/*
    Name:       BatterySourceCharger.ino
    Author:     Andreas

    ATMega328P 16MHz (ProMini)

	I2C 0x40	INA238	MainDC
	I2C 0x41	INA238	SolarDC
	I2C 0x43	INA238	HelpDC
	I2C 0x44	INA238	DCDC
	I2C 0x45	INA238	Battery
	I2C 0x48	ADS1015	ADC-2xTemp-Fan
	I2C 0x60	MCP4725	DAC
	
	PD5			SW-SolarDC
	PD4			SW_HelpDC
	PD7			SW-MainDC
	PD6			SW-Charge    HIGH active, LOW = OFF
	PC1			DCDC_EN
	
	Input
	PC0			DCDC_PG
	
	PD2			Button_Set
	ADC6		Button_Down 460
	ADC6		Button_Up   703
	ADC6		Button_OK	383
	
	ADC7		LightIntensity Display
	
	Display		GMG12864-06D    ST7565
	// CS  = PB2
	// RSE = PB0   // Reset
	// RS  = PB1   // A0 / Data-Command
	// SCL = PB5
	// SI  = PB3
	
	UART		Serial communication
*/

#include <Arduino.h>
#include <Wire.h>
#include <avr/pgmspace.h>

// ============================================================
// System Parameters
// ============================================================

#define UART_BAUDRATE              115200
#define I2C_CLOCK_HZ               100000UL
#define MEASURE_INTERVAL_MS        100UL
#define DISPLAY_INTERVAL_MS        500UL         // Slow display refresh for readable values.


// ============================================================
// Charger Communication Protocol
// ============================================================
// Common Source <-> Assist protocol layout.
// BatteryProtocolMonitor can passively decode the same frames.

#define PROTOCOL_START_BYTE_1       0xA5U
#define PROTOCOL_START_BYTE_2       0x5AU
#define PROTOCOL_VERSION            0x01U

#define MESSAGE_MASTER_REQUEST      0x0001U
#define MESSAGE_CLIENT_RESPONSE     0x0002U
#define MESSAGE_COMMAND             0x0003U
#define MESSAGE_COMMAND_RESPONSE    0x0004U
#define MESSAGE_DIAGNOSTIC          0x0005U

#define PROTOCOL_PAYLOAD_LENGTH     21U
#define PROTOCOL_FRAME_LENGTH       30U

#define COMMUNICATION_INTERVAL_MS       1000UL
#define COMMUNICATION_RX_TIMEOUT_MS       20UL		// Partial-frame timeout at 115200 baud. Fix!Me
#define COMMUNICATION_LINK_TIMEOUT_MS    3000UL		// No valid Assist response for this time marks the link stale. Fix!Me

struct MasterRequestPayload
{
    uint16_t batteryVoltage_10mV;
    int16_t batteryCurrent_10mA;
    uint16_t sourcePower_W;
    uint16_t safeSourcePower_W;
    uint16_t grantedAssistPower_W;
    uint8_t batterySoc_percent;
    uint16_t sourceMode;
    uint16_t mpptState;
    uint16_t dcdcState;
    uint16_t warningFlags;
    uint16_t errorFlags;
};

struct ClientResponsePayload
{
    uint16_t batteryVoltage_10mV;
    int16_t batteryCurrent_10mA;
    uint16_t requestedInputPower_W;
    uint16_t actualInputPower_W;
    uint8_t batterySoc_percent;
    uint16_t chargeState;
    uint16_t operatingState;
    uint16_t batteryCapacity_Ah;
    uint16_t batteryInfoCode;
    uint16_t warningFlags;
    uint16_t errorFlags;
};

// Last valid CLIENT_RESPONSE received from BatteryAssistCharger.
// These values are diagnostic/system information; Source charge control remains autonomous.
ClientResponsePayload receivedClientResponse = {};
bool receivedClientResponseValid = false;
bool communicationLinkTimeout = false;
bool communicationAwaitingResponse = false;
uint8_t receivedClientResponseSequence = 0U;
uint8_t communicationNextSequence = 0U;
uint8_t communicationLastSentSequence = 0U;
uint32_t receivedClientResponseTime_ms = 0UL;
uint32_t communicationFirstRequestTime_ms = 0UL;
uint32_t communicationReceiveLastByte_ms = 0UL;

uint32_t communicationRxCrcErrors = 0UL;
uint32_t communicationRxFormatErrors = 0UL;
uint32_t communicationRxDataErrors = 0UL;
uint32_t communicationRxTimeoutErrors = 0UL;
uint32_t communicationRxSequenceErrors = 0UL;

static uint8_t communicationReceiveFrame[PROTOCOL_FRAME_LENGTH];
static uint8_t communicationReceiveIndex = 0U;

// Communication transmit and receive functions.
void Communication_Task(uint32_t now_ms);
void Communication_SendFrame(uint16_t messageType, uint8_t sequence, const uint8_t *payload);
void Communication_WriteU16(uint8_t *buffer, uint8_t *index, uint16_t value);
uint16_t Communication_UpdateCrc16(uint16_t crc, uint8_t value);
void CommunicationReceive_Task();
bool CommunicationReceive_ValidateFrame(const uint8_t *frame);
bool CommunicationReceive_ClientDataPlausible(const ClientResponsePayload *data);
uint16_t CommunicationReceive_ReadU16(const uint8_t *buffer, uint8_t *index);
void CommunicationReceive_Reset();

// ============================================================
// I2C Addresses
// ============================================================

#define INA238_MAINDC_ADDR         0x40
#define INA238_SOLARDC_ADDR        0x41
#define INA238_HELPDC_ADDR         0x43
#define INA238_DCDC_ADDR           0x44
#define INA238_BATTERY_ADDR        0x45

#define ADS1015_ADDR               0x48
#define MCP4725_ADDR               0x60


// ============================================================
// MCP4725 DAC Parameters
// ============================================================

#define DAC_CODE_MIN               0
#define DAC_CODE_MAX               4095
#define DAC_VREF_mV                5000L

// DCDC output model from two 24V measurement runs:
// DCDC_mV = 30113 - 2.404 * DAC_Code
#define DCDC_DAC_OFFSET_mV         30113L
#define DCDC_DAC_SLOPE_uV_CODE     2404L        // measured: 2.404mV per DAC code

#define DCDC_VOUT_MIN_mV           20250L
#define DCDC_VOUT_MAX_mV           28200L

#define DAC_RAMP_STEP_CODE         2000         // Fast step for open Battery-DCDC switch test.
#define DAC_RAMP_STEP_DELAY_MS     0UL

// ============================================================
// DCDC State Machine Parameters
// ============================================================

#define DCDC_TASK_INTERVAL_MS      100UL
#define DCDC_TARGET_TOLERANCE_mV   50L
#define DCDC_TARGET_OFFSET_mV       0L
#define DCDC_SAFE_START_CODE        DAC_CODE_MAX // Highest DAC code gives lowest DCDC voltage.
#define DCDC_MAX_POWER_W             350L		// Configurable continuous DC/DC limit; thermal derating uses this ceiling. Fix!Me

// ============================================================
// DCDC Pre-Charge Self Test
// ============================================================
// SW-Charge remains open while INA_DCDC verifies the real converter output.
// Function test and battery connection use separate voltage levels:
// 1. Match the real DCDC output to the battery.
// 2. Raise the open output by 100mV and verify the rise with INA_DCDC.
// 3. Set the DAC to the calculated battery +20mV connection level.
//    The output capacitors are not required to discharge to this level before CHARGE closes.
#define DCDC_TEST_MIN_OUTPUT_mV          18000L
#define DCDC_TEST_RAISE_OFFSET_mV          100L   // Open-output function-test voltage above battery.
#define DCDC_TEST_RISE_MIN_mV               50L   // Minimum measured rise proving DCDC response.
#define DCDC_TEST_MATCH_TOL_mV               20L   // Allowed difference while matching battery voltage.
#define DCDC_TEST_DAC_STEP_CODE               2U   // Fine DAC steps during function test.
#define DCDC_CONNECT_OFFSET_mV                20L   // DAC target above battery when SW-Charge closes.
#define DCDC_CONNECT_SETTLE_MS               300UL  // Let the new DAC command settle; do not wait for COUT discharge.
#define DCDC_TEST_STABLE_COUNT                3U
#define DCDC_TEST_PHASE_TIMEOUT_MS         5000UL
#define DCDC_REVERSE_CURRENT_LIMIT_mA       -50L   // More negative than this starts the reverse-current timer.
#define DCDC_REVERSE_CURRENT_TIME_MS       4000UL  // Allow the normal current ramp several seconds to recover.
#define DCDC_REVERSE_RETRY_MS             60000UL  // Retry after another charger caused persistent reverse current. Fix!Me

#define SOURCE_WARNING_THERMAL_DERATING   0x0001U  // Bit 0: DC/DC power is thermally derated.
#define SOURCE_WARNING_RECOVERY_LONG      0x0002U  // Bit 1: battery BMS recovery has been active unusually long.
#define SOURCE_ERROR_DCDC_PRECHARGE       0x0001U  // Bit 0: DC/DC self-test/precharge failure.
#define SOURCE_ERROR_BATTERY_OVERVOLTAGE  0x0002U  // Bit 1: battery reached the hard-stop voltage.
#define SOURCE_ERROR_OVERTEMPERATURE      0x0004U  // Bit 2: FET or coil reached thermal shutdown.

// ============================================================
// 24V Battery / Recovery Parameters
// ============================================================

#define BATTERY_24V_HARD_STOP_mV       28200L
#define BATTERY_24V_BMS_LOW_mV         21000L

// External battery BMS recovery.
// Below 21V the BMS may have disconnected the cells from the external terminals.
// The DC/DC first holds about 21.5V with a very small current ceiling. As the BMS
// wakes and terminal voltage rises, the current limit is increased only after 23V.
#define BATTERY_RECOVERY_TARGET_mV       21500L		// Practically tested wake voltage for this external BMS.
#define BATTERY_RECOVERY_WAKE_mV         22000L		// Transition region; still keep the initial low current. Fix!Me
#define BATTERY_RECOVERY_RAMP_mV         23000L		// Start the slow current ramp only above this terminal voltage. Fix!Me
#define BATTERY_RECOVERY_CURRENT_mA        100L		// Current ceiling while the BMS is still waking. Fix!Me
#define BATTERY_RECOVERY_RAMP_INTERVAL_MS 30000UL		// Delay between recovery current stages. Fix!Me
#define BATTERY_RECOVERY_LONG_MS        7200000UL		// Warning after two hours; recovery continues if otherwise safe. Fix!Me

#define BATTERY_RECOVERY_STAGE_0_mA BATTERY_RECOVERY_CURRENT_mA		// Initial BMS wake current ceiling.
#define BATTERY_RECOVERY_STAGE_1_mA        250L		// Recovery ramp stage. Fix!Me
#define BATTERY_RECOVERY_STAGE_2_mA        500L		// Recovery ramp stage. Fix!Me
#define BATTERY_RECOVERY_STAGE_3_mA       1000L		// Recovery ramp stage. Fix!Me
#define BATTERY_RECOVERY_STAGE_4_mA       2000L		// Recovery ramp stage. Fix!Me
#define BATTERY_RECOVERY_STAGE_5_mA       3000L		// Recovery ramp stage. Fix!Me
#define BATTERY_RECOVERY_STAGE_6_mA       4000L		// Recovery ramp stage. Fix!Me
#define BATTERY_RECOVERY_STAGE_7_mA BATTERY_MAX_CHARGE_CURRENT_mA		// Final recovery stage before normal control. Fix!Me

#define CHARGE_START_CURRENT_mA              50L
#define BATTERY_MAX_CHARGE_CURRENT_mA      5000L		// Present Source/Assist battery-current ceiling. Fix!Me
#define CHARGE_TAPER_START_mV             28000L
#define CHARGE_TAPER_END_mV               28100L
#define CHARGE_TAPER_MIN_mA                  50L

// Absolute DAC floor from the measured 24V transfer curve.
// Code 796 corresponds to approximately 28.20V. Software voltage protection
// remains authoritative and disconnects before the BMS high cutoff.
#define DCDC_DAC_MIN_SAFE                    796

#define MAIN_CHARGE_CURRENT_SET_mA  BATTERY_MAX_CHARGE_CURRENT_mA
#define MAIN_CHARGE_CURRENT_MAX_mA  BATTERY_MAX_CHARGE_CURRENT_mA
#define HELP_CHARGE_CURRENT_SET_mA  BATTERY_MAX_CHARGE_CURRENT_mA
#define HELP_CHARGE_CURRENT_MAX_mA  BATTERY_MAX_CHARGE_CURRENT_mA
#define SOLAR_CHARGE_CURRENT_MIN_mA CHARGE_START_CURRENT_mA
#define SOLAR_CHARGE_CURRENT_SET_mA BATTERY_MAX_CHARGE_CURRENT_mA
#define SOLAR_CHARGE_CURRENT_MAX_mA BATTERY_MAX_CHARGE_CURRENT_mA


// Solar hybrid power P&O regulation:
// P&O perturbs an INPUT-POWER request, not the battery-current request directly.
// The requested input power is converted to a safe battery-current target using
// a conservative efficiency estimate. This permits Buck, Buck-Boost and Boost
// operation without demanding more output power than the source can provide.
#define SOLAR_REGULATION_INTERVAL_BUCK_MS       500UL
#define SOLAR_REGULATION_INTERVAL_TRANS_MS      750UL
#define SOLAR_REGULATION_INTERVAL_NEAR_MS      1000UL
#define SOLAR_REGULATION_INTERVAL_BOOST_MS     1500UL

// Dynamic operating regions relative to the actual battery voltage.
#define SOLAR_BUCK_HEADROOM_GOOD_mV             4000L
#define SOLAR_BUCK_HEADROOM_MIN_mV              1500L
#define SOLAR_NEAR_BATTERY_BAND_mV                 0L

// Input-power perturbation sizes.
#define SOLAR_POWER_STEP_BUCK_mW                1000L
#define SOLAR_POWER_STEP_TRANS_mW                250L
#define SOLAR_POWER_STEP_NEAR_mW                 100L
#define SOLAR_POWER_STEP_BOOST_mW                 50L
#define SOLAR_POWER_STEP_MIN_mW                   50L
#define SOLAR_POWER_FAST_DOWN_mW                2000L
#define SOLAR_POWER_CRITICAL_DOWN_mW            5000L
#define SOLAR_POWER_HYST_mW                      100L
#define SOLAR_POWER_GAIN_MIN_mW                  100L

// Conservative conversion from available input power to battery current.
#define SOLAR_EFF_BUCK_PERMILLE                  900L
#define SOLAR_EFF_TRANS_PERMILLE                 850L
#define SOLAR_EFF_NEAR_PERMILLE                  820L
#define SOLAR_EFF_BOOST_PERMILLE                 780L

// Weak-source and collapse handling. 18V operation is intentionally allowed.
#define SOLAR_ACTIVE_HOLD_MIN_mV               10000L
#define SOLAR_POWER_HARD_FLOOR_mV              10000L
#define SOLAR_RAPID_DROP_mV                     1500L
#define SOLAR_SOFT_DROP_mV                       250L

// Source-voltage reserve control:
// Remember the unloaded/healthy source voltage and allow approximately 1V drop.
// A larger drop without a useful power gain means that the source current limit
// or the weak-source knee has been reached. P&O then holds or reduces power.
#define SOLAR_VOLTAGE_DROP_TARGET_mV             6800L
#define SOLAR_VOLTAGE_DROP_HYST_mV                150L
#define SOLAR_REFERENCE_REBASE_CURRENT_mA         150L
#define SOLAR_REFERENCE_REBASE_CYCLES               3U

// Fast weak-source guard. This runs in the normal 100ms control path, not only
// in the slower P&O interval. It removes load before a current-limited source
// or a shaded solar panel can collapse deeply and excite the DCDC magnetics.
#define SOLAR_FAST_GUARD_HALF_DROP_mV             7500L
#define SOLAR_FAST_GUARD_QUARTER_DROP_mV          9000L
#define SOLAR_FAST_GUARD_COLLAPSE_DROP_mV        11000L
#define SOLAR_FAST_GUARD_COLLAPSE_PERCENT            70L
#define SOLAR_FAST_GUARD_RETRIGGER_MS               200UL
#define SOLAR_FAST_GUARD_RECOVERY_MS               2000UL
#define SOLAR_FAST_GUARD_DAC_RELEASE_CODE             20U

// Invalidate an old learned source limit when the available source power has
// changed substantially. Medium changes keep only history points below the new
// safe estimate; large changes discard the stable history completely.
#define SOLAR_SAFE_KEEP_PERCENT                        85L
#define SOLAR_SAFE_CLEAR_PERCENT                       70L

// Learned recovery using recent measurements and confirmed stable operating
// points. After a collapse, return quickly near the last safe power and then
// continue with small steps while watching the input-voltage trend.
#define SOLAR_TREND_HISTORY_SIZE                      10U
#define SOLAR_STABLE_HISTORY_SIZE                      8U
#define SOLAR_STABLE_VDROP_MAX_mV                   5680L
#define SOLAR_TREND_SOFT_DROP_mV                     150L
#define SOLAR_TREND_POWER_GAIN_MIN_mW                 50L
#define SOLAR_SAFE_POWER_MARGIN_mW                  1500L
#define SOLAR_RECOVERY_FAST_STEP_mW                 1000L
#define SOLAR_RECOVERY_MID_STEP_mW                   500L
#define SOLAR_RECOVERY_FINE_STEP_mW                  100L
#define SOLAR_RECOVERY_FINE_BAND_mW                 1500L
#define SOLAR_RECOVERY_SOFT_HOLD_MS                 1000UL
#define SOLAR_RECOVERY_TARGET_STABLE_MS             2000UL

#define SOLAR_TRACK_TOL_mA                       100L
#define SOLAR_POWER_SET_MIN_mW                  1000L
#define SOLAR_POWER_SET_MAX_mW                180000L


// LOW_SOLAR mode for sunrise / sunset.
// Below the normal MPPT current range the controller no longer performs P&O.
// Instead it slowly increases/decreases the battery-current request while
// keeping the solar input near a usable voltage floor. This avoids repeated
// collapse/restart cycles at very low irradiance.
#define SOLAR_LOW_ENTER_CURRENT_mA              350L
#define SOLAR_LOW_EXIT_CURRENT_mA               450L
#define SOLAR_LOW_ENTER_TIME_MS                5000UL
#define SOLAR_LOW_EXIT_TIME_MS                 5000UL

// LOW_SOLAR knee detection.
#define SOLAR_LOW_CURRENT_STEP_mA                 50L
#define SOLAR_LOW_CONTROL_INTERVAL_MS           1000UL
#define SOLAR_LOW_DROP_MIN_mV                    1000L
#define SOLAR_LOW_DROP_INCREASE_mV                500L
#define SOLAR_LOW_BAT_MARGIN_mV                   750L
#define SOLAR_LOW_HOLD_MS                       10000UL

// True source loss is separate from merely operating below 34V.
#define SOLAR_LOW_SOURCE_LOST_MARGIN_mV             0L
#define SOLAR_LOW_SOURCE_LOST_TIME_MS            3000UL
#define SOLAR_LOW_OFF_CURRENT_mA                  100L
#define SOLAR_LOW_OFF_TIME_MS                   30000UL
#define SOLAR_LOW_RETRY_PAUSE_MS               120000UL

// MPPT step sizes retained by the low-solar state machine.
#define SOLAR_MPPT_STEP_MIN_mA                    50L
#define SOLAR_MPPT_STEP_NORMAL_mA                100L
#define SOLAR_MPPT_STEP_LARGE_mA                 250L
#define SOLAR_MPPT_FAST_DOWN_mA                 1000L
#define SOLAR_MPPT_POWER_HYST_uW              100000LL
#define SOLAR_MPPT_FINE_DIFF_uW               500000LL
#define SOLAR_MPPT_LARGE_DIFF_uW             2000000LL

// Input current limits per input source:
// The charge current target is reduced when the measured input current reaches this limit.
#define MAIN_INPUT_CURRENT_MAX_mA    9000L
#define SOLAR_INPUT_CURRENT_MAX_mA   6000L
#define HELP_INPUT_CURRENT_MAX_mA    2000L

// Input current regulation:
// Reduce charge current fast at input overcurrent, increase it slowly when input current has margin again.
#define INPUT_CURRENT_REG_INTERVAL_MS 100UL
#define INPUT_CURRENT_HYST_mA        200L
#define INPUT_CURRENT_STEP_UP_mA     100L
#define INPUT_CURRENT_STEP_DOWN_mA   300L
#define INPUT_CURRENT_FAST_DOWN_mA   1000L

// ============================================================
// Battery Charge End / Float Parameters
// ============================================================
// EnerSys SBS60: 12V, 51Ah, 1.80V/cell end of discharge, float voltage 2.29V/cell.
// Typical car battery note: 12V, 120Ah, similar basic voltage ranges, but larger capacity.
// 24V charging strategy. The normal full target remains below the 28.25V BMS cutoff.
#define BATTERY_FLOAT_mV             27800L
#define BATTERY_HIGH_CURRENT_MIN_mV  21000L
#define BATTERY_HIGH_CURRENT_MAX_mV  27800L
#define BATTERY_ABSORPTION_mV        28100L
#define BATTERY_FLOAT_CURRENT_mA        50L
#define BATTERY_VOLTAGE_TAPER_mV       100L   // Full detection current. Car battery 120Ah: typical 0.2A to 0.5A.
#define BATTERY_ABSORB_TOLERANCE_mV     50L   // Voltage must be near absorption before full current is accepted.

// One complete 28.10V charge cycle after a real night / long solar absence.
// Short clouds must not arm another full cycle.
#define BATTERY_DAILY_RESET_NO_SOLAR_MS  14400000UL  // 4 hours

#define CHARGE_CURRENT_HYST_mA       20L          // Normal current-control dead band.
#define CHARGE_MAX_OFFSET_mV          300L        // DCDC may rise only this much above battery voltage.
#define CHARGE_RAMP_INTERVAL_MS     100UL        // Current control interval.
#define CHARGE_POSITIVE_START_MS    1500UL       // After SW-Charge ON, only search in positive direction.

// Current control stabilizing:
// The INA current value is filtered before control.
// This avoids reaction to single noisy samples.
#define CHARGE_CURRENT_FILTER_DIV   4L           // 4 = slow and stable, 2 = faster.
// Current safety limits are deliberately separated from the 5000mA MPPT ceiling.
// A small overshoot is corrected by one DAC code and is not treated as a fault.
#define CHARGE_ABSOLUTE_MAX_mA      6000L          // Moderate fast reduction above this raw/filtered current.
#define CHARGE_SEVERE_OVER_mA       7000L          // Emergency reduction above this raw/filtered current.
#define CHARGE_CORRECT_STEP_CODE       1           // Normal correction: approximately 2.4mV.
#define CHARGE_FAST_STEP_CODE          4           // Fast correction: approximately 9.6mV.
#define CHARGE_EMERGENCY_STEP_CODE    12           // Emergency correction: approximately 28.8mV.
#define CHARGE_STABLE_COUNT           10           // 1s stable at final target before READY.

// ============================================================
// ST7565 Display Pins - Port B
// ============================================================

#define LCD_RST                    PB0
#define LCD_A0                     PB1
#define LCD_CS                     PB2
#define LCD_SDA                    PB3
#define LCD_SCK                    PB5

#define LCD_RST_M                  _BV(LCD_RST)
#define LCD_A0_M                   _BV(LCD_A0)
#define LCD_CS_M                   _BV(LCD_CS)
#define LCD_SDA_M                  _BV(LCD_SDA)
#define LCD_SCK_M                  _BV(LCD_SCK)

#define LCD_WIDTH                  128
#define LCD_PAGES                  8

// ============================================================
// Port D Outputs
// ============================================================
// Source switches are active LOW.
// Charge switch PD6 is active HIGH.


#define PIN_DISPLAY_BACKLIGHT      PD3
#define PIN_SW_SOLARDC             PD5
#define PIN_SW_HELPDC              PD4
#define PIN_SW_MAINDC              PD7
#define PIN_SW_CHARGE              PD6

#define MASK_DISPLAY_BACKLIGHT     _BV(PIN_DISPLAY_BACKLIGHT)
#define MASK_SW_SOLARDC            _BV(PIN_SW_SOLARDC)
#define MASK_SW_HELPDC             _BV(PIN_SW_HELPDC)
#define MASK_SW_MAINDC             _BV(PIN_SW_MAINDC)
#define MASK_SW_CHARGE             _BV(PIN_SW_CHARGE)

#define PORTD_OUTPUT_MASK          (MASK_DISPLAY_BACKLIGHT | MASK_SW_SOLARDC | MASK_SW_HELPDC | MASK_SW_MAINDC | MASK_SW_CHARGE)

// ============================================================
// Port C Inputs / Outputs
// ============================================================

#define PIN_EN_DCDC                PC1
#define PIN_PG_DCDC                PC0

#define MASK_EN_DCDC               _BV(PIN_EN_DCDC)
#define MASK_PG_DCDC               _BV(PIN_PG_DCDC)

// ============================================================
// Button / ADC Parameters
// ============================================================

#define PIN_BUTTON_SET             PD2
#define MASK_BUTTON_SET            _BV(PIN_BUTTON_SET)

#define ADC_BUTTONS                6
#define ADC_LIGHT                  7

#define ADC_BTN_OK_VALUE           383
#define ADC_BTN_DOWN_VALUE         460
#define ADC_BTN_UP_VALUE           703
#define BUTTON_ADC_TOLERANCE       60

#define BUTTON_NONE                0
#define BUTTON_OK                  1
#define BUTTON_DOWN                2
#define BUTTON_UP                  3

// ============================================================
// Display Backlight
// ============================================================

#define DISPLAY_LIGHT_TIME_MS      60000UL

// ============================================================
// Display Layout
// ============================================================

#define DISPLAY_LEFT_X             2
#define DISPLAY_RIGHT_X            66
#define DISPLAY_COLUMN_W           60
#define DISPLAY_LABEL_Y            0
#define DISPLAY_SEPARATOR_Y       10            // Small line below the battery headline.
#define DISPLAY_VOLTAGE_Y          16
#define DISPLAY_CURRENT_Y          40            // Leave one empty text line between voltage and current.
#define DISPLAY_TEMP_Y             56            // Bottom line, right side.
#define DISPLAY_STATE_Y            56            // Bottom line, left side.
#define DISPLAY_CURRENT_MIN_mA     50L          // Hide current below this value.

// ============================================================

// ============================================================
// INA238 Parameters
// ============================================================

#define INA238_CURRENT_LSB_uA      1000L       // 1mA/bit, supports up to about 32A

#define SHUNT_MAINDC_mOHM          9           // 0.009 Ohm
#define SHUNT_SOLARDC_mOHM         9           // 0.009 Ohm
#define SHUNT_DCDC_mOHM            5           // 0.005 Ohm
#define SHUNT_BATTERY_mOHM         5           // 0.005 Ohm
#define SHUNT_HELPDC_mOHM          50          // 0.050 Ohm

// INA238 current correction.
// 1000 = no correction.
// 2000 = current value x 2.000.
// Use this for final calibration with a lab power supply or current meter.
#define INA238_CORR_MAINDC_PERMILLE   2000L    // Input current correction. Car battery 120Ah: same calibration method.
#define INA238_CORR_SOLARDC_PERMILLE  1000L    // 24V test calibration: INA value matched to laboratory PSU current.
#define INA238_CORR_HELPDC_PERMILLE   2000L    // Input current correction. Car battery 120Ah: same calibration method.
#define INA238_CORR_DCDC_PERMILLE     1000L    // Charge converter current correction.
#define INA238_CORR_BATTERY_PERMILLE  1000L    // Battery current correction.

#define INA238_REG_CONFIG          0x00
#define INA238_REG_ADC_CONFIG      0x01
#define INA238_REG_SHUNT_CAL       0x02
#define INA238_REG_VSHUNT          0x04
#define INA238_REG_VBUS            0x05
#define INA238_REG_CURRENT         0x07
#define INA238_REG_DEVICE_ID       0x3F

#define INA238_CONFIG_VALUE        0x0000
#define INA238_ADC_CONFIG_VALUE    0xFB68

#define CH_MAINDC                  0
#define CH_SOLARDC                 1
#define CH_DCDC                    2
#define CH_BATTERY                 3
#define CH_HELPDC                  4

struct INA238_Channel
{
	uint8_t address;
	uint16_t shunt_mOhm;
	int32_t currentCorrPermille;
	int32_t voltage_mV;
	int32_t current_mA;
	bool online;
};

INA238_Channel ina238[] =
{
	{ INA238_MAINDC_ADDR,  SHUNT_MAINDC_mOHM,  INA238_CORR_MAINDC_PERMILLE,  0, 0, false },
	{ INA238_SOLARDC_ADDR, SHUNT_SOLARDC_mOHM, INA238_CORR_SOLARDC_PERMILLE, 0, 0, false },
	{ INA238_DCDC_ADDR,    SHUNT_DCDC_mOHM,    INA238_CORR_DCDC_PERMILLE,    0, 0, false },
	{ INA238_BATTERY_ADDR, SHUNT_BATTERY_mOHM, INA238_CORR_BATTERY_PERMILLE, 0, 0, false },
	{ INA238_HELPDC_ADDR,  SHUNT_HELPDC_mOHM,  INA238_CORR_HELPDC_PERMILLE,  0, 0, false }
};

#define INA238_COUNT (sizeof(ina238) / sizeof(ina238[0]))

// ============================================================
// Input Source Auto Switch Parameters
// ============================================================

#define SOLAR_ON_mV        18000L
#define MAIN_ON_mV         24000L
#define HELPDC_ON_mV       15000L

// ============================================================

// Start delay sequence:
// 1. Wait before an available input source is switched ON.
// 2. Wait before DCDC is enabled.
// 3. Wait before SW-Charge connects DCDC to the battery.
#define SOURCE_ON_DELAY_MS  1000UL
#define DCDC_ON_DELAY_MS    1000UL
#define CHARGE_ON_DELAY_MS  1000UL

// ============================================================
// DCDC State Machine
// ============================================================

enum DCDC_State
{
	DCDC_STATE_OFF = 0,
	DCDC_STATE_START,
	DCDC_STATE_CHARGE_ON,
	DCDC_STATE_RAMP_CURRENT,
	DCDC_STATE_READY
};

// Internal self-test state. It is not part of the communication protocol.
enum DCDC_PreChargeTest_State
{
	DCDC_TEST_IDLE = 0,
	DCDC_TEST_MATCH_BATTERY,
	DCDC_TEST_RAISE_OUTPUT,
	DCDC_TEST_CONNECT_LEVEL,
	DCDC_TEST_PASSED,
	DCDC_TEST_FAILED
};

enum InputSource_Id
{
	INPUT_SOURCE_NONE = 0,
	INPUT_SOURCE_MAIN,
	INPUT_SOURCE_SOLAR,
	INPUT_SOURCE_HELP
};

enum SolarRegulation_Status
{
	SOLAR_REG_IDLE = 0,
	SOLAR_REG_FAST_DOWN,
	SOLAR_REG_DOWN,
	SOLAR_REG_SLOW_DOWN,
	SOLAR_REG_HOLD,
	SOLAR_REG_UP
};

// Battery charge state:
// WAIT: no charge until the battery is low enough.
// BULK: charge with current regulation up to absorption voltage.
// ABS: hold absorption voltage until battery current is below the full threshold.
// FULL: charger off until restart voltage is reached.
enum BatteryCharge_State
{
	BATTERY_CHARGE_WAIT = 0,
	BATTERY_CHARGE_BULK,
	BATTERY_CHARGE_ABS,
	BATTERY_CHARGE_FULL
};

bool DCDC_ControlOneStepToCurrent(uint32_t now_ms);
void DCDC_ResetCurrentControl();
void ChargeCurrent_Task(uint32_t now_ms);
int32_t ChargeCurrent_GetSet_mA();
int32_t ChargeCurrent_GetMax_mA();
int32_t BatteryCharge_SelectCurrent_mA(int32_t normalCurrent_mA, int32_t maximumCurrent_mA);
int32_t InputCurrentLimit_Task(uint32_t now_ms, uint8_t source, int32_t requestedSet_mA);
int32_t InputCurrent_Get_mA(uint8_t source);
int32_t InputVoltage_Get_mV(uint8_t source);
int32_t InputCurrent_GetMax_mA(uint8_t source);
void SolarChargeCurrent_Task(uint32_t now_ms);
int32_t SolarPowerToBatteryCurrent_mA(int32_t inputPower_mW, int32_t batteryVoltage_mV, int32_t efficiency_permille);
int32_t SolarBatteryCurrentToInputPower_mW(int32_t batteryCurrent_mA, int32_t batteryVoltage_mV, int32_t efficiency_permille);
void SolarMPPT_Reset();
void SolarHistory_Reset();
void SolarTrend_Add(int32_t powerSet_mW, int32_t voltage_mV, int32_t measuredPower_mW);
bool SolarTrend_Get(int32_t *voltageDelta_mV, int32_t *powerDelta_mW);
void SolarStableHistory_Add(int32_t powerSet_mW, int32_t voltage_mV, int32_t measuredPower_mW);
void SolarStableHistory_Clear();
void SolarStableHistory_RemoveAbove(int32_t maximumPower_mW);
int32_t SolarStableHistory_FindSafePower_mW(int32_t failedPower_mW);
void BatteryCharge_Task();
bool BatteryCharge_IsAllowed();
const char* BatteryCharge_StateText(BatteryCharge_State state);
bool DCDC_SetStartVoltageFromBattery();
int32_t DCDC_GetStartTargetFromBattery_mV();
int32_t DCDC_GetMaxTargetFromBattery_mV();
int32_t DCDC_GetTargetFromBattery_mV();
void BatteryRecovery_Task(uint32_t now_ms);
void BatteryRecovery_Start(uint32_t now_ms);
void BatteryRecovery_Stop();
int32_t BatteryRecovery_GetStageCurrent_mA(uint8_t stage);
int32_t DCDC_GetAllowedPower_W();
int32_t DCDC_GetPowerLimitedCurrent_mA();
void TemperatureProtection_Task();
void DCDC_PreChargeTest_Reset();
void DCDC_PreChargeTest_Fail();
bool DCDC_PreChargeTest_AdjustToVoltage(int32_t target_mV);
bool DCDC_PreChargeTest_Task(uint32_t now_ms);
void DCDC_ReverseCurrentCheck_Reset();
bool DCDC_ReverseCurrentCheck_Task(uint32_t now_ms);
void DCDC_AllOff();
void ButtonBacklight_Task(uint32_t now_ms);

// ============================================================
// ADS1015 / NTC Parameters
// ============================================================

#define ADS1015_REG_CONVERSION     0x00
#define ADS1015_REG_CONFIG         0x01
#define ADS1015_REG_LO_THRESH      0x02
#define ADS1015_REG_HI_THRESH      0x03

#define ADS1015_OS_SINGLE          0x8000
#define ADS1015_PGA_6V144          0x0000
#define ADS1015_MODE_SINGLE        0x0100
#define ADS1015_MODE_CONTINUOUS    0x0000
#define ADS1015_DR_1600SPS         0x0080
#define ADS1015_COMP_DISABLE       0x0003

#define ADS1015_MUX_A0_A1          0x0000
#define ADS1015_MUX_A2_A3          0x3000

#define ADS1015_COMP_QUE_1         0x0000
#define ADS1015_COMP_MODE_TRAD     0x0000
#define ADS1015_COMP_POL_LOW       0x0000
#define ADS1015_COMP_LAT_NON       0x0000

// ============================================================
// Fan / Temperature Control
// ============================================================

// Fan and thermal protection follow the BatteryCharger_24V strategy.
#define FAN_CHECK_INTERVAL_MS        1000UL		// Temperature sampling interval. Fix!Me
#define FAN_AFTER_RUN_OFF_C            35		// Fan stops only when FET and coil are both below this temperature.
#define DCDC_DERATING_START_C          65		// Full configured power is allowed up to this temperature. Fix!Me
#define DCDC_DERATING_70C_POWER_W     260L		// Allowed power at 70C. Fix!Me
#define DCDC_DERATING_75C_POWER_W     175L		// Allowed power at 75C. Fix!Me
#define DCDC_DERATING_80C_POWER_W     100L		// Allowed power at 80C. Fix!Me
#define DCDC_SHUTDOWN_TEMP_C           85		// Charging stops when either sensor reaches this temperature. Fix!Me
#define DCDC_RESTART_TEMP_C            55		// Restart only when both sensors are at or below this temperature. Fix!Me

struct NTC_TableEntry
{
	int16_t temp_C;
	int16_t raw;
};

const NTC_TableEntry ntcTable[] PROGMEM =
{
	{ -10, 880 },
	{   0, 790 },
	{  10, 690 },
	{  20, 585 },
	{  25, 535 },
	{  30, 490 },
	{  40, 405 },
	{  50, 330 },
	{  60, 270 },
	{  70, 220 },
	{  80, 180 }
};

#define NTC_TABLE_COUNT (sizeof(ntcTable) / sizeof(ntcTable[0]))

// ============================================================
// Runtime Variables
// ============================================================

// Timing variables are separated by task.
uint32_t lastMeasure_ms = 0;             // INA238 measurement interval.
uint32_t lastControl_ms = 0;             // Charge current control interval.
uint32_t lastDisplay_ms = 0;             // LCD display refresh interval.
uint32_t lastDcdcTask_ms = 0;            // DCDC state machine interval.
uint32_t dcdcDelayStart_ms = 0;          // Delay timer for DCDC start sequence.
uint32_t sourceDelayStart_ms = 0;        // Delay timer before source switch ON.
uint8_t inputSourcePending = INPUT_SOURCE_NONE; // Source waiting for SOURCE_ON_DELAY_MS.
uint32_t lastSolarRegulation_ms = 0;     // Solar current regulation interval.
uint32_t lastInputCurrentRegulation_ms = 0; // Input current limit regulation interval.
uint32_t lastFanCheck_ms = 0;            // Fan temperature check interval.
uint32_t lightTimer_ms = 0;              // Display backlight timeout.
bool lastBacklightSetPressed = false;      // Last SET button state for backlight trigger.
uint8_t lastBacklightAdcButton = BUTTON_NONE; // Last ADC button state for backlight trigger.



volatile bool buttonSetIrqFlag = false;


bool stateSolarDC = false;
bool stateHelpDC = false;
bool stateMainDC = false;
bool stateCharge = false;
bool stateDCDC = false;
bool fanState = false;

int16_t fetTemperature_C = 0;
int16_t coilTemperature_C = 0;

uint16_t dacCode = 0;
uint32_t chargePositiveStart_ms = 0;
int32_t chargeCurrentFiltered_mA = 0;
int32_t activeChargeCurrentSet_mA = 0L;
int32_t activeChargeCurrentMax_mA = 0L;
int32_t solarChargeCurrentSet_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
int32_t inputLimitedChargeCurrentSet_mA = 0L;
uint8_t inputLimitSource = INPUT_SOURCE_NONE;
SolarRegulation_Status solarRegulationStatus = SOLAR_REG_IDLE;
int64_t solarMpptLastPower_uW = 0LL;
int64_t solarMpptPower_uW = 0LL;
int64_t solarMpptPowerDiff_uW = 0LL;
int32_t solarMpptStep_mA = SOLAR_MPPT_STEP_NORMAL_mA;
int8_t solarMpptDirection = 1;
bool solarMpptValid = false;
int32_t solarMpptPowerSet_mW = SOLAR_POWER_SET_MIN_mW;
int32_t solarMpptLastVoltage_mV = 0L;
int32_t solarMpptPowerStep_mW = SOLAR_POWER_STEP_BUCK_mW;
int32_t solarMpptEfficiency_permille = SOLAR_EFF_BUCK_PERMILLE;
int32_t solarMpptHeadroom_mV = 0L;
int32_t solarMpptReferenceVoltage_mV = 0L;
int32_t solarMpptVoltageDrop_mV = 0L;
uint8_t solarMpptRebaseCounter = 0U;
uint32_t solarFastGuardLastTrip_ms = 0UL;
uint32_t solarFastGuardLastAction_ms = 0UL;
bool solarFastGuardHold = false;


// LOW_SOLAR runtime state.
bool solarLowPowerMode = false;
bool solarLowEnterTiming = false;
bool solarLowExitTiming = false;
bool solarLowOffTiming = false;
bool solarLowPauseActive = false;
uint32_t solarLowEnterStart_ms = 0UL;
uint32_t solarLowExitStart_ms = 0UL;
uint32_t solarLowOffStart_ms = 0UL;
uint32_t solarLowPauseUntil_ms = 0UL;
uint32_t solarLowLastControl_ms = 0UL;
uint32_t solarLowHoldUntil_ms = 0UL;
uint32_t solarLowSourceLostStart_ms = 0UL;
int32_t solarLowChargeCurrent_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
int32_t solarLowStepStartVoltage_mV = 0L;
int32_t solarLowLastDrop_mV = 0L;
bool solarLowStepPending = false;
bool solarLowSourceLostTiming = false;

struct SolarHistoryPoint
{
	int32_t powerSet_mW;
	int32_t inputVoltage_mV;
	int32_t measuredPower_mW;
};

SolarHistoryPoint solarStableHistory[SOLAR_STABLE_HISTORY_SIZE];
uint8_t solarStableHistoryIndex = 0U;
uint8_t solarStableHistoryCount = 0U;
int32_t solarTrendPowerSet_mW[SOLAR_TREND_HISTORY_SIZE];
int32_t solarTrendVoltage_mV[SOLAR_TREND_HISTORY_SIZE];
int32_t solarTrendMeasuredPower_mW[SOLAR_TREND_HISTORY_SIZE];
uint8_t solarTrendHistoryIndex = 0U;
uint8_t solarTrendHistoryCount = 0U;
int32_t solarLastSafePower_mW = SOLAR_POWER_SET_MIN_mW;
int32_t solarRecoveryTargetPower_mW = SOLAR_POWER_SET_MIN_mW;
int32_t solarRecoveryFailedPower_mW = SOLAR_POWER_SET_MIN_mW;
bool solarRecoveryActive = false;
bool solarRecoveryFine = false;
bool solarRecoveryTargetLocked = false;
uint32_t solarRecoverySoftHoldUntil_ms = 0UL;
uint32_t solarRecoveryTargetStableSince_ms = 0UL;

BatteryCharge_State batteryChargeState = BATTERY_CHARGE_WAIT;
uint32_t batteryNoSolarStart_ms = 0UL;    // Long solar absence arms next 28.10V daily cycle.
bool chargeCurrentFilterValid = false;
uint8_t chargeStableCounter = 0;
DCDC_State dcdcState = DCDC_STATE_OFF;
DCDC_PreChargeTest_State dcdcPreChargeTestState = DCDC_TEST_IDLE;
uint32_t dcdcPreChargeTestPhaseStart_ms = 0UL;
int32_t dcdcPreChargeTestBaseVoltage_mV = 0L;
uint8_t dcdcPreChargeTestStableCounter = 0U;
bool dcdcPreChargeTestFault = false;
bool dcdcReverseCurrentTiming = false;
uint32_t dcdcReverseCurrentStart_ms = 0UL;
uint32_t dcdcReverseBlockedUntil_ms = 0UL;

bool batteryOvervoltageFault = false;
bool thermalShutdown = false;

bool batteryRecoveryActive = false;
bool batteryRecoveryLongWarning = false;
uint8_t batteryRecoveryStage = 0U;
int32_t batteryRecoveryCurrentLimit_mA = BATTERY_RECOVERY_CURRENT_mA;
uint32_t batteryRecoveryStart_ms = 0UL;
uint32_t batteryRecoveryLastRamp_ms = 0UL;

// ============================================================
// Display Texts in Flash
// ============================================================

const char txtBattery[] PROGMEM = "BATTERY";
const char txtSolar[]   PROGMEM = "SOLAR";
const char txtMain[]    PROGMEM = "MAIN";
const char txtHelpDC[]  PROGMEM = "LOW";
const char txtCharge[]  PROGMEM = "CHARGE";
const char txtDCDC[]    PROGMEM = "DCDC";
const char txtOn[]      PROGMEM = "ON";
const char txtOff[]     PROGMEM = "OFF";

// ============================================================

// ============================================================
// Arduino Setup
// ============================================================

void setup()
{
	Serial.begin(UART_BAUDRATE);

	Wire.begin();
	Wire.setClock(I2C_CLOCK_HZ);

	IO_Init();
	LCD_Init();
	LCD_Clear();
	ButtonSet_InterruptInit();


	for (uint8_t i = 0; i < INA238_COUNT; i++)
	{
		ina238[i].online = INA238_Init(&ina238[i]);
	}

	DisplayBacklight_Off();

	SolarDC_Off();
	HelpDC_Off();
	MainDC_Off();
	Charge_Off();
	DCDC_Disable();
	DAC_WriteCode(DCDC_SAFE_START_CODE);
	ADS1015_AlertForceOff();
}

// ============================================================
// Arduino Loop
// ============================================================

void loop()
{
	uint32_t now_ms = millis();

	CommunicationReceive_Task();
	Communication_Task(now_ms);

	ButtonBacklight_Task(now_ms);
	DisplayBacklight_Task(now_ms);
	FanTemp_Task(now_ms);
	TemperatureProtection_Task();

	if ((uint32_t)(now_ms - lastMeasure_ms) >= MEASURE_INTERVAL_MS)
	{
		lastMeasure_ms = now_ms;

		for (uint8_t i = 0; i < INA238_COUNT; i++)
		{
			if (ina238[i].online)
			{
				INA238_ReadValues(&ina238[i]);
			}
		}

		// Measurement and control run at MEASURE_INTERVAL_MS.
		// The display is refreshed slower to avoid unreadable flicker.
		if ((uint32_t)(now_ms - lastDisplay_ms) >= DISPLAY_INTERVAL_MS)
		{
			lastDisplay_ms = now_ms;
			LCD_PrintMeasurements();
		}

		InputSource_Task();
		DCDC_Task(now_ms);
	}

}

const char* BatteryCharge_StateText(BatteryCharge_State state)
{
	switch (state)
	{
		case BATTERY_CHARGE_WAIT: return "WAIT";
		case BATTERY_CHARGE_BULK: return "BULK";
		case BATTERY_CHARGE_ABS:  return "ABS";
		case BATTERY_CHARGE_FULL: return "FULL";
	}

	return "?";
}

// ============================================================
// MCP4725 DAC Low Level
// ============================================================

bool DAC_WriteCode(uint16_t code)
{
	if (code > DAC_CODE_MAX)
	code = DAC_CODE_MAX;

	Wire.beginTransmission(MCP4725_ADDR);
	Wire.write(0x40);                              // Fast write, DAC register only.
	Wire.write((uint8_t)(code >> 4));
	Wire.write((uint8_t)((code & 0x000F) << 4));

	if (Wire.endTransmission() != 0)
	{
		return false;
	}

	dacCode = code;
	return true;
}

uint16_t DAC_VoltageToCode_mV(uint16_t voltage_mV)
{
	uint32_t code;

	if (voltage_mV > DAC_VREF_mV)
	voltage_mV = DAC_VREF_mV;

	code = ((uint32_t)voltage_mV * 4095UL + (DAC_VREF_mV / 2)) / DAC_VREF_mV;

	if (code > DAC_CODE_MAX)
	code = DAC_CODE_MAX;

	return (uint16_t)code;
}

uint16_t DAC_CodeToVoltage_mV(uint16_t code)
{
	if (code > DAC_CODE_MAX)
	code = DAC_CODE_MAX;

	return (uint16_t)(((uint32_t)code * DAC_VREF_mV + 2047UL) / 4095UL);
}

// ============================================================
// DAC Output Voltage Model
// ============================================================

uint16_t DCDC_CalcDacCodeForVout_mV(uint16_t vout_mV)
{
	int32_t code;

	if (vout_mV < DCDC_VOUT_MIN_mV)
	vout_mV = DCDC_VOUT_MIN_mV;

	if (vout_mV > DCDC_VOUT_MAX_mV)
	vout_mV = DCDC_VOUT_MAX_mV;

	// DCDC_mV = 30113 - 2.404 * DAC_Code
	// DAC_Code = (30113 - DCDC_mV) / 2.404
	code = ((DCDC_DAC_OFFSET_mV - (int32_t)vout_mV) * 1000L + (DCDC_DAC_SLOPE_uV_CODE / 2)) / DCDC_DAC_SLOPE_uV_CODE;

	if (code < DAC_CODE_MIN)
	code = DAC_CODE_MIN;

	if (code > DAC_CODE_MAX)
	code = DAC_CODE_MAX;

	return (uint16_t)code;
}

int32_t DCDC_CalcVoutFromDacCode_mV(uint16_t code)
{
	int32_t vout_mV;

	if (code > DAC_CODE_MAX)
	code = DAC_CODE_MAX;

	// DCDC_mV = 30113 - 2.404 * DAC_Code
	vout_mV = DCDC_DAC_OFFSET_mV - (((int32_t)code * DCDC_DAC_SLOPE_uV_CODE + 500L) / 1000L);

	return vout_mV;
}

bool DCDC_SetVout_mV(uint16_t vout_mV)
{
	uint16_t targetCode;

	targetCode = DCDC_CalcDacCodeForVout_mV(vout_mV);


	return DAC_RampToCode(targetCode);
}

bool DAC_RampToCode(uint16_t targetCode)
{
	uint16_t nextCode;

	if (targetCode > DAC_CODE_MAX)
	targetCode = DAC_CODE_MAX;

	while (dacCode != targetCode)
	{
		if (dacCode < targetCode)
		{
			nextCode = dacCode + DAC_RAMP_STEP_CODE;
			if (nextCode > targetCode)
			nextCode = targetCode;
		}
		else
		{
			if (dacCode > DAC_RAMP_STEP_CODE)
			nextCode = dacCode - DAC_RAMP_STEP_CODE;
			else
			nextCode = 0;

			if (nextCode < targetCode)
			nextCode = targetCode;
		}

		if (!DAC_WriteCode(nextCode))
		return false;

		if (DAC_RAMP_STEP_DELAY_MS > 0)
		delay(DAC_RAMP_STEP_DELAY_MS);
	}

	return true;
}

// ============================================================
// IO Low Level
// ============================================================

void IO_Init()
{
	DDRD |= PORTD_OUTPUT_MASK;
	DDRD &= ~MASK_BUTTON_SET;

	// Source switches are inverted: HIGH = OFF, LOW = ON.
	PORTD |= MASK_SW_SOLARDC;
	PORTD |= MASK_SW_HELPDC;
	PORTD |= MASK_SW_MAINDC;

	// Charge switch is not inverted: LOW = OFF, HIGH = ON.
	PORTD &= ~MASK_SW_CHARGE;
	PORTD &= ~MASK_DISPLAY_BACKLIGHT;
	PORTD |= MASK_BUTTON_SET;

	DDRC |= MASK_EN_DCDC;
	DDRC &= ~MASK_PG_DCDC;

	// DCDC inverted: HIGH = OFF, LOW = ON
	PORTC |= MASK_EN_DCDC;
	PORTC |= MASK_PG_DCDC;
}

// ============================================================
// Output Control
// ============================================================

void DisplayBacklight_On()  { PORTD |=  MASK_DISPLAY_BACKLIGHT; }
void DisplayBacklight_Off() { PORTD &= ~MASK_DISPLAY_BACKLIGHT; }

void SolarDC_On()           { PORTD &= ~MASK_SW_SOLARDC; stateSolarDC = true; }
void SolarDC_Off()          { PORTD |=  MASK_SW_SOLARDC; stateSolarDC = false; }

void HelpDC_On()            { PORTD &= ~MASK_SW_HELPDC; stateHelpDC = true; }
void HelpDC_Off()           { PORTD |=  MASK_SW_HELPDC; stateHelpDC = false; }

void MainDC_On()            { PORTD &= ~MASK_SW_MAINDC; stateMainDC = true; }
void MainDC_Off()           { PORTD |=  MASK_SW_MAINDC; stateMainDC = false; }

void Charge_On()            {
#if CHARGE_ENABLE_ALLOWED
	PORTD |=  MASK_SW_CHARGE; stateCharge = true;
#else
	PORTD &= ~MASK_SW_CHARGE; stateCharge = false;
#endif
}
void Charge_Off()           { PORTD &= ~MASK_SW_CHARGE; stateCharge = false; }

void DCDC_Enable()          { PORTC &= ~MASK_EN_DCDC; stateDCDC = true; }
void DCDC_Disable()         { PORTC |=  MASK_EN_DCDC; stateDCDC = false; }

// ============================================================
// Input Read
// ============================================================

bool DCDC_PowerGood()
{
	// PG_DCDC is inverted by hardware: LOW = power good.
	return ((PINC & MASK_PG_DCDC) == 0);
}

bool ButtonSet_Pressed()
{
	return !(PIND & MASK_BUTTON_SET);
}

// ============================================================
// Backlight Timer
// ============================================================

void DisplayBacklight_ResetTimer(uint32_t now_ms)
{
	DisplayBacklight_On();
	lightTimer_ms = now_ms + DISPLAY_LIGHT_TIME_MS;
}

void DisplayBacklight_Task(uint32_t now_ms)
{
	if ((PORTD & MASK_DISPLAY_BACKLIGHT) == 0)
	return;

	if ((int32_t)(now_ms - lightTimer_ms) >= 0)
	DisplayBacklight_Off();
}

bool DisplayBacklight_IsOn()
{
	return (PORTD & MASK_DISPLAY_BACKLIGHT);
}

void ButtonBacklight_Task(uint32_t now_ms)
{
	bool setPressed = ButtonSet_Pressed();
	uint8_t adcButton = ButtonADC_Read();

	// Every new button press switches the backlight on for the full light time.
	if ((setPressed && !lastBacklightSetPressed) || ((adcButton != BUTTON_NONE) && (lastBacklightAdcButton == BUTTON_NONE)))
	DisplayBacklight_ResetTimer(now_ms);

	lastBacklightSetPressed = setPressed;
	lastBacklightAdcButton = adcButton;
}

// ============================================================
// INA238 Low Level I2C
// ============================================================

bool INA238_Write16(uint8_t address, uint8_t reg, uint16_t value)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.write((uint8_t)(value >> 8));
	Wire.write((uint8_t)(value & 0xFF));
	return (Wire.endTransmission() == 0);
}

bool INA238_Read16(uint8_t address, uint8_t reg, uint16_t* value)
{
	Wire.beginTransmission(address);
	Wire.write(reg);

	if (Wire.endTransmission(false) != 0)
	return false;

	if (Wire.requestFrom(address, (uint8_t)2) != 2)
	return false;

	*value = ((uint16_t)Wire.read() << 8) | Wire.read();
	return true;
}

uint16_t INA238_CalcCalibration(uint16_t shunt_mOhm)
{
	uint32_t cal;

	// CAL = 819200000 * CurrentLSB_A * Rshunt_Ohm
	// CurrentLSB_A = uA / 1000000
	// Rshunt_Ohm   = mOhm / 1000
	cal = 8192UL * (uint32_t)INA238_CURRENT_LSB_uA * (uint32_t)shunt_mOhm;
	cal = (cal + 5000UL) / 10000UL;

	if (cal > 65535UL)
	cal = 65535UL;

	return (uint16_t)cal;
}

// ============================================================
// INA238 Driver
// ============================================================

bool INA238_Init(INA238_Channel* ch)
{
	uint16_t dummy = 0;

	if (!INA238_Read16(ch->address, INA238_REG_DEVICE_ID, &dummy))
	return false;

	if (!INA238_Write16(ch->address, INA238_REG_CONFIG, INA238_CONFIG_VALUE))
	return false;

	if (!INA238_Write16(ch->address, INA238_REG_ADC_CONFIG, INA238_ADC_CONFIG_VALUE))
	return false;

	if (!INA238_Write16(ch->address, INA238_REG_SHUNT_CAL, INA238_CalcCalibration(ch->shunt_mOhm)))
	return false;

	return true;
}

bool INA238_ReadValues(INA238_Channel* ch)
{
	uint16_t rawBus = 0;
	uint16_t rawCurrent = 0;

	if (!INA238_Read16(ch->address, INA238_REG_VBUS, &rawBus))
	{
		ch->online = false;
		return false;
	}

	if (!INA238_Read16(ch->address, INA238_REG_CURRENT, &rawCurrent))
	{
		ch->online = false;
		return false;
	}

	// VBUS LSB = 3.125mV
	ch->voltage_mV = ((int32_t)(int16_t)rawBus * 3125L) / 1000L;

	// CURRENT LSB = INA238_CURRENT_LSB_uA.
	// The correction factor is used for board calibration only.
	ch->current_mA = ((int32_t)(int16_t)rawCurrent * INA238_CURRENT_LSB_uA) / 1000L;
	ch->current_mA = (ch->current_mA * ch->currentCorrPermille) / 1000L;

	return true;
}

// ============================================================
// Button Set Interrupt
// ============================================================

void ButtonSet_InterruptInit()
{
	EICRA |= _BV(ISC01);     // INT0 falling edge
	EICRA &= ~_BV(ISC00);
	EIFR |= _BV(INTF0);
	EIMSK |= _BV(INT0);
}

ISR(INT0_vect)
{
	buttonSetIrqFlag = true;
}


uint8_t ButtonADC_Read()
{
	uint16_t adc = analogRead(ADC_BUTTONS);

	if (ADC_IsNear(adc, ADC_BTN_OK_VALUE))
	return BUTTON_OK;

	if (ADC_IsNear(adc, ADC_BTN_DOWN_VALUE))
	return BUTTON_DOWN;

	if (ADC_IsNear(adc, ADC_BTN_UP_VALUE))
	return BUTTON_UP;

	return BUTTON_NONE;
}

bool ADC_IsNear(uint16_t value, uint16_t target)
{
	if (value > target)
	return ((value - target) <= BUTTON_ADC_TOLERANCE);
	else
	return ((target - value) <= BUTTON_ADC_TOLERANCE);
}


// ============================================================
// ST7565 Low Level, direct page output, no framebuffer
// ============================================================

void LCD_PinInit()
{
	DDRB |= LCD_CS_M | LCD_A0_M | LCD_RST_M | LCD_SDA_M | LCD_SCK_M;

	PORTB |= LCD_CS_M;
	PORTB &= ~LCD_SCK_M;
	PORTB &= ~LCD_SDA_M;
	PORTB |= LCD_RST_M;
}

void LCD_WriteByte(uint8_t data)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		if (data & 0x80) PORTB |= LCD_SDA_M;
		else             PORTB &= ~LCD_SDA_M;

		PORTB |= LCD_SCK_M;
		PORTB &= ~LCD_SCK_M;

		data <<= 1;
	}
}

void LCD_Command(uint8_t cmd)
{
	PORTB &= ~LCD_A0_M;
	PORTB &= ~LCD_CS_M;
	LCD_WriteByte(cmd);
	PORTB |= LCD_CS_M;
}

void LCD_Data(uint8_t data)
{
	PORTB |= LCD_A0_M;
	PORTB &= ~LCD_CS_M;
	LCD_WriteByte(data);
	PORTB |= LCD_CS_M;
}

void LCD_Init()
{
	LCD_PinInit();

	PORTB &= ~LCD_RST_M;
	delay(50);
	PORTB |= LCD_RST_M;
	delay(50);

	LCD_Command(0xAE);    // Display OFF
	LCD_Command(0xA2);    // Bias 1/9
	LCD_Command(0xA0);    // ADC normal
	LCD_Command(0xC8);    // COM reverse
	LCD_Command(0x40);    // Start line 0
	LCD_Command(0x25);    // Resistor ratio
	LCD_Command(0x81);    // Electronic volume
	LCD_Command(0x28);    // Contrast
	LCD_Command(0x2F);    // Power control
	delay(50);
	LCD_Command(0xA6);    // Normal display
	LCD_Command(0xA4);    // RAM content
	LCD_Command(0xAF);    // Display ON
}

void LCD_SetPos(uint8_t x, uint8_t page)
{
	LCD_Command(0xB0 | (page & 0x07));
	LCD_Command(0x10 | (x >> 4));
	LCD_Command(0x00 | (x & 0x0F));
}

void LCD_Clear()
{
	for (uint8_t page = 0; page < LCD_PAGES; page++)
	{
		LCD_SetPos(0, page);
		for (uint8_t x = 0; x < LCD_WIDTH; x++)
		LCD_Data(0x00);
	}
}

void LCD_Update()
{
	// Not required without framebuffer.
}

void LCD_SetPixel(uint8_t x, uint8_t y)
{
	// Not used in direct text mode.
}

// ============================================================
// ST7565 Text
// ============================================================

void LCD_Char(uint8_t x, uint8_t y, char c)
{
	uint8_t pattern[5];

	LCD_GetCharPattern(c, pattern);
	LCD_SetPos(x, y >> 3);

	for (uint8_t col = 0; col < 5; col++)
	LCD_Data(pattern[col]);

	LCD_Data(0x00);
}

void LCD_Text(uint8_t x, uint8_t y, const char* txt)
{
	LCD_SetPos(x, y >> 3);

	while (*txt)
	{
		uint8_t pattern[5];
		LCD_GetCharPattern(*txt++, pattern);

		for (uint8_t col = 0; col < 5; col++)
		LCD_Data(pattern[col]);

		LCD_Data(0x00);
	}
}

void LCD_Text_P(uint8_t x, uint8_t y, const char* txt)
{
	LCD_SetPos(x, y >> 3);

	char c;
	while ((c = pgm_read_byte(txt++)))
	{
		uint8_t pattern[5];
		LCD_GetCharPattern(c, pattern);

		for (uint8_t col = 0; col < 5; col++)
		LCD_Data(pattern[col]);

		LCD_Data(0x00);
	}
}

void LCD_HLine(uint8_t x1, uint8_t x2, uint8_t y)
{
	uint8_t mask;

	mask = (uint8_t)(1 << (y & 7));
	LCD_SetPos(x1, y >> 3);

	while (x1 <= x2)
	{
		LCD_Data(mask);
		x1++;
	}
}


// ============================================================
// Display Measurements
// ============================================================

void LCD_PrintMeasurements()
{
	uint8_t sourceChannel;
	const char* sourceText_P;
	int32_t batteryCurrent_mA;
	int32_t sourceCurrent_mA;

	LCD_Clear();

	sourceChannel = LCD_GetSelectedSourceChannel();
	sourceText_P = LCD_GetSelectedSourceText_P(sourceChannel);
	batteryCurrent_mA = LCD_GetDisplayBatteryCurrent_mA();
	sourceCurrent_mA = ina238[sourceChannel].current_mA;

	// Left column: battery values.
	LCD_Text_P(LCD_CenterX(DISPLAY_LEFT_X, DISPLAY_COLUMN_W, 7), DISPLAY_LABEL_Y, txtBattery);
	LCD_HLine(DISPLAY_LEFT_X, DISPLAY_LEFT_X + DISPLAY_COLUMN_W - 4, DISPLAY_SEPARATOR_Y);
	LCD_PrintBigSignedFixed1(DISPLAY_LEFT_X, DISPLAY_COLUMN_W, DISPLAY_VOLTAGE_Y, ina238[CH_BATTERY].voltage_mV, 1000L, 'V');

	if (LCD_CurrentFlows(batteryCurrent_mA))
	LCD_PrintBigSignedFixed1(DISPLAY_LEFT_X, DISPLAY_COLUMN_W, DISPLAY_CURRENT_Y, batteryCurrent_mA, 1000L, 'A');

	// Right column: show source values only if at least one source has minimum voltage.
	if (LCD_AnySourceHasMinimumVoltage())
	{
		LCD_Text_P(LCD_CenterX(DISPLAY_RIGHT_X, DISPLAY_COLUMN_W, LCD_TextLength_P(sourceText_P)), DISPLAY_LABEL_Y, sourceText_P);
		LCD_PrintBigSignedFixed1(DISPLAY_RIGHT_X, DISPLAY_COLUMN_W, DISPLAY_VOLTAGE_Y, ina238[sourceChannel].voltage_mV, 1000L, 'V');

		if (LCD_CurrentFlows(sourceCurrent_mA))
		LCD_PrintBigSignedFixed1(DISPLAY_RIGHT_X, DISPLAY_COLUMN_W, DISPLAY_CURRENT_Y, sourceCurrent_mA, 1000L, 'A');
	}

	LCD_PrintSystemState();
	LCD_PrintHighestTemperature();
}

uint8_t LCD_GetSelectedSourceChannel()
{
	if (stateMainDC)
	return CH_MAINDC;

	if (stateSolarDC)
	return CH_SOLARDC;

	if (stateHelpDC)
	return CH_HELPDC;

	// No source is switched on: show the best available source voltage.
	if (ina238[CH_MAINDC].voltage_mV >= MAIN_ON_mV)
	return CH_MAINDC;

	if (ina238[CH_SOLARDC].voltage_mV >= SOLAR_ON_mV)
	return CH_SOLARDC;

	if (ina238[CH_HELPDC].voltage_mV >= HELPDC_ON_mV)
	return CH_HELPDC;

	return CH_MAINDC;
}

const char* LCD_GetSelectedSourceText_P(uint8_t channel)
{
	if (channel == CH_SOLARDC)
	return txtSolar;

	if (channel == CH_HELPDC)
	return txtHelpDC;

	return txtMain;
}

int32_t LCD_GetDisplayBatteryCurrent_mA()
{
	// The filtered current is easier to read while charging.
	if (chargeCurrentFilterValid)
	return chargeCurrentFiltered_mA;

	return ina238[CH_BATTERY].current_mA;
}

bool LCD_CurrentFlows(int32_t current_mA)
{
	if (current_mA < 0)
	current_mA = -current_mA;

	return (current_mA >= DISPLAY_CURRENT_MIN_mA);
}

bool LCD_AnySourceHasMinimumVoltage()
{
	if (ina238[CH_MAINDC].voltage_mV >= MAIN_ON_mV)
	return true;

	if (ina238[CH_SOLARDC].voltage_mV >= SOLAR_ON_mV)
	return true;

	if (ina238[CH_HELPDC].voltage_mV >= HELPDC_ON_mV)
	return true;

	return false;
}

uint8_t LCD_TextLength_P(const char* txt)
{
	uint8_t len = 0;

	while (pgm_read_byte(txt++))
	len++;

	return len;
}

uint8_t LCD_CenterX(uint8_t x, uint8_t width, uint8_t chars)
{
	uint8_t textWidth = chars * 6;

	if (textWidth >= width)
	return x;

	return x + ((width - textWidth) / 2);
}

void LCD_PrintSystemState()
{
	// Local protection states have highest priority.
	if (dcdcPreChargeTestFault)
	{
		LCD_Text(0, DISPLAY_STATE_Y, "DCDC ERROR");
		return;
	}

	if (batteryOvervoltageFault)
	{
		LCD_Text(0, DISPLAY_STATE_Y, "BAT OVP");
		return;
	}

	if (thermalShutdown)
	{
		LCD_Text(0, DISPLAY_STATE_Y, "OVER TEMP");
		return;
	}

	if (batteryRecoveryActive)
	{
		LCD_Text(0, DISPLAY_STATE_Y, "RECOVERY");
		return;
	}

	// LOW_PAUSE has priority over LOW_SOLAR because the converter is
	// intentionally waiting before the next weak-source retry.
	if (solarLowPauseActive)
	{
		LCD_Text(0, DISPLAY_STATE_Y, "LOW PAUSE");
		return;
	}

	// Weak-source knee search is active.
	if (solarLowPowerMode)
	{
		LCD_Text(0, DISPLAY_STATE_Y, "LOW SOLAR");
		return;
	}

	// Normal display line.
	LCD_Text(0, DISPLAY_STATE_Y, DCDC_StateText(dcdcState));
	LCD_Text(42, DISPLAY_STATE_Y, BatteryCharge_StateText(batteryChargeState));
}

void LCD_PrintHighestTemperature()
{
	int16_t highestTemp_C;

	if (fetTemperature_C >= coilTemperature_C)
	highestTemp_C = fetTemperature_C;
	else
	highestTemp_C = coilTemperature_C;

	// Bottom right temperature, no label.
	LCD_PrintTemperatureC(110, DISPLAY_TEMP_Y, highestTemp_C);
}

void LCD_PrintBigSignedFixed1(uint8_t columnX, uint8_t columnWidth, uint8_t y, int32_t value, int32_t scale, char unit)
{
	char buf[7];
	uint8_t len = 0;
	uint16_t tenths;
	uint16_t ip;
	uint8_t fp;
	uint8_t startX;
	uint8_t numberWidth;

	if (value < 0)
	{
		buf[len++] = '-';
		value = -value;
	}

	// One decimal digit with simple rounding.
	tenths = (uint16_t)((value + (scale / 20L)) / (scale / 10L));
	ip = tenths / 10;
	fp = tenths % 10;

	if (ip >= 100)
	buf[len++] = '0' + ((ip / 100) % 10);

	if (ip >= 10)
	buf[len++] = '0' + ((ip / 10) % 10);

	buf[len++] = '0' + (ip % 10);
	buf[len++] = '.';
	buf[len++] = '0' + fp;
	buf[len] = 0;

	// Big number, small unit character.
	numberWidth = len * 12;
	if ((numberWidth + 6) >= columnWidth)
	startX = columnX;
	else
	startX = columnX + ((columnWidth - numberWidth - 6) / 2);

	LCD_BigText(startX, y, buf);
	LCD_Char(startX + numberWidth, y + 8, unit);
}


void LCD_PrintBigIntFixed(uint8_t x, uint8_t y, uint16_t val, uint8_t minDigits)
{
	char buf[5];
	uint8_t i = 0;

	do
	{
		buf[i++] = '0' + (val % 10);
		val /= 10;
	}
	while ((val > 0 || i < minDigits) && i < sizeof(buf));

	while (i > 0)
	{
		i--;
		LCD_BigChar(x, y, buf[i]);
		x += 12;
	}
}

void LCD_BigText(uint8_t x, uint8_t y, const char* txt)
{
	while (*txt)
	{
		LCD_BigChar(x, y, *txt++);
		x += 12;
	}
}

void LCD_BigChar(uint8_t x, uint8_t y, char c)
{
	uint8_t pattern[5];
	uint8_t top[5];
	uint8_t bottom[5];

	LCD_GetCharPattern(c, pattern);

	for (uint8_t col = 0; col < 5; col++)
	{
		top[col] = 0;
		bottom[col] = 0;

		for (uint8_t row = 0; row < 7; row++)
		{
			if (pattern[col] & (1 << row))
			{
				uint8_t bigRow = row * 2;

				if (bigRow < 8)
				top[col] |= (1 << bigRow);
				else
				bottom[col] |= (1 << (bigRow - 8));

				bigRow++;

				if (bigRow < 8)
				top[col] |= (1 << bigRow);
				else
				bottom[col] |= (1 << (bigRow - 8));
			}
		}
	}

	LCD_SetPos(x, y >> 3);
	for (uint8_t col = 0; col < 5; col++)
	{
		LCD_Data(top[col]);
		LCD_Data(top[col]);
	}
	LCD_Data(0x00);
	LCD_Data(0x00);

	LCD_SetPos(x, (y >> 3) + 1);
	for (uint8_t col = 0; col < 5; col++)
	{
		LCD_Data(bottom[col]);
		LCD_Data(bottom[col]);
	}
	LCD_Data(0x00);
	LCD_Data(0x00);
}

void LCD_PrintValueLine_mV(uint8_t x, uint8_t y, const char* label_P, int32_t value_mV, char unit)
{
	LCD_Text_P(x, y, label_P);
	LCD_PrintSignedFixed2(78, y, value_mV, 1000L);
	LCD_Char(114, y, unit);
}

void LCD_PrintValueLine_mA(uint8_t x, uint8_t y, const char* label_P, int32_t value_mA, char unit)
{
	LCD_Text_P(x, y, label_P);
	LCD_PrintSignedFixed2(78, y, value_mA, 1000L);
	LCD_Char(114, y, unit);
}

void LCD_PrintSignedFixed2(uint8_t x, uint8_t y, int32_t value, int32_t scale)
{
	if (value < 0)
	{
		LCD_Char(72, y, '-');
		value = -value;
	}

	uint16_t ip = (uint16_t)(value / scale);
	uint8_t fp = (uint8_t)((value % scale) / (scale / 100L));

	LCD_PrintIntFixed(x, y, ip, 2);
	LCD_Char(x + 12, y, '.');
	LCD_Char(x + 18, y, '0' + (fp / 10));
	LCD_Char(x + 24, y, '0' + (fp % 10));
}

void LCD_PrintTemperatureC(uint8_t x, uint8_t y, int16_t temp_C)
{
	if (temp_C < 0)
	{
		LCD_Char(x, y, '-');
		x += 6;
		temp_C = -temp_C;
	}

	LCD_PrintIntFixed(x, y, (uint16_t)temp_C, 2);
	LCD_Char(x + 12, y, 'C');
}

void LCD_PrintStatusLine()
{
	// Not used by the large symmetric measurement screen.
}

void LCD_PrintIntFixed(uint8_t x, uint8_t y, uint16_t val, uint8_t minDigits)
{
	char buf[5];
	uint8_t i = 0;

	do
	{
		buf[i++] = '0' + (val % 10);
		val /= 10;
	}
	while ((val > 0 || i < minDigits) && i < sizeof(buf));

	while (i > 0)
	{
		i--;
		LCD_Char(x, y, buf[i]);
		x += 6;
	}
}

// ============================================================
// Minimal 5x7 Font
// ============================================================

void LCD_GetCharPattern(char c, uint8_t* p)
{
	for (uint8_t i = 0; i < 5; i++)
	p[i] = 0x00;

	// The internal font table contains uppercase letters only.
	// Convert lowercase display text to uppercase before lookup.
	if ((c >= 'a') && (c <= 'z'))
	c = (char)(c - 32);

	switch (c)
	{
		case ' ': break;

		case '0': p[0]=0x3E; p[1]=0x51; p[2]=0x49; p[3]=0x45; p[4]=0x3E; break;
		case '1': p[0]=0x00; p[1]=0x42; p[2]=0x7F; p[3]=0x40; p[4]=0x00; break;
		case '2': p[0]=0x42; p[1]=0x61; p[2]=0x51; p[3]=0x49; p[4]=0x46; break;
		case '3': p[0]=0x21; p[1]=0x41; p[2]=0x45; p[3]=0x4B; p[4]=0x31; break;
		case '4': p[0]=0x18; p[1]=0x14; p[2]=0x12; p[3]=0x7F; p[4]=0x10; break;
		case '5': p[0]=0x27; p[1]=0x45; p[2]=0x45; p[3]=0x45; p[4]=0x39; break;
		case '6': p[0]=0x3C; p[1]=0x4A; p[2]=0x49; p[3]=0x49; p[4]=0x30; break;
		case '7': p[0]=0x01; p[1]=0x71; p[2]=0x09; p[3]=0x05; p[4]=0x03; break;
		case '8': p[0]=0x36; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x36; break;
		case '9': p[0]=0x06; p[1]=0x49; p[2]=0x49; p[3]=0x29; p[4]=0x1E; break;

		case '.': p[0]=0x00; p[1]=0x60; p[2]=0x60; p[3]=0x00; p[4]=0x00; break;
		case '-': p[0]=0x08; p[1]=0x08; p[2]=0x08; p[3]=0x08; p[4]=0x08; break;
		case '>': p[0]=0x41; p[1]=0x22; p[2]=0x14; p[3]=0x08; p[4]=0x00; break;
		case '*': p[0]=0x14; p[1]=0x08; p[2]=0x3E; p[3]=0x08; p[4]=0x14; break;

		case 'A': p[0]=0x7E; p[1]=0x11; p[2]=0x11; p[3]=0x11; p[4]=0x7E; break;
		case 'B': p[0]=0x7F; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x36; break;
		case 'C': p[0]=0x3E; p[1]=0x41; p[2]=0x41; p[3]=0x41; p[4]=0x22; break;
		case 'D': p[0]=0x7F; p[1]=0x41; p[2]=0x41; p[3]=0x22; p[4]=0x1C; break;
		case 'E': p[0]=0x7F; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x41; break;
		case 'F': p[0]=0x7F; p[1]=0x09; p[2]=0x09; p[3]=0x09; p[4]=0x01; break;
		case 'G': p[0]=0x3E; p[1]=0x41; p[2]=0x49; p[3]=0x49; p[4]=0x7A; break;
		case 'H': p[0]=0x7F; p[1]=0x08; p[2]=0x08; p[3]=0x08; p[4]=0x7F; break;
		case 'I': p[0]=0x00; p[1]=0x41; p[2]=0x7F; p[3]=0x41; p[4]=0x00; break;
		case 'K': p[0]=0x7F; p[1]=0x08; p[2]=0x14; p[3]=0x22; p[4]=0x41; break;
		case 'L': p[0]=0x7F; p[1]=0x40; p[2]=0x40; p[3]=0x40; p[4]=0x40; break;
		case 'M': p[0]=0x7F; p[1]=0x02; p[2]=0x04; p[3]=0x02; p[4]=0x7F; break;
		case 'N': p[0]=0x7F; p[1]=0x04; p[2]=0x08; p[3]=0x10; p[4]=0x7F; break;
		case 'O': p[0]=0x3E; p[1]=0x41; p[2]=0x41; p[3]=0x41; p[4]=0x3E; break;
		case 'P': p[0]=0x7F; p[1]=0x09; p[2]=0x09; p[3]=0x09; p[4]=0x06; break;
		case 'R': p[0]=0x7F; p[1]=0x09; p[2]=0x19; p[3]=0x29; p[4]=0x46; break;
		case 'S': p[0]=0x46; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x31; break;
		case 'T': p[0]=0x01; p[1]=0x01; p[2]=0x7F; p[3]=0x01; p[4]=0x01; break;
		case 'U': p[0]=0x3F; p[1]=0x40; p[2]=0x40; p[3]=0x40; p[4]=0x3F; break;
		case 'V': p[0]=0x1F; p[1]=0x20; p[2]=0x40; p[3]=0x20; p[4]=0x1F; break;
		case 'W': p[0]=0x7F; p[1]=0x20; p[2]=0x18; p[3]=0x20; p[4]=0x7F; break;
		case 'Y': p[0]=0x07; p[1]=0x08; p[2]=0x70; p[3]=0x08; p[4]=0x07; break;
	}
}

// ============================================================
// ADS1015 Low Level
// ============================================================

bool ADS1015_WriteRegister(uint8_t reg, uint16_t value)
{
	Wire.beginTransmission(ADS1015_ADDR);
	Wire.write(reg);
	Wire.write((uint8_t)(value >> 8));
	Wire.write((uint8_t)(value & 0xFF));
	return (Wire.endTransmission() == 0);
}

bool ADS1015_WriteConfig(uint16_t config)
{
	return ADS1015_WriteRegister(ADS1015_REG_CONFIG, config);
}

bool ADS1015_ReadConversion(int16_t* value)
{
	Wire.beginTransmission(ADS1015_ADDR);
	Wire.write(ADS1015_REG_CONVERSION);

	if (Wire.endTransmission(false) != 0)
	return false;

	if (Wire.requestFrom(ADS1015_ADDR, (uint8_t)2) != 2)
	return false;

	uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
	*value = ((int16_t)raw) >> 4;

	return true;
}

bool ADS1015_ReadDiff(uint16_t mux, int16_t* raw)
{
	uint16_t config = 0;

	config |= ADS1015_OS_SINGLE;
	config |= mux;
	config |= ADS1015_PGA_6V144;
	config |= ADS1015_MODE_SINGLE;
	config |= ADS1015_DR_1600SPS;
	config |= ADS1015_COMP_DISABLE;

	if (!ADS1015_WriteConfig(config))
	return false;

	delayMicroseconds(1000);

	return ADS1015_ReadConversion(raw);
}

void ADS1015_ReadTemperatures()
{
	int16_t raw1 = 0;
	int16_t raw2 = 0;

	if (ADS1015_ReadDiff(ADS1015_MUX_A0_A1, &raw1))
	fetTemperature_C = NTC_RawToTempC(raw1);

	if (ADS1015_ReadDiff(ADS1015_MUX_A2_A3, &raw2))
	coilTemperature_C = NTC_RawToTempC(raw2);
}

int16_t NTC_RawToTempC(int16_t raw)
{
	int16_t t1, t2;
	int16_t r1, r2;

	if (raw >= pgm_read_word(&ntcTable[0].raw))
	return pgm_read_word(&ntcTable[0].temp_C);

	for (uint8_t i = 0; i < NTC_TABLE_COUNT - 1; i++)
	{
		t1 = pgm_read_word(&ntcTable[i].temp_C);
		r1 = pgm_read_word(&ntcTable[i].raw);
		t2 = pgm_read_word(&ntcTable[i + 1].temp_C);
		r2 = pgm_read_word(&ntcTable[i + 1].raw);

		if ((raw <= r1) && (raw >= r2))
		return t1 + ((int32_t)(r1 - raw) * (t2 - t1)) / (r1 - r2);
	}

	return pgm_read_word(&ntcTable[NTC_TABLE_COUNT - 1].temp_C);
}

// ============================================================
// Fan Temperature Task, ALERT used as fan open-drain output
// ============================================================

void FanTemp_Task(uint32_t now_ms)
{
	if ((uint32_t)(now_ms - lastFanCheck_ms) < FAN_CHECK_INTERVAL_MS)
	return;

	lastFanCheck_ms = now_ms;
	ADS1015_ReadTemperatures();

	// The fan cools the internal coil and moves air across the FET heatsink.
	// It runs whenever the converter/charge path is active and continues until
	// both measured locations have cooled below the after-run threshold.
	if (dcdcState != DCDC_STATE_OFF || stateCharge)
		fanState = true;
	else if ((fetTemperature_C <= FAN_AFTER_RUN_OFF_C) &&
	         (coilTemperature_C <= FAN_AFTER_RUN_OFF_C))
		fanState = false;
	else
		fanState = true;

	if (fanState)
	ADS1015_AlertForceOn();
	else
	ADS1015_AlertForceOff();
}

void ADS1015_AlertForceOn()
{
	uint16_t config = 0;

	// Force comparator active. ALERT active LOW.
	ADS1015_WriteRegister(ADS1015_REG_LO_THRESH, 0x8000);
	ADS1015_WriteRegister(ADS1015_REG_HI_THRESH, 0x0000);

	config |= ADS1015_MUX_A0_A1;
	config |= ADS1015_PGA_6V144;
	config |= ADS1015_MODE_CONTINUOUS;
	config |= ADS1015_DR_1600SPS;
	config |= ADS1015_COMP_MODE_TRAD;
	config |= ADS1015_COMP_POL_LOW;
	config |= ADS1015_COMP_LAT_NON;
	config |= ADS1015_COMP_QUE_1;

	ADS1015_WriteConfig(config);
}

void ADS1015_AlertForceOff()
{
	uint16_t config = 0;

	config |= ADS1015_MUX_A0_A1;
	config |= ADS1015_PGA_6V144;
	config |= ADS1015_MODE_SINGLE;
	config |= ADS1015_DR_1600SPS;
	config |= ADS1015_COMP_DISABLE;

	ADS1015_WriteConfig(config);
}


// ============================================================
// DCDC State Machine
// ============================================================
//
// Simple charge start and hold sequence:
// 1. Select an available input source.
// 2. Set DCDC equal to the battery voltage.
//    This avoids a large voltage step when SW-Charge is switched on.
// 3. Enable DCDC.
// 4. Switch PD6 / SW-Charge ON. PD6 is HIGH active.
// 5. Slowly lower the DAC code. This raises the DCDC output voltage.
// 6. Stop the voltage ramp at battery voltage + CHARGE_MAX_OFFSET_mV.
// 7. Control the ramp by the battery INA current.
// 8. Hold the active charge current target with a small hysteresis.
// 9. If the input source is removed, switch charge and DCDC OFF immediately.
//
// Current direction:
// INA current is positive from IN+ pin 10 to IN- pin 9.
// The charge current is measured with INA_BATTERY.
// INA_DCDC current is shown as converter diagnostic information.

const char* DCDC_StateText(DCDC_State state)
{
	switch (state)
	{
		case DCDC_STATE_OFF:          return "OFF";
		case DCDC_STATE_START:        return "START";
		case DCDC_STATE_CHARGE_ON:    return "CHGON";
		case DCDC_STATE_RAMP_CURRENT: return "IRAMP";
		case DCDC_STATE_READY:        return "READY";
	}

	return "?";
}

bool InputSource_Available()
{
	if (stateMainDC)
	return (ina238[CH_MAINDC].online && ina238[CH_MAINDC].voltage_mV >= MAIN_ON_mV);

	if (stateSolarDC)
	return (ina238[CH_SOLARDC].online && ina238[CH_SOLARDC].voltage_mV >= SOLAR_ACTIVE_HOLD_MIN_mV);

	if (stateHelpDC)
	return (ina238[CH_HELPDC].online && ina238[CH_HELPDC].voltage_mV >= HELPDC_ON_mV);

	return false;
}

void DCDC_SetState(DCDC_State newState)
{
	if (dcdcState == newState)
	return;


	dcdcState = newState;
}

void DCDC_AllOff()
{
	Charge_Off();
	DCDC_Disable();
	DAC_WriteCode(DCDC_SAFE_START_CODE);
	DCDC_ResetCurrentControl();
	DCDC_PreChargeTest_Reset();
	inputLimitedChargeCurrentSet_mA = 0L;
	inputLimitSource = INPUT_SOURCE_NONE;
	SolarMPPT_Reset();
	DCDC_SetState(DCDC_STATE_OFF);
}

void ChargeCurrent_Task(uint32_t now_ms)
{
    if (!InputSource_Available())
    {
        activeChargeCurrentSet_mA = 0L;
        activeChargeCurrentMax_mA = BATTERY_MAX_CHARGE_CURRENT_mA;
        inputLimitedChargeCurrentSet_mA = 0L;
        inputLimitSource = INPUT_SOURCE_NONE;
        SolarMPPT_Reset();
        return;
    }

    // During BMS recovery the charger deliberately avoids the normal MPPT/current request.
    // It behaves as a voltage source with a strictly limited battery-current ceiling.
    if (batteryRecoveryActive)
    {
        activeChargeCurrentSet_mA = batteryRecoveryCurrentLimit_mA;
        activeChargeCurrentMax_mA = batteryRecoveryCurrentLimit_mA;
        inputLimitedChargeCurrentSet_mA = batteryRecoveryCurrentLimit_mA;
        inputLimitSource = stateMainDC ? INPUT_SOURCE_MAIN :
                           stateSolarDC ? INPUT_SOURCE_SOLAR : INPUT_SOURCE_HELP;
        solarRegulationStatus = SOLAR_REG_IDLE;
        return;
    }

    int32_t requestedSet_mA = CHARGE_START_CURRENT_mA;

    if (stateSolarDC)
    {
        SolarChargeCurrent_Task(now_ms);
        requestedSet_mA = solarChargeCurrentSet_mA;
    }
    else if (stateMainDC)
    {
        SolarMPPT_Reset();
        solarRegulationStatus = SOLAR_REG_IDLE;
        requestedSet_mA = MAIN_CHARGE_CURRENT_SET_mA;
    }
    else if (stateHelpDC)
    {
        SolarMPPT_Reset();
        solarRegulationStatus = SOLAR_REG_IDLE;
        requestedSet_mA = HELP_CHARGE_CURRENT_SET_mA;
    }

    if (requestedSet_mA > BATTERY_MAX_CHARGE_CURRENT_mA)
        requestedSet_mA = BATTERY_MAX_CHARGE_CURRENT_mA;
    if (requestedSet_mA < 0L)
        requestedSet_mA = 0L;

    // Preserve the existing high-voltage taper.
    int32_t voltageLimitedSet_mA = requestedSet_mA;
    const int32_t battery_mV = ina238[CH_BATTERY].voltage_mV;
    if (battery_mV >= CHARGE_TAPER_END_mV)
    {
        voltageLimitedSet_mA = 0L;
    }
    else if (battery_mV > CHARGE_TAPER_START_mV)
    {
        const int32_t remaining_mV = CHARGE_TAPER_END_mV - battery_mV;
        const int32_t taperSpan_mV = CHARGE_TAPER_END_mV - CHARGE_TAPER_START_mV;
        voltageLimitedSet_mA = CHARGE_TAPER_MIN_mA +
            ((requestedSet_mA - CHARGE_TAPER_MIN_mA) * remaining_mV) /
            taperSpan_mV;
        if (voltageLimitedSet_mA < CHARGE_TAPER_MIN_mA)
            voltageLimitedSet_mA = CHARGE_TAPER_MIN_mA;
    }

    int32_t powerLimited_mA = DCDC_GetPowerLimitedCurrent_mA();
    if (voltageLimitedSet_mA > powerLimited_mA)
        voltageLimitedSet_mA = powerLimited_mA;

    const uint8_t source = stateMainDC ? INPUT_SOURCE_MAIN :
                           stateSolarDC ? INPUT_SOURCE_SOLAR : INPUT_SOURCE_HELP;
    activeChargeCurrentSet_mA = InputCurrentLimit_Task(now_ms, source, voltageLimitedSet_mA);
    activeChargeCurrentMax_mA = BATTERY_MAX_CHARGE_CURRENT_mA;
}

int32_t InputCurrentLimit_Task(uint32_t now_ms, uint8_t source, int32_t requestedSet_mA)
{
	int32_t inputCurrent_mA;
	int32_t inputCurrentMax_mA;

	if (inputLimitSource != source)
	{
		inputLimitSource = source;
		inputLimitedChargeCurrentSet_mA = requestedSet_mA;
		lastInputCurrentRegulation_ms = now_ms;
		return inputLimitedChargeCurrentSet_mA;
	}

	if (inputLimitedChargeCurrentSet_mA > requestedSet_mA)
	inputLimitedChargeCurrentSet_mA = requestedSet_mA;

	if ((uint32_t)(now_ms - lastInputCurrentRegulation_ms) < INPUT_CURRENT_REG_INTERVAL_MS)
	return inputLimitedChargeCurrentSet_mA;

	lastInputCurrentRegulation_ms = now_ms;
	inputCurrent_mA = InputCurrent_Get_mA(source);
	inputCurrentMax_mA = InputCurrent_GetMax_mA(source);

	// P&O intentionally allows Buck, Buck-Boost and Boost operation. Do not use
	// a fixed 21.5V guard here: it would discard useful low-light solar power.
	// Only a true loss of source voltage forces the request back to minimum.
	if (source == INPUT_SOURCE_SOLAR)
	{
		const int32_t inputVoltage_mV = InputVoltage_Get_mV(source);
		if (inputVoltage_mV < SOLAR_POWER_HARD_FLOOR_mV)
		{
			inputLimitedChargeCurrentSet_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
			solarChargeCurrentSet_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
			solarMpptPowerSet_mW = SOLAR_POWER_SET_MIN_mW;
			solarMpptDirection = -1;
			solarMpptValid = false;
			solarRegulationStatus = SOLAR_REG_FAST_DOWN;
			return inputLimitedChargeCurrentSet_mA;
		}
	}

	// Input current is too high: reduce charge current target quickly.
	if (inputCurrent_mA > (inputCurrentMax_mA + INPUT_CURRENT_HYST_mA))
	{
		if (inputLimitedChargeCurrentSet_mA > INPUT_CURRENT_FAST_DOWN_mA)
		inputLimitedChargeCurrentSet_mA -= INPUT_CURRENT_FAST_DOWN_mA;
		else
		inputLimitedChargeCurrentSet_mA = 0L;

		return inputLimitedChargeCurrentSet_mA;
	}

	// Input current is near the limit: reduce charge current target gently.
	if (inputCurrent_mA > inputCurrentMax_mA)
	{
		if (inputLimitedChargeCurrentSet_mA > INPUT_CURRENT_STEP_DOWN_mA)
		inputLimitedChargeCurrentSet_mA -= INPUT_CURRENT_STEP_DOWN_mA;
		else
		inputLimitedChargeCurrentSet_mA = 0L;

		return inputLimitedChargeCurrentSet_mA;
	}

	// Input current has enough margin again: increase charge current target slowly up to requested value.
	if (inputCurrent_mA < (inputCurrentMax_mA - INPUT_CURRENT_HYST_mA))
	{
		if (inputLimitedChargeCurrentSet_mA < (requestedSet_mA - INPUT_CURRENT_STEP_UP_mA))
		inputLimitedChargeCurrentSet_mA += INPUT_CURRENT_STEP_UP_mA;
		else
		inputLimitedChargeCurrentSet_mA = requestedSet_mA;
	}

	return inputLimitedChargeCurrentSet_mA;
}

int32_t InputVoltage_Get_mV(uint8_t source)
{
	if (source == INPUT_SOURCE_MAIN)
		return ina238[CH_MAINDC].voltage_mV;

	if (source == INPUT_SOURCE_SOLAR)
		return ina238[CH_SOLARDC].voltage_mV;

	if (source == INPUT_SOURCE_HELP)
		return ina238[CH_HELPDC].voltage_mV;

	return 0L;
}

int32_t InputCurrent_Get_mA(uint8_t source)
{
	if (source == INPUT_SOURCE_MAIN)
	return ina238[CH_MAINDC].current_mA;

	if (source == INPUT_SOURCE_SOLAR)
	return ina238[CH_SOLARDC].current_mA;

	if (source == INPUT_SOURCE_HELP)
	return ina238[CH_HELPDC].current_mA;

	return 0L;
}

int32_t InputCurrent_GetMax_mA(uint8_t source)
{
	if (source == INPUT_SOURCE_MAIN)
	return MAIN_INPUT_CURRENT_MAX_mA;

	if (source == INPUT_SOURCE_SOLAR)
	return SOLAR_INPUT_CURRENT_MAX_mA;

	if (source == INPUT_SOURCE_HELP)
	return HELP_INPUT_CURRENT_MAX_mA;

	return 0L;
}

int32_t ChargeCurrent_GetSet_mA()
{
	int32_t target_mV = (batteryChargeState == BATTERY_CHARGE_FULL) ?
	                    BATTERY_FLOAT_mV : BATTERY_ABSORPTION_mV;
	int32_t battery_mV = ina238[CH_BATTERY].voltage_mV;
	int32_t limited_mA = activeChargeCurrentSet_mA;

	if (thermalShutdown || batteryOvervoltageFault)
	return 0L;

	if (batteryRecoveryActive)
	{
		if (limited_mA > batteryRecoveryCurrentLimit_mA)
		limited_mA = batteryRecoveryCurrentLimit_mA;
		return limited_mA;
	}

	if (battery_mV >= BATTERY_24V_HARD_STOP_mV)
	return 0L;

	if (battery_mV >= target_mV)
	return 0L;

	if (battery_mV > (target_mV - BATTERY_VOLTAGE_TAPER_mV))
	{
		limited_mA = (limited_mA * (target_mV - battery_mV)) / BATTERY_VOLTAGE_TAPER_mV;
		if (limited_mA < 0L) limited_mA = 0L;
	}

	return limited_mA;
}

int32_t ChargeCurrent_GetMax_mA()
{
	return activeChargeCurrentMax_mA;
}

int32_t BatteryCharge_SelectCurrent_mA(int32_t normalCurrent_mA, int32_t maximumCurrent_mA)
{
	int32_t battery_mV;

	if (!ina238[CH_BATTERY].online)
	return normalCurrent_mA;

	battery_mV = ina238[CH_BATTERY].voltage_mV;

	// Use maximum current only in the lower voltage charge range.
	if ((battery_mV >= BATTERY_HIGH_CURRENT_MIN_mV) &&
		(battery_mV <= BATTERY_HIGH_CURRENT_MAX_mV))
	return maximumCurrent_mA;

	return normalCurrent_mA;
}

void BatteryCharge_Task()
{
    int32_t battery_mV;
    int32_t batteryCurrent_mA;
    uint32_t now_ms = millis();

    if (!ina238[CH_BATTERY].online)
        return;

    battery_mV = ina238[CH_BATTERY].voltage_mV;
    batteryCurrent_mA = chargeCurrentFiltered_mA;

    // Recovery has its own current/voltage progression; normal daily charge-state
    // transitions resume only after the BMS recovery ramp is complete.
    if (batteryRecoveryActive)
    {
        if (batteryChargeState == BATTERY_CHARGE_WAIT)
            batteryChargeState = BATTERY_CHARGE_BULK;
        return;
    }

    // A real night / long period without selected solar arms exactly one new
    // 28.10V cycle. Short clouds do not reset the daily full-charge state.
    if (!stateSolarDC)
    {
        if (batteryNoSolarStart_ms == 0UL)
            batteryNoSolarStart_ms = now_ms;
        else if ((uint32_t)(now_ms - batteryNoSolarStart_ms) >=
                 BATTERY_DAILY_RESET_NO_SOLAR_MS)
            batteryChargeState = BATTERY_CHARGE_WAIT;
    }
    else
    {
        batteryNoSolarStart_ms = 0UL;
    }

    // First usable source after boot/night: always perform one complete cycle.
    if (batteryChargeState == BATTERY_CHARGE_WAIT)
    {
        batteryChargeState = BATTERY_CHARGE_BULK;
        return;
    }

    if (batteryChargeState == BATTERY_CHARGE_BULK)
    {
        if (battery_mV >= BATTERY_ABSORPTION_mV)
            batteryChargeState = BATTERY_CHARGE_ABS;
        return;
    }

    if (batteryChargeState == BATTERY_CHARGE_ABS)
    {
        if (stateCharge && chargeCurrentFilterValid &&
            (battery_mV >= (BATTERY_ABSORPTION_mV - BATTERY_ABSORB_TOLERANCE_mV)) &&
            (batteryCurrent_mA >= 0L) &&
            (batteryCurrent_mA < BATTERY_FLOAT_CURRENT_mA))
        {
            batteryChargeState = BATTERY_CHARGE_FULL; // 27.80V float after daily 28.10V cycle.
        }
        return;
    }

    // FULL remains float. Do not restart another 28.10V cycle during the same day.
    // The next complete cycle is armed only by BATTERY_DAILY_RESET_NO_SOLAR_MS.
}

bool BatteryCharge_IsAllowed()
{
    uint32_t now_ms = millis();

    if (thermalShutdown)
        return false;

    if (batteryOvervoltageFault)
        return false;

    if (dcdcPreChargeTestFault)
        return false;

    if ((int32_t)(now_ms - dcdcReverseBlockedUntil_ms) < 0)
        return false;

    if (stateSolarDC && solarLowPauseActive)
    {
        if ((int32_t)(now_ms - solarLowPauseUntil_ms) < 0)
            return false;

        solarLowPauseActive = false;
        solarLowPowerMode = true;
        solarLowChargeCurrent_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
        solarLowOffTiming = false;
        solarLowOffStart_ms = 0UL;
        SolarMPPT_Reset();
    }

    if (batteryRecoveryActive)
        return true;

    return (batteryChargeState == BATTERY_CHARGE_BULK ||
            batteryChargeState == BATTERY_CHARGE_ABS ||
            batteryChargeState == BATTERY_CHARGE_FULL);
}

void SolarMPPT_Reset()
{
	solarChargeCurrentSet_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
	solarLowChargeCurrent_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
	solarLowEnterTiming = false;
	solarLowExitTiming = false;
	solarLowOffTiming = false;
	solarLowStepPending = false;
	solarLowSourceLostTiming = false;
	solarLowLastDrop_mV = 0L;
	solarLowStepStartVoltage_mV = 0L;
	solarLowHoldUntil_ms = 0UL;
	solarRegulationStatus = SOLAR_REG_IDLE;
	solarMpptLastPower_uW = 0LL;
	solarMpptPower_uW = 0LL;
	solarMpptPowerDiff_uW = 0LL;
	solarMpptStep_mA = SOLAR_MPPT_STEP_NORMAL_mA;
	solarMpptDirection = 1;
	solarMpptValid = false;
	solarMpptPowerSet_mW = SOLAR_POWER_SET_MIN_mW;
	solarMpptLastVoltage_mV = 0L;
	solarMpptPowerStep_mW = SOLAR_POWER_STEP_BUCK_mW;
	solarMpptEfficiency_permille = SOLAR_EFF_BUCK_PERMILLE;
	solarMpptHeadroom_mV = 0L;
	solarMpptReferenceVoltage_mV = 0L;
	solarMpptVoltageDrop_mV = 0L;
	solarMpptRebaseCounter = 0U;
	solarFastGuardLastTrip_ms = 0UL;
	solarFastGuardLastAction_ms = 0UL;
	solarFastGuardHold = false;
	SolarHistory_Reset();
}

void SolarHistory_Reset()
{
	solarStableHistoryIndex = 0U;
	solarStableHistoryCount = 0U;
	solarTrendHistoryIndex = 0U;
	solarTrendHistoryCount = 0U;
	solarLastSafePower_mW = SOLAR_POWER_SET_MIN_mW;
	solarRecoveryTargetPower_mW = SOLAR_POWER_SET_MIN_mW;
	solarRecoveryFailedPower_mW = SOLAR_POWER_SET_MIN_mW;
	solarRecoveryActive = false;
	solarRecoveryFine = false;
	solarRecoveryTargetLocked = false;
	solarRecoverySoftHoldUntil_ms = 0UL;
	solarRecoveryTargetStableSince_ms = 0UL;
}

void SolarTrend_Add(int32_t powerSet_mW, int32_t voltage_mV, int32_t measuredPower_mW)
{
	solarTrendPowerSet_mW[solarTrendHistoryIndex] = powerSet_mW;
	solarTrendVoltage_mV[solarTrendHistoryIndex] = voltage_mV;
	solarTrendMeasuredPower_mW[solarTrendHistoryIndex] = measuredPower_mW;
	solarTrendHistoryIndex++;
	if (solarTrendHistoryIndex >= SOLAR_TREND_HISTORY_SIZE)
		solarTrendHistoryIndex = 0U;
	if (solarTrendHistoryCount < SOLAR_TREND_HISTORY_SIZE)
		solarTrendHistoryCount++;
}

bool SolarTrend_Get(int32_t *voltageDelta_mV, int32_t *powerDelta_mW)
{
	int32_t oldVoltageSum_mV = 0L;
	int32_t newVoltageSum_mV = 0L;
	int32_t oldPowerSum_mW = 0L;
	int32_t newPowerSum_mW = 0L;
	uint8_t oldestIndex;
	uint8_t index;
	uint8_t i;

	if (solarTrendHistoryCount < SOLAR_TREND_HISTORY_SIZE)
		return false;

	// The write index points to the oldest entry when the ring is full.
	// Compare the average of the oldest three samples with the average of
	// the newest three samples. This gives about 0.9s observation time while
	// suppressing single-sample noise.
	oldestIndex = solarTrendHistoryIndex;
	for (i = 0U; i < 3U; i++)
	{
		index = oldestIndex + i;
		if (index >= SOLAR_TREND_HISTORY_SIZE)
			index -= SOLAR_TREND_HISTORY_SIZE;
		oldVoltageSum_mV += solarTrendVoltage_mV[index];
		oldPowerSum_mW += solarTrendMeasuredPower_mW[index];
	}

	for (i = 0U; i < 3U; i++)
	{
		index = oldestIndex + SOLAR_TREND_HISTORY_SIZE - 3U + i;
		while (index >= SOLAR_TREND_HISTORY_SIZE)
			index -= SOLAR_TREND_HISTORY_SIZE;
		newVoltageSum_mV += solarTrendVoltage_mV[index];
		newPowerSum_mW += solarTrendMeasuredPower_mW[index];
	}

	*voltageDelta_mV = (newVoltageSum_mV / 3L) - (oldVoltageSum_mV / 3L);
	*powerDelta_mW = (newPowerSum_mW / 3L) - (oldPowerSum_mW / 3L);
	return true;
}

void SolarStableHistory_Clear()
{
	uint8_t i;

	for (i = 0U; i < SOLAR_STABLE_HISTORY_SIZE; i++)
	{
		solarStableHistory[i].powerSet_mW = 0L;
		solarStableHistory[i].inputVoltage_mV = 0L;
		solarStableHistory[i].measuredPower_mW = 0L;
	}

	solarStableHistoryIndex = 0U;
	solarStableHistoryCount = 0U;
	solarLastSafePower_mW = SOLAR_POWER_SET_MIN_mW;
}

void SolarStableHistory_RemoveAbove(int32_t maximumPower_mW)
{
	SolarHistoryPoint retained[SOLAR_STABLE_HISTORY_SIZE];
	uint8_t retainedCount = 0U;
	uint8_t i;

	for (i = 0U; i < solarStableHistoryCount; i++)
	{
		if (solarStableHistory[i].powerSet_mW <= maximumPower_mW)
		{
			retained[retainedCount] = solarStableHistory[i];
			retainedCount++;
		}
	}

	SolarStableHistory_Clear();
	for (i = 0U; i < retainedCount; i++)
	{
		SolarStableHistory_Add(retained[i].powerSet_mW,
		                       retained[i].inputVoltage_mV,
		                       retained[i].measuredPower_mW);
	}
}

void SolarStableHistory_Add(int32_t powerSet_mW, int32_t voltage_mV, int32_t measuredPower_mW)
{
	SolarHistoryPoint *point = &solarStableHistory[solarStableHistoryIndex];
	point->powerSet_mW = powerSet_mW;
	point->inputVoltage_mV = voltage_mV;
	point->measuredPower_mW = measuredPower_mW;

	solarStableHistoryIndex++;
	if (solarStableHistoryIndex >= SOLAR_STABLE_HISTORY_SIZE)
		solarStableHistoryIndex = 0U;
	if (solarStableHistoryCount < SOLAR_STABLE_HISTORY_SIZE)
		solarStableHistoryCount++;

	if (powerSet_mW > solarLastSafePower_mW)
		solarLastSafePower_mW = powerSet_mW;
}

int32_t SolarStableHistory_FindSafePower_mW(int32_t failedPower_mW)
{
	int32_t safePower_mW = SOLAR_POWER_SET_MIN_mW;
	uint8_t i;

	for (i = 0U; i < solarStableHistoryCount; i++)
	{
		int32_t candidate_mW = solarStableHistory[i].powerSet_mW;
		if ((candidate_mW < failedPower_mW) && (candidate_mW > safePower_mW))
			safePower_mW = candidate_mW;
	}

	if (safePower_mW > (SOLAR_POWER_SET_MIN_mW + SOLAR_SAFE_POWER_MARGIN_mW))
		safePower_mW -= SOLAR_SAFE_POWER_MARGIN_mW;
	else
		safePower_mW = SOLAR_POWER_SET_MIN_mW;

	return safePower_mW;
}

int32_t SolarPowerToBatteryCurrent_mA(int32_t inputPower_mW,
                                     int32_t batteryVoltage_mV,
                                     int32_t efficiency_permille)
{
	int64_t current_mA;

	if ((inputPower_mW <= 0L) || (batteryVoltage_mV <= 0L))
		return 0L;

	current_mA = ((int64_t)inputPower_mW * (int64_t)efficiency_permille) /
	             (int64_t)batteryVoltage_mV;
	if (current_mA > 32767LL)
		current_mA = 32767LL;
	return (int32_t)current_mA;
}

int32_t SolarBatteryCurrentToInputPower_mW(int32_t batteryCurrent_mA,
                                          int32_t batteryVoltage_mV,
                                          int32_t efficiency_permille)
{
	int64_t power_mW;

	if ((batteryCurrent_mA <= 0L) || (batteryVoltage_mV <= 0L) ||
	    (efficiency_permille <= 0L))
		return 0L;

	power_mW = ((int64_t)batteryCurrent_mA * (int64_t)batteryVoltage_mV) /
	           (int64_t)efficiency_permille;
	if (power_mW > SOLAR_POWER_SET_MAX_mW)
		power_mW = SOLAR_POWER_SET_MAX_mW;
	return (int32_t)power_mW;
}

void SolarChargeCurrent_Task(uint32_t now_ms)
{
	int32_t solarVoltage_mV;
	int32_t solarCurrent_mA;
	int32_t batteryVoltage_mV;
	int32_t solarCurrentLimit_mA;
	int32_t maxPowerSet_mW;
	int32_t measuredPower_mW;
	int32_t powerDiff_mW;
	int32_t voltageDiff_mV;
	int32_t trackedCurrent_mA;
	int32_t requestedCurrent_mA;
	int32_t downStep_mW;
	int32_t trendVoltageDelta_mV = 0L;
	int32_t trendPowerDelta_mW = 0L;
	int32_t recoveryStep_mW;
	bool trendValid;
	bool sourceDropGuardActive;
	uint32_t regulationInterval_ms;
	int64_t solarPower_uW;

	if (!ina238[CH_SOLARDC].online || !ina238[CH_BATTERY].online)
	{
		SolarMPPT_Reset();
		return;
	}

	if (dcdcState != DCDC_STATE_READY)
	{
		solarChargeCurrentSet_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
		solarRegulationStatus = SOLAR_REG_HOLD;
		solarMpptValid = false;
		lastSolarRegulation_ms = now_ms;
		return;
	}

	solarVoltage_mV = ina238[CH_SOLARDC].voltage_mV;
	solarCurrent_mA = ina238[CH_SOLARDC].current_mA;
	batteryVoltage_mV = ina238[CH_BATTERY].voltage_mV;
	if (solarCurrent_mA < 0L)
		solarCurrent_mA = 0L;

	solarPower_uW = (int64_t)solarVoltage_mV * (int64_t)solarCurrent_mA;
	measuredPower_mW = (int32_t)(solarPower_uW / 1000LL);
	SolarTrend_Add(solarMpptPowerSet_mW, solarVoltage_mV, measuredPower_mW);
	trendValid = SolarTrend_Get(&trendVoltageDelta_mV, &trendPowerDelta_mW);

	solarMpptHeadroom_mV = solarVoltage_mV - batteryVoltage_mV;

	// Maintain the healthy source reference continuously so the fast guard can
	// react before the slower P&O interval expires. The reference rises
	// immediately, but a load-induced collapse is never tracked downward here.
	if (solarMpptReferenceVoltage_mV <= 0L ||
	    solarVoltage_mV > solarMpptReferenceVoltage_mV)
	{
		solarMpptReferenceVoltage_mV = solarVoltage_mV;
		solarMpptRebaseCounter = 0U;
	}

	solarMpptVoltageDrop_mV = solarMpptReferenceVoltage_mV - solarVoltage_mV;
	if (solarMpptVoltageDrop_mV < 0L)
		solarMpptVoltageDrop_mV = 0L;

	// ------------------------------------------------------------
	// LOW_SOLAR: sunrise / sunset operation without normal P&O.
	// Probe +50mA, observe panel-voltage reaction, and detect the knee.
	// ------------------------------------------------------------
	if (!solarLowPowerMode)
	{
		if (solarCurrent_mA <= SOLAR_LOW_ENTER_CURRENT_mA)
		{
			if (!solarLowEnterTiming)
			{
				solarLowEnterTiming = true;
				solarLowEnterStart_ms = now_ms;
			}
			else if ((uint32_t)(now_ms - solarLowEnterStart_ms) >= SOLAR_LOW_ENTER_TIME_MS)
			{
				solarLowPowerMode = true;
				solarLowEnterTiming = false;
				solarLowExitTiming = false;
				solarLowStepPending = false;
				solarLowLastDrop_mV = 0L;
				solarLowHoldUntil_ms = 0UL;
				solarLowSourceLostTiming = false;
				solarLowChargeCurrent_mA = solarChargeCurrentSet_mA;
				if (solarLowChargeCurrent_mA < SOLAR_CHARGE_CURRENT_MIN_mA)
					solarLowChargeCurrent_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
				solarMpptValid = false;
				solarRecoveryActive = false;
				solarRecoveryFine = false;
			}
		}
		else
			solarLowEnterTiming = false;
	}

	if (solarLowPowerMode)
	{
		if (solarCurrent_mA >= SOLAR_LOW_EXIT_CURRENT_mA)
		{
			if (!solarLowExitTiming)
			{
				solarLowExitTiming = true;
				solarLowExitStart_ms = now_ms;
			}
			else if ((uint32_t)(now_ms - solarLowExitStart_ms) >= SOLAR_LOW_EXIT_TIME_MS)
			{
				solarLowPowerMode = false;
				solarLowExitTiming = false;
				solarLowOffTiming = false;
				solarLowStepPending = false;
				solarLowSourceLostTiming = false;
				solarMpptValid = false;
				solarMpptPowerSet_mW = measuredPower_mW;
				if (solarMpptPowerSet_mW < SOLAR_POWER_SET_MIN_mW)
					solarMpptPowerSet_mW = SOLAR_POWER_SET_MIN_mW;
				lastSolarRegulation_ms = now_ms;
				return;
			}
		}
		else
			solarLowExitTiming = false;

		// Tunnel / source really gone: only battery voltage itself is the
		// immediate source-loss criterion. Falling below the old 34V value
		// alone no longer switches the converter off.
		if (solarVoltage_mV <= (batteryVoltage_mV + SOLAR_LOW_SOURCE_LOST_MARGIN_mV))
		{
			if (!solarLowSourceLostTiming)
			{
				solarLowSourceLostTiming = true;
				solarLowSourceLostStart_ms = now_ms;
			}
			else if ((uint32_t)(now_ms - solarLowSourceLostStart_ms) >= SOLAR_LOW_SOURCE_LOST_TIME_MS)
			{
				solarLowPauseActive = true;
				solarLowPauseUntil_ms = now_ms + SOLAR_LOW_RETRY_PAUSE_MS;
				solarLowSourceLostTiming = false;
				solarLowStepPending = false;
				solarLowLastDrop_mV = 0L;
				solarLowChargeCurrent_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
				solarChargeCurrentSet_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
				inputLimitedChargeCurrentSet_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
				solarRegulationStatus = SOLAR_REG_HOLD;
				return;
			}
		}
		else
			solarLowSourceLostTiming = false;

		// Night / practically no useful solar energy.
		if (solarCurrent_mA < SOLAR_LOW_OFF_CURRENT_mA)
		{
			if (!solarLowOffTiming)
			{
				solarLowOffTiming = true;
				solarLowOffStart_ms = now_ms;
			}
			else if ((uint32_t)(now_ms - solarLowOffStart_ms) >= SOLAR_LOW_OFF_TIME_MS)
			{
				solarLowPauseActive = true;
				solarLowPauseUntil_ms = now_ms + SOLAR_LOW_RETRY_PAUSE_MS;
				solarLowOffTiming = false;
				solarLowStepPending = false;
				solarLowLastDrop_mV = 0L;
				solarLowChargeCurrent_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
				solarChargeCurrentSet_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
				inputLimitedChargeCurrentSet_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
				solarRegulationStatus = SOLAR_REG_HOLD;
				return;
			}
		}
		else
			solarLowOffTiming = false;

		// After a detected knee, stay at the previous useful point for 10s.
		if ((int32_t)(now_ms - solarLowHoldUntil_ms) < 0)
		{
			solarChargeCurrentSet_mA = solarLowChargeCurrent_mA;
			inputLimitedChargeCurrentSet_mA = solarLowChargeCurrent_mA;
			solarRegulationStatus = SOLAR_REG_HOLD;
			solarMpptLastPower_uW = solarPower_uW;
			solarMpptLastVoltage_mV = solarVoltage_mV;
			return;
		}

		if ((uint32_t)(now_ms - solarLowLastControl_ms) >= SOLAR_LOW_CONTROL_INTERVAL_MS)
		{
			solarLowLastControl_ms = now_ms;

			if (solarLowStepPending)
			{
				int32_t newDrop_mV = solarLowStepStartVoltage_mV - solarVoltage_mV;
				if (newDrop_mV < 0L)
					newDrop_mV = 0L;

				bool nearBattery =
					solarVoltage_mV <= (batteryVoltage_mV + SOLAR_LOW_BAT_MARGIN_mV);

				bool kneeDetected =
					(newDrop_mV >= SOLAR_LOW_DROP_MIN_mV) &&
					(newDrop_mV >= (solarLowLastDrop_mV + SOLAR_LOW_DROP_INCREASE_mV));

				if (nearBattery || kneeDetected)
				{
					solarLowChargeCurrent_mA -= SOLAR_LOW_CURRENT_STEP_mA;
					if (solarLowChargeCurrent_mA < SOLAR_CHARGE_CURRENT_MIN_mA)
						solarLowChargeCurrent_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
					solarLowHoldUntil_ms = now_ms + SOLAR_LOW_HOLD_MS;
					solarLowLastDrop_mV = 0L;
				}
				else
				{
					solarLowLastDrop_mV = newDrop_mV;
				}

				solarLowStepPending = false;
			}
			else
			{
				solarLowStepStartVoltage_mV = solarVoltage_mV;
				solarLowChargeCurrent_mA += SOLAR_LOW_CURRENT_STEP_mA;
				if (solarLowChargeCurrent_mA > SOLAR_CHARGE_CURRENT_MAX_mA)
					solarLowChargeCurrent_mA = SOLAR_CHARGE_CURRENT_MAX_mA;
				solarLowStepPending = true;
			}
		}

		solarChargeCurrentSet_mA = solarLowChargeCurrent_mA;
		inputLimitedChargeCurrentSet_mA = solarLowChargeCurrent_mA;
		solarRegulationStatus = SOLAR_REG_HOLD;
		solarMpptLastPower_uW = solarPower_uW;
		solarMpptLastVoltage_mV = solarVoltage_mV;
		return;
	}

	// Immediate weak-source protection. One action is permitted every 200ms.
	// A moderate collapse halves the requested power. A deeper collapse cuts it
	// to one quarter, and a severe collapse returns directly to minimum power.
	if ((solarMpptReferenceVoltage_mV > 0L) &&
	    ((uint32_t)(now_ms - solarFastGuardLastAction_ms) >=
	     SOLAR_FAST_GUARD_RETRIGGER_MS))
	{
		int32_t guardedPower_mW = solarMpptPowerSet_mW;
		bool guardTrip = false;
		bool severeGuardTrip = false;

		if ((solarMpptVoltageDrop_mV >= SOLAR_FAST_GUARD_COLLAPSE_DROP_mV) ||
		    (solarVoltage_mV <
		     (solarMpptReferenceVoltage_mV * SOLAR_FAST_GUARD_COLLAPSE_PERCENT) / 100L))
		{
			guardedPower_mW = SOLAR_POWER_SET_MIN_mW;
			guardTrip = true;
			severeGuardTrip = true;
		}
		else if (solarMpptVoltageDrop_mV >= SOLAR_FAST_GUARD_QUARTER_DROP_mV)
		{
			guardedPower_mW = solarMpptPowerSet_mW / 4L;
			guardTrip = true;
		}
		else if (solarMpptVoltageDrop_mV >= SOLAR_FAST_GUARD_HALF_DROP_mV)
		{
			guardedPower_mW = solarMpptPowerSet_mW / 2L;
			guardTrip = true;
		}

		if (guardTrip)
		{
			// Capture the failed point and recovery target only on the first trip.
			// Further guard actions during the same collapse may reduce Pset again,
			// but must never overwrite the previously learned safe target.
			if (!solarRecoveryTargetLocked)
			{
				int32_t previousSafePower_mW = solarLastSafePower_mW;
				int32_t newSafeEstimate_mW;

				solarRecoveryFailedPower_mW = solarMpptPowerSet_mW;
				newSafeEstimate_mW = solarRecoveryFailedPower_mW -
				                         SOLAR_SAFE_POWER_MARGIN_mW;
				if (newSafeEstimate_mW < SOLAR_POWER_SET_MIN_mW)
					newSafeEstimate_mW = SOLAR_POWER_SET_MIN_mW;

				if ((previousSafePower_mW > SOLAR_POWER_SET_MIN_mW) &&
				    (solarRecoveryFailedPower_mW <
				     (previousSafePower_mW * SOLAR_SAFE_CLEAR_PERCENT) / 100L))
				{
					// Source capability changed strongly: discard the old history.
					SolarStableHistory_Clear();
					solarRecoveryTargetPower_mW = newSafeEstimate_mW;
				}
				else if ((previousSafePower_mW > SOLAR_POWER_SET_MIN_mW) &&
				         (solarRecoveryFailedPower_mW <
				          (previousSafePower_mW * SOLAR_SAFE_KEEP_PERCENT) / 100L))
				{
					// Source capability changed moderately: keep only lower points.
					SolarStableHistory_RemoveAbove(newSafeEstimate_mW);
					solarRecoveryTargetPower_mW = newSafeEstimate_mW;
				}
				else
				{
					solarRecoveryTargetPower_mW =
						SolarStableHistory_FindSafePower_mW(solarRecoveryFailedPower_mW);
				}

				solarLastSafePower_mW = solarRecoveryTargetPower_mW;
				solarRecoveryTargetLocked = true;
				solarRecoveryTargetStableSince_ms = 0UL;
			}
			solarRecoveryActive = false;
			solarRecoveryFine = false;

			if (guardedPower_mW < SOLAR_POWER_SET_MIN_mW)
				guardedPower_mW = SOLAR_POWER_SET_MIN_mW;

			solarMpptPowerSet_mW = guardedPower_mW;
			requestedCurrent_mA = SolarPowerToBatteryCurrent_mA(
				solarMpptPowerSet_mW, batteryVoltage_mV,
				solarMpptEfficiency_permille);
			if (requestedCurrent_mA < SOLAR_CHARGE_CURRENT_MIN_mA)
				requestedCurrent_mA = SOLAR_CHARGE_CURRENT_MIN_mA;

			solarChargeCurrentSet_mA = requestedCurrent_mA;
			inputLimitedChargeCurrentSet_mA = requestedCurrent_mA;

			// A severe source collapse needs an immediate hardware unload in
			// addition to the new current request. Higher DAC code means less
			// DCDC output voltage and therefore less input power.
			if (severeGuardTrip)
			{
				uint16_t releasedCode = dacCode + SOLAR_FAST_GUARD_DAC_RELEASE_CODE;
				if ((releasedCode < dacCode) || (releasedCode > DAC_CODE_MAX))
					releasedCode = DAC_CODE_MAX;
				DAC_WriteCode(releasedCode);
			}

			solarMpptDirection = -1;
			solarMpptValid = false;
			solarRegulationStatus = SOLAR_REG_FAST_DOWN;
			solarFastGuardLastAction_ms = now_ms;
			solarFastGuardLastTrip_ms = now_ms;
			solarFastGuardHold = true;
			lastSolarRegulation_ms = now_ms;
			return;
		}
	}

	// After a collapse, keep the reduced request unchanged for two seconds. This
	// lets the source voltage and the DCDC current loop settle before P&O searches
	// upward again. A new or deeper collapse above can still retrigger the guard.
	if (solarFastGuardHold)
	{
		if ((uint32_t)(now_ms - solarFastGuardLastTrip_ms) <
		    SOLAR_FAST_GUARD_RECOVERY_MS)
		{
			solarRegulationStatus = SOLAR_REG_HOLD;
			lastSolarRegulation_ms = now_ms;
			return;
		}

		solarFastGuardHold = false;
		solarMpptValid = false;
		solarMpptDirection = 1;
		solarRecoveryActive =
			(solarRecoveryTargetPower_mW > solarMpptPowerSet_mW);
		solarRecoveryFine = !solarRecoveryActive;
		lastSolarRegulation_ms = now_ms;
		return;
	}

	if ((int32_t)(now_ms - solarRecoverySoftHoldUntil_ms) < 0)
	{
		solarRegulationStatus = SOLAR_REG_HOLD;
		lastSolarRegulation_ms = now_ms;
		return;
	}

	if (solarMpptHeadroom_mV >= SOLAR_BUCK_HEADROOM_GOOD_mV)
	{
		regulationInterval_ms = SOLAR_REGULATION_INTERVAL_BUCK_MS;
		solarMpptPowerStep_mW = SOLAR_POWER_STEP_BUCK_mW;
		solarMpptEfficiency_permille = SOLAR_EFF_BUCK_PERMILLE;
	}
	else if (solarMpptHeadroom_mV >= SOLAR_BUCK_HEADROOM_MIN_mV)
	{
		regulationInterval_ms = SOLAR_REGULATION_INTERVAL_TRANS_MS;
		solarMpptPowerStep_mW = SOLAR_POWER_STEP_TRANS_mW;
		solarMpptEfficiency_permille = SOLAR_EFF_TRANS_PERMILLE;
	}
	else if (solarMpptHeadroom_mV >= SOLAR_NEAR_BATTERY_BAND_mV)
	{
		regulationInterval_ms = SOLAR_REGULATION_INTERVAL_NEAR_MS;
		solarMpptPowerStep_mW = SOLAR_POWER_STEP_NEAR_mW;
		solarMpptEfficiency_permille = SOLAR_EFF_NEAR_PERMILLE;
	}
	else
	{
		regulationInterval_ms = SOLAR_REGULATION_INTERVAL_BOOST_MS;
		solarMpptPowerStep_mW = SOLAR_POWER_STEP_BOOST_mW;
		solarMpptEfficiency_permille = SOLAR_EFF_BOOST_PERMILLE;
	}

	if ((uint32_t)(now_ms - lastSolarRegulation_ms) < regulationInterval_ms)
		return;
	lastSolarRegulation_ms = now_ms;

	solarCurrentLimit_mA = BatteryCharge_SelectCurrent_mA(SOLAR_CHARGE_CURRENT_SET_mA,
	                                                     SOLAR_CHARGE_CURRENT_MAX_mA);
	if (solarCurrentLimit_mA > SOLAR_CHARGE_CURRENT_MAX_mA)
		solarCurrentLimit_mA = SOLAR_CHARGE_CURRENT_MAX_mA;
	if (solarCurrentLimit_mA < SOLAR_CHARGE_CURRENT_MIN_mA)
		solarCurrentLimit_mA = SOLAR_CHARGE_CURRENT_MIN_mA;

	maxPowerSet_mW = SolarBatteryCurrentToInputPower_mW(solarCurrentLimit_mA,
	                                                   batteryVoltage_mV,
	                                                   solarMpptEfficiency_permille);
	if (maxPowerSet_mW < SOLAR_POWER_SET_MIN_mW)
		maxPowerSet_mW = SOLAR_POWER_SET_MIN_mW;
	if (solarMpptPowerSet_mW > maxPowerSet_mW)
		solarMpptPowerSet_mW = maxPowerSet_mW;

	solarMpptPower_uW = solarPower_uW;
	powerDiff_mW = measuredPower_mW - (int32_t)(solarMpptLastPower_uW / 1000LL);
	voltageDiff_mV = solarVoltage_mV - solarMpptLastVoltage_mV;
	solarMpptPowerDiff_uW = (int64_t)powerDiff_mW * 1000LL;

	// When the external source voltage is deliberately changed, the old reference
	// must eventually be discarded. Rebase only at minimum requested power and low
	// input current, so a normal current-limit collapse cannot redefine itself as
	// a healthy source voltage.
	if ((solarMpptPowerSet_mW <= SOLAR_POWER_SET_MIN_mW) &&
	    (solarCurrent_mA <= SOLAR_REFERENCE_REBASE_CURRENT_mA))
	{
		if (solarMpptRebaseCounter < SOLAR_REFERENCE_REBASE_CYCLES)
			solarMpptRebaseCounter++;

		if (solarMpptRebaseCounter >= SOLAR_REFERENCE_REBASE_CYCLES)
		{
			solarMpptReferenceVoltage_mV = solarVoltage_mV;
			solarMpptVoltageDrop_mV = 0L;
			solarMpptRebaseCounter = 0U;
		}
	}
	else
	{
		solarMpptRebaseCounter = 0U;
	}

	// The 1V reserve is a collapse guard, not a fixed-voltage MPPT. It becomes
	// active only when the extra loading no longer produces a useful power gain.
	sourceDropGuardActive =
		(solarMpptVoltageDrop_mV >=
		 (SOLAR_VOLTAGE_DROP_TARGET_mV - SOLAR_VOLTAGE_DROP_HYST_mV)) &&
		(powerDiff_mW < SOLAR_POWER_GAIN_MIN_mW);

	// Store only confirmed stable points. These entries form the FIFO used after
	// a source collapse. A point is stable when the current loop has settled, the
	// source voltage is close to its healthy reference and the short trend is not
	// falling without useful power gain.
	trackedCurrent_mA = chargeCurrentFiltered_mA;
	if (trackedCurrent_mA < 0L)
		trackedCurrent_mA = 0L;
	requestedCurrent_mA = SolarPowerToBatteryCurrent_mA(solarMpptPowerSet_mW,
	                                                   batteryVoltage_mV,
	                                                   solarMpptEfficiency_permille);
	if (!solarFastGuardHold &&
	    !solarRecoveryTargetLocked &&
	    ((trackedCurrent_mA + SOLAR_TRACK_TOL_mA) >= requestedCurrent_mA) &&
	    (solarMpptVoltageDrop_mV <= SOLAR_STABLE_VDROP_MAX_mV) &&
	    (!trendValid ||
	     (trendVoltageDelta_mV > -SOLAR_TREND_SOFT_DROP_mV) ||
	     (trendPowerDelta_mW >= SOLAR_TREND_POWER_GAIN_MIN_mW)))
	{
		SolarStableHistory_Add(solarMpptPowerSet_mW,
		                       solarVoltage_mV, measuredPower_mW);
	}

	// During recovery, normal P&O is completely bypassed. First move quickly
	// to the locked safe target. Only after reaching the target does the fine
	// search resume. A falling averaged voltage trend aborts the increase.
	if ((solarRecoveryActive || solarRecoveryFine) &&
	    ((int32_t)(now_ms - solarRecoverySoftHoldUntil_ms) >= 0))
	{
		if (trendValid &&
		    (trendVoltageDelta_mV <= -SOLAR_TREND_SOFT_DROP_mV) &&
		    (trendPowerDelta_mW < SOLAR_TREND_POWER_GAIN_MIN_mW))
		{
			solarMpptPowerSet_mW = solarRecoveryTargetPower_mW;
			if (solarMpptPowerSet_mW < SOLAR_POWER_SET_MIN_mW)
				solarMpptPowerSet_mW = SOLAR_POWER_SET_MIN_mW;
			solarMpptDirection = -1;
			solarMpptValid = false;
			solarRegulationStatus = SOLAR_REG_HOLD;
			solarRecoveryActive = false;
			solarRecoveryFine = true;
			solarRecoverySoftHoldUntil_ms = now_ms + SOLAR_RECOVERY_SOFT_HOLD_MS;
			solarRecoveryTargetStableSince_ms = 0UL;
		}
		else if (solarRecoveryActive)
		{
			int32_t distance_mW =
				solarRecoveryTargetPower_mW - solarMpptPowerSet_mW;

			if (distance_mW > 3000L)
				recoveryStep_mW = SOLAR_RECOVERY_FAST_STEP_mW;
			else if (distance_mW > 1000L)
				recoveryStep_mW = SOLAR_RECOVERY_MID_STEP_mW;
			else
				recoveryStep_mW = SOLAR_RECOVERY_FINE_STEP_mW;

			if (recoveryStep_mW > distance_mW)
				recoveryStep_mW = distance_mW;
			if (recoveryStep_mW > 0L)
				solarMpptPowerSet_mW += recoveryStep_mW;

			if (solarMpptPowerSet_mW > maxPowerSet_mW)
				solarMpptPowerSet_mW = maxPowerSet_mW;

			if (solarMpptPowerSet_mW >= solarRecoveryTargetPower_mW)
			{
				solarMpptPowerSet_mW = solarRecoveryTargetPower_mW;
				solarRecoveryActive = false;
				solarRecoveryFine = true;
				solarRecoveryTargetStableSince_ms = now_ms;
			}
			solarMpptDirection = 1;
			solarMpptValid = true;
			solarRegulationStatus = SOLAR_REG_UP;
		}
		else
		{
			// Hold the learned target for two stable seconds before unlocking it.
			// Afterwards normal P&O may continue with the normal regional step.
			if (solarRecoveryTargetStableSince_ms == 0UL)
				solarRecoveryTargetStableSince_ms = now_ms;

			if ((uint32_t)(now_ms - solarRecoveryTargetStableSince_ms) >=
			    SOLAR_RECOVERY_TARGET_STABLE_MS)
			{
				solarRecoveryFine = false;
				solarRecoveryTargetLocked = false;
				solarRecoveryTargetStableSince_ms = 0UL;
				solarMpptValid = false;
			}
			else
			{
				solarRegulationStatus = SOLAR_REG_HOLD;
			}
		}

		requestedCurrent_mA = SolarPowerToBatteryCurrent_mA(
			solarMpptPowerSet_mW, batteryVoltage_mV,
			solarMpptEfficiency_permille);
		if (requestedCurrent_mA < SOLAR_CHARGE_CURRENT_MIN_mA)
			requestedCurrent_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
		if (requestedCurrent_mA > solarCurrentLimit_mA)
			requestedCurrent_mA = solarCurrentLimit_mA;
		solarChargeCurrentSet_mA = requestedCurrent_mA;
		inputLimitedChargeCurrentSet_mA = requestedCurrent_mA;
		solarMpptLastPower_uW = solarPower_uW;
		solarMpptLastVoltage_mV = solarVoltage_mV;
		return;
	}

	// A real source collapse is handled quickly, but normal Boost operation down
	// to 18V remains valid and is not treated as a fault.
	if (solarVoltage_mV < SOLAR_POWER_HARD_FLOOR_mV)
	{
		solarMpptPowerSet_mW = SOLAR_POWER_SET_MIN_mW;
		solarMpptDirection = -1;
		solarMpptValid = false;
		solarRegulationStatus = SOLAR_REG_FAST_DOWN;
	}
	else if (solarMpptValid &&
	         ((voltageDiff_mV <= -SOLAR_RAPID_DROP_mV) ||
	          (solarMpptVoltageDrop_mV >=
	           (SOLAR_VOLTAGE_DROP_TARGET_mV + SOLAR_RAPID_DROP_mV))))
	{
		downStep_mW = SOLAR_POWER_CRITICAL_DOWN_mW;
		if (solarMpptPowerSet_mW > (SOLAR_POWER_SET_MIN_mW + downStep_mW))
			solarMpptPowerSet_mW -= downStep_mW;
		else
			solarMpptPowerSet_mW = SOLAR_POWER_SET_MIN_mW;
		solarMpptDirection = -1;
		solarRegulationStatus = SOLAR_REG_FAST_DOWN;
	}
	else if (solarMpptValid &&
	         sourceDropGuardActive &&
	         (solarMpptVoltageDrop_mV >
	          (SOLAR_VOLTAGE_DROP_TARGET_mV + SOLAR_VOLTAGE_DROP_HYST_mV)))
	{
		// The source has moved beyond the allowed approximately 1V reserve.
		// Reduce one region-dependent P&O step and let the inner current loop settle.
		downStep_mW = solarMpptPowerStep_mW;
		if (downStep_mW < SOLAR_POWER_STEP_MIN_mW)
			downStep_mW = SOLAR_POWER_STEP_MIN_mW;

		if (solarMpptPowerSet_mW > (SOLAR_POWER_SET_MIN_mW + downStep_mW))
			solarMpptPowerSet_mW -= downStep_mW;
		else
			solarMpptPowerSet_mW = SOLAR_POWER_SET_MIN_mW;

		solarMpptDirection = -1;
		solarRegulationStatus = SOLAR_REG_DOWN;
	}
	else if (!solarMpptValid)
	{
		solarMpptValid = true;
		solarMpptDirection = 1;
		solarRegulationStatus = SOLAR_REG_HOLD;
	}
	else if (solarRecoveryActive)
	{
		// The recovery block above already selected the next request.
	}
	else
	{
		// Do not perturb upward while the inner battery-current loop has not yet
		// reached the current corresponding to the previous power request.
		trackedCurrent_mA = chargeCurrentFiltered_mA;
		if (trackedCurrent_mA < 0L)
			trackedCurrent_mA = 0L;
		requestedCurrent_mA = SolarPowerToBatteryCurrent_mA(solarMpptPowerSet_mW,
		                                                       batteryVoltage_mV,
		                                                       solarMpptEfficiency_permille);

		if ((solarMpptDirection > 0) &&
		    (trackedCurrent_mA + SOLAR_TRACK_TOL_mA < requestedCurrent_mA))
		{
			solarRegulationStatus = SOLAR_REG_HOLD;
		}
		else if (sourceDropGuardActive &&
		         (solarMpptVoltageDrop_mV >=
		          (SOLAR_VOLTAGE_DROP_TARGET_mV - SOLAR_VOLTAGE_DROP_HYST_mV)))
		{
			// Stay close to the approximately 1V source-voltage reduction. Do not
			// demand more power until the source voltage or available power recovers.
			solarRegulationStatus = SOLAR_REG_HOLD;
		}
		else
		{
			// Classic P&O with voltage-collapse assistance. A perturbation that
			// lowers power reverses direction. If an upward perturbation produces
			// almost no power gain but lowers voltage, it also reverses early.
			if (solarMpptDirection > 0)
			{
				if ((powerDiff_mW < -SOLAR_POWER_HYST_mW) ||
				    ((voltageDiff_mV < -SOLAR_SOFT_DROP_mV) &&
				     (powerDiff_mW < SOLAR_POWER_GAIN_MIN_mW)))
					solarMpptDirection = -1;
			}
			else
			{
				if (powerDiff_mW < -SOLAR_POWER_HYST_mW)
					solarMpptDirection = 1;
			}

			if (solarMpptDirection > 0)
			{
				if (solarMpptPowerSet_mW < (maxPowerSet_mW - solarMpptPowerStep_mW))
					solarMpptPowerSet_mW += solarMpptPowerStep_mW;
				else
					solarMpptPowerSet_mW = maxPowerSet_mW;
				solarRegulationStatus = SOLAR_REG_UP;
			}
			else
			{
				if (solarMpptPowerSet_mW > (SOLAR_POWER_SET_MIN_mW + solarMpptPowerStep_mW))
					solarMpptPowerSet_mW -= solarMpptPowerStep_mW;
				else
					solarMpptPowerSet_mW = SOLAR_POWER_SET_MIN_mW;
				solarRegulationStatus = SOLAR_REG_DOWN;
			}
		}
	}

	solarMpptLastPower_uW = solarPower_uW;
	solarMpptLastVoltage_mV = solarVoltage_mV;

	requestedCurrent_mA = SolarPowerToBatteryCurrent_mA(solarMpptPowerSet_mW,
	                                                   batteryVoltage_mV,
	                                                   solarMpptEfficiency_permille);
	if (requestedCurrent_mA < SOLAR_CHARGE_CURRENT_MIN_mA)
		requestedCurrent_mA = SOLAR_CHARGE_CURRENT_MIN_mA;
	if (requestedCurrent_mA > solarCurrentLimit_mA)
		requestedCurrent_mA = solarCurrentLimit_mA;
	solarChargeCurrentSet_mA = requestedCurrent_mA;
	solarMpptStep_mA = SolarPowerToBatteryCurrent_mA(solarMpptPowerStep_mW,
	                                               batteryVoltage_mV,
	                                               solarMpptEfficiency_permille);
}


// ============================================================
// DCDC Charge State Machine
// ============================================================

void DCDC_Task(uint32_t now_ms)
{
	if ((uint32_t)(now_ms - lastDcdcTask_ms) < DCDC_TASK_INTERVAL_MS)
	return;

	lastDcdcTask_ms = now_ms;
	BatteryRecovery_Task(now_ms);
	ChargeCurrent_Task(now_ms);
	BatteryCharge_Task();

	// Absolute battery protection. Disconnect before the internal BMS hard cutoff.
	if (ina238[CH_BATTERY].online &&
	    ina238[CH_BATTERY].voltage_mV >= BATTERY_24V_HARD_STOP_mV)
	{
		batteryOvervoltageFault = true;
		batteryChargeState = BATTERY_CHARGE_FULL;
		DCDC_AllOff();
		return;
	}

	// Source removed: disconnect the battery and switch off DCDC.
	// A latched DCDC test fault is cleared only after the source disappears.
	if (!InputSource_Available())
	{
		dcdcPreChargeTestFault = false;
		batteryOvervoltageFault = false;
		BatteryRecovery_Stop();
		DCDC_AllOff();
		return;
	}

	// Never reconnect the battery to a DCDC that failed the real INA test.
	if (dcdcPreChargeTestFault)
	{
		Charge_Off();
		DCDC_Disable();
		DCDC_ResetCurrentControl();
		DCDC_SetState(DCDC_STATE_OFF);
		return;
	}

	// Battery full or not low enough: keep the input selected, but keep DCDC and Charge OFF.
	if (!BatteryCharge_IsAllowed())
	{
		Charge_Off();
		DCDC_Disable();
		DCDC_ResetCurrentControl();
		DCDC_PreChargeTest_Reset();
		DCDC_SetState(DCDC_STATE_OFF);
		return;
	}

	if (dcdcState == DCDC_STATE_OFF)
	{
		Charge_Off();
		DCDC_Disable();
		DCDC_ResetCurrentControl();
		DCDC_PreChargeTest_Reset();

		// Set the DAC close to battery voltage before DCDC start.
		if (DCDC_SetStartVoltageFromBattery())
		{
			dcdcDelayStart_ms = now_ms;
			DCDC_SetState(DCDC_STATE_START);
		}

		return;
	}

	if (dcdcState == DCDC_STATE_START)
	{
		// Wait after the input source switch is ON before DCDC is enabled.
		if ((uint32_t)(now_ms - dcdcDelayStart_ms) < DCDC_ON_DELAY_MS)
		return;

		DCDC_Enable();
		lastControl_ms = now_ms;
		dcdcDelayStart_ms = now_ms;
		dcdcPreChargeTestPhaseStart_ms = now_ms;
		dcdcPreChargeTestState = DCDC_TEST_MATCH_BATTERY;
		dcdcPreChargeTestStableCounter = 0U;
		DCDC_SetState(DCDC_STATE_CHARGE_ON);
		return;
	}

	if (dcdcState == DCDC_STATE_CHARGE_ON)
	{
		// Keep SW-Charge open until INA_DCDC proves that the converter follows the DAC.
		Charge_Off();

		if (!DCDC_PreChargeTest_Task(now_ms))
		return;

		// Normal charging connects slightly above battery voltage. During BMS recovery
		// the converter instead connects at the defined 21.5V wake level.
		Charge_On();
		chargePositiveStart_ms = now_ms;
		DCDC_ReverseCurrentCheck_Reset();
		DCDC_SetState(DCDC_STATE_RAMP_CURRENT);
		return;
	}

	if (dcdcState == DCDC_STATE_RAMP_CURRENT)
	{
		// Give the existing slow current ramp time to take over. A continuously
		// negative battery current indicates that the DCDC did not take control.
		if (!DCDC_ReverseCurrentCheck_Task(now_ms))
		return;

		if (DCDC_ControlOneStepToCurrent(now_ms))
		DCDC_SetState(DCDC_STATE_READY);

		return;
	}

	if (dcdcState == DCDC_STATE_READY)
	{
		Charge_On();

		// Keep the current near the target.
		DCDC_ControlOneStepToCurrent(now_ms);
	}
}

bool DCDC_ControlOneStepToCurrent(uint32_t now_ms)
{
	int32_t iBatRaw_mA;
	uint16_t minCode;

	if (!ina238[CH_BATTERY].online)
	return false;

	if ((uint32_t)(now_ms - lastControl_ms) < CHARGE_RAMP_INTERVAL_MS)
	return false;

	lastControl_ms = now_ms;
	iBatRaw_mA = ina238[CH_BATTERY].current_mA;

	// Simple integer low pass filter:
	// First valid sample loads the filter. Later samples are averaged slowly.
	if (!chargeCurrentFilterValid)
	{
		chargeCurrentFiltered_mA = iBatRaw_mA;
		chargeCurrentFilterValid = true;
	}
	else
	chargeCurrentFiltered_mA += (iBatRaw_mA - chargeCurrentFiltered_mA) / CHARGE_CURRENT_FILTER_DIV;

	// Lower DAC code means higher DCDC output voltage and more charge current.
	// Never raise DCDC more than CHARGE_MAX_OFFSET_mV above the actual battery voltage.
	minCode = DCDC_CalcDacCodeForVout_mV((uint16_t)DCDC_GetMaxTargetFromBattery_mV());
	if (minCode < DCDC_DAC_MIN_SAFE) minCode = DCDC_DAC_MIN_SAFE;

	// Solar source control is performed by the outer power-P&O loop. The inner
	// loop remains a pure battery-current regulator; no fixed input-voltage DAC
	// override is used because valid low-light Boost operation may occur at 18V.


	// Emergency current reduction. This threshold is intentionally well above
	// the normal regulation dead band so measurement ripple cannot trigger it.
	if ((iBatRaw_mA >= CHARGE_SEVERE_OVER_mA) ||
	    (chargeCurrentFiltered_mA >= CHARGE_SEVERE_OVER_mA))
	{
		if (dacCode < (DAC_CODE_MAX - CHARGE_EMERGENCY_STEP_CODE))
			DAC_WriteCode(dacCode + CHARGE_EMERGENCY_STEP_CODE);
		else
			DAC_WriteCode(DAC_CODE_MAX);

		chargeStableCounter = 0;
		return false;
	}

	// Moderate fast reduction for a real overcurrent, not for a few mA of overshoot.
	if ((iBatRaw_mA >= CHARGE_ABSOLUTE_MAX_mA) ||
	    (chargeCurrentFiltered_mA >= CHARGE_ABSOLUTE_MAX_mA))
	{
		if (dacCode < (DAC_CODE_MAX - CHARGE_FAST_STEP_CODE))
			DAC_WriteCode(dacCode + CHARGE_FAST_STEP_CODE);
		else
			DAC_WriteCode(DAC_CODE_MAX);

		chargeStableCounter = 0;
		return false;
	}

	// Positive start phase after SW-Charge ON.
	// Capacitors may cause a temporary negative current reading.
	// During this time, the control only searches toward positive charge current.
	if ((uint32_t)(now_ms - chargePositiveStart_ms) < CHARGE_POSITIVE_START_MS)
	{
		if (chargeCurrentFiltered_mA < (ChargeCurrent_GetSet_mA() - CHARGE_CURRENT_HYST_mA))
		{
			if (dacCode > (minCode + CHARGE_CORRECT_STEP_CODE))
			DAC_WriteCode(dacCode - CHARGE_CORRECT_STEP_CODE);
			else
			DAC_WriteCode(minCode);
		}

		chargeStableCounter = 0;
		return false;
	}

	// Slow upward ramp when current is too low.
	if (chargeCurrentFiltered_mA < (ChargeCurrent_GetSet_mA() - CHARGE_CURRENT_HYST_mA))
	{
		if (dacCode > (minCode + CHARGE_CORRECT_STEP_CODE))
		DAC_WriteCode(dacCode - CHARGE_CORRECT_STEP_CODE);
		else
		DAC_WriteCode(minCode);

		chargeStableCounter = 0;
		return false;
	}

	// Downward current correction. During solar CV limiting, use a proportional
	// step so the actual current follows a rapidly reduced CVset without a long tail.
	if (chargeCurrentFiltered_mA > (ChargeCurrent_GetSet_mA() + CHARGE_CURRENT_HYST_mA))
	{
		uint16_t currentStep = CHARGE_CORRECT_STEP_CODE;
		int32_t currentExcess_mA =
			chargeCurrentFiltered_mA - ChargeCurrent_GetSet_mA();

		if (stateSolarDC)
		{
			if (currentExcess_mA > 1000L)
				currentStep = CHARGE_FAST_STEP_CODE;
			else if (currentExcess_mA > 300L)
				currentStep = 4U;
		}

		if (dacCode < (DAC_CODE_MAX - currentStep))
			DAC_WriteCode(dacCode + currentStep);
		else
			DAC_WriteCode(DAC_CODE_MAX);

		chargeStableCounter = 0;
		return false;
	}

	// Current is inside the dead band. READY is accepted only after several stable samples.
	if (chargeStableCounter < CHARGE_STABLE_COUNT)
	chargeStableCounter++;

return (chargeStableCounter >= CHARGE_STABLE_COUNT);
}

// ============================================================
// Battery BMS Recovery / Thermal Protection
// ============================================================

int32_t BatteryRecovery_GetStageCurrent_mA(uint8_t stage)
{
	switch (stage)
	{
		case 0U: return BATTERY_RECOVERY_STAGE_0_mA;
		case 1U: return BATTERY_RECOVERY_STAGE_1_mA;
		case 2U: return BATTERY_RECOVERY_STAGE_2_mA;
		case 3U: return BATTERY_RECOVERY_STAGE_3_mA;
		case 4U: return BATTERY_RECOVERY_STAGE_4_mA;
		case 5U: return BATTERY_RECOVERY_STAGE_5_mA;
		case 6U: return BATTERY_RECOVERY_STAGE_6_mA;
		default: return BATTERY_RECOVERY_STAGE_7_mA;
	}
}

void BatteryRecovery_Start(uint32_t now_ms)
{
	batteryRecoveryActive = true;
	batteryRecoveryLongWarning = false;
	batteryRecoveryStage = 0U;
	batteryRecoveryCurrentLimit_mA = BATTERY_RECOVERY_CURRENT_mA;
	batteryRecoveryStart_ms = now_ms;
	batteryRecoveryLastRamp_ms = now_ms;
	batteryChargeState = BATTERY_CHARGE_BULK;
}

void BatteryRecovery_Stop()
{
	batteryRecoveryActive = false;
	batteryRecoveryLongWarning = false;
	batteryRecoveryStage = 0U;
	batteryRecoveryCurrentLimit_mA = BATTERY_RECOVERY_CURRENT_mA;
	batteryRecoveryStart_ms = 0UL;
	batteryRecoveryLastRamp_ms = 0UL;
}

void BatteryRecovery_Task(uint32_t now_ms)
{
	if (!ina238[CH_BATTERY].online)
	return;

	int32_t battery_mV = ina238[CH_BATTERY].voltage_mV;

	if (!batteryRecoveryActive)
	{
		if (battery_mV < BATTERY_24V_BMS_LOW_mV)
		BatteryRecovery_Start(now_ms);
		return;
	}

	if (!batteryRecoveryLongWarning &&
	    batteryRecoveryStart_ms != 0UL &&
	    (uint32_t)(now_ms - batteryRecoveryStart_ms) >= BATTERY_RECOVERY_LONG_MS)
	{
		batteryRecoveryLongWarning = true;
	}

	// Below 22V the BMS is still in its wake region. Keep the tested low current ceiling.
	if (battery_mV < BATTERY_RECOVERY_WAKE_mV)
	{
		batteryRecoveryStage = 0U;
		batteryRecoveryCurrentLimit_mA = BATTERY_RECOVERY_CURRENT_mA;
		batteryRecoveryLastRamp_ms = now_ms;
		return;
	}

	// Between about 22V and 23V the BMS may already be internally active, but it still
	// limits the pack. Do not increase current yet.
	if (battery_mV < BATTERY_RECOVERY_RAMP_mV)
	{
		batteryRecoveryStage = 0U;
		batteryRecoveryCurrentLimit_mA = BATTERY_RECOVERY_CURRENT_mA;
		batteryRecoveryLastRamp_ms = now_ms;
		return;
	}

	if ((uint32_t)(now_ms - batteryRecoveryLastRamp_ms) < BATTERY_RECOVERY_RAMP_INTERVAL_MS)
	return;

	batteryRecoveryLastRamp_ms = now_ms;

	if (batteryRecoveryStage < 7U)
	{
		batteryRecoveryStage++;
		batteryRecoveryCurrentLimit_mA = BatteryRecovery_GetStageCurrent_mA(batteryRecoveryStage);
		return;
	}

	// One full interval at the normal-current stage has completed. Normal charge control may resume.
	BatteryRecovery_Stop();
}

int32_t DCDC_GetAllowedPower_W()
{
	int16_t hottest_C = (fetTemperature_C > coilTemperature_C) ?
	                   fetTemperature_C : coilTemperature_C;

	if (hottest_C >= DCDC_SHUTDOWN_TEMP_C)
	return 0L;

	if (hottest_C >= 80)
	return DCDC_DERATING_80C_POWER_W;

	if (hottest_C >= 75)
	return DCDC_DERATING_75C_POWER_W -
	       ((int32_t)(hottest_C - 75) *
	        (DCDC_DERATING_75C_POWER_W - DCDC_DERATING_80C_POWER_W) / 5L);

	if (hottest_C >= 70)
	return DCDC_DERATING_70C_POWER_W -
	       ((int32_t)(hottest_C - 70) *
	        (DCDC_DERATING_70C_POWER_W - DCDC_DERATING_75C_POWER_W) / 5L);

	if (hottest_C >= DCDC_DERATING_START_C)
	return DCDC_MAX_POWER_W -
	       ((int32_t)(hottest_C - DCDC_DERATING_START_C) *
	        (DCDC_MAX_POWER_W - DCDC_DERATING_70C_POWER_W) / 5L);

	return DCDC_MAX_POWER_W;
}

int32_t DCDC_GetPowerLimitedCurrent_mA()
{
	int32_t battery_mV = ina238[CH_BATTERY].voltage_mV;
	int32_t allowedPower_W = DCDC_GetAllowedPower_W();

	if (batteryRecoveryActive && battery_mV < BATTERY_RECOVERY_TARGET_mV)
	battery_mV = BATTERY_RECOVERY_TARGET_mV;

	if (battery_mV <= 0L || allowedPower_W <= 0L)
	return 0L;

	return (int32_t)(((int64_t)allowedPower_W * 1000000LL) / battery_mV);
}

void TemperatureProtection_Task()
{
	if (!thermalShutdown)
	{
		if ((fetTemperature_C >= DCDC_SHUTDOWN_TEMP_C) ||
		    (coilTemperature_C >= DCDC_SHUTDOWN_TEMP_C))
		{
			thermalShutdown = true;
			DCDC_AllOff();
		}
	}
	else
	{
		if ((fetTemperature_C <= DCDC_RESTART_TEMP_C) &&
		    (coilTemperature_C <= DCDC_RESTART_TEMP_C))
		{
			thermalShutdown = false;
		}
	}
}

// ============================================================
// DCDC Pre-Charge Self Test
// ============================================================

void DCDC_PreChargeTest_Reset()
{
	dcdcPreChargeTestState = DCDC_TEST_IDLE;
	dcdcPreChargeTestPhaseStart_ms = 0UL;
	dcdcPreChargeTestBaseVoltage_mV = 0L;
	dcdcPreChargeTestStableCounter = 0U;
	DCDC_ReverseCurrentCheck_Reset();
}

void DCDC_PreChargeTest_Fail()
{
	dcdcPreChargeTestState = DCDC_TEST_FAILED;
	dcdcPreChargeTestFault = true;
	Charge_Off();
	DCDC_Disable();
	DAC_WriteCode(DCDC_SAFE_START_CODE);
	DCDC_ResetCurrentControl();
	DCDC_SetState(DCDC_STATE_OFF);
}

bool DCDC_PreChargeTest_AdjustToVoltage(int32_t target_mV)
{
	int32_t dcdc_mV;

	if (!ina238[CH_DCDC].online)
	return false;

	dcdc_mV = ina238[CH_DCDC].voltage_mV;

	if (dcdc_mV < (target_mV - DCDC_TEST_MATCH_TOL_mV))
	{
		dcdcPreChargeTestStableCounter = 0U;
		// Lower DAC code raises DCDC output voltage.
		if (dacCode > DCDC_TEST_DAC_STEP_CODE)
		DAC_WriteCode(dacCode - DCDC_TEST_DAC_STEP_CODE);
		else
		DAC_WriteCode(DAC_CODE_MIN);
		return false;
	}

	if (dcdc_mV > (target_mV + DCDC_TEST_MATCH_TOL_mV))
	{
		dcdcPreChargeTestStableCounter = 0U;
		// Higher DAC code lowers DCDC output voltage.
		if (dacCode < (DAC_CODE_MAX - DCDC_TEST_DAC_STEP_CODE))
		DAC_WriteCode(dacCode + DCDC_TEST_DAC_STEP_CODE);
		else
		DAC_WriteCode(DAC_CODE_MAX);
		return false;
	}

	return true;
}

bool DCDC_PreChargeTest_Task(uint32_t now_ms)
{
	int32_t battery_mV;
	int32_t dcdc_mV;
	int32_t testTarget_mV;

	if (!ina238[CH_BATTERY].online || !ina238[CH_DCDC].online)
	{
		DCDC_PreChargeTest_Fail();
		return false;
	}

	battery_mV = ina238[CH_BATTERY].voltage_mV;
	dcdc_mV = ina238[CH_DCDC].voltage_mV;

	// With an open external BMS, terminal voltage cannot be used as the normal
	// precharge reference. Verify the DC/DC at the tested 21.5V wake level.
	if (batteryRecoveryActive)
	battery_mV = BATTERY_RECOVERY_TARGET_mV;

	// A healthy 24V DCDC must not remain far below the normal 24V battery range.
	if ((uint32_t)(now_ms - dcdcDelayStart_ms) >= CHARGE_ON_DELAY_MS)
	{
		if (dcdc_mV < DCDC_TEST_MIN_OUTPUT_mV)
		{
			DCDC_PreChargeTest_Fail();
			return false;
		}
	}

	if ((uint32_t)(now_ms - dcdcPreChargeTestPhaseStart_ms) > DCDC_TEST_PHASE_TIMEOUT_MS)
	{
		DCDC_PreChargeTest_Fail();
		return false;
	}

	if (dcdcPreChargeTestState == DCDC_TEST_MATCH_BATTERY)
	{
		// First bring the real DCDC output close to the real battery voltage.
		if (!DCDC_PreChargeTest_AdjustToVoltage(battery_mV))
		return false;

		if (dcdcPreChargeTestStableCounter < DCDC_TEST_STABLE_COUNT)
		{
			dcdcPreChargeTestStableCounter++;
			return false;
		}

		dcdcPreChargeTestBaseVoltage_mV = dcdc_mV;
		dcdcPreChargeTestStableCounter = 0U;
		dcdcPreChargeTestPhaseStart_ms = now_ms;
		dcdcPreChargeTestState = DCDC_TEST_RAISE_OUTPUT;
		return false;
	}

	if (dcdcPreChargeTestState == DCDC_TEST_RAISE_OUTPUT)
	{
		// Function test with CHARGE still open. Raise the real output clearly enough
		// that INA_DCDC can prove that the LM5176 power stage follows the DAC.
		testTarget_mV = battery_mV + DCDC_TEST_RAISE_OFFSET_mV;
		if (testTarget_mV > DCDC_VOUT_MAX_mV)
		testTarget_mV = DCDC_VOUT_MAX_mV;

		if (!DCDC_PreChargeTest_AdjustToVoltage(testTarget_mV))
		return false;

		if ((dcdc_mV - dcdcPreChargeTestBaseVoltage_mV) < DCDC_TEST_RISE_MIN_mV)
		{
			DCDC_PreChargeTest_Fail();
			return false;
		}

		if (dcdcPreChargeTestStableCounter < DCDC_TEST_STABLE_COUNT)
		{
			dcdcPreChargeTestStableCounter++;
			return false;
		}

		// Function test passed. Now command the DAC directly to the small positive
		// connection level. COUT has no discharge load, so do not wait for the real
		// open-circuit output voltage to fall to this value.
		testTarget_mV = batteryRecoveryActive ?
		                BATTERY_RECOVERY_TARGET_mV :
		                (battery_mV + DCDC_CONNECT_OFFSET_mV);
		if (testTarget_mV > DCDC_VOUT_MAX_mV)
		testTarget_mV = DCDC_VOUT_MAX_mV;
		DAC_WriteCode(DCDC_CalcDacCodeForVout_mV((uint16_t)testTarget_mV));
		dcdcPreChargeTestStableCounter = 0U;
		dcdcPreChargeTestPhaseStart_ms = now_ms;
		dcdcPreChargeTestState = DCDC_TEST_CONNECT_LEVEL;
		return false;
	}

	if (dcdcPreChargeTestState == DCDC_TEST_CONNECT_LEVEL)
	{
		// The DAC is already set to battery +20mV. Allow a short settling time for
		// the control input, then permit SW-Charge to close. The remaining charge in
		// the output capacitors is intentionally not used as a pass/fail criterion.
		if ((uint32_t)(now_ms - dcdcPreChargeTestPhaseStart_ms) < DCDC_CONNECT_SETTLE_MS)
		return false;

		dcdcPreChargeTestState = DCDC_TEST_PASSED;
		return true;
	}

	return (dcdcPreChargeTestState == DCDC_TEST_PASSED);
}

void DCDC_ReverseCurrentCheck_Reset()
{
	dcdcReverseCurrentTiming = false;
	dcdcReverseCurrentStart_ms = 0UL;
}

bool DCDC_ReverseCurrentCheck_Task(uint32_t now_ms)
{
	if (!ina238[CH_BATTERY].online)
	{
		DCDC_PreChargeTest_Fail();
		return false;
	}

	if (ina238[CH_BATTERY].current_mA < DCDC_REVERSE_CURRENT_LIMIT_mA)
	{
		if (!dcdcReverseCurrentTiming)
		{
			dcdcReverseCurrentTiming = true;
			dcdcReverseCurrentStart_ms = now_ms;
		}
		else if ((uint32_t)(now_ms - dcdcReverseCurrentStart_ms) >= DCDC_REVERSE_CURRENT_TIME_MS)
		{
			Charge_Off();
			DCDC_Disable();
			DAC_WriteCode(DCDC_SAFE_START_CODE);
			DCDC_ResetCurrentControl();
			DCDC_PreChargeTest_Reset();
			DCDC_SetState(DCDC_STATE_OFF);
			dcdcReverseBlockedUntil_ms = now_ms + DCDC_REVERSE_RETRY_MS;
			return false;
		}
	}
	else
	{
		DCDC_ReverseCurrentCheck_Reset();
	}

	return true;
}

void DCDC_ResetCurrentControl()
{
	chargeCurrentFiltered_mA = 0;
	chargeCurrentFilterValid = false;
	chargeStableCounter = 0;
	chargePositiveStart_ms = 0;
}

bool DCDC_SetStartVoltageFromBattery()
{
	int32_t target_mV;

	if (!ina238[CH_BATTERY].online)
	return false;

	target_mV = DCDC_GetStartTargetFromBattery_mV();

	if (target_mV < DCDC_VOUT_MIN_mV)
	target_mV = DCDC_VOUT_MIN_mV;

	if (target_mV > DCDC_VOUT_MAX_mV)
	target_mV = DCDC_VOUT_MAX_mV;

	return DAC_WriteCode(DCDC_CalcDacCodeForVout_mV((uint16_t)target_mV));
}

int32_t DCDC_GetStartTargetFromBattery_mV()
{
	if (batteryRecoveryActive)
	return BATTERY_RECOVERY_TARGET_mV;

	return ina238[CH_BATTERY].voltage_mV;
}

int32_t DCDC_GetMaxTargetFromBattery_mV()
{
	int32_t target_mV;

	target_mV = ina238[CH_BATTERY].voltage_mV + CHARGE_MAX_OFFSET_mV;

	if (batteryRecoveryActive && target_mV < BATTERY_RECOVERY_TARGET_mV)
	target_mV = BATTERY_RECOVERY_TARGET_mV;

	if (target_mV < DCDC_VOUT_MIN_mV)
	target_mV = DCDC_VOUT_MIN_mV;

	// Select 28.10V full-charge or 27.80V float ceiling.
	int32_t chargeTarget_mV = (batteryChargeState == BATTERY_CHARGE_FULL) ?
	                          BATTERY_FLOAT_mV : BATTERY_ABSORPTION_mV;
	if (target_mV > chargeTarget_mV)
	target_mV = chargeTarget_mV;

	if (target_mV > DCDC_VOUT_MAX_mV)
	target_mV = DCDC_VOUT_MAX_mV;

	return target_mV;
}

int32_t DCDC_GetTargetFromBattery_mV()
{
	return DCDC_GetMaxTargetFromBattery_mV();
}

// ============================================================
// Input Source Auto Switch
// ============================================================
//
// Source switch outputs:
// LOW  = ON
// HIGH = OFF
//
// Charge switch output PD6:
// HIGH = ON
// LOW  = OFF
//
// Priority:
// 1. Main
// 2. Solar
// 3. HelpDC

void InputSource_Task()
{
	bool mainAvailable;
	bool solarAvailable;
	bool helpAvailable;
	uint8_t wantedSource;
	uint32_t now_ms;

	mainAvailable  = ina238[CH_MAINDC].online  && (ina238[CH_MAINDC].voltage_mV >= MAIN_ON_mV);
	solarAvailable = ina238[CH_SOLARDC].online && (ina238[CH_SOLARDC].voltage_mV >= SOLAR_ON_mV);
	helpAvailable  = ina238[CH_HELPDC].online  && (ina238[CH_HELPDC].voltage_mV >= HELPDC_ON_mV);

	wantedSource = INPUT_SOURCE_NONE;

	if (mainAvailable)
	wantedSource = INPUT_SOURCE_MAIN;
	else if (stateSolarDC && ina238[CH_SOLARDC].online &&
	         ina238[CH_SOLARDC].voltage_mV >= SOLAR_ACTIVE_HOLD_MIN_mV)
	{
		// Keep an already selected solar source connected during a temporary
		// current-limit sag. The voltage guard reduces converter loading.
		wantedSource = INPUT_SOURCE_SOLAR;
	}
	else if (solarAvailable)
	wantedSource = INPUT_SOURCE_SOLAR;
	else if (helpAvailable)
	wantedSource = INPUT_SOURCE_HELP;

	if (wantedSource == INPUT_SOURCE_NONE)
	{
		inputSourcePending = INPUT_SOURCE_NONE;
		MainDC_Off();
		SolarDC_Off();
		HelpDC_Off();
		return;
	}

	// Keep the already selected source ON.
	if ((wantedSource == INPUT_SOURCE_MAIN  && stateMainDC) ||
		(wantedSource == INPUT_SOURCE_SOLAR && stateSolarDC) ||
		(wantedSource == INPUT_SOURCE_HELP  && stateHelpDC))
	{
		inputSourcePending = INPUT_SOURCE_NONE;
		return;
	}

	now_ms = millis();

	// A new source was detected. Keep all source switches OFF during the delay.
	if (inputSourcePending != wantedSource)
	{
		inputSourcePending = wantedSource;
		sourceDelayStart_ms = now_ms;
		MainDC_Off();
		SolarDC_Off();
		HelpDC_Off();
		return;
	}

	// Wait before the selected source switch is closed.
	if ((uint32_t)(now_ms - sourceDelayStart_ms) < SOURCE_ON_DELAY_MS)
	return;

	if (wantedSource == INPUT_SOURCE_MAIN)
	{
		MainDC_On();
		SolarDC_Off();
		HelpDC_Off();
		return;
	}

	if (wantedSource == INPUT_SOURCE_SOLAR)
	{
		MainDC_Off();
		SolarDC_On();
		HelpDC_Off();
		return;
	}

	MainDC_Off();
	SolarDC_Off();
	HelpDC_On();
}



// ============================================================
// Charger Communication Protocol: payload plausibility validation
// ============================================================
// Received communication is validated and stored only. It does not affect charge control.

void CommunicationReceive_Task()
{
    uint32_t now_ms = millis();

    if (communicationReceiveIndex > 0U &&
        (uint32_t)(now_ms - communicationReceiveLastByte_ms) > COMMUNICATION_RX_TIMEOUT_MS)
    {
        communicationRxTimeoutErrors++;
        CommunicationReceive_Reset();
    }

    while (Serial.available() > 0)
    {
        uint8_t value = (uint8_t)Serial.read();
        communicationReceiveLastByte_ms = millis();

        if (communicationReceiveIndex == 0U)
        {
            if (value == PROTOCOL_START_BYTE_1)
                communicationReceiveFrame[communicationReceiveIndex++] = value;
            continue;
        }

        if (communicationReceiveIndex == 1U)
        {
            if (value == PROTOCOL_START_BYTE_2)
                communicationReceiveFrame[communicationReceiveIndex++] = value;
            else if (value == PROTOCOL_START_BYTE_1)
                communicationReceiveFrame[0] = value;
            else
            {
                communicationRxFormatErrors++;
                CommunicationReceive_Reset();
            }
            continue;
        }

        communicationReceiveFrame[communicationReceiveIndex++] = value;

        if (communicationReceiveIndex >= PROTOCOL_FRAME_LENGTH)
        {
            CommunicationReceive_ValidateFrame(communicationReceiveFrame);
            CommunicationReceive_Reset();
        }
    }
}

bool CommunicationReceive_ValidateFrame(const uint8_t *frame)
{
    if (frame[0] != PROTOCOL_START_BYTE_1 || frame[1] != PROTOCOL_START_BYTE_2)
    {
        communicationRxFormatErrors++;
        return false;
    }

    if (frame[2] != PROTOCOL_VERSION)
    {
        communicationRxFormatErrors++;
        return false;
    }

    uint16_t messageType = ((uint16_t)frame[3] << 8) | frame[4];
    if (messageType != MESSAGE_CLIENT_RESPONSE)
    {
        communicationRxFormatErrors++;
        return false;
    }

    if (frame[6] != PROTOCOL_PAYLOAD_LENGTH)
    {
        communicationRxFormatErrors++;
        return false;
    }

    uint16_t calculatedCrc = 0xFFFFU;
    for (uint8_t i = 2U; i < (PROTOCOL_FRAME_LENGTH - 2U); i++)
        calculatedCrc = Communication_UpdateCrc16(calculatedCrc, frame[i]);

    uint16_t receivedCrc = ((uint16_t)frame[PROTOCOL_FRAME_LENGTH - 2U] << 8) |
                           frame[PROTOCOL_FRAME_LENGTH - 1U];

    if (calculatedCrc != receivedCrc)
    {
        communicationRxCrcErrors++;
        return false;
    }

    ClientResponsePayload decoded = {};
    uint8_t valueIndex = 7U;
    decoded.batteryVoltage_10mV = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.batteryCurrent_10mA = (int16_t)CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.requestedInputPower_W = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.actualInputPower_W = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.batterySoc_percent = frame[valueIndex++];
    decoded.chargeState = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.operatingState = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.batteryCapacity_Ah = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.batteryInfoCode = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.warningFlags = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.errorFlags = CommunicationReceive_ReadU16(frame, &valueIndex);

    if (valueIndex != PROTOCOL_FRAME_LENGTH - 2U ||
        !CommunicationReceive_ClientDataPlausible(&decoded))
    {
        communicationRxDataErrors++;
        return false;
    }

    uint8_t sequence = frame[5];
    if (!communicationAwaitingResponse || sequence != communicationLastSentSequence)
    {
        communicationRxSequenceErrors++;
        return false;
    }

    receivedClientResponse = decoded;
    receivedClientResponseSequence = sequence;
    receivedClientResponseTime_ms = millis();
    receivedClientResponseValid = true;
    communicationAwaitingResponse = false;
    communicationLinkTimeout = false;
    return true;
}

bool CommunicationReceive_ClientDataPlausible(const ClientResponsePayload *data)
{
    if (data->batteryVoltage_10mV > 1800U) return false;       // Maximum 18.00V.
    if (data->batteryCurrent_10mA < -2000 || data->batteryCurrent_10mA > 2000) return false;
    if (data->requestedInputPower_W > 500U) return false;
    if (data->actualInputPower_W > 500U) return false;
    if (data->batterySoc_percent > 100U) return false;
    if (data->chargeState > 4U) return false; // Assist adds FLOAT as protocol state 3; FULL is 4.
    if (data->operatingState > DCDC_STATE_READY) return false;
    if (data->batteryCapacity_Ah > 2000U) return false; // 0 = unknown; plausibility ceiling. Fix!Me
    if (data->batteryInfoCode > 7U) return false;
    return true;
}

uint16_t CommunicationReceive_ReadU16(const uint8_t *buffer, uint8_t *index)
{
    uint16_t value = ((uint16_t)buffer[*index] << 8) | buffer[*index + 1U];
    *index += 2U;
    return value;
}

void CommunicationReceive_Reset()
{
    communicationReceiveIndex = 0U;
    communicationReceiveLastByte_ms = 0UL;
}

void Communication_Task(uint32_t now_ms)
{
    static uint32_t lastSend_ms = 0UL;

    if (receivedClientResponseValid &&
        (uint32_t)(now_ms - receivedClientResponseTime_ms) > COMMUNICATION_LINK_TIMEOUT_MS)
    {
        communicationLinkTimeout = true;
        receivedClientResponseValid = false;
    }
    else if (!receivedClientResponseValid && communicationFirstRequestTime_ms != 0UL &&
             (uint32_t)(now_ms - communicationFirstRequestTime_ms) > COMMUNICATION_LINK_TIMEOUT_MS)
    {
        communicationLinkTimeout = true;
    }

    if ((uint32_t)(now_ms - lastSend_ms) < COMMUNICATION_INTERVAL_MS)
        return;

    lastSend_ms = now_ms;

    int32_t batteryVoltage_10mV = ina238[CH_BATTERY].voltage_mV / 10L;
    int32_t batteryCurrent_10mA = ina238[CH_BATTERY].current_mA / 10L;
    int32_t sourceVoltage_mV = InputVoltage_Get_mV(inputLimitSource);
    int32_t sourceCurrent_mA = InputCurrent_Get_mA(inputLimitSource);
    int32_t sourcePower_W = 0L;
    int32_t safeSourcePower_W = 0L;

    if (sourceVoltage_mV > 0L && sourceCurrent_mA > 0L)
        sourcePower_W = (int32_t)(((int64_t)sourceVoltage_mV * sourceCurrent_mA) / 1000000LL);

    if (inputLimitSource == INPUT_SOURCE_SOLAR)
        safeSourcePower_W = solarLastSafePower_mW / 1000L;
    else
        safeSourcePower_W = sourcePower_W;

    if (batteryVoltage_10mV < 0L) batteryVoltage_10mV = 0L;
    if (batteryVoltage_10mV > 65535L) batteryVoltage_10mV = 65535L;
    if (batteryCurrent_10mA < -32768L) batteryCurrent_10mA = -32768L;
    if (batteryCurrent_10mA > 32767L) batteryCurrent_10mA = 32767L;
    if (sourcePower_W < 0L) sourcePower_W = 0L;
    if (sourcePower_W > 65535L) sourcePower_W = 65535L;
    if (safeSourcePower_W < 0L) safeSourcePower_W = 0L;
    if (safeSourcePower_W > 65535L) safeSourcePower_W = 65535L;

    uint8_t payload[PROTOCOL_PAYLOAD_LENGTH];
    uint8_t index = 0U;

    Communication_WriteU16(payload, &index, (uint16_t)batteryVoltage_10mV);
    Communication_WriteU16(payload, &index, (uint16_t)(int16_t)batteryCurrent_10mA);
    Communication_WriteU16(payload, &index, (uint16_t)sourcePower_W);
    Communication_WriteU16(payload, &index, (uint16_t)safeSourcePower_W);
    Communication_WriteU16(payload, &index, 0U);                             // Assist grant reserved; not active yet.
    payload[index++] = 0U;                                                   // SOC reserved; not calculated yet.
    Communication_WriteU16(payload, &index, (uint16_t)inputLimitSource);

    uint16_t transmittedMpptState = (uint16_t)solarRegulationStatus;
    if (solarLowPauseActive)
        transmittedMpptState = 7U;
    else if (solarLowPowerMode)
        transmittedMpptState = 6U;

    Communication_WriteU16(payload, &index, transmittedMpptState);
    Communication_WriteU16(payload, &index, (uint16_t)dcdcState);

    uint16_t warningFlags = 0U;
    if (DCDC_GetAllowedPower_W() < DCDC_MAX_POWER_W && !thermalShutdown)
        warningFlags |= SOURCE_WARNING_THERMAL_DERATING;
    if (batteryRecoveryLongWarning)
        warningFlags |= SOURCE_WARNING_RECOVERY_LONG;
    Communication_WriteU16(payload, &index, warningFlags);

    uint16_t errorFlags = 0U;
    if (dcdcPreChargeTestFault)
        errorFlags |= SOURCE_ERROR_DCDC_PRECHARGE;
    if (batteryOvervoltageFault)
        errorFlags |= SOURCE_ERROR_BATTERY_OVERVOLTAGE;
    if (thermalShutdown)
        errorFlags |= SOURCE_ERROR_OVERTEMPERATURE;
    Communication_WriteU16(payload, &index, errorFlags);

    communicationLastSentSequence = communicationNextSequence++;
    communicationAwaitingResponse = true;
    if (communicationFirstRequestTime_ms == 0UL)
        communicationFirstRequestTime_ms = now_ms;
    Communication_SendFrame(MESSAGE_MASTER_REQUEST, communicationLastSentSequence, payload);
}

void Communication_SendFrame(uint16_t messageType, uint8_t sequence, const uint8_t *payload)
{
    uint8_t frame[PROTOCOL_FRAME_LENGTH];
    uint8_t index = 0U;
    uint16_t crc = 0xFFFFU;

    frame[index++] = PROTOCOL_START_BYTE_1;
    frame[index++] = PROTOCOL_START_BYTE_2;
    frame[index++] = PROTOCOL_VERSION;
    frame[index++] = (uint8_t)(messageType >> 8);
    frame[index++] = (uint8_t)messageType;
    frame[index++] = sequence;
    frame[index++] = PROTOCOL_PAYLOAD_LENGTH;

    for (uint8_t i = 2U; i < index; i++)
    {
        crc = Communication_UpdateCrc16(crc, frame[i]);
    }

    for (uint8_t i = 0U; i < PROTOCOL_PAYLOAD_LENGTH; i++)
    {
        frame[index++] = payload[i];
        crc = Communication_UpdateCrc16(crc, payload[i]);
    }

    frame[index++] = (uint8_t)(crc >> 8);
    frame[index++] = (uint8_t)crc;

    Serial.write(frame, PROTOCOL_FRAME_LENGTH);
}

void Communication_WriteU16(uint8_t *buffer, uint8_t *index, uint16_t value)
{
    buffer[(*index)++] = (uint8_t)(value >> 8);
    buffer[(*index)++] = (uint8_t)value;
}

uint16_t Communication_UpdateCrc16(uint16_t crc, uint8_t value)
{
    crc ^= (uint16_t)value << 8;

    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        if ((crc & 0x8000U) != 0U)
        {
            crc = (uint16_t)((crc << 1) ^ 0x1021U);
        }
        else
        {
            crc <<= 1;
        }
    }

    return crc;
}


