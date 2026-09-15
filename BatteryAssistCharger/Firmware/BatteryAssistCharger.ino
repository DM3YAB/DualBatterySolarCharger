/*
    Name:       BatteryAssistCharger.ino
    Author:     Andreas

    ATMega328P 16MHz (ProMini)

	I2C 0x40	INA238	HIGH_POWER
	I2C 0x41	INA238	MID_POWER / SolarShare from 24V battery
	I2C 0x43	INA238	LOW_POWER
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
// Charger Communication Protocol - Step 6
// ============================================================
// This section only defines the common protocol layout.
// No communication data affects the existing charge control yet.

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

#define COMMUNICATION_INTERVAL_MS       1000UL		// Response cadence follows valid Source requests.
#define COMMUNICATION_RX_TIMEOUT_MS       20UL		// Maximum gap inside one received frame. Fix!Me
#define COMMUNICATION_LINK_TIMEOUT_MS    3000UL		// Source data becomes invalid after this time. Fix!Me

// ============================================================
// 12V Battery Night Observation and Autonomous Support Charge - Step 5b
// ============================================================
// The RAM ring buffer covers six hours with one sample every ten minutes.
// The RAM history remains diagnostic. A critically low 12V battery may start
// an autonomous 8W support charge from the 24V input connected to SOLARDC.

#define BATTERY_HISTORY_INTERVAL_MS          600000UL   // 10 minutes
#define BATTERY_HISTORY_COUNT                36U        // 6 hours
#define BATTERY_RECOVERY_TIME_MS             1800000UL  // 30 minutes
#define BATTERY_LOW_WARNING_mV               12100L
#define BATTERY_SUPPORT_START_mV             11800L		// Warning/emergency threshold only. Fix!Me
#define BATTERY_SUPPORT_STOP_mV              12400L		// End autonomous support after recovery. Fix!Me
#define BATTERY_FAST_DROP_mV                 300L       // 0.30V within one hour
#define BATTERY_FAST_DETECT_MAX_mV           12600L      // Evaluate FAST_DISCHARGE only at/below 12.6V.
#define BATTERY_FAST_DROP_WINDOW_MS          3600000UL
#define BATTERY_LOAD_CURRENT_mA              -100L
#define BATTERY_NO_CHARGE_CURRENT_MAX_mA     100L

// External 12V charger / alternator protection.
#define EXTERNAL_CHARGE_REVERSE_mA           -100L
#define EXTERNAL_CHARGE_RELEASE_mV            13100L
#define EXTERNAL_CHARGE_RELEASE_TIME_MS       60000UL
#define BATTERY_SUPPORT_POWER_W               8U		// Autonomous support power from the 24V battery. Fix!Me
#define BATTERY_SUPPORT_CURRENT_MAX_mA         800L		// Absolute support-current ceiling. Fix!Me
#define BATTERY_SUPPORT_SOURCE_START_mV        24500L		// 24V battery must be above this level to start support. Fix!Me
#define BATTERY_SUPPORT_SOURCE_STOP_mV         24000L		// Stop support before the 24V battery is discharged further. Fix!Me

// Latched user warnings. Acknowledging clears only the display/communication
// latch; it never changes the charge or protection state machines.
#define BATTERY_WARNING_LOW_BIT                0x0001U
#define BATTERY_WARNING_FAST_BIT               0x0002U
#define BATTERY_LOW_REARM_mV                   12200L   // LOW may trigger again only after recovery above this level.
#define WARNING_ACK_HOLD_MS                    2000UL   // Hold OK for two seconds to acknowledge warnings.

#define ASSIST_BATTERY_CAPACITY_Ah              100U		// Installed 12V battery capacity reported to Source. 0 = unknown. Fix!Me
#define BATTERY_CAPACITY_MAX_Ah                2000U		// Protocol plausibility limit; uint16_t wire range remains larger. Fix!Me

#define ASSIST_WARNING_LOW_BATTERY          0x0001U
#define ASSIST_WARNING_FAST_DISCHARGE       0x0002U
#define ASSIST_WARNING_THERMAL_DERATING     0x0004U
#define ASSIST_ERROR_DCDC_PRECHARGE         0x0001U
#define ASSIST_ERROR_BATTERY_OVERVOLTAGE    0x0002U
#define ASSIST_ERROR_OVERTEMPERATURE        0x0004U

enum BatteryObservation_Info
{
    BATTERY_INFO_NORMAL = 0,
    BATTERY_INFO_LOW = 1,
    BATTERY_INFO_LOAD_ACTIVE = 2,
    BATTERY_INFO_RECOVERY = 3,
    BATTERY_INFO_FAST_DISCHARGE = 4,
    BATTERY_INFO_SUPPORT_REQUEST = 5,
    BATTERY_INFO_CHECK_BATTERY = 6,
    BATTERY_INFO_EXTERNAL_CHARGE = 7
};

struct BatteryHistoryEntry
{
    uint32_t time_ms;
    int16_t voltage_10mV;
    int16_t current_10mA;
};

static BatteryHistoryEntry batteryHistory[BATTERY_HISTORY_COUNT];
static uint8_t batteryHistoryWriteIndex = 0U;
static uint8_t batteryHistoryEntries = 0U;
static uint32_t batteryHistoryLastSample_ms = 0UL;
static uint32_t batteryLoadEnded_ms = 0UL;
static bool batteryLoadWasActive = false;
static bool batterySupportRequestActive = false; // Battery needs autonomous support charge.
static bool batterySupportSourceReady = false;
static bool batterySupportCycleCompleted = false;
static uint8_t batteryObservationInfo = BATTERY_INFO_NORMAL;

static bool externalChargeActive = false;
static uint32_t externalChargeReleaseStart_ms = 0UL;

// Warning latches are independent from the charge logic.
// Once acknowledged, an unchanged active condition stays acknowledged until it
// clears and occurs again.
static uint16_t batteryWarningLatchedFlags = 0U;
static bool batteryLowConditionActive = false;
static bool batteryFastConditionActive = false;
static bool warningAckTiming = false;
static bool warningAckDoneWhileHeld = false;
static uint32_t warningAckStart_ms = 0UL;

void BatteryObservation_Task(uint32_t now_ms);
void BatteryWarning_UpdateConditions(bool fastDischarge, int32_t voltage_mV);
void BatteryWarning_Acknowledge_Task(uint32_t now_ms);
bool BatteryObservation_IsFastDischarge(uint32_t now_ms, int32_t voltage_mV);
bool BatterySupportCharge_IsActive();
int32_t BatterySupportCharge_GetCurrent_mA();
bool SolarShare_IsValid(uint32_t now_ms);
int32_t SolarShare_GetGrantedCurrent_mA(uint32_t now_ms);
bool ExternalCharge_Task(uint32_t now_ms);

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

// Last valid MASTER_REQUEST received from BatterySourceCharger.
// grantedAssistPower_W directly limits normal MID_POWER / SolarShare charging.
MasterRequestPayload receivedMasterRequest = {};
bool receivedMasterRequestValid = false;
uint8_t receivedMasterRequestSequence = 0U;
uint32_t receivedMasterRequestTime_ms = 0UL;
bool communicationResponsePending = false;

static uint8_t communicationReceiveFrame[PROTOCOL_FRAME_LENGTH];
static uint8_t communicationReceiveIndex = 0U;
static uint32_t communicationReceiveLastByte_ms = 0UL;
static uint32_t communicationRxCrcErrors = 0UL;
static uint32_t communicationRxFormatErrors = 0UL;
static uint32_t communicationRxDataErrors = 0UL;
static uint32_t communicationRxTimeoutErrors = 0UL;

// Step 5 communication transmit and receive functions.
void Communication_Task(uint32_t now_ms);
void CommunicationTest_SendFrame(uint16_t messageType, uint8_t sequence, const uint8_t *payload);
void CommunicationTest_WriteU16(uint8_t *buffer, uint8_t *index, uint16_t value);
uint16_t CommunicationTest_UpdateCrc16(uint16_t crc, uint8_t value);
void CommunicationReceive_Task();
bool CommunicationReceive_ValidateFrame(const uint8_t *frame);
bool CommunicationReceive_MasterDataPlausible(const MasterRequestPayload *data);
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

// DCDC output model from measurement:
// DCDC_mV = 16150 - 1.303 * DAC_Code
#define DCDC_DAC_OFFSET_mV         16150L
#define DCDC_DAC_SLOPE_uV_CODE     1303L        // 1.303mV per DAC code

#define DCDC_VOUT_MIN_mV           10500L
#define DCDC_VOUT_MAX_mV           16150L

#define DAC_RAMP_STEP_CODE         2000         // Fast step for open Battery-DCDC switch test.
#define DAC_RAMP_STEP_DELAY_MS     0UL

// ============================================================
// DCDC State Machine Parameters
// ============================================================

#define DCDC_TASK_INTERVAL_MS      100UL
#define DCDC_TARGET_TOLERANCE_mV   50L
#define DCDC_TARGET_OFFSET_mV       0L
#define DCDC_SAFE_START_CODE        DAC_CODE_MAX // Highest DAC code gives lowest DCDC voltage.

// Charge current limits per input source:
// MAIN and SOLAR may use a higher absolute limit for later tests.
// The normal set current is used as the regulation target.
#define MAIN_CHARGE_CURRENT_SET_mA  5000L
#define MAIN_CHARGE_CURRENT_MAX_mA  9000L

#define HELP_CHARGE_CURRENT_SET_mA  2000L
#define HELP_CHARGE_CURRENT_MAX_mA  2000L

#define SOLAR_CHARGE_CURRENT_MIN_mA 0L
#define SOLAR_CHARGE_CURRENT_SET_mA 5000L
#define SOLAR_CHARGE_CURRENT_MAX_mA 9000L

// Input current limits per input source:
// The charge current target is reduced when the measured input current reaches this limit.
#define MAIN_INPUT_CURRENT_MAX_mA    9000L
#define SOLAR_INPUT_CURRENT_MAX_mA   9000L
#define HELP_INPUT_CURRENT_MAX_mA    2000L

// Input current regulation:
// Reduce charge current fast at input overcurrent, increase it slowly when input current has margin again.
#define INPUT_CURRENT_REG_INTERVAL_MS 300UL
#define INPUT_CURRENT_HYST_mA        200L
#define INPUT_CURRENT_STEP_UP_mA     100L
#define INPUT_CURRENT_STEP_DOWN_mA   300L
#define INPUT_CURRENT_FAST_DOWN_mA   1000L

// ============================================================
// Battery Charge End / Float Parameters
// ============================================================
// EnerSys SBS60: 12V, 51Ah, 1.80V/cell end of discharge, float voltage 2.29V/cell.
// Typical car battery note: 12V, 120Ah, similar basic voltage ranges, but larger capacity.
// 12V charging strategy:
// - Below 12.60V a complete 14.20V charge cycle is requested.
// - After the absorption end-current is reached, 13.20V float is maintained
//   while an external source is available.
#define BATTERY_BULK_START_mV        12600L
#define BATTERY_RESTART_mV           12600L
#define BATTERY_FLOAT_mV             13200L
#define BATTERY_HIGH_CURRENT_MIN_mV  11000L
#define BATTERY_HIGH_CURRENT_MAX_mV  13200L
#define BATTERY_ABSORPTION_mV        14200L
#define BATTERY_FLOAT_CURRENT_mA       100L
#define BATTERY_VOLTAGE_TAPER_mV       100L
#define BATTERY_ABSORB_TOLERANCE_mV     50L   // Voltage must be near absorption before full current is accepted.
#define ABS_MIN_TIME_MS              900000UL		// Minimum absorption time: 15 minutes. Fix!Me
#define ABS_MAX_TIME_MS            10800000UL		// Maximum absorption time: 3 hours. Fix!Me

// Assist charge demand from the 12V battery state.
// >12.80V: leave a resting/full battery alone.
// <12.80V: SolarShare may restore/maintain float voltage.
// <=12.60V: request one complete bulk/absorption cycle before returning to float.
#define BATTERY_FLOAT_START_mV          12800L		// Start float maintenance below this resting voltage. Fix!Me
#define BATTERY_FULL_START_mV           12600L		// Start one complete charge cycle at/below this voltage. Fix!Me
#define SOLAR_SHARE_DATA_TIMEOUT_MS      3000UL		// Grant is invalid when Source communication is stale. Fix!Me

#define CHARGE_CURRENT_HYST_mA      10L          // Simple dead band around target current.
#define CHARGE_MAX_OFFSET_mV        1000L        // DCDC may rise only this much above battery voltage.
#define CHARGE_RAMP_STEP_CODE       1            // Lower DAC code raises DCDC voltage.
#define CHARGE_RAMP_INTERVAL_MS     100UL        // Current control interval.
#define CHARGE_POSITIVE_START_MS    1500UL       // After SW-Charge ON, only search in positive direction.

// Current control stabilizing:
// The INA current value is filtered before control.
// This avoids reaction to single noisy samples.
#define CHARGE_CURRENT_FILTER_DIV   4L           // 4 = slow and stable, 2 = faster.
#define CHARGE_OVERCURRENT_mA       250L         // Fast DAC step above target + this value.
#define CHARGE_FAST_STEP_CODE       8            // Fast current reduction step.
#define CHARGE_STABLE_COUNT         5            // Stable samples before READY state.

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
#define INA238_CORR_SOLARDC_PERMILLE  2000L    // Input current correction. Car battery 120Ah: same calibration method.
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

// Start delay sequence:
// 1. Wait before an available input source is switched ON.
// 2. Wait before DCDC is enabled.
// 3. Wait before SW-Charge connects DCDC to the battery.
#define SOURCE_ON_DELAY_MS  1000UL
#define DCDC_ON_DELAY_MS    1000UL
#define CHARGE_ON_DELAY_MS  1000UL

// ============================================================
// DCDC Pre-Charge Self Test - Step 9B.2
// ============================================================
// SW-Charge remains open while INA_DCDC verifies the real converter output.
// Function test and battery connection use separate voltage levels:
// 1. Match the real DCDC output to the battery.
// 2. Raise the open output by 80mV and verify the rise with INA_DCDC.
// 3. Set the DAC to the calculated battery +30mV connection level.
//    The output capacitors are not required to discharge to this level before CHARGE closes.
#define DCDC_TEST_MIN_OUTPUT_mV          10000L
#define DCDC_TEST_RAISE_OFFSET_mV           80L   // Open-output function-test voltage above battery.
#define DCDC_TEST_RISE_MIN_mV               40L   // Minimum measured rise proving DCDC response.
#define DCDC_TEST_MATCH_TOL_mV               20L   // Allowed difference while matching battery voltage.
#define DCDC_TEST_DAC_STEP_CODE               2U   // Fine DAC steps during function test.
#define DCDC_CONNECT_OFFSET_mV                30L   // DAC target above battery when SW-Charge closes.
#define DCDC_CONNECT_SETTLE_MS               300UL  // Let the new DAC command settle; do not wait for COUT discharge.
#define DCDC_TEST_STABLE_COUNT                3U
#define DCDC_TEST_PHASE_TIMEOUT_MS         5000UL
#define DCDC_REVERSE_CURRENT_LIMIT_mA       -50L   // More negative than this starts the reverse-current timer.
#define DCDC_REVERSE_CURRENT_TIME_MS       4000UL		// Reverse current must persist before disconnect.
#define DCDC_REVERSE_RETRY_MS             60000UL		// Retry delay after another charger takes over. Fix!Me
#define DCDC_ERROR_BIT                    0x0001U  // Communication error bit: local DCDC failure.

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

// Internal service state. It is deliberately not part of the communication
// protocol, so the existing protocol state values remain unchanged.
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
	SOLAR_REG_UP,
	SOLAR_REG_LOW_SOLAR,   // 6: SOURCE weak-solar knee search
	SOLAR_REG_LOW_PAUSE    // 7: SOURCE weak-solar retry pause
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
	BATTERY_CHARGE_FLOAT,
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
int32_t InputCurrent_GetMax_mA(uint8_t source);
void BatteryCharge_Task();
bool BatteryCharge_IsAllowed();
const char* BatteryCharge_StateText(BatteryCharge_State state);
bool DCDC_SetStartVoltageFromBattery();
int32_t DCDC_GetStartTargetFromBattery_mV();
int32_t DCDC_GetMaxTargetFromBattery_mV();
int32_t DCDC_GetTargetFromBattery_mV();
bool DCDC_PreChargeTest_Task(uint32_t now_ms);
int32_t DCDC_GetAllowedPower_W();
int32_t DCDC_GetPowerLimitedCurrent_mA();
void TemperatureProtection_Task();
void DCDC_PreChargeTest_Reset();
void DCDC_PreChargeTest_Fail();
bool DCDC_PreChargeTest_AdjustToVoltage(int32_t target_mV);
bool DCDC_ReverseCurrentCheck_Task(uint32_t now_ms);
void DCDC_ReverseCurrentCheck_Reset();
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

#define DCDC_MAX_POWER_W                 350L		// Maximum continuous DC/DC output power. Fix!Me
#define FAN_CHECK_INTERVAL_MS           1000UL		// Temperature supervision interval. Fix!Me
#define FAN_AFTER_RUN_OFF_C                35		// Fan stops only when FET and coil are both below this temperature.
#define DCDC_DERATING_START_C              65		// Full configured power is allowed up to this temperature. Fix!Me
#define DCDC_DERATING_70C_POWER_W         260L		// Allowed power at 70C. Fix!Me
#define DCDC_DERATING_75C_POWER_W         175L		// Allowed power at 75C. Fix!Me
#define DCDC_DERATING_80C_POWER_W         100L		// Allowed power at 80C. Fix!Me
#define DCDC_SHUTDOWN_TEMP_C               85		// Charging stops when either sensor reaches this temperature. Fix!Me
#define DCDC_RESTART_TEMP_C                55		// Restart only after both sensors have cooled below this value. Fix!Me

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
uint32_t lastInputCurrentRegulation_ms = 0; // Input current limit regulation interval.
uint32_t lastFanCheck_ms = 0;            // Fan temperature check interval.
uint32_t lightTimer_ms = 0;              // Display backlight timeout.
bool lastBacklightSetPressed = false;      // Last SET button state for backlight trigger.
uint8_t lastBacklightAdcButton = BUTTON_NONE; // Last ADC button state for backlight trigger.


bool stateSolarDC = false;
bool stateHelpDC = false;
bool stateMainDC = false;
bool stateCharge = false;
bool stateDCDC = false;
bool fanState = false;
bool thermalShutdown = false;
uint32_t dcdcReverseBlockedUntil_ms = 0UL;

int16_t fetTemperature_C = 0;
int16_t coilTemperature_C = 0;

uint16_t dacCode = 0;
uint32_t chargePositiveStart_ms = 0;
int32_t chargeCurrentFiltered_mA = 0;
int32_t activeChargeCurrentSet_mA = MAIN_CHARGE_CURRENT_SET_mA;
int32_t activeChargeCurrentMax_mA = MAIN_CHARGE_CURRENT_MAX_mA;
int32_t inputLimitedChargeCurrentSet_mA = 0L;
uint8_t inputLimitSource = INPUT_SOURCE_NONE;
BatteryCharge_State batteryChargeState = BATTERY_CHARGE_WAIT;
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

// ============================================================
// Display Texts in Flash
// ============================================================

const char txtBattery[] PROGMEM = "BATTERY";
const char txtSolar[]   PROGMEM = "MID";
const char txtMain[]    PROGMEM = "HIGH";
const char txtHelpDC[]  PROGMEM = "LOW";
const char txtCharge[]  PROGMEM = "CHARGE";
const char txtDCDC[]    PROGMEM = "DCDC";
const char txtOn[]      PROGMEM = "ON";
const char txtOff[]     PROGMEM = "OFF";

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
	BatteryWarning_Acknowledge_Task(now_ms);
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

        BatteryObservation_Task(now_ms);

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

// ============================================================
// Battery Charge State Text
// ============================================================

const char* BatteryCharge_StateText(BatteryCharge_State state)
{
	switch (state)
	{
		case BATTERY_CHARGE_WAIT: return "WAIT";
		case BATTERY_CHARGE_BULK: return "BULK";
		case BATTERY_CHARGE_ABS:   return "ABS";
		case BATTERY_CHARGE_FLOAT: return "FLOAT";
		case BATTERY_CHARGE_FULL:  return "FULL";
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

	// DCDC_mV = 16150 - 1.303 * DAC_Code
	// DAC_Code = (16150 - DCDC_mV) / 1.303
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

	// DCDC_mV = 16150 - 1.303 * DAC_Code
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

void Charge_On()            { PORTD |=  MASK_SW_CHARGE; stateCharge = true; }
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
// ADC Button Input
// ============================================================

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
	int32_t batteryCurrent_mA;
	int32_t dcdcCurrent_mA;

	LCD_Clear();

	// Diagnostic display:
	// Left  = raw battery INA (total current at the 12V battery)
	// Right = DCDC INA (current through the ASSIST DCDC path)
	//
	// No control or protocol values are changed by this display-only test.
	batteryCurrent_mA = ina238[CH_BATTERY].current_mA;
	dcdcCurrent_mA = ina238[CH_DCDC].current_mA;

	// Left column: BATTERY INA.
	LCD_Text(LCD_CenterX(DISPLAY_LEFT_X, DISPLAY_COLUMN_W, 3), DISPLAY_LABEL_Y, "BAT");
	LCD_HLine(DISPLAY_LEFT_X, DISPLAY_LEFT_X + DISPLAY_COLUMN_W - 4, DISPLAY_SEPARATOR_Y);
	LCD_PrintBigSignedFixed1(DISPLAY_LEFT_X, DISPLAY_COLUMN_W, DISPLAY_VOLTAGE_Y,
	                        ina238[CH_BATTERY].voltage_mV, 1000L, 'V');

	if (LCD_CurrentFlows(batteryCurrent_mA))
		LCD_PrintBigSignedFixed1(DISPLAY_LEFT_X, DISPLAY_COLUMN_W, DISPLAY_CURRENT_Y,
		                        batteryCurrent_mA, 1000L, 'A');

	// Right column: DCDC INA.
	LCD_Text(LCD_CenterX(DISPLAY_RIGHT_X, DISPLAY_COLUMN_W, 4), DISPLAY_LABEL_Y, "DCDC");
	LCD_PrintBigSignedFixed1(DISPLAY_RIGHT_X, DISPLAY_COLUMN_W, DISPLAY_VOLTAGE_Y,
	                        ina238[CH_DCDC].voltage_mV, 1000L, 'V');

	if (LCD_CurrentFlows(dcdcCurrent_mA))
		LCD_PrintBigSignedFixed1(DISPLAY_RIGHT_X, DISPLAY_COLUMN_W, DISPLAY_CURRENT_Y,
		                        dcdcCurrent_mA, 1000L, 'A');

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
	// Fault and battery warnings have priority over the normal state text.
	if (dcdcPreChargeTestFault)
	{
		LCD_Text(0, DISPLAY_STATE_Y, "DCDC ERROR");
		return;
	}

	if (externalChargeActive)
	{
		LCD_Text(0, DISPLAY_STATE_Y, "EXT CHARGE");
		return;
	}

	if ((batteryWarningLatchedFlags & (BATTERY_WARNING_LOW_BIT | BATTERY_WARNING_FAST_BIT)) ==
	    (BATTERY_WARNING_LOW_BIT | BATTERY_WARNING_FAST_BIT))
	{
		LCD_Text(0, DISPLAY_STATE_Y, "LOW + FAST");
		return;
	}

	if (batteryWarningLatchedFlags & BATTERY_WARNING_LOW_BIT)
	{
		LCD_Text(0, DISPLAY_STATE_Y, "LOW BATTERY");
		return;
	}

	if (batteryWarningLatchedFlags & BATTERY_WARNING_FAST_BIT)
	{
		LCD_Text(0, DISPLAY_STATE_Y, "FAST DISCHARGE");
		return;
	}

	// Last display line: state text only, no label.
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

	if ((dcdcState != DCDC_STATE_OFF) || stateCharge)
		fanState = true;
	else if ((fetTemperature_C <= FAN_AFTER_RUN_OFF_C) && (coilTemperature_C <= FAN_AFTER_RUN_OFF_C))
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
// INA_DCDC current is shown only for debug.

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
	return (ina238[CH_SOLARDC].online && ina238[CH_SOLARDC].voltage_mV >= SOLAR_ON_mV);

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
	DCDC_SetState(DCDC_STATE_OFF);
}

void ChargeCurrent_Task(uint32_t now_ms)
{
	int32_t requestedSet_mA = 0L;

	// HIGH_POWER (Main) always has priority and performs normal 12V charging.
	if (stateMainDC)
	{
		requestedSet_mA = BatteryCharge_SelectCurrent_mA(MAIN_CHARGE_CURRENT_SET_mA, MAIN_CHARGE_CURRENT_MAX_mA);
		activeChargeCurrentSet_mA = InputCurrentLimit_Task(now_ms, INPUT_SOURCE_MAIN, requestedSet_mA);
		activeChargeCurrentMax_mA = MAIN_CHARGE_CURRENT_MAX_mA;
		return;
	}

	// MID_POWER is the 24V battery. Autonomous 8W support is independent of SolarShare grant.
	if (stateSolarDC && BatterySupportCharge_IsActive())
	{
		requestedSet_mA = BatterySupportCharge_GetCurrent_mA();
		activeChargeCurrentSet_mA = InputCurrentLimit_Task(now_ms, INPUT_SOURCE_SOLAR, requestedSet_mA);
		activeChargeCurrentMax_mA = BATTERY_SUPPORT_CURRENT_MAX_mA;
		return;
	}

	// Normal MID_POWER charging may use only the power explicitly granted by Source.
	if (stateSolarDC && SolarShare_IsValid(now_ms))
	{
		requestedSet_mA = SolarShare_GetGrantedCurrent_mA(now_ms);
		activeChargeCurrentSet_mA = InputCurrentLimit_Task(now_ms, INPUT_SOURCE_SOLAR, requestedSet_mA);
		activeChargeCurrentMax_mA = SOLAR_CHARGE_CURRENT_MAX_mA;
		return;
	}

	if (stateHelpDC)
	{
		requestedSet_mA = BatteryCharge_SelectCurrent_mA(HELP_CHARGE_CURRENT_SET_mA, HELP_CHARGE_CURRENT_MAX_mA);
		activeChargeCurrentSet_mA = InputCurrentLimit_Task(now_ms, INPUT_SOURCE_HELP, requestedSet_mA);
		activeChargeCurrentMax_mA = HELP_CHARGE_CURRENT_MAX_mA;
		return;
	}

	activeChargeCurrentSet_mA = 0L;
	activeChargeCurrentMax_mA = 0L;
	inputLimitedChargeCurrentSet_mA = 0L;
	inputLimitSource = INPUT_SOURCE_NONE;
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
	int32_t target_mV = (batteryChargeState == BATTERY_CHARGE_FLOAT) ?
	                    BATTERY_FLOAT_mV : BATTERY_ABSORPTION_mV;
	int32_t battery_mV = ina238[CH_BATTERY].voltage_mV;
	int32_t limited_mA = activeChargeCurrentSet_mA;
	int32_t powerLimited_mA = DCDC_GetPowerLimitedCurrent_mA();
	if (powerLimited_mA < limited_mA) limited_mA = powerLimited_mA;

	// Voltage control is implemented as a current taper near the active target.
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
	int32_t max_mA = activeChargeCurrentMax_mA;
	int32_t powerLimited_mA = DCDC_GetPowerLimitedCurrent_mA();
	if (powerLimited_mA < max_mA) max_mA = powerLimited_mA;
	return max_mA;
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
	static uint32_t absorptionStart_ms = 0UL;
	uint32_t now_ms = millis();
	int32_t battery_mV;
	int32_t batteryCurrent_mA;

	if (!ina238[CH_BATTERY].online)
		return;

	battery_mV = ina238[CH_BATTERY].voltage_mV;
	batteryCurrent_mA = chargeCurrentFiltered_mA;

	// A resting/full battery above 12.8V is left alone.
	if (batteryChargeState == BATTERY_CHARGE_WAIT || batteryChargeState == BATTERY_CHARGE_FULL)
	{
		if (battery_mV <= BATTERY_FULL_START_mV)
		{
			batteryChargeState = BATTERY_CHARGE_BULK;
			absorptionStart_ms = 0UL;
		}
		else if (battery_mV < BATTERY_FLOAT_START_mV)
		{
			batteryChargeState = BATTERY_CHARGE_FLOAT;
		}
		else
		{
			batteryChargeState = BATTERY_CHARGE_FULL;
		}
		return;
	}

	if (batteryChargeState == BATTERY_CHARGE_BULK)
	{
		if (battery_mV >= BATTERY_ABSORPTION_mV)
		{
			batteryChargeState = BATTERY_CHARGE_ABS;
			absorptionStart_ms = now_ms;
		}
		return;
	}

	if (batteryChargeState == BATTERY_CHARGE_ABS)
	{
		uint32_t absTime_ms = (absorptionStart_ms == 0UL) ? 0UL : (uint32_t)(now_ms - absorptionStart_ms);
		if (absTime_ms >= ABS_MAX_TIME_MS)
		{
			batteryChargeState = BATTERY_CHARGE_FLOAT;
			return;
		}
		if ((absTime_ms >= ABS_MIN_TIME_MS) && stateCharge && chargeCurrentFilterValid &&
			(battery_mV >= (BATTERY_ABSORPTION_mV - BATTERY_ABSORB_TOLERANCE_mV)) &&
			(batteryCurrent_mA >= 0L) && (batteryCurrent_mA < BATTERY_FLOAT_CURRENT_mA))
		{
			batteryChargeState = BATTERY_CHARGE_FLOAT;
		}
		return;
	}

	// FLOAT is entered only after the battery has fallen below 12.8V or after a full cycle.
	// The voltage controller then maintains 13.2V while an allowed source is present.
}


bool BatteryCharge_IsAllowed()
{
	uint32_t now_ms = millis();

	if (thermalShutdown || dcdcPreChargeTestFault)
		return false;
	if ((int32_t)(now_ms - dcdcReverseBlockedUntil_ms) < 0)
		return false;

	if (stateMainDC)
		return (batteryChargeState == BATTERY_CHARGE_BULK || batteryChargeState == BATTERY_CHARGE_ABS || batteryChargeState == BATTERY_CHARGE_FLOAT);

	if (stateSolarDC)
	{
		if (BatterySupportCharge_IsActive())
			return true;
		if (!SolarShare_IsValid(now_ms))
			return false;
		return (batteryChargeState == BATTERY_CHARGE_BULK || batteryChargeState == BATTERY_CHARGE_ABS || batteryChargeState == BATTERY_CHARGE_FLOAT);
	}

	if (stateHelpDC)
		return (batteryChargeState == BATTERY_CHARGE_BULK || batteryChargeState == BATTERY_CHARGE_ABS || batteryChargeState == BATTERY_CHARGE_FLOAT);

	return false;
}








bool ExternalCharge_Task(uint32_t now_ms)
{
    if (!ina238[CH_BATTERY].online)
        return externalChargeActive;

    int32_t battery_mV = ina238[CH_BATTERY].voltage_mV;
    int32_t batteryCurrent_mA = ina238[CH_BATTERY].current_mA;

    if (!externalChargeActive &&
        dcdcState == DCDC_STATE_READY &&
        stateCharge &&
        batteryCurrent_mA <= EXTERNAL_CHARGE_REVERSE_mA)
    {
        externalChargeActive = true;
        externalChargeReleaseStart_ms = 0UL;

        Charge_Off();
        DCDC_Disable();
        DCDC_ResetCurrentControl();
        DCDC_PreChargeTest_Reset();
        DCDC_SetState(DCDC_STATE_OFF);

        batteryObservationInfo = BATTERY_INFO_EXTERNAL_CHARGE;
        return true;
    }

    if (!externalChargeActive)
        return false;

    Charge_Off();
    DCDC_Disable();
    DCDC_ResetCurrentControl();
    DCDC_PreChargeTest_Reset();
    DCDC_SetState(DCDC_STATE_OFF);
    batteryObservationInfo = BATTERY_INFO_EXTERNAL_CHARGE;

    if (battery_mV <= EXTERNAL_CHARGE_RELEASE_mV)
    {
        if (externalChargeReleaseStart_ms == 0UL)
            externalChargeReleaseStart_ms = now_ms;
        else if ((uint32_t)(now_ms - externalChargeReleaseStart_ms) >= EXTERNAL_CHARGE_RELEASE_TIME_MS)
        {
            externalChargeActive = false;
            externalChargeReleaseStart_ms = 0UL;
            batteryObservationInfo = BATTERY_INFO_NORMAL;
            return false;
        }
    }
    else
    {
        externalChargeReleaseStart_ms = 0UL;
    }

    return true;
}

void DCDC_Task(uint32_t now_ms)
{
	if ((uint32_t)(now_ms - lastDcdcTask_ms) < DCDC_TASK_INTERVAL_MS)
	return;

	lastDcdcTask_ms = now_ms;

	// Decide first whether the SOURCE currently has enough real solar power for
	// normal 12V charging. Support charging remains independent of this gate.

	ChargeCurrent_Task(now_ms);
	BatteryCharge_Task();

    // The autonomous support cycle is independent of the normal battery
    // charge-state machine. During support charging the 8W current target
    // remains active until the 12V battery reaches the support stop voltage.
    bool supportChargeActive = BatterySupportCharge_IsActive();

    if (ExternalCharge_Task(now_ms))
        return;

	// Source removed: disconnect the battery and switch off DCDC.
	// Removing the source also clears a latched DCDC pre-charge test fault,
	// so the next source connection gets one new test attempt.
	if (!InputSource_Available())
	{
		dcdcPreChargeTestFault = false;
		DCDC_AllOff();
		return;
	}

	// A failed DCDC test is latched while the input source remains present.
	// Never connect the battery to a DCDC that did not pass the real INA test.
	if (dcdcPreChargeTestFault)
	{
		Charge_Off();
		DCDC_Disable();
		DCDC_ResetCurrentControl();
		DCDC_SetState(DCDC_STATE_OFF);
		return;
	}

    // A requested support charge may only use the 24V source while its
    // voltage is inside the protected range.
    if (stateSolarDC && batterySupportRequestActive && !batterySupportSourceReady)
    {
        Charge_Off();
        DCDC_Disable();
        DCDC_ResetCurrentControl();
        DCDC_PreChargeTest_Reset();
        DCDC_SetState(DCDC_STATE_OFF);
        return;
    }

	// Battery full or not low enough: keep the input selected, but keep DCDC and Charge OFF.
    // Do not apply the normal charge-state gate to autonomous support charging.
	if (!supportChargeActive && !BatteryCharge_IsAllowed())
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
		// The following pre-charge test uses INA_DCDC for the real voltage.
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
		// Step 9B.2: SW-Charge stays OFF until function test and connect-level setup passed.
		Charge_Off();

		if (!DCDC_PreChargeTest_Task(now_ms))
		return;

		// The DCDC function test passed and the DAC is now commanded to the small
		// positive connection level. Only now connect the battery.
		Charge_On();
		chargePositiveStart_ms = now_ms;
		DCDC_ReverseCurrentCheck_Reset();
		DCDC_SetState(DCDC_STATE_RAMP_CURRENT);
		return;
	}

	if (dcdcState == DCDC_STATE_RAMP_CURRENT)
	{
		// After SW-Charge closes, allow the normal slow current ramp to work.
		// A continuously negative battery current indicates that the DCDC does
		// not take over the battery voltage and may be accepting reverse current.
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

// ============================================================
// DCDC Pre-Charge Self Test - Step 9A
// ============================================================

int32_t DCDC_GetAllowedPower_W()
{
	int16_t hottest_C = (fetTemperature_C > coilTemperature_C) ? fetTemperature_C : coilTemperature_C;
	if (hottest_C >= DCDC_SHUTDOWN_TEMP_C) return 0L;
	if (hottest_C >= 80) return DCDC_DERATING_80C_POWER_W;
	if (hottest_C >= 75) return DCDC_DERATING_75C_POWER_W - ((int32_t)(hottest_C - 75) * (DCDC_DERATING_75C_POWER_W - DCDC_DERATING_80C_POWER_W) / 5L);
	if (hottest_C >= 70) return DCDC_DERATING_70C_POWER_W - ((int32_t)(hottest_C - 70) * (DCDC_DERATING_70C_POWER_W - DCDC_DERATING_75C_POWER_W) / 5L);
	if (hottest_C >= DCDC_DERATING_START_C) return DCDC_MAX_POWER_W - ((int32_t)(hottest_C - DCDC_DERATING_START_C) * (DCDC_MAX_POWER_W - DCDC_DERATING_70C_POWER_W) / 5L);
	return DCDC_MAX_POWER_W;
}

int32_t DCDC_GetPowerLimitedCurrent_mA()
{
	int32_t battery_mV = ina238[CH_BATTERY].voltage_mV;
	int32_t allowedPower_W = DCDC_GetAllowedPower_W();
	if (battery_mV <= 0L || allowedPower_W <= 0L) return 0L;
	return (int32_t)(((int64_t)allowedPower_W * 1000000LL) / battery_mV);
}

void TemperatureProtection_Task()
{
	if (!thermalShutdown)
	{
		if ((fetTemperature_C >= DCDC_SHUTDOWN_TEMP_C) || (coilTemperature_C >= DCDC_SHUTDOWN_TEMP_C))
		{
			thermalShutdown = true;
			DCDC_AllOff();
		}
	}
	else if ((fetTemperature_C <= DCDC_RESTART_TEMP_C) && (coilTemperature_C <= DCDC_RESTART_TEMP_C))
	{
		thermalShutdown = false;
	}
}

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

	// A healthy 12V DCDC must not remain below 10V after it has been enabled.
	// Give the converter a short settling time before evaluating this limit.
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

		// Function test passed. Command the DAC directly to the small positive
		// connection level. COUT has no discharge load, so do not wait for the real
		// open-circuit output voltage to fall to this value.
		testTarget_mV = battery_mV + DCDC_CONNECT_OFFSET_mV;
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
		// The DAC is already set to battery +30mV. Allow a short settling time for
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
		DCDC_ReverseCurrentCheck_Reset();

	return true;
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

	// Absolute safety correction.
	// This parameter is user adjustable, but the software reacts immediately here.
	if ((iBatRaw_mA > ChargeCurrent_GetMax_mA()) || (chargeCurrentFiltered_mA > ChargeCurrent_GetMax_mA()))
	{
		if (dacCode < (DAC_CODE_MAX - CHARGE_FAST_STEP_CODE))
		DAC_WriteCode(dacCode + CHARGE_FAST_STEP_CODE);
		else
		DAC_WriteCode(DAC_CODE_MAX);

		chargeStableCounter = 0;
		return false;
	}

	// Fast safety correction for large overcurrent peaks.
	if (iBatRaw_mA > (ChargeCurrent_GetSet_mA() + CHARGE_OVERCURRENT_mA))
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
			if (dacCode > (minCode + CHARGE_RAMP_STEP_CODE))
			DAC_WriteCode(dacCode - CHARGE_RAMP_STEP_CODE);
			else
			DAC_WriteCode(minCode);
		}

		chargeStableCounter = 0;
		return false;
	}

	// Slow upward ramp when current is too low.
	if (chargeCurrentFiltered_mA < (ChargeCurrent_GetSet_mA() - CHARGE_CURRENT_HYST_mA))
	{
		if (dacCode > (minCode + CHARGE_RAMP_STEP_CODE))
		DAC_WriteCode(dacCode - CHARGE_RAMP_STEP_CODE);
		else
		DAC_WriteCode(minCode);

		chargeStableCounter = 0;
		return false;
	}

	// Moderate downward correction when filtered current is too high.
	if (chargeCurrentFiltered_mA > (ChargeCurrent_GetSet_mA() + CHARGE_CURRENT_HYST_mA))
	{
		if (dacCode < DAC_CODE_MAX)
		DAC_WriteCode(dacCode + CHARGE_RAMP_STEP_CODE);

		chargeStableCounter = 0;
		return false;
	}

	// Current is inside the dead band. READY is accepted only after several stable samples.
	if (chargeStableCounter < CHARGE_STABLE_COUNT)
	chargeStableCounter++;

	return (chargeStableCounter >= CHARGE_STABLE_COUNT);
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
	return ina238[CH_BATTERY].voltage_mV;
}

int32_t DCDC_GetMaxTargetFromBattery_mV()
{
	int32_t target_mV;

	target_mV = ina238[CH_BATTERY].voltage_mV + CHARGE_MAX_OFFSET_mV;

	if (target_mV < DCDC_VOUT_MIN_mV)
	target_mV = DCDC_VOUT_MIN_mV;

	// Select the voltage ceiling from the active charge state.
	int32_t chargeTarget_mV = (batteryChargeState == BATTERY_CHARGE_FLOAT) ?
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
// Charger Communication Protocol - Step 6: payload plausibility validation
// ============================================================
// Valid Source data controls the MID_POWER SolarShare grant; invalid/stale data cannot enable normal MID_POWER charging.

void CommunicationReceive_Task()
{
    uint32_t now_ms = millis();
    if (communicationReceiveIndex > 0U && (uint32_t)(now_ms - communicationReceiveLastByte_ms) > COMMUNICATION_RX_TIMEOUT_MS)
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
            {
                communicationReceiveFrame[communicationReceiveIndex++] = value;
            }
            continue;
        }

        if (communicationReceiveIndex == 1U)
        {
            if (value == PROTOCOL_START_BYTE_2)
            {
                communicationReceiveFrame[communicationReceiveIndex++] = value;
            }
            else if (value == PROTOCOL_START_BYTE_1)
            {
                communicationReceiveFrame[0] = value;
            }
            else
            {
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
        return false;
    }

    uint16_t messageType = ((uint16_t)frame[3] << 8) | frame[4];
    if (messageType != MESSAGE_MASTER_REQUEST)
    {
        return false;
    }

    if (frame[6] != PROTOCOL_PAYLOAD_LENGTH)
    {
        return false;
    }

    uint16_t calculatedCrc = 0xFFFFU;
    for (uint8_t i = 2U; i < (PROTOCOL_FRAME_LENGTH - 2U); i++)
    {
        calculatedCrc = CommunicationTest_UpdateCrc16(calculatedCrc, frame[i]);
    }

    uint16_t receivedCrc = ((uint16_t)frame[PROTOCOL_FRAME_LENGTH - 2U] << 8) |
                           frame[PROTOCOL_FRAME_LENGTH - 1U];

    if (calculatedCrc != receivedCrc)
    {
        communicationRxCrcErrors++;
        return false;
    }

    MasterRequestPayload decoded = {};
    uint8_t valueIndex = 7U;
    decoded.batteryVoltage_10mV = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.batteryCurrent_10mA = (int16_t)CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.sourcePower_W = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.safeSourcePower_W = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.grantedAssistPower_W = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.batterySoc_percent = frame[valueIndex++];
    decoded.sourceMode = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.mpptState = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.dcdcState = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.warningFlags = CommunicationReceive_ReadU16(frame, &valueIndex);
    decoded.errorFlags = CommunicationReceive_ReadU16(frame, &valueIndex);

    if (valueIndex != PROTOCOL_FRAME_LENGTH - 2U ||
        !CommunicationReceive_MasterDataPlausible(&decoded))
    {
        communicationRxDataErrors++;
        return false;
    }

    receivedMasterRequest = decoded;
    receivedMasterRequestSequence = frame[5];
    receivedMasterRequestTime_ms = millis();
    receivedMasterRequestValid = true;
    communicationResponsePending = true;

    return true;
}

bool CommunicationReceive_MasterDataPlausible(const MasterRequestPayload *data)
{
    if (data->batteryVoltage_10mV > 3600U) return false;       // Maximum 36.00V.
    if (data->batteryCurrent_10mA < -5000 || data->batteryCurrent_10mA > 5000) return false;
    if (data->sourcePower_W > 1500U) return false;
    if (data->safeSourcePower_W > 1500U) return false;
    if (data->grantedAssistPower_W > 500U) return false;
    if (data->batterySoc_percent > 100U) return false;
    if (data->sourceMode > INPUT_SOURCE_HELP) return false;
    if (data->mpptState > SOLAR_REG_LOW_PAUSE) return false;
    if (data->dcdcState > DCDC_STATE_READY) return false;
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
}

// ============================================================
// Normal Solar Day/Night Charge Gate - Step 9D
// ============================================================





// ============================================================
// 12V Battery Observation - Step 5
// ============================================================

void BatteryObservation_Task(uint32_t now_ms)
{
    if (!ina238[CH_BATTERY].online)
    {
        batteryObservationInfo = BATTERY_INFO_NORMAL;
        batterySupportRequestActive = false;
        batterySupportSourceReady = false;
        batteryLowConditionActive = false;
        batteryFastConditionActive = false;
        return;
    }

    int32_t voltage_mV = ina238[CH_BATTERY].voltage_mV;
    int32_t current_mA = ina238[CH_BATTERY].current_mA;
    int32_t sourceVoltage_mV = ina238[CH_SOLARDC].voltage_mV;

    // Protect the 24V source with start/stop hysteresis.
    if (batterySupportSourceReady)
    {
        if (!ina238[CH_SOLARDC].online || sourceVoltage_mV <= BATTERY_SUPPORT_SOURCE_STOP_mV)
        {
            batterySupportSourceReady = false;
        }
    }
    else if (ina238[CH_SOLARDC].online && sourceVoltage_mV >= BATTERY_SUPPORT_SOURCE_START_mV)
    {
        batterySupportSourceReady = true;
    }

    // Existing charger activity excludes the battery from night observation.
    bool chargerActive = stateCharge || stateDCDC || (current_mA > BATTERY_NO_CHARGE_CURRENT_MAX_mA);
    bool loadActive = (!chargerActive && current_mA < BATTERY_LOAD_CURRENT_mA);

    if (loadActive)
    {
        batteryLoadWasActive = true;
        batteryObservationInfo = BATTERY_INFO_LOAD_ACTIVE;
    }
    else if (batteryLoadWasActive)
    {
        batteryLoadWasActive = false;
        batteryLoadEnded_ms = now_ms;
    }

    bool recoveryActive = (batteryLoadEnded_ms != 0UL) &&
                          ((uint32_t)(now_ms - batteryLoadEnded_ms) < BATTERY_RECOVERY_TIME_MS);

    if (!chargerActive &&
        ((batteryHistoryEntries == 0U) ||
         ((uint32_t)(now_ms - batteryHistoryLastSample_ms) >= BATTERY_HISTORY_INTERVAL_MS)))
    {
        batteryHistoryLastSample_ms = now_ms;
        batteryHistory[batteryHistoryWriteIndex].time_ms = now_ms;
        batteryHistory[batteryHistoryWriteIndex].voltage_10mV = (int16_t)(voltage_mV / 10L);
        batteryHistory[batteryHistoryWriteIndex].current_10mA = (int16_t)(current_mA / 10L);

        batteryHistoryWriteIndex++;
        if (batteryHistoryWriteIndex >= BATTERY_HISTORY_COUNT)
        {
            batteryHistoryWriteIndex = 0U;
        }
        if (batteryHistoryEntries < BATTERY_HISTORY_COUNT)
        {
            batteryHistoryEntries++;
        }
    }

    bool fastDischarge = false;
    if (!chargerActive && !loadActive && !recoveryActive)
    {
        fastDischarge = BatteryObservation_IsFastDischarge(now_ms, voltage_mV);
    }

    // Update only the user-visible warning latches. This does not influence
    // batterySupportRequestActive or any charging decision.
    BatteryWarning_UpdateConditions(fastDischarge, voltage_mV);

    // Hysteresis keeps the autonomous support cycle active until 12.4V.
    if (batterySupportRequestActive)
    {
        if (voltage_mV >= BATTERY_SUPPORT_STOP_mV)
        {
            batterySupportRequestActive = false;
            batterySupportCycleCompleted = true;
        }
    }
    else
    {
        // A critically low battery must request support independently of the
        // present DCDC and charge-switch states. Those states become active
        // during the support cycle and must not cancel or block the request.
        if (voltage_mV <= BATTERY_SUPPORT_START_mV)
        {
            batterySupportRequestActive = true;
            batterySupportCycleCompleted = false;
        }
        else if (fastDischarge && voltage_mV < BATTERY_LOW_WARNING_mV)
        {
            batterySupportRequestActive = true;
            batterySupportCycleCompleted = false;
        }
    }

    if (batterySupportRequestActive)
    {
        if (BatterySupportCharge_IsActive())
        {
            batteryObservationInfo = BATTERY_INFO_SUPPORT_REQUEST;
        }
        else
        {
            batteryObservationInfo = fastDischarge ? BATTERY_INFO_CHECK_BATTERY : BATTERY_INFO_LOW;
        }
    }
    else if (fastDischarge)
    {
        batteryObservationInfo = BATTERY_INFO_FAST_DISCHARGE;
    }
    else if (loadActive)
    {
        batteryObservationInfo = BATTERY_INFO_LOAD_ACTIVE;
    }
    else if (recoveryActive && voltage_mV < BATTERY_LOW_WARNING_mV)
    {
        batteryObservationInfo = BATTERY_INFO_RECOVERY;
    }
    else if (!chargerActive && voltage_mV < BATTERY_LOW_WARNING_mV)
    {
        batteryObservationInfo = BATTERY_INFO_LOW;
    }
    else
    {
        batteryObservationInfo = BATTERY_INFO_NORMAL;
    }
}

void BatteryWarning_UpdateConditions(bool fastDischarge, int32_t voltage_mV)
{
    // LOW BATTERY uses hysteresis for re-arming. Acknowledging while still low
    // therefore does not make the warning appear again every 100ms.
    bool lowNow = batteryLowConditionActive;

    if (batteryLowConditionActive)
    {
        if (voltage_mV >= BATTERY_LOW_REARM_mV)
            lowNow = false;
    }
    else if (voltage_mV <= BATTERY_LOW_WARNING_mV)
    {
        lowNow = true;
    }

    if (lowNow && !batteryLowConditionActive)
        batteryWarningLatchedFlags |= BATTERY_WARNING_LOW_BIT;

    batteryLowConditionActive = lowNow;

    // FAST DISCHARGE is latched on a new occurrence. If it is acknowledged
    // while still active, it stays acknowledged until the condition clears
    // once and is detected again later.
    if (fastDischarge && !batteryFastConditionActive)
        batteryWarningLatchedFlags |= BATTERY_WARNING_FAST_BIT;

    batteryFastConditionActive = fastDischarge;
}

void BatteryWarning_Acknowledge_Task(uint32_t now_ms)
{
    uint8_t adcButton = ButtonADC_Read();
    bool okPressed = (adcButton == BUTTON_OK);

    if (!okPressed)
    {
        warningAckTiming = false;
        warningAckDoneWhileHeld = false;
        warningAckStart_ms = 0UL;
        return;
    }

    if (!warningAckTiming)
    {
        warningAckTiming = true;
        warningAckStart_ms = now_ms;
        return;
    }

    if (!warningAckDoneWhileHeld &&
        (uint32_t)(now_ms - warningAckStart_ms) >= WARNING_ACK_HOLD_MS)
    {
        // ACK means only "I have seen the warning". Do not touch support
        // charging, battery observation or the latched DCDC hardware fault.
        batteryWarningLatchedFlags = 0U;
        warningAckDoneWhileHeld = true;
        DisplayBacklight_ResetTimer(now_ms);
    }
}

bool BatterySupportCharge_IsActive()
{
    // A valid support request and a healthy 24V source are sufficient.
    // Source selection and DCDC enable remain protected by the existing
    // InputSource_Task() and InputSource_Available() logic.
    return batterySupportRequestActive && batterySupportSourceReady;
}

int32_t BatterySupportCharge_GetCurrent_mA()
{
    int32_t batteryVoltage_mV = ina238[CH_BATTERY].voltage_mV;
    if (batteryVoltage_mV <= 0L)
    {
        return 0L;
    }

    int32_t current_mA = (int32_t)(((int64_t)BATTERY_SUPPORT_POWER_W * 1000000LL) / batteryVoltage_mV);
    if (current_mA > BATTERY_SUPPORT_CURRENT_MAX_mA)
    {
        current_mA = BATTERY_SUPPORT_CURRENT_MAX_mA;
    }
    if (current_mA < 0L)
    {
        current_mA = 0L;
    }
    return current_mA;
}

bool SolarShare_IsValid(uint32_t now_ms)
{
	if (!receivedMasterRequestValid)
		return false;
	if ((uint32_t)(now_ms - receivedMasterRequestTime_ms) > SOLAR_SHARE_DATA_TIMEOUT_MS)
		return false;
	if (receivedMasterRequest.errorFlags != 0U)
		return false;
	return (receivedMasterRequest.grantedAssistPower_W > 0U);
}

int32_t SolarShare_GetGrantedCurrent_mA(uint32_t now_ms)
{
	if (!SolarShare_IsValid(now_ms))
		return 0L;
	int32_t battery_mV = ina238[CH_BATTERY].voltage_mV;
	if (battery_mV <= 0L)
		return 0L;
	int32_t current_mA = (int32_t)(((int64_t)receivedMasterRequest.grantedAssistPower_W * 1000000LL) / battery_mV);
	if (current_mA > SOLAR_CHARGE_CURRENT_MAX_mA) current_mA = SOLAR_CHARGE_CURRENT_MAX_mA;
	if (current_mA < 0L) current_mA = 0L;
	return current_mA;
}

bool BatteryObservation_IsFastDischarge(uint32_t now_ms, int32_t voltage_mV)
{
    // Ignore normal voltage relaxation after charging and short high-voltage
    // drops. FAST_DISCHARGE becomes diagnostically relevant only once the
    // battery has entered the normal discharge range.
    if (voltage_mV > BATTERY_FAST_DETECT_MAX_mV)
    {
        return false;
    }

    if (batteryHistoryEntries < 2U)
    {
        return false;
    }

    bool referenceFound = false;
    int32_t referenceVoltage_mV = voltage_mV;
    for (uint8_t n = 0U; n < batteryHistoryEntries; n++)
    {
        int16_t index = (int16_t)batteryHistoryWriteIndex - 1 - (int16_t)n;
        if (index < 0)
        {
            index += BATTERY_HISTORY_COUNT;
        }

        const BatteryHistoryEntry &entry = batteryHistory[(uint8_t)index];
        if ((uint32_t)(now_ms - entry.time_ms) > BATTERY_FAST_DROP_WINDOW_MS)
        {
            break;
        }

        // Ignore samples that clearly show an active external load.
        if (entry.current_10mA >= (BATTERY_LOAD_CURRENT_mA / 10L))
        {
            referenceVoltage_mV = (int32_t)entry.voltage_10mV * 10L;
            referenceFound = true;
        }
    }

    if (!referenceFound)
    {
        return false;
    }

    return (referenceVoltage_mV - voltage_mV) >= BATTERY_FAST_DROP_mV;
}

void Communication_Task(uint32_t now_ms)
{
    if (!receivedMasterRequestValid || !communicationResponsePending)
        return;
    if ((uint32_t)(now_ms - receivedMasterRequestTime_ms) > COMMUNICATION_LINK_TIMEOUT_MS)
    {
        communicationResponsePending = false;
        return;
    }

    // Reply once to every valid MASTER_REQUEST and echo its sequence number.
    communicationResponsePending = false;

    int32_t batteryVoltage_10mV = ina238[CH_BATTERY].voltage_mV / 10L;
    int32_t batteryCurrent_10mA = ina238[CH_BATTERY].current_mA / 10L;
    int32_t requestedPower_W = 0L;
    int32_t actualPower_W = 0L;

    // The Assist decides the autonomous support charge locally.
    // Therefore no power request is sent to the Source charger.
    requestedPower_W = 0L;

    if (ina238[CH_BATTERY].voltage_mV > 0L && ina238[CH_BATTERY].current_mA > 0L)
    {
        actualPower_W = (int32_t)(((int64_t)ina238[CH_BATTERY].voltage_mV * ina238[CH_BATTERY].current_mA) / 1000000LL);
    }

    if (batteryVoltage_10mV < 0L) batteryVoltage_10mV = 0L;
    if (batteryVoltage_10mV > 65535L) batteryVoltage_10mV = 65535L;
    if (batteryCurrent_10mA < -32768L) batteryCurrent_10mA = -32768L;
    if (batteryCurrent_10mA > 32767L) batteryCurrent_10mA = 32767L;
    if (requestedPower_W < 0L) requestedPower_W = 0L;
    if (requestedPower_W > 65535L) requestedPower_W = 65535L;
    if (actualPower_W < 0L) actualPower_W = 0L;
    if (actualPower_W > 65535L) actualPower_W = 65535L;

    uint8_t payload[PROTOCOL_PAYLOAD_LENGTH];
    uint8_t index = 0U;

    CommunicationTest_WriteU16(payload, &index, (uint16_t)batteryVoltage_10mV);
    CommunicationTest_WriteU16(payload, &index, (uint16_t)(int16_t)batteryCurrent_10mA);
    CommunicationTest_WriteU16(payload, &index, (uint16_t)requestedPower_W);
    CommunicationTest_WriteU16(payload, &index, (uint16_t)actualPower_W);
    payload[index++] = 0U;                                                   // SOC not calculated yet.
    CommunicationTest_WriteU16(payload, &index, (uint16_t)batteryChargeState);
    CommunicationTest_WriteU16(payload, &index, (uint16_t)dcdcState);
    CommunicationTest_WriteU16(payload, &index, ASSIST_BATTERY_CAPACITY_Ah);
    uint16_t communicationInfoCode = externalChargeActive ? BATTERY_INFO_EXTERNAL_CHARGE : batteryObservationInfo;
    CommunicationTest_WriteU16(payload, &index, communicationInfoCode); // Battery observation INFO code.
    // Warning flags are the acknowledged/user-visible latches.
    // Bit 0 = LOW BATTERY, Bit 1 = FAST DISCHARGE.
    uint16_t warningFlags = batteryWarningLatchedFlags;
    if (DCDC_GetAllowedPower_W() < DCDC_MAX_POWER_W && !thermalShutdown) warningFlags |= ASSIST_WARNING_THERMAL_DERATING;
    CommunicationTest_WriteU16(payload, &index, warningFlags);

    uint16_t errorFlags = 0U;
    if (dcdcPreChargeTestFault)
    {
        errorFlags |= ASSIST_ERROR_DCDC_PRECHARGE;
    }
    if (thermalShutdown) errorFlags |= ASSIST_ERROR_OVERTEMPERATURE;
    CommunicationTest_WriteU16(payload, &index, errorFlags);

    CommunicationTest_SendFrame(MESSAGE_CLIENT_RESPONSE, receivedMasterRequestSequence, payload);
}

void CommunicationTest_SendFrame(uint16_t messageType, uint8_t sequence, const uint8_t *payload)
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
        crc = CommunicationTest_UpdateCrc16(crc, frame[i]);
    }

    for (uint8_t i = 0U; i < PROTOCOL_PAYLOAD_LENGTH; i++)
    {
        frame[index++] = payload[i];
        crc = CommunicationTest_UpdateCrc16(crc, payload[i]);
    }

    frame[index++] = (uint8_t)(crc >> 8);
    frame[index++] = (uint8_t)crc;

    Serial.write(frame, PROTOCOL_FRAME_LENGTH);
}

void CommunicationTest_WriteU16(uint8_t *buffer, uint8_t *index, uint16_t value)
{
    buffer[(*index)++] = (uint8_t)(value >> 8);
    buffer[(*index)++] = (uint8_t)value;
}

uint16_t CommunicationTest_UpdateCrc16(uint16_t crc, uint8_t value)
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

