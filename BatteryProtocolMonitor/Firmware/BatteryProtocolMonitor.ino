/*
	Name:       BatteryProtocolMonitor.ino
	Author:     Andreas
	
    Board:      NUCLEO-F103RB

    Connections:
        Serial1 RX = PA10
        Serial3 RX = PB11
        Serial2 TX = ST-LINK virtual COM port

    Important:
        Serial1 and Serial3 are treated equally.
        The received MessageType determines whether a frame is:
            0x0001 = MASTER_REQUEST  (SOLAR)
            0x0002 = CLIENT_RESPONSE (ASSIST)

        Therefore SOLAR and ASSIST may be connected to either input.

    Protocol frame:
        A5 5A Version TypeHigh TypeLow Sequence Length Payload... CrcHigh CrcLow

    CRC:
        CRC16-CCITT
        Polynomial: 0x1021
        Initial value: 0xFFFF
        Calculated from Version through the last payload byte.
        Header bytes A5 5A are not included.

    Frame length:
        Total bytes = 9 + PayloadLength

    Output:
        Fixed-width line for terminal display.
        One line is printed when:
            - MASTER and CLIENT with the same sequence were received, or
            - no matching CLIENT was received before PAIR_TIMEOUT_MS.

    Reject diagnostics:
        CRC-valid but unsupported or implausible frames, CRC errors and frame
        timeouts are buffered and written to daily REJECT.CSV together with
        UART number, parser state, reason and the received raw bytes.
*/

#include <Arduino.h>
#include <RTClock.h>
#include <SPI.h>
#include <SD.h>

// Roger Clark / Arduino_STM32 (libmaple) RTC library.
// The NUCLEO-F103RB uses the external 32.768kHz crystal (LSE).
RTClock rtc(RTCSEL_LSE);

// ============================================================
// Parameters
// ============================================================

#define CHARGER_BAUDRATE             115200UL
#define OUTPUT_BAUDRATE              115200UL

#define START_BYTE_1                 0xA5U
#define START_BYTE_2                 0x5AU
#define PROTOCOL_VERSION             0x01U

#define MESSAGE_MASTER_REQUEST       0x0001U
#define MESSAGE_CLIENT_RESPONSE      0x0002U

#define PROTOCOL_PAYLOAD_LENGTH      21U
#define PROTOCOL_FRAME_LENGTH        30U
#define MAX_PAYLOAD_LENGTH           PROTOCOL_PAYLOAD_LENGTH
#define RAW_FRAME_BUFFER_SIZE        32U		// Complete 30-byte frame plus diagnostic reserve
#define REJECT_QUEUE_SIZE             8U		// Buffered reject records before SD write
#define REJECT_REASON_LENGTH        192U		// Diagnostic text for implausible fields

#define FRAME_TIMEOUT_MS             20UL
#define PAIR_TIMEOUT_MS              300UL
#define HEADER_REPEAT_LINES          100UL
#define DEVICE_ERROR_DCDC_BIT         0x0001U
#define DEVICE_WARNING_LOW_BIT       0x0001U
#define DEVICE_WARNING_FAST_BIT      0x0002U

// ============================================================
// RTC - Step 10A
// ============================================================
// NUCLEO-F103RB: use the onboard 32.768kHz LSE crystal.
// If the RTC contains no plausible date, initialize it once from the PC compile time.
// With VBAT connected, later resets/uploads keep the existing RTC counter.
#define RTC_USE_COMPILE_TIME_IF_UNSET  1
#define RTC_VALID_EPOCH_MIN            1735689600UL  // 2025-01-01 00:00:00 UTC-like epoch

// ============================================================
// SD Card - Step 10B.1: initialization only
// ============================================================
// SPI1: PA5=SCK, PA6=MISO, PA7=MOSI
// CS is wired to PB6 (Arduino D10 on this setup).
#define SD_CS_PIN                      PB6

#define SD_TEST_DIR                  "/TEST"
#define SD_TEST_FILE                 "/TEST/TEST.TXT"
#define SD_TEST_SIGNATURE            "Battery Protocol Monitor SD Test"

// ============================================================
// Logger - Step 10D
// ============================================================
// One row is written for every monitor output pair (normally about 1 s).
// EVENT.CSV contains only abnormal conditions and their direct protection response.
#define LOG_EVENT_LOW_BIT              0x0001U
#define LOG_EVENT_FAST_BIT             0x0002U

static bool sdLoggerReady = false;
static uint16_t logCurrentYear = 0U;
static uint8_t logCurrentMonth = 0U;
static uint8_t logCurrentDay = 0U;
static uint8_t logCurrentHour = 255U;

static bool logPreviousPairComplete = true;
static bool logPairStateInitialized = false;
static uint16_t logPreviousSourceWarnings = 0U;
static uint16_t logPreviousSourceErrors = 0U;
static uint16_t logPreviousAssistWarnings = 0U;
static uint16_t logPreviousAssistErrors = 0U;
static uint16_t logPreviousAssistInfo = 0U;
static bool logEventStateInitialized = false;
static uint32_t logPreviousCrcErrors = 0UL;
static uint32_t logPreviousFormatErrors = 0UL;
static uint32_t logPreviousDataErrors = 0UL;
static uint32_t logPreviousTimeoutErrors = 0UL;

// ============================================================
// Parser states
// ============================================================

enum ParserState
{
    WAIT_START_1,
    WAIT_START_2,
    READ_VERSION,
    READ_TYPE_HIGH,
    READ_TYPE_LOW,
    READ_SEQUENCE,
    READ_LENGTH,
    READ_PAYLOAD,
    READ_CRC_HIGH,
    READ_CRC_LOW
};

// ============================================================
// Reject diagnostics
// ============================================================

enum RejectType
{
    REJECT_NONE,
    REJECT_VERSION,
    REJECT_MESSAGE_TYPE,
    REJECT_LENGTH,
    REJECT_CRC,
    REJECT_DATA,
    REJECT_TIMEOUT
};

// ============================================================
// Protocol structures
// ============================================================

struct ProtocolFrame
{
    uint8_t version;
    uint16_t messageType;
    uint8_t sequence;
    uint8_t payloadLength;
    uint8_t payload[MAX_PAYLOAD_LENGTH];
    uint16_t receivedCrc;
};

struct Parser
{
    ParserState state;
    ProtocolFrame frame;

    uint8_t payloadIndex;
    uint16_t calculatedCrc;
    uint32_t frameStartTime_ms;
    uint32_t lastByteTime_ms;

    uint8_t rawBuffer[RAW_FRAME_BUFFER_SIZE];
    uint8_t rawLength;

    uint32_t byteCount;
    uint32_t validFrameCount;
    uint32_t crcErrorCount;
    uint32_t formatErrorCount;
    uint32_t dataErrorCount;
    uint32_t timeoutErrorCount;
};

struct RejectRecord
{
    RejectType type;
    uint8_t uartNumber;
    ParserState parserState;
    uint8_t version;
    uint16_t messageType;
    uint8_t sequence;
    uint8_t payloadLength;
    uint16_t receivedCrc;
    uint16_t calculatedCrc;
    uint32_t eventTime_ms;
    uint8_t rawLength;
    uint8_t rawBuffer[RAW_FRAME_BUFFER_SIZE];
    char reason[REJECT_REASON_LENGTH];
};

struct SequenceStatistics
{
    uint8_t lastSequence;
    bool initialized;
    uint32_t lostFrameCount;
    uint32_t duplicateFrameCount;
    uint32_t reverseFrameCount;
};

// ============================================================
// Decoded data
// ============================================================

struct MasterData
{
    uint16_t voltage_10mV;
    int16_t current_10mA;
    uint16_t sourcePower_W;
    uint16_t safePower_W;
    uint16_t grantPower_W;
    uint8_t soc_percent;
    uint16_t sourceMode;
    uint16_t mpptState;
    uint16_t powerManagerState;
    uint16_t warningFlags;
    uint16_t errorFlags;

    uint8_t sequence;
    uint8_t uartNumber;
    bool valid;
};

struct ClientData
{
    uint16_t voltage_10mV;
    int16_t current_10mA;
    uint16_t requestedPower_W;
    uint16_t actualPower_W;
    uint8_t soc_percent;
    uint16_t chargeState;
    uint16_t operatingState;
    uint16_t batteryCapacity_Ah;
    uint16_t batteryInfoCode;
    uint16_t warningFlags;
    uint16_t errorFlags;

    uint8_t sequence;
    uint8_t uartNumber;
    bool valid;
};

// ============================================================
// Global variables
// ============================================================

Parser parser1;
Parser parser3;

MasterData masterData;
MasterData previousMasterData;

ClientData clientData;
ClientData previousClientData;

uint32_t masterReceiveTime_ms = 0UL;
uint32_t outputLineCount = 0UL;

uint32_t previousParser1CrcErrors = 0UL;
uint32_t previousParser1FormatErrors = 0UL;
uint32_t previousParser1DataErrors = 0UL;
uint32_t previousParser1TimeoutErrors = 0UL;

uint32_t previousParser3CrcErrors = 0UL;
uint32_t previousParser3FormatErrors = 0UL;
uint32_t previousParser3DataErrors = 0UL;
uint32_t previousParser3TimeoutErrors = 0UL;

bool masterWaitingForClient = false;

SequenceStatistics masterSequenceStats = {};
SequenceStatistics clientSequenceStats = {};

RejectRecord rejectQueue[REJECT_QUEUE_SIZE];
uint8_t rejectQueueRead = 0U;
uint8_t rejectQueueWrite = 0U;
uint8_t rejectQueueCount = 0U;
uint32_t rejectQueueDropped = 0UL;

// ============================================================
// Function prototypes
// ============================================================

void ResetParser(Parser *parser);
void ProcessSerial1(uint32_t now_ms);
void ProcessSerial3(uint32_t now_ms);
void ProcessReceivedByte(Parser *parser, uint8_t value, uint8_t uartNumber, uint32_t now_ms);
void CheckParserTimeout(Parser *parser, uint32_t now_ms);
void HandleCompleteFrame(Parser *parser, uint8_t uartNumber, uint32_t now_ms);

bool DecodeMasterFrame(const ProtocolFrame *frame, uint8_t uartNumber, char *reason, size_t reasonSize);
bool DecodeClientFrame(const ProtocolFrame *frame, uint8_t uartNumber, char *reason, size_t reasonSize);
bool MasterDataPlausible(const MasterData *data, char *reason, size_t reasonSize);
bool ClientDataPlausible(const ClientData *data, char *reason, size_t reasonSize);
void AppendReason(char *reason, size_t reasonSize, const char *field, long value, long minimum, long maximum);
void ParserStartRawFrame(Parser *parser, uint8_t value, uint32_t now_ms);
void ParserAppendRawByte(Parser *parser, uint8_t value);
void QueueReject(const Parser *parser, uint8_t uartNumber, RejectType type, ParserState stateAtError, uint32_t now_ms, const char *reason);
void ProcessRejectQueue();
const char *RejectTypeText(RejectType type);
const char *ParserStateText(ParserState state);
void UpdateSequenceStatistics(SequenceStatistics *statistics, uint8_t sequence);

void TryPrintCompletePair(uint32_t now_ms);
void PrintMissingClientIfTimedOut(uint32_t now_ms);
void PrintHeader();
void PrintDataLine(uint32_t now_ms, bool pairComplete);

char DetermineDelta(bool pairComplete);
const char *DetermineInfo(bool pairComplete, uint16_t warnings, uint16_t errors);
bool MasterValuesChanged();
bool ClientValuesChanged();
bool ErrorCountersChanged();

uint16_t ReadU16(const uint8_t *buffer, uint8_t *index);
uint16_t UpdateCrc16(uint16_t crc, uint8_t value);

void PrintVoltage(uint16_t value_10mV);
void PrintCurrent(int16_t value_10mA);
void PrintHex16(uint16_t value);
void PrintColumnText(const char *text, uint8_t width);
void PrintColumnUnsigned(uint32_t value, uint8_t width);
void PrintColumnVoltage(uint16_t value_10mV, uint8_t width);
void PrintColumnCurrent(int16_t value_10mA, uint8_t width);
void PrintColumnHex16(uint16_t value, uint8_t width);

bool SD_Init();
bool SD_TestDirectory();
bool SD_WriteTestFile();
bool SD_ReadVerifyTestFile();
bool SD_RemoveTestFile();
bool SD_PrintCardInfo();
const char* SD_CardTypeText(uint8_t cardType);
bool Log_CreateStructure();
bool Log_CreateHourHeader(const char *filePath, uint16_t year, uint8_t month, uint8_t day, uint8_t hour);
bool Log_CreateEventHeader(const char *filePath, uint16_t year, uint8_t month, uint8_t day);
bool Log_CreateRejectHeader(const char *filePath, uint16_t year, uint8_t month, uint8_t day);
bool Log_EnsureCurrentFiles();
void Log_ProcessDataLine(bool pairComplete, const char *infoText);
bool Log_WriteHourlyData(bool pairComplete, const char *infoText);
void Log_ProcessEvents(bool pairComplete);
void Log_WriteEvent(const char *device, const char *eventName, const char *action, const char *valueText, const char *infoText);
void Log_WriteReject(const RejectRecord *record);
void Log_PrintTime(File &file);
void Log_PrintDate(File &file);
void Log_PrintVoltage10mV(File &file, uint16_t value_10mV);
void Log_PrintCurrent10mA(File &file, int16_t value_10mA);
void RTC_PrintDateTimeToFile(File &file);
void RTC_Init();
void RTC_PrintDateTime();
void RTC_SetFromCompileTime();
uint8_t RTC_CompileMonth(const char *dateText);
uint8_t RTC_CalcWeekDay(uint16_t year, uint8_t month, uint8_t day);

// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial1.begin(CHARGER_BAUDRATE);
    Serial2.begin(OUTPUT_BAUDRATE);
    Serial3.begin(CHARGER_BAUDRATE);

    ResetParser(&parser1);
    ResetParser(&parser3);

    masterData.valid = false;
    previousMasterData.valid = false;

    clientData.valid = false;
    previousClientData.valid = false;

    delay(300);

    RTC_Init();

    Serial2.println("MONITOR_START_STEP_10D1_DE_CSV");
    Serial2.print("DATE/TIME: ");
    RTC_PrintDateTime();
    Serial2.println("RTC: OK");
    Serial2.println();

    Serial2.println("SD: INIT...");
    if (SD_Init())
    {
        Serial2.println("SD: OK");

        if (SD_TestDirectory())
        {
            if (SD_WriteTestFile())
            {
                Serial2.println("SD: WRITE OK");

                if (SD_ReadVerifyTestFile())
                {
                    Serial2.println("SD: READ OK");

                    if (SD_RemoveTestFile())
                        Serial2.println("SD: TEST FILE REMOVED");
                    else
                        Serial2.println("SD: REMOVE ERROR");

                    // Low-level card information temporarily takes over the SPI/SD hardware.
                    Serial2.println();
                    Serial2.println("SD: CARD INFO...");
                    if (!SD_PrintCardInfo())
                        Serial2.println("SD: CARD INFO ERROR");

                    // Restore the proven normal SD.h object after the low-level diagnostic.
                    // All runtime logging uses only this normal SD object.
                    Serial2.println();
                    Serial2.println("SD: LOGGER REINIT...");
                    if (SD_Init())
                    {
                        Serial2.println("SD: LOGGER READY");
                        Serial2.println();
                        Serial2.println("LOG: STRUCTURE...");
                        if (Log_CreateStructure())
                        {
                            tm_t logNow;
                            rtc.getTime(logNow);
                            logCurrentYear = (uint16_t)logNow.year + 1970U;
                            logCurrentMonth = logNow.month;
                            logCurrentDay = logNow.day;
                            logCurrentHour = logNow.hour;
                            sdLoggerReady = true;
                            Serial2.println("LOG: STRUCTURE OK");
                            Serial2.println("LOG: DATA/EVENT READY");
                        }
                        else
                        {
                            Serial2.println("LOG: STRUCTURE ERROR");
                        }
                    }
                    else
                    {
                        Serial2.println("SD: LOGGER REINIT ERROR");
                    }
                }
                else
                {
                    Serial2.println("SD: READ ERROR");
                }
            }
            else
            {
                Serial2.println("SD: WRITE ERROR");
            }
        }
    }
    else
    {
        Serial2.println("SD: NOT READY");
    }

    Serial2.println();
    PrintHeader();
}

// ============================================================
// SD Card - Step 10B.1
// ============================================================

bool SD_Init()
{
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);

    // Default SPI interface on this NUCLEO/Roger-core setup is SPI1:
    // PA5=SCK, PA6=MISO, PA7=MOSI.
    SPI.begin();

    // Initialization only. Without a separate card-detect contact, false
    // cannot reliably distinguish "no card" from an initialization error.
    return SD.begin(SD_CS_PIN);
}



bool SD_TestDirectory()
{
    if (SD.exists(SD_TEST_DIR))
    {
        Serial2.println("SD: TEST DIR OK");
        return true;
    }

    if (!SD.mkdir(SD_TEST_DIR))
    {
        Serial2.println("SD: DIR ERROR");
        return false;
    }

    Serial2.println("SD: TEST DIR CREATED");
    return true;
}

bool SD_WriteTestFile()
{
    // A previous interrupted test must not turn this test into an append test.
    if (SD.exists(SD_TEST_FILE))
        SD.remove(SD_TEST_FILE);

    File file = SD.open(SD_TEST_FILE, FILE_WRITE);
    if (!file)
    {
        Serial2.println("SD: OPEN WRITE ERROR");
        return false;
    }

    file.println(SD_TEST_SIGNATURE);
    file.println("Step: 10C");
    file.print("DATE/TIME: ");
    RTC_PrintDateTimeToFile(file);
    file.println("RTC: OK");
    file.println("SD INIT: OK");
    file.println("WRITE TEST: OK");

    file.flush();
    bool writeOk = (file.size() > 0U);
    file.close();

    return writeOk;
}

bool SD_ReadVerifyTestFile()
{
    File file = SD.open(SD_TEST_FILE, FILE_READ);
    if (!file)
    {
        Serial2.println("SD: OPEN READ ERROR");
        return false;
    }

    const char expected[] = SD_TEST_SIGNATURE;
    uint8_t index = 0U;
    bool match = true;

    while (index < (sizeof(expected) - 1U))
    {
        if (!file.available())
        {
            match = false;
            break;
        }

        char value = (char)file.read();
        if (value != expected[index])
        {
            match = false;
            break;
        }
        index++;
    }

    file.close();
    return match && (index == (sizeof(expected) - 1U));
}

bool SD_RemoveTestFile()
{
    if (!SD.exists(SD_TEST_FILE))
        return true;

    if (!SD.remove(SD_TEST_FILE))
        return false;

    return !SD.exists(SD_TEST_FILE);
}

const char* SD_CardTypeText(uint8_t cardType)
{
    switch (cardType)
    {
        case SD_CARD_TYPE_SD1:  return "SD1";
        case SD_CARD_TYPE_SD2:  return "SD2";
        case SD_CARD_TYPE_SDHC: return "SDHC";
        default:                return "UNKNOWN";
    }
}

bool SD_PrintCardInfo()
{
    // SD.h wraps an older SdFat implementation.  Use a separate low-level
    // card/volume instance only for diagnostics; the normal logger continues
    // to use the already proven SD object above.
    Sd2Card infoCard;
    SdVolume infoVolume;
    cid_t cid;

    if (!infoCard.init(SPI_HALF_SPEED, SD_CS_PIN))
    {
        Serial2.print("SD INFO: CARD INIT ERROR 0x");
        Serial2.println(infoCard.errorCode(), HEX);
        return false;
    }

    if (!infoVolume.init(infoCard))
    {
        Serial2.println("SD INFO: FAT VOLUME ERROR");
        return false;
    }

    Serial2.print("SD TYPE: ");
    Serial2.println(SD_CardTypeText(infoCard.type()));

    if (infoCard.readCID(&cid))
    {
        char productName[6];
        for (uint8_t i = 0U; i < 5U; i++)
            productName[i] = (char)cid.pnm[i];
        productName[5] = '\0';

        Serial2.print("SD NAME: ");
        Serial2.println(productName);

        Serial2.print("SD MID : 0x");
        Serial2.println(cid.mid, HEX);
    }
    else
    {
        Serial2.println("SD NAME: NOT AVAILABLE");
    }

    Serial2.print("SD FAT : FAT");
    Serial2.println(infoVolume.fatType(), DEC);

    // One SD block is 512 bytes. Total capacity is available with this old
    // SD/SdVolume implementation. Free-cluster information is deliberately
    // omitted because this library version does not provide freeClusterCount().
    uint32_t blocksPerCluster = (uint32_t)infoVolume.blocksPerCluster();
    uint32_t totalClusters = infoVolume.clusterCount();

    if (blocksPerCluster == 0U || totalClusters == 0U)
    {
        Serial2.println("SD SPACE: READ ERROR");
        return false;
    }

    uint64_t totalBlocks = (uint64_t)blocksPerCluster * (uint64_t)totalClusters;
    uint32_t totalMB = (uint32_t)(totalBlocks / 2048ULL); // 2048 * 512 B = 1 MiB

    Serial2.print("SD TOTAL: ");
    Serial2.print(totalMB);
    Serial2.println(" MB");

    return true;
}

// ============================================================
// Log structure - Step 10C
// ============================================================

bool Log_CreateHourHeader(const char *filePath, uint16_t year, uint8_t month, uint8_t day, uint8_t hour)
{
    if (SD.exists(filePath))
    {
        Serial2.print("LOG: HOUR FILE OK  ");
        Serial2.println(filePath);
        return true;
    }

    File file = SD.open(filePath, FILE_WRITE);
    if (!file)
    {
        Serial2.print("LOG: HOUR OPEN ERROR ");
        Serial2.println(filePath);
        return false;
    }

    file.println("# Battery Service Monitor");
    file.println("# FileType=HOURLY_DATA");
    file.print("# Date=");
    if (year < 1000U) file.print('0');
    if (year < 100U) file.print('0');
    if (year < 10U) file.print('0');
    file.print(year);
    file.print('-');
    if (month < 10U) file.print('0');
    file.print(month);
    file.print('-');
    if (day < 10U) file.print('0');
    file.println(day);
    file.print("# Hour=");
    if (hour < 10U) file.print('0');
    file.println(hour);
    file.println("# Firmware=STEP_10D2_EXTERNAL_CHARGE_EVENT");
    file.println("# Protocol=1");
    file.println("Time;Seq;Pair;U24_V;I24_A;Psrc_W;Safe_W;Grant_W;SOC24;Src;MPPT;PM;U12_V;I12_A;Req_W;Act_W;SOC12;Charge;Operate;Capacity_Ah;InfoCode;SrcWarn;SrcErr;AstWarn;AstErr;State");
    file.flush();
    bool ok = (file.size() > 0U);
    file.close();

    if (ok)
    {
        Serial2.print("LOG: HOUR CREATED  ");
        Serial2.println(filePath);
    }
    return ok;
}

bool Log_CreateEventHeader(const char *filePath, uint16_t year, uint8_t month, uint8_t day)
{
    if (SD.exists(filePath))
    {
        Serial2.print("LOG: EVENT FILE OK ");
        Serial2.println(filePath);
        return true;
    }

    File file = SD.open(filePath, FILE_WRITE);
    if (!file)
    {
        Serial2.print("LOG: EVENT OPEN ERROR ");
        Serial2.println(filePath);
        return false;
    }

    file.println("# Battery Service Monitor");
    file.println("# FileType=EVENT_LOG");
    file.print("# Date=");
    if (year < 1000U) file.print('0');
    if (year < 100U) file.print('0');
    if (year < 10U) file.print('0');
    file.print(year);
    file.print('-');
    if (month < 10U) file.print('0');
    file.print(month);
    file.print('-');
    if (day < 10U) file.print('0');
    file.println(day);
    file.println("# Firmware=STEP_10D2_EXTERNAL_CHARGE_EVENT");
    file.println("# Protocol=1");
    file.println("Date;Time;Device;Event;Action;Value;Info");
    file.flush();
    bool ok = (file.size() > 0U);
    file.close();

    if (ok)
    {
        Serial2.print("LOG: EVENT CREATED ");
        Serial2.println(filePath);
    }
    return ok;
}

bool Log_CreateRejectHeader(const char *filePath, uint16_t year, uint8_t month, uint8_t day)
{
    if (SD.exists(filePath))
    {
        Serial2.print("LOG: REJECT FILE OK ");
        Serial2.println(filePath);
        return true;
    }

    File file = SD.open(filePath, FILE_WRITE);
    if (!file)
    {
        Serial2.print("LOG: REJECT OPEN ERROR ");
        Serial2.println(filePath);
        return false;
    }

    file.println("# Battery Service Monitor");
    file.println("# FileType=REJECT_LOG");
    file.print("# Date=");
    if (year < 1000U) file.print('0');
    if (year < 100U) file.print('0');
    if (year < 10U) file.print('0');
    file.print(year);
    file.print('-');
    if (month < 10U) file.print('0');
    file.print(month);
    file.print('-');
    if (day < 10U) file.print('0');
    file.println(day);
    file.println("# Firmware=PROTOCOL_MONITOR_REJECT_DIAGNOSTICS");
    file.println("# Protocol=1");
    file.println("Date;Time;Millis;UART;Reject;ParserState;Version;Type;Seq;Length;RxCRC;CalcCRC;Reason;Raw");
    file.flush();
    bool ok = (file.size() > 0U);
    file.close();

    if (ok)
    {
        Serial2.print("LOG: REJECT CREATED ");
        Serial2.println(filePath);
    }
    return ok;
}

bool Log_CreateStructure()
{
    tm_t now;
    rtc.getTime(now);

    uint16_t year = (uint16_t)now.year + 1970U;
    uint8_t month = now.month;
    uint8_t day = now.day;
    uint8_t hour = now.hour;

    if (year < 2025U || year > 2099U || month < 1U || month > 12U || day < 1U || day > 31U || hour > 23U)
    {
        Serial2.println("LOG: RTC INVALID");
        return false;
    }

    const char rootDir[] = "/LOG";
    char dayDir[20];
    char hourFile[32];
    char eventFile[32];
    char rejectFile[32];

    snprintf(dayDir, sizeof(dayDir), "/LOG/%04u%02u%02u", year, month, day);
    snprintf(hourFile, sizeof(hourFile), "%s/%02u.CSV", dayDir, hour);
    snprintf(eventFile, sizeof(eventFile), "%s/EVENT.CSV", dayDir);
    snprintf(rejectFile, sizeof(rejectFile), "%s/REJECT.CSV", dayDir);

    if (!SD.exists(rootDir))
    {
        if (!SD.mkdir(rootDir))
        {
            Serial2.println("LOG: ROOT DIR ERROR");
            return false;
        }
        Serial2.println("LOG: ROOT CREATED /LOG");
    }
    else
    {
        Serial2.println("LOG: ROOT DIR OK /LOG");
    }

    if (!SD.exists(dayDir))
    {
        if (!SD.mkdir(dayDir))
        {
            Serial2.print("LOG: DAY DIR ERROR ");
            Serial2.println(dayDir);
            return false;
        }
        Serial2.print("LOG: DAY CREATED  ");
        Serial2.println(dayDir);
    }
    else
    {
        Serial2.print("LOG: DAY DIR OK   ");
        Serial2.println(dayDir);
    }

    if (!Log_CreateHourHeader(hourFile, year, month, day, hour))
        return false;

    if (!Log_CreateEventHeader(eventFile, year, month, day))
        return false;

    if (!Log_CreateRejectHeader(rejectFile, year, month, day))
        return false;

    return true;
}

// ============================================================
// Runtime data/event/reject logger
// ============================================================

bool Log_EnsureCurrentFiles()
{
    if (!sdLoggerReady)
        return false;

    tm_t now;
    rtc.getTime(now);
    uint16_t year = (uint16_t)now.year + 1970U;

    if (year == logCurrentYear && now.month == logCurrentMonth &&
        now.day == logCurrentDay && now.hour == logCurrentHour)
        return true;

    if (!Log_CreateStructure())
    {
        Serial2.println("LOG: ROLLOVER ERROR");
        sdLoggerReady = false;
        return false;
    }

    logCurrentYear = year;
    logCurrentMonth = now.month;
    logCurrentDay = now.day;
    logCurrentHour = now.hour;
    Serial2.println("LOG: NEW HOUR/DAY READY");
    return true;
}

void Log_PrintTime(File &file)
{
    tm_t now;
    rtc.getTime(now);
    if (now.hour < 10U) file.print('0');
    file.print(now.hour); file.print(':');
    if (now.minute < 10U) file.print('0');
    file.print(now.minute); file.print(':');
    if (now.second < 10U) file.print('0');
    file.print(now.second);
}

void Log_PrintDate(File &file)
{
    tm_t now;
    rtc.getTime(now);
    uint16_t year = (uint16_t)now.year + 1970U;
    file.print(year); file.print('-');
    if (now.month < 10U) file.print('0');
    file.print(now.month); file.print('-');
    if (now.day < 10U) file.print('0');
    file.print(now.day);
}

void Log_PrintVoltage10mV(File &file, uint16_t value_10mV)
{
    file.print(value_10mV / 100U);
    file.print(',');
    uint16_t frac = value_10mV % 100U;
    if (frac < 10U) file.print('0');
    file.print(frac);
}

void Log_PrintCurrent10mA(File &file, int16_t value_10mA)
{
    int32_t v = value_10mA;
    if (v < 0)
    {
        file.print('-');
        v = -v;
    }
    file.print(v / 100L);
    file.print(',');
    int32_t frac = v % 100L;
    if (frac < 10L) file.print('0');
    file.print(frac);
}

bool Log_WriteHourlyData(bool pairComplete, const char *infoText)
{
    if (!Log_EnsureCurrentFiles())
        return false;

    tm_t now;
    rtc.getTime(now);
    uint16_t year = (uint16_t)now.year + 1970U;
    char path[32];
    snprintf(path, sizeof(path), "/LOG/%04u%02u%02u/%02u.CSV", year, now.month, now.day, now.hour);

    File file = SD.open(path, FILE_WRITE);
    if (!file)
    {
        Serial2.println("LOG: DATA OPEN ERROR");
        return false;
    }

    Log_PrintTime(file); file.print(';');
    if (masterData.valid) file.print(masterData.sequence); file.print(';');
    file.print(pairComplete ? "OK" : "NOCL"); file.print(';');

    if (masterData.valid)
    {
        Log_PrintVoltage10mV(file, masterData.voltage_10mV); file.print(';');
        Log_PrintCurrent10mA(file, masterData.current_10mA); file.print(';');
        file.print(masterData.sourcePower_W); file.print(';');
        file.print(masterData.safePower_W); file.print(';');
        file.print(masterData.grantPower_W); file.print(';');
        file.print(masterData.soc_percent); file.print(';');
        file.print(masterData.sourceMode); file.print(';');
        file.print(masterData.mpptState); file.print(';');
        file.print(masterData.powerManagerState); file.print(';');
    }
    else
    {
        file.print(";;;;;;;;;");
    }

    if (pairComplete && clientData.valid)
    {
        Log_PrintVoltage10mV(file, clientData.voltage_10mV); file.print(';');
        Log_PrintCurrent10mA(file, clientData.current_10mA); file.print(';');
        file.print(clientData.requestedPower_W); file.print(';');
        file.print(clientData.actualPower_W); file.print(';');
        file.print(clientData.soc_percent); file.print(';');
        file.print(clientData.chargeState); file.print(';');
        file.print(clientData.operatingState); file.print(';');
        file.print(clientData.batteryCapacity_Ah); file.print(';');
        file.print(clientData.batteryInfoCode); file.print(';');
    }
    else
    {
        file.print(";;;;;;;;;");
    }

    if (masterData.valid)
    {
        file.print(masterData.warningFlags, HEX); file.print(';');
        file.print(masterData.errorFlags, HEX); file.print(';');
    }
    else file.print(";;");

    if (pairComplete && clientData.valid)
    {
        file.print(clientData.warningFlags, HEX); file.print(';');
        file.print(clientData.errorFlags, HEX); file.print(';');
    }
    else file.print(";;");

    file.println(infoText ? infoText : "");
    file.flush();
    bool ok = file.size() > 0U;
    file.close();
    if (!ok) Serial2.println("LOG: DATA WRITE ERROR");
    return ok;
}

void Log_WriteEvent(const char *device, const char *eventName, const char *action,
                    const char *valueText, const char *infoText)
{
    if (!Log_EnsureCurrentFiles())
        return;

    tm_t now;
    rtc.getTime(now);
    uint16_t year = (uint16_t)now.year + 1970U;
    char path[32];
    snprintf(path, sizeof(path), "/LOG/%04u%02u%02u/EVENT.CSV", year, now.month, now.day);

    File file = SD.open(path, FILE_WRITE);
    if (!file)
    {
        Serial2.println("LOG: EVENT OPEN ERROR");
        return;
    }

    Log_PrintDate(file); file.print(';');
    Log_PrintTime(file); file.print(';');
    file.print(device); file.print(';');
    file.print(eventName); file.print(';');
    file.print(action); file.print(';');
    file.print(valueText ? valueText : ""); file.print(';');
    file.println(infoText ? infoText : "");
    file.flush();
    file.close();
}

void Log_WriteReject(const RejectRecord *record)
{
    if (!Log_EnsureCurrentFiles())
        return;

    tm_t now;
    rtc.getTime(now);
    uint16_t year = (uint16_t)now.year + 1970U;
    char path[32];
    snprintf(path, sizeof(path), "/LOG/%04u%02u%02u/REJECT.CSV", year, now.month, now.day);

    File file = SD.open(path, FILE_WRITE);
    if (!file)
    {
        Serial2.println("LOG: REJECT WRITE OPEN ERROR");
        return;
    }

    Log_PrintDate(file); file.print(';');
    Log_PrintTime(file); file.print(';');
    file.print(record->eventTime_ms); file.print(';');
    file.print(record->uartNumber); file.print(';');
    file.print(RejectTypeText(record->type)); file.print(';');
    file.print(ParserStateText(record->parserState)); file.print(';');
    file.print(record->version); file.print(';');
    file.print("0x"); if (record->messageType < 0x1000U) file.print('0'); if (record->messageType < 0x0100U) file.print('0'); if (record->messageType < 0x0010U) file.print('0'); file.print(record->messageType, HEX); file.print(';');
    file.print(record->sequence); file.print(';');
    file.print(record->payloadLength); file.print(';');
    file.print("0x"); if (record->receivedCrc < 0x1000U) file.print('0'); if (record->receivedCrc < 0x0100U) file.print('0'); if (record->receivedCrc < 0x0010U) file.print('0'); file.print(record->receivedCrc, HEX); file.print(';');
    file.print("0x"); if (record->calculatedCrc < 0x1000U) file.print('0'); if (record->calculatedCrc < 0x0100U) file.print('0'); if (record->calculatedCrc < 0x0010U) file.print('0'); file.print(record->calculatedCrc, HEX); file.print(';');
    file.print(record->reason); file.print(';');

    for (uint8_t i = 0U; i < record->rawLength; i++)
    {
        if (i > 0U) file.print(' ');
        if (record->rawBuffer[i] < 0x10U) file.print('0');
        file.print(record->rawBuffer[i], HEX);
    }
    file.println();
    file.flush();
    file.close();
}

void Log_ProcessEvents(bool pairComplete)
{
    uint16_t sourceWarn = masterData.valid ? masterData.warningFlags : 0U;
    uint16_t sourceErr  = masterData.valid ? masterData.errorFlags : 0U;
    uint16_t assistWarn = (pairComplete && clientData.valid) ? clientData.warningFlags : 0U;
    uint16_t assistErr  = (pairComplete && clientData.valid) ? clientData.errorFlags : 0U;
    uint16_t assistInfo = (pairComplete && clientData.valid) ? clientData.batteryInfoCode : 0U;

    char value[48];
    char info[80];

    // Communication loss/restoration is itself an abnormal event.
    if (!logPairStateInitialized || pairComplete != logPreviousPairComplete)
    {
        if (logPairStateInitialized)
        {
            if (!pairComplete)
                Log_WriteEvent("MONITOR", "ASSIST_LINK", "LOST", "", "No matching CLIENT frame");
            else
                Log_WriteEvent("MONITOR", "ASSIST_LINK", "RESTORED", "", "Valid SOURCE/ASSIST pair received");
        }
        logPreviousPairComplete = pairComplete;
        logPairStateInitialized = true;
    }

    if (!logEventStateInitialized)
    {
        logPreviousSourceWarnings = sourceWarn;
        logPreviousSourceErrors = sourceErr;
        logPreviousAssistWarnings = assistWarn;
        logPreviousAssistErrors = assistErr;
        logPreviousAssistInfo = assistInfo;
        logEventStateInitialized = true;

        // Existing abnormal state at monitor startup is worth recording once.
        if (assistWarn & DEVICE_WARNING_LOW_BIT)
        {
            snprintf(value, sizeof(value), "U12=%u.%02uV", clientData.voltage_10mV/100U, clientData.voltage_10mV%100U);
            Log_WriteEvent("ASSIST", "LOW_BATTERY", "DETECTED", value, "Warning already active at logger start");
        }
        if (assistWarn & DEVICE_WARNING_FAST_BIT)
        {
            snprintf(value, sizeof(value), "U12=%u.%02uV I12=%d", clientData.voltage_10mV/100U, clientData.voltage_10mV%100U, clientData.current_10mA);
            Log_WriteEvent("ASSIST", "FAST_DISCHARGE", "DETECTED", value, "Warning already active at logger start");
        }
        if (assistErr & DEVICE_ERROR_DCDC_BIT)
            Log_WriteEvent("ASSIST", "DCDC_ERROR", "DETECTED", "", "Charge protection locked");
        if (sourceErr & DEVICE_ERROR_DCDC_BIT)
            Log_WriteEvent("SOURCE", "DCDC_ERROR", "DETECTED", "", "Charge protection locked");
        if (pairComplete && clientData.valid && assistInfo == 7U)
        {
            snprintf(value, sizeof(value), "U12=%u.%02uV I12=%d",
                     clientData.voltage_10mV/100U, clientData.voltage_10mV%100U, clientData.current_10mA);
            Log_WriteEvent("ASSIST", "EXTERNAL_CHARGE", "ACTIVE", value,
                           "External charger already active at logger start; ASSIST inhibited");
        }
    }
    else
    {
        if (pairComplete && clientData.valid)
        {
            uint16_t changedAssistWarn = assistWarn ^ logPreviousAssistWarnings;
            if (changedAssistWarn & DEVICE_WARNING_LOW_BIT)
            {
                snprintf(value, sizeof(value), "U12=%u.%02uV I12=%d", clientData.voltage_10mV/100U, clientData.voltage_10mV%100U, clientData.current_10mA);
                const char *clearReason = (clientData.voltage_10mV >= 1220U) ? "Battery recovered to normal range" : "Warning acknowledged by user";
                Log_WriteEvent("ASSIST", "LOW_BATTERY", (assistWarn & DEVICE_WARNING_LOW_BIT) ? "SET" : "CLEAR", value,
                               (assistWarn & DEVICE_WARNING_LOW_BIT) ? "Battery below allowed range" : clearReason);
            }
            if (changedAssistWarn & DEVICE_WARNING_FAST_BIT)
            {
                snprintf(value, sizeof(value), "U12=%u.%02uV I12=%d", clientData.voltage_10mV/100U, clientData.voltage_10mV%100U, clientData.current_10mA);
                const char *clearReason = (clientData.voltage_10mV >= 1220U) ? "Battery/voltage condition normalized" : "Warning acknowledged by user";
                Log_WriteEvent("ASSIST", "FAST_DISCHARGE", (assistWarn & DEVICE_WARNING_FAST_BIT) ? "SET" : "CLEAR", value,
                               (assistWarn & DEVICE_WARNING_FAST_BIT) ? "Voltage drop faster than allowed" : clearReason);
            }

            uint16_t changedAssistErr = assistErr ^ logPreviousAssistErrors;
            if (changedAssistErr & DEVICE_ERROR_DCDC_BIT)
            {
                snprintf(value, sizeof(value), "U12=%u.%02uV", clientData.voltage_10mV/100U, clientData.voltage_10mV%100U);
                Log_WriteEvent("ASSIST", "DCDC_ERROR", (assistErr & DEVICE_ERROR_DCDC_BIT) ? "SET" : "CLEAR", value,
                               (assistErr & DEVICE_ERROR_DCDC_BIT) ? "DCDC self-test failed; charge locked" : "DCDC fault cleared after valid restart/test");
            }
        }

        uint16_t changedSourceErr = sourceErr ^ logPreviousSourceErrors;
        if (changedSourceErr & DEVICE_ERROR_DCDC_BIT)
        {
            snprintf(value, sizeof(value), "U24=%u.%02uV", masterData.voltage_10mV/100U, masterData.voltage_10mV%100U);
            Log_WriteEvent("SOURCE", "DCDC_ERROR", (sourceErr & DEVICE_ERROR_DCDC_BIT) ? "SET" : "CLEAR", value,
                           (sourceErr & DEVICE_ERROR_DCDC_BIT) ? "DCDC self-test failed; charge locked" : "DCDC fault cleared after valid restart/test");
        }

        // SUPPORT_CHARGE is not normal routine logging here; it is recorded because it is
        // the direct protection response to a weak/rapidly discharged 12V battery.
        if (pairComplete && clientData.valid)
        {
            bool supportNow = (assistInfo == 5U);
            bool supportBefore = (logPreviousAssistInfo == 5U);
            if (supportNow != supportBefore)
            {
                snprintf(value, sizeof(value), "U12=%u.%02uV U24=%u.%02uV", clientData.voltage_10mV/100U, clientData.voltage_10mV%100U,
                         masterData.voltage_10mV/100U, masterData.voltage_10mV%100U);
                snprintf(info, sizeof(info), "Req=%uW Act=%uW ChargeState=%u", clientData.requestedPower_W, clientData.actualPower_W, clientData.chargeState);
                Log_WriteEvent("ASSIST", "SUPPORT_CHARGE", supportNow ? "START" : "END", value, info);
            }

            bool externalNow = (assistInfo == 7U);
            bool externalBefore = (logPreviousAssistInfo == 7U);
            if (externalNow != externalBefore)
            {
                snprintf(value, sizeof(value), "U12=%u.%02uV I12=%d",
                         clientData.voltage_10mV/100U, clientData.voltage_10mV%100U, clientData.current_10mA);
                Log_WriteEvent("ASSIST", "EXTERNAL_CHARGE",
                               externalNow ? "START" : "END",
                               value,
                               externalNow ? "Reverse current detected; CHARGE/DCDC inhibited"
                                           : "U12 <= 13.10V for 60s; ASSIST released");
            }

            logPreviousAssistWarnings = assistWarn;
            logPreviousAssistErrors = assistErr;
            logPreviousAssistInfo = assistInfo;
        }

        logPreviousSourceWarnings = sourceWarn;
        logPreviousSourceErrors = sourceErr;
    }

    uint32_t crcNow = parser1.crcErrorCount + parser3.crcErrorCount;
    uint32_t fmtNow = parser1.formatErrorCount + parser3.formatErrorCount;
    uint32_t dataNow = parser1.dataErrorCount + parser3.dataErrorCount;
    uint32_t toNow = parser1.timeoutErrorCount + parser3.timeoutErrorCount;
    if (crcNow != logPreviousCrcErrors || fmtNow != logPreviousFormatErrors ||
        dataNow != logPreviousDataErrors || toNow != logPreviousTimeoutErrors)
    {
        snprintf(value, sizeof(value), "CRC=%lu FMT=%lu DATA=%lu TO=%lu",
                 (unsigned long)crcNow, (unsigned long)fmtNow, (unsigned long)dataNow, (unsigned long)toNow);
        Log_WriteEvent("MONITOR", "UART_ERROR", "COUNT_CHANGED", value, "Protocol/parser error counter changed");
        logPreviousCrcErrors = crcNow;
        logPreviousFormatErrors = fmtNow;
        logPreviousDataErrors = dataNow;
        logPreviousTimeoutErrors = toNow;
    }
}

void Log_ProcessDataLine(bool pairComplete, const char *infoText)
{
    if (!sdLoggerReady)
        return;

    Log_WriteHourlyData(pairComplete, infoText);
    Log_ProcessEvents(pairComplete);
}

void RTC_PrintDateTimeToFile(File &file)
{
    tm_t now;
    rtc.getTime(now);

    uint16_t year = (uint16_t)now.year + 1970U;

    if (year < 1000U) file.print('0');
    if (year < 100U)  file.print('0');
    if (year < 10U)   file.print('0');
    file.print(year);
    file.print('-');
    if (now.month < 10U) file.print('0');
    file.print(now.month);
    file.print('-');
    if (now.day < 10U) file.print('0');
    file.print(now.day);
    file.print(' ');
    if (now.hour < 10U) file.print('0');
    file.print(now.hour);
    file.print(':');
    if (now.minute < 10U) file.print('0');
    file.print(now.minute);
    file.print(':');
    if (now.second < 10U) file.print('0');
    file.println(now.second);
}

// ============================================================
// Main loop
// ============================================================

void loop()
{
    uint32_t now_ms = millis();

    ProcessSerial1(now_ms);
    ProcessSerial3(now_ms);

    CheckParserTimeout(&parser1, now_ms);
    CheckParserTimeout(&parser3, now_ms);

    TryPrintCompletePair(now_ms);
    PrintMissingClientIfTimedOut(now_ms);

    ProcessRejectQueue();
}

// ============================================================
// RTC - Step 10A
// ============================================================

void RTC_Init()
{
#if RTC_USE_COMPILE_TIME_IF_UNSET
    // Roger Clark RTClock stores Unix-like seconds in the F1 RTC counter.
    // A fresh/invalid backup domain normally returns a very small value.
    if ((uint32_t)rtc.getTime() < RTC_VALID_EPOCH_MIN)
    {
        RTC_SetFromCompileTime();
    }
#endif
}

void RTC_PrintDateTime()
{
    tm_t nowTm;
    rtc.getTime(nowTm);

    uint16_t fullYear = (uint16_t)(1970U + nowTm.year);

    Serial2.print(fullYear);
    Serial2.print('-');
    if (nowTm.month < 10U) Serial2.print('0');
    Serial2.print(nowTm.month);
    Serial2.print('-');
    if (nowTm.day < 10U) Serial2.print('0');
    Serial2.print(nowTm.day);
    Serial2.print(' ');
    if (nowTm.hour < 10U) Serial2.print('0');
    Serial2.print(nowTm.hour);
    Serial2.print(':');
    if (nowTm.minute < 10U) Serial2.print('0');
    Serial2.print(nowTm.minute);
    Serial2.print(':');
    if (nowTm.second < 10U) Serial2.print('0');
    Serial2.println(nowTm.second);
}

void RTC_SetFromCompileTime()
{
    const char *dateText = __DATE__;  // Example: "Aug  9 2026"
    const char *timeText = __TIME__;  // Example: "15:42:17"

    uint8_t month = RTC_CompileMonth(dateText);
    uint8_t day = (uint8_t)(((dateText[4] == ' ') ? 0 : (dateText[4] - '0')) * 10 +
                            (dateText[5] - '0'));
    uint16_t fullYear = (uint16_t)((dateText[7] - '0') * 1000U +
                                   (dateText[8] - '0') * 100U +
                                   (dateText[9] - '0') * 10U +
                                   (dateText[10] - '0'));

    tm_t compileTm;
    compileTm.year = (uint8_t)(fullYear - 1970U);
    compileTm.month = month;
    compileTm.day = day;
    compileTm.weekday = RTC_CalcWeekDay(fullYear, month, day);
    compileTm.hour = (uint8_t)((timeText[0] - '0') * 10 + (timeText[1] - '0'));
    compileTm.minute = (uint8_t)((timeText[3] - '0') * 10 + (timeText[4] - '0'));
    compileTm.second = (uint8_t)((timeText[6] - '0') * 10 + (timeText[7] - '0'));

    rtc.setTime(compileTm);
}

uint8_t RTC_CompileMonth(const char *dateText)
{
    const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";

    for (uint8_t i = 0U; i < 12U; i++)
    {
        uint8_t p = (uint8_t)(i * 3U);
        if (dateText[0] == months[p] &&
            dateText[1] == months[p + 1U] &&
            dateText[2] == months[p + 2U])
        {
            return (uint8_t)(i + 1U);
        }
    }

    return 1U;
}

uint8_t RTC_CalcWeekDay(uint16_t year, uint8_t month, uint8_t day)
{
    // Sakamoto algorithm. Roger Clark tm_t uses Time-library style weekday: Sunday=1 ... Saturday=7.
    static const uint8_t monthOffset[12] = { 0U, 3U, 2U, 5U, 0U, 3U, 5U, 1U, 4U, 6U, 2U, 4U };
    uint16_t y = year;

    if (month < 3U)
        y--;

    uint8_t sundayZero = (uint8_t)((y + y / 4U - y / 100U + y / 400U +
                                    monthOffset[month - 1U] + day) % 7U);

    return (uint8_t)(sundayZero + 1U);
}

// ============================================================
// UART input
// ============================================================

void ProcessSerial1(uint32_t now_ms)
{
    while (Serial1.available() > 0)
    {
        uint8_t value = (uint8_t)Serial1.read();

        parser1.byteCount++;
        ProcessReceivedByte(&parser1, value, 1U, now_ms);
    }
}

void ProcessSerial3(uint32_t now_ms)
{
    while (Serial3.available() > 0)
    {
        uint8_t value = (uint8_t)Serial3.read();

        parser3.byteCount++;
        ProcessReceivedByte(&parser3, value, 3U, now_ms);
    }
}

// ============================================================
// Frame parser
// ============================================================

void ResetParser(Parser *parser)
{
    parser->state = WAIT_START_1;
    parser->payloadIndex = 0U;
    parser->calculatedCrc = 0xFFFFU;
    parser->frameStartTime_ms = 0UL;
    parser->rawLength = 0U;

    parser->frame.version = 0U;
    parser->frame.messageType = 0U;
    parser->frame.sequence = 0U;
    parser->frame.payloadLength = 0U;
    parser->frame.receivedCrc = 0U;
}

void ParserStartRawFrame(Parser *parser, uint8_t value, uint32_t now_ms)
{
    parser->rawLength = 0U;
    parser->frameStartTime_ms = now_ms;
    ParserAppendRawByte(parser, value);
}

void ParserAppendRawByte(Parser *parser, uint8_t value)
{
    if (parser->rawLength < RAW_FRAME_BUFFER_SIZE)
        parser->rawBuffer[parser->rawLength++] = value;
}

void QueueReject(const Parser *parser, uint8_t uartNumber, RejectType type,
                 ParserState stateAtError, uint32_t now_ms, const char *reason)
{
    if (rejectQueueCount >= REJECT_QUEUE_SIZE)
    {
        rejectQueueDropped++;
        return;
    }

    RejectRecord *record = &rejectQueue[rejectQueueWrite];
    record->type = type;
    record->uartNumber = uartNumber;
    record->parserState = stateAtError;
    record->version = parser->frame.version;
    record->messageType = parser->frame.messageType;
    record->sequence = parser->frame.sequence;
    record->payloadLength = parser->frame.payloadLength;
    record->receivedCrc = parser->frame.receivedCrc;
    record->calculatedCrc = parser->calculatedCrc;
    record->eventTime_ms = now_ms;
    record->rawLength = parser->rawLength;

    for (uint8_t i = 0U; i < record->rawLength; i++)
        record->rawBuffer[i] = parser->rawBuffer[i];

    if (reason)
    {
        strncpy(record->reason, reason, REJECT_REASON_LENGTH - 1U);
        record->reason[REJECT_REASON_LENGTH - 1U] = '\0';
    }
    else
    {
        record->reason[0] = '\0';
    }

    rejectQueueWrite = (uint8_t)((rejectQueueWrite + 1U) % REJECT_QUEUE_SIZE);
    rejectQueueCount++;
}

void ProcessRejectQueue()
{
    while (rejectQueueCount > 0U)
    {
        RejectRecord *record = &rejectQueue[rejectQueueRead];

        Serial2.print("REJECT UART");
        Serial2.print(record->uartNumber);
        Serial2.print(' ');
        Serial2.print(RejectTypeText(record->type));
        Serial2.print(" Seq=");
        Serial2.print(record->sequence);
        Serial2.print(" Reason=");
        Serial2.println(record->reason);

        Log_WriteReject(record);

        rejectQueueRead = (uint8_t)((rejectQueueRead + 1U) % REJECT_QUEUE_SIZE);
        rejectQueueCount--;
    }

    if (rejectQueueDropped > 0UL)
    {
        char value[32];
        snprintf(value, sizeof(value), "Dropped=%lu", (unsigned long)rejectQueueDropped);
        Serial2.print("REJECT QUEUE DROPPED=");
        Serial2.println(rejectQueueDropped);
        Log_WriteEvent("MONITOR", "REJECT_QUEUE", "DROPPED", value, "Reject queue overflow; some raw frames could not be stored");
        rejectQueueDropped = 0UL;
    }
}

const char *RejectTypeText(RejectType type)
{
    switch (type)
    {
        case REJECT_VERSION:      return "VERSION_ERROR";
        case REJECT_MESSAGE_TYPE: return "UNKNOWN_TYPE";
        case REJECT_LENGTH:       return "LENGTH_ERROR";
        case REJECT_CRC:          return "CRC_ERROR";
        case REJECT_DATA:         return "DATA_IMPLAUSIBLE";
        case REJECT_TIMEOUT:      return "FRAME_TIMEOUT";
        default:                  return "UNKNOWN";
    }
}

const char *ParserStateText(ParserState state)
{
    switch (state)
    {
        case WAIT_START_1:   return "WAIT_START_1";
        case WAIT_START_2:   return "WAIT_START_2";
        case READ_VERSION:   return "READ_VERSION";
        case READ_TYPE_HIGH: return "READ_TYPE_HIGH";
        case READ_TYPE_LOW:  return "READ_TYPE_LOW";
        case READ_SEQUENCE:  return "READ_SEQUENCE";
        case READ_LENGTH:    return "READ_LENGTH";
        case READ_PAYLOAD:   return "READ_PAYLOAD";
        case READ_CRC_HIGH:  return "READ_CRC_HIGH";
        case READ_CRC_LOW:   return "READ_CRC_LOW";
        default:             return "UNKNOWN";
    }
}

void ProcessReceivedByte(Parser *parser, uint8_t value, uint8_t uartNumber, uint32_t now_ms)
{
    parser->lastByteTime_ms = now_ms;

    switch (parser->state)
    {
        case WAIT_START_1:
            if (value == START_BYTE_1)
            {
                ParserStartRawFrame(parser, value, now_ms);
                parser->state = WAIT_START_2;
            }
            break;

        case WAIT_START_2:
            if (value == START_BYTE_2)
            {
                ParserAppendRawByte(parser, value);
                parser->calculatedCrc = 0xFFFFU;
                parser->payloadIndex = 0U;
                parser->state = READ_VERSION;
            }
            else if (value == START_BYTE_1)
            {
                // Start a new possible header. Header noise is not written to REJECT.CSV.
                ParserStartRawFrame(parser, value, now_ms);
                parser->state = WAIT_START_2;
            }
            else
            {
                parser->formatErrorCount++;
                ResetParser(parser);
            }
            break;

        case READ_VERSION:
            ParserAppendRawByte(parser, value);
            parser->frame.version = value;
            parser->calculatedCrc = UpdateCrc16(parser->calculatedCrc, value);
            parser->state = READ_TYPE_HIGH;
            break;

        case READ_TYPE_HIGH:
            ParserAppendRawByte(parser, value);
            parser->frame.messageType = (uint16_t)value << 8;
            parser->calculatedCrc = UpdateCrc16(parser->calculatedCrc, value);
            parser->state = READ_TYPE_LOW;
            break;

        case READ_TYPE_LOW:
            ParserAppendRawByte(parser, value);
            parser->frame.messageType |= value;
            parser->calculatedCrc = UpdateCrc16(parser->calculatedCrc, value);
            parser->state = READ_SEQUENCE;
            break;

        case READ_SEQUENCE:
            ParserAppendRawByte(parser, value);
            parser->frame.sequence = value;
            parser->calculatedCrc = UpdateCrc16(parser->calculatedCrc, value);
            parser->state = READ_LENGTH;
            break;

        case READ_LENGTH:
        {
            ParserAppendRawByte(parser, value);
            parser->frame.payloadLength = value;
            parser->calculatedCrc = UpdateCrc16(parser->calculatedCrc, value);
            parser->payloadIndex = 0U;

            if (value != PROTOCOL_PAYLOAD_LENGTH)
            {
                char reason[64];
                snprintf(reason, sizeof(reason), "PayloadLength=%u expected=%u", value, PROTOCOL_PAYLOAD_LENGTH);
                parser->formatErrorCount++;
                QueueReject(parser, uartNumber, REJECT_LENGTH, READ_LENGTH, now_ms, reason);
                ResetParser(parser);
            }
            else
            {
                parser->state = READ_PAYLOAD;
            }
            break;
        }

        case READ_PAYLOAD:
            ParserAppendRawByte(parser, value);
            parser->frame.payload[parser->payloadIndex] = value;
            parser->payloadIndex++;
            parser->calculatedCrc = UpdateCrc16(parser->calculatedCrc, value);

            if (parser->payloadIndex >= parser->frame.payloadLength)
                parser->state = READ_CRC_HIGH;
            break;

        case READ_CRC_HIGH:
            ParserAppendRawByte(parser, value);
            parser->frame.receivedCrc = (uint16_t)value << 8;
            parser->state = READ_CRC_LOW;
            break;

        case READ_CRC_LOW:
            ParserAppendRawByte(parser, value);
            parser->frame.receivedCrc |= value;

            if (parser->frame.receivedCrc == parser->calculatedCrc)
            {
                parser->validFrameCount++;
                HandleCompleteFrame(parser, uartNumber, now_ms);
            }
            else
            {
                char reason[80];
                snprintf(reason, sizeof(reason), "RxCRC=0x%04X CalcCRC=0x%04X", parser->frame.receivedCrc, parser->calculatedCrc);
                parser->crcErrorCount++;
                QueueReject(parser, uartNumber, REJECT_CRC, READ_CRC_LOW, now_ms, reason);
            }

            ResetParser(parser);
            break;
    }
}

void CheckParserTimeout(Parser *parser, uint32_t now_ms)
{
    if (parser->state == WAIT_START_1)
        return;

    if ((uint32_t)(now_ms - parser->lastByteTime_ms) >= FRAME_TIMEOUT_MS)
    {
        char reason[96];
        ParserState stateAtError = parser->state;
        snprintf(reason, sizeof(reason), "NoByteFor=%lums rawBytes=%u frameAge=%lums",
                 (unsigned long)(now_ms - parser->lastByteTime_ms), parser->rawLength,
                 (unsigned long)(now_ms - parser->frameStartTime_ms));
        parser->timeoutErrorCount++;
        QueueReject(parser, (parser == &parser1) ? 1U : 3U, REJECT_TIMEOUT, stateAtError, now_ms, reason);
        ResetParser(parser);
    }
}

// ============================================================
// Frame classification
// ============================================================

void HandleCompleteFrame(Parser *parser, uint8_t uartNumber, uint32_t now_ms)
{
    if (parser->frame.version != PROTOCOL_VERSION)
    {
        char reason[64];
        snprintf(reason, sizeof(reason), "Version=%u expected=%u", parser->frame.version, PROTOCOL_VERSION);
        parser->formatErrorCount++;
        QueueReject(parser, uartNumber, REJECT_VERSION, READ_CRC_LOW, now_ms, reason);
        return;
    }

    char reason[REJECT_REASON_LENGTH];
    reason[0] = '\0';

    switch (parser->frame.messageType)
    {
        case MESSAGE_MASTER_REQUEST:
            if (DecodeMasterFrame(&parser->frame, uartNumber, reason, sizeof(reason)))
            {
                masterReceiveTime_ms = now_ms;
                masterWaitingForClient = true;
            }
            else
            {
                parser->dataErrorCount++;
                QueueReject(parser, uartNumber, REJECT_DATA, READ_CRC_LOW, now_ms, reason);
            }
            break;

        case MESSAGE_CLIENT_RESPONSE:
            if (!DecodeClientFrame(&parser->frame, uartNumber, reason, sizeof(reason)))
            {
                parser->dataErrorCount++;
                QueueReject(parser, uartNumber, REJECT_DATA, READ_CRC_LOW, now_ms, reason);
            }
            break;

        default:
        {
            char typeReason[64];
            snprintf(typeReason, sizeof(typeReason), "MessageType=0x%04X unknown", parser->frame.messageType);
            parser->formatErrorCount++;
            QueueReject(parser, uartNumber, REJECT_MESSAGE_TYPE, READ_CRC_LOW, now_ms, typeReason);
            break;
        }
    }
}

// ============================================================
// Payload decoding
// ============================================================

bool DecodeMasterFrame(const ProtocolFrame *frame, uint8_t uartNumber, char *reason, size_t reasonSize)
{
    MasterData decoded = {};
    uint8_t index = 0U;

    if (frame->payloadLength != PROTOCOL_PAYLOAD_LENGTH)
    {
        snprintf(reason, reasonSize, "PayloadLength=%u expected=%u", frame->payloadLength, PROTOCOL_PAYLOAD_LENGTH);
        return false;
    }

    decoded.voltage_10mV = ReadU16(frame->payload, &index);
    decoded.current_10mA = (int16_t)ReadU16(frame->payload, &index);
    decoded.sourcePower_W = ReadU16(frame->payload, &index);
    decoded.safePower_W = ReadU16(frame->payload, &index);
    decoded.grantPower_W = ReadU16(frame->payload, &index);
    decoded.soc_percent = frame->payload[index++];
    decoded.sourceMode = ReadU16(frame->payload, &index);
    decoded.mpptState = ReadU16(frame->payload, &index);
    decoded.powerManagerState = ReadU16(frame->payload, &index);
    decoded.warningFlags = ReadU16(frame->payload, &index);
    decoded.errorFlags = ReadU16(frame->payload, &index);

    if (index != PROTOCOL_PAYLOAD_LENGTH)
    {
        snprintf(reason, reasonSize, "DecodeIndex=%u expected=%u", index, PROTOCOL_PAYLOAD_LENGTH);
        return false;
    }

    if (!MasterDataPlausible(&decoded, reason, reasonSize))
        return false;

    decoded.sequence = frame->sequence;
    decoded.uartNumber = uartNumber;
    decoded.valid = true;
    UpdateSequenceStatistics(&masterSequenceStats, decoded.sequence);
    masterData = decoded;
    return true;
}

bool DecodeClientFrame(const ProtocolFrame *frame, uint8_t uartNumber, char *reason, size_t reasonSize)
{
    ClientData decoded = {};
    uint8_t index = 0U;

    if (frame->payloadLength != PROTOCOL_PAYLOAD_LENGTH)
    {
        snprintf(reason, reasonSize, "PayloadLength=%u expected=%u", frame->payloadLength, PROTOCOL_PAYLOAD_LENGTH);
        return false;
    }

    decoded.voltage_10mV = ReadU16(frame->payload, &index);
    decoded.current_10mA = (int16_t)ReadU16(frame->payload, &index);
    decoded.requestedPower_W = ReadU16(frame->payload, &index);
    decoded.actualPower_W = ReadU16(frame->payload, &index);
    decoded.soc_percent = frame->payload[index++];
    decoded.chargeState = ReadU16(frame->payload, &index);
    decoded.operatingState = ReadU16(frame->payload, &index);
    decoded.batteryCapacity_Ah = ReadU16(frame->payload, &index);
    decoded.batteryInfoCode = ReadU16(frame->payload, &index);
    decoded.warningFlags = ReadU16(frame->payload, &index);
    decoded.errorFlags = ReadU16(frame->payload, &index);

    if (index != PROTOCOL_PAYLOAD_LENGTH)
    {
        snprintf(reason, reasonSize, "DecodeIndex=%u expected=%u", index, PROTOCOL_PAYLOAD_LENGTH);
        return false;
    }

    if (!ClientDataPlausible(&decoded, reason, reasonSize))
        return false;

    decoded.sequence = frame->sequence;
    decoded.uartNumber = uartNumber;
    decoded.valid = true;
    UpdateSequenceStatistics(&clientSequenceStats, decoded.sequence);
    clientData = decoded;
    return true;
}

void AppendReason(char *reason, size_t reasonSize, const char *field, long value, long minimum, long maximum)
{
    size_t used = strlen(reason);
    if (used >= reasonSize - 1U)
        return;

    snprintf(reason + used, reasonSize - used, "%s%s=%ld allowed=%ld..%ld",
             (used > 0U) ? "," : "", field, value, minimum, maximum);
}

bool MasterDataPlausible(const MasterData *data, char *reason, size_t reasonSize)
{
    reason[0] = '\0';

    if (data->voltage_10mV > 3600U) AppendReason(reason, reasonSize, "U24_10mV", data->voltage_10mV, 0, 3600);
    if (data->current_10mA < -5000 || data->current_10mA > 5000) AppendReason(reason, reasonSize, "I24_10mA", data->current_10mA, -5000, 5000);
    if (data->sourcePower_W > 1500U) AppendReason(reason, reasonSize, "SourcePower_W", data->sourcePower_W, 0, 1500);
    if (data->safePower_W > 1500U) AppendReason(reason, reasonSize, "SafePower_W", data->safePower_W, 0, 1500);
    if (data->grantPower_W > 500U) AppendReason(reason, reasonSize, "GrantPower_W", data->grantPower_W, 0, 500);
    if (data->soc_percent > 100U) AppendReason(reason, reasonSize, "SOC24", data->soc_percent, 0, 100);
    if (data->sourceMode > 3U) AppendReason(reason, reasonSize, "SourceMode", data->sourceMode, 0, 3);
    if (data->mpptState > 7U) AppendReason(reason, reasonSize, "MPPT", data->mpptState, 0, 7);
    if (data->powerManagerState > 4U) AppendReason(reason, reasonSize, "PowerManager", data->powerManagerState, 0, 4);

    return reason[0] == '\0';
}

bool ClientDataPlausible(const ClientData *data, char *reason, size_t reasonSize)
{
    reason[0] = '\0';

    if (data->voltage_10mV > 1800U) AppendReason(reason, reasonSize, "U12_10mV", data->voltage_10mV, 0, 1800);
    if (data->current_10mA < -2000 || data->current_10mA > 2000) AppendReason(reason, reasonSize, "I12_10mA", data->current_10mA, -2000, 2000);
    if (data->requestedPower_W > 500U) AppendReason(reason, reasonSize, "Requested_W", data->requestedPower_W, 0, 500);
    if (data->actualPower_W > 500U) AppendReason(reason, reasonSize, "Actual_W", data->actualPower_W, 0, 500);
    if (data->soc_percent > 100U) AppendReason(reason, reasonSize, "SOC12", data->soc_percent, 0, 100);
    if (data->chargeState > 4U) AppendReason(reason, reasonSize, "ChargeState", data->chargeState, 0, 4);
    if (data->operatingState > 4U) AppendReason(reason, reasonSize, "OperatingState", data->operatingState, 0, 4);
    if (data->batteryCapacity_Ah > 2000U) AppendReason(reason, reasonSize, "CapacityAh", data->batteryCapacity_Ah, 0, 2000);
    if (data->batteryInfoCode > 7U) AppendReason(reason, reasonSize, "InfoCode", data->batteryInfoCode, 0, 7);

    return reason[0] == '\0';
}

// ============================================================
// Sequence diagnostics
// ============================================================

void UpdateSequenceStatistics(SequenceStatistics *statistics, uint8_t sequence)
{
    uint8_t forwardDistance;

    if (!statistics->initialized)
    {
        statistics->lastSequence = sequence;
        statistics->initialized = true;
        return;
    }

    forwardDistance = (uint8_t)(sequence - statistics->lastSequence);

    if (forwardDistance == 0U)
    {
        statistics->duplicateFrameCount++;
        return;
    }

    if (forwardDistance < 128U)
    {
        if (forwardDistance > 1U)
        {
            statistics->lostFrameCount += (uint32_t)(forwardDistance - 1U);
        }
        statistics->lastSequence = sequence;
        return;
    }

    statistics->reverseFrameCount++;
}

// ============================================================
// Pair handling
// ============================================================

void TryPrintCompletePair(uint32_t now_ms)
{
    if (!masterWaitingForClient)
    {
        return;
    }

    if (!masterData.valid || !clientData.valid)
    {
        return;
    }

    if (clientData.sequence != masterData.sequence)
    {
        return;
    }

    PrintDataLine(now_ms, true);
    masterWaitingForClient = false;
}

void PrintMissingClientIfTimedOut(uint32_t now_ms)
{
    if (!masterWaitingForClient)
    {
        return;
    }

    if ((uint32_t)(now_ms - masterReceiveTime_ms) < PAIR_TIMEOUT_MS)
    {
        return;
    }

    PrintDataLine(now_ms, false);
    masterWaitingForClient = false;
}

// ============================================================
// TAB output
// ============================================================

void PrintHeader()
{
    PrintColumnText("FRAME", 23);
    PrintColumnText("SOURCE (24V)", 45);
    PrintColumnText("ASSIST (12V)", 45);
    PrintColumnText("FLAGS", 12);
    PrintColumnText("UART", 27);
    PrintColumnText("SEQUENCE", 38);
    PrintColumnText("INFO", 16);
    Serial2.println();

    PrintColumnText("Time", 7); PrintColumnText("Seq", 4); PrintColumnText("Pair", 6);
    PrintColumnText("MU", 3); PrintColumnText("CU", 3);
    PrintColumnText("U24", 6); PrintColumnText("I24", 7); PrintColumnText("Psrc", 5);
    PrintColumnText("Safe", 5); PrintColumnText("Grant", 6); PrintColumnText("SOC", 4);
    PrintColumnText("Src", 4); PrintColumnText("MPPT", 5); PrintColumnText("PM", 3);
    PrintColumnText("U12", 6); PrintColumnText("I12", 7); PrintColumnText("Req", 5);
    PrintColumnText("Act", 5); PrintColumnText("SOC", 4); PrintColumnText("Chg", 4);
    PrintColumnText("Oper", 5); PrintColumnText("Ah", 5); PrintColumnText("Info", 4);
    PrintColumnText("Warn", 5); PrintColumnText("Err", 5); PrintColumnText("D", 2);
    PrintColumnText("U1OK", 6); PrintColumnText("U3OK", 6); PrintColumnText("CRC", 5);
    PrintColumnText("FMT", 5); PrintColumnText("DATA", 6); PrintColumnText("TO", 5);
    PrintColumnText("MLost", 7); PrintColumnText("MDup", 6); PrintColumnText("MRev", 6);
    PrintColumnText("CLost", 7); PrintColumnText("CDup", 6); PrintColumnText("CRev", 6);
    PrintColumnText("State", 16);
    Serial2.println();
}

void PrintDataLine(uint32_t now_ms, bool pairComplete)
{
    uint16_t combinedWarnings = 0U;
    uint16_t combinedErrors = 0U;
    uint32_t combinedCrcErrors;
    uint32_t combinedFormatErrors;
    uint32_t combinedDataErrors;
    uint32_t combinedTimeoutErrors;
    char deltaText[2];
    const char *infoText;

    if (outputLineCount > 0UL && (outputLineCount % HEADER_REPEAT_LINES) == 0UL) PrintHeader();

    if (masterData.valid)
    {
        combinedWarnings |= masterData.warningFlags;
        combinedErrors |= masterData.errorFlags;
    }
    if (pairComplete && clientData.valid)
    {
        combinedWarnings |= clientData.warningFlags;
        combinedErrors |= clientData.errorFlags;
    }

    combinedCrcErrors = parser1.crcErrorCount + parser3.crcErrorCount;
    combinedFormatErrors = parser1.formatErrorCount + parser3.formatErrorCount;
    combinedDataErrors = parser1.dataErrorCount + parser3.dataErrorCount;
    combinedTimeoutErrors = parser1.timeoutErrorCount + parser3.timeoutErrorCount;

    deltaText[0] = DetermineDelta(pairComplete);
    deltaText[1] = '\0';
    infoText = DetermineInfo(pairComplete, combinedWarnings, combinedErrors);

    PrintColumnUnsigned(now_ms, 7);
    masterData.valid ? PrintColumnUnsigned(masterData.sequence, 4) : PrintColumnText("-", 4);
    PrintColumnText(pairComplete ? "OK" : "NOCL", 6);
    masterData.valid ? PrintColumnUnsigned(masterData.uartNumber, 3) : PrintColumnText("-", 3);
    (pairComplete && clientData.valid) ? PrintColumnUnsigned(clientData.uartNumber, 3) : PrintColumnText("-", 3);

    if (masterData.valid)
    {
        PrintColumnVoltage(masterData.voltage_10mV, 6);
        PrintColumnCurrent(masterData.current_10mA, 7);
        PrintColumnUnsigned(masterData.sourcePower_W, 5);
        PrintColumnUnsigned(masterData.safePower_W, 5);
        PrintColumnUnsigned(masterData.grantPower_W, 6);
        PrintColumnUnsigned(masterData.soc_percent, 4);
        PrintColumnUnsigned(masterData.sourceMode, 4);
        PrintColumnUnsigned(masterData.mpptState, 5);
        PrintColumnUnsigned(masterData.powerManagerState, 3);
    }
    else
    {
        PrintColumnText("-", 6); PrintColumnText("-", 7); PrintColumnText("-", 5);
        PrintColumnText("-", 5); PrintColumnText("-", 6); PrintColumnText("-", 4);
        PrintColumnText("-", 4); PrintColumnText("-", 5); PrintColumnText("-", 3);
    }

    if (pairComplete && clientData.valid)
    {
        PrintColumnVoltage(clientData.voltage_10mV, 6);
        PrintColumnCurrent(clientData.current_10mA, 7);
        PrintColumnUnsigned(clientData.requestedPower_W, 5);
        PrintColumnUnsigned(clientData.actualPower_W, 5);
        PrintColumnUnsigned(clientData.soc_percent, 4);
        PrintColumnUnsigned(clientData.chargeState, 4);
        PrintColumnUnsigned(clientData.operatingState, 5);
        PrintColumnUnsigned(clientData.batteryCapacity_Ah, 5);
        PrintColumnUnsigned(clientData.batteryInfoCode, 4);
    }
    else
    {
        PrintColumnText("-", 6); PrintColumnText("-", 7); PrintColumnText("-", 5);
        PrintColumnText("-", 5); PrintColumnText("-", 4); PrintColumnText("-", 4);
        PrintColumnText("-", 5); PrintColumnText("-", 5); PrintColumnText("-", 4);
    }

    PrintColumnHex16(combinedWarnings, 5);
    PrintColumnHex16(combinedErrors, 5);
    PrintColumnText(deltaText, 2);
    PrintColumnUnsigned(parser1.validFrameCount, 6);
    PrintColumnUnsigned(parser3.validFrameCount, 6);
    PrintColumnUnsigned(combinedCrcErrors, 5);
    PrintColumnUnsigned(combinedFormatErrors, 5);
    PrintColumnUnsigned(combinedDataErrors, 6);
    PrintColumnUnsigned(combinedTimeoutErrors, 5);
    PrintColumnUnsigned(masterSequenceStats.lostFrameCount, 7);
    PrintColumnUnsigned(masterSequenceStats.duplicateFrameCount, 6);
    PrintColumnUnsigned(masterSequenceStats.reverseFrameCount, 6);
    PrintColumnUnsigned(clientSequenceStats.lostFrameCount, 7);
    PrintColumnUnsigned(clientSequenceStats.duplicateFrameCount, 6);
    PrintColumnUnsigned(clientSequenceStats.reverseFrameCount, 6);
    PrintColumnText(infoText, 16);
    Serial2.println();

    // Persist the same decoded data that is visible in the terminal.
    // This call occurs before previous* is updated, so EVENT transition detection
    // sees the true old/new state.
    Log_ProcessDataLine(pairComplete, infoText);

    previousMasterData = masterData;
    if (pairComplete) previousClientData = clientData;

    previousParser1CrcErrors = parser1.crcErrorCount;
    previousParser1FormatErrors = parser1.formatErrorCount;
    previousParser1DataErrors = parser1.dataErrorCount;
    previousParser1TimeoutErrors = parser1.timeoutErrorCount;
    previousParser3CrcErrors = parser3.crcErrorCount;
    previousParser3FormatErrors = parser3.formatErrorCount;
    previousParser3DataErrors = parser3.dataErrorCount;
    previousParser3TimeoutErrors = parser3.timeoutErrorCount;
    outputLineCount++;
}

// ============================================================
// Delta evaluation
// ============================================================

char DetermineDelta(bool pairComplete)
{
    bool masterChanged;
    bool clientChanged;

    if (ErrorCountersChanged())
    {
        return 'E';
    }

    if (!pairComplete)
    {
        return 'N';
    }

    if (masterData.valid &&
        previousMasterData.valid &&
        masterData.grantPower_W != previousMasterData.grantPower_W)
    {
        return 'G';
    }

    if ((masterData.valid &&
         previousMasterData.valid &&
         masterData.sourcePower_W != previousMasterData.sourcePower_W) ||
        (clientData.valid &&
         previousClientData.valid &&
         (clientData.requestedPower_W != previousClientData.requestedPower_W ||
          clientData.actualPower_W != previousClientData.actualPower_W)))
    {
        return 'P';
    }

    masterChanged = MasterValuesChanged();
    clientChanged = ClientValuesChanged();

    if (masterChanged && clientChanged)
    {
        return 'B';
    }

    if (masterChanged)
    {
        return 'S';
    }

    if (clientChanged)
    {
        return 'A';
    }

    return '.';
}

const char *DetermineInfo(bool pairComplete, uint16_t warnings, uint16_t errors)
{
    if (ErrorCountersChanged()) return "UART_ERROR";
    if (!pairComplete) return "NO_ASSIST";

    if (clientData.valid && (clientData.errorFlags & DEVICE_ERROR_DCDC_BIT))
        return "ASSIST_DCDC_ERROR";

    if (masterData.valid && (masterData.errorFlags & DEVICE_ERROR_DCDC_BIT))
        return "SOURCE_DCDC_ERROR";

    if (errors != 0U) return "DEVICE_ERROR";

    // ASSIST warning flags are latched until the user acknowledges them.
    // Bit 0 = LOW BATTERY, bit 1 = FAST DISCHARGE.
    if (clientData.valid)
    {
        uint16_t assistWarnings = clientData.warningFlags;

        if ((assistWarnings & (DEVICE_WARNING_LOW_BIT | DEVICE_WARNING_FAST_BIT)) ==
            (DEVICE_WARNING_LOW_BIT | DEVICE_WARNING_FAST_BIT))
            return "LOW+FAST";

        if (assistWarnings & DEVICE_WARNING_LOW_BIT)
            return "LOW_BATTERY";

        if (assistWarnings & DEVICE_WARNING_FAST_BIT)
            return "FAST_DISCHARGE";
    }


    // Keep only live non-warning observation states here. Warning-like batteryInfoCode
    // states are deliberately suppressed after acknowledgement.
    if (clientData.valid)
    {
        switch (clientData.batteryInfoCode)
        {
            case 2U: return "LOAD_ACTIVE";
            case 3U: return "RECOVERY";
            case 5U: return "SUPPORT_CHARGE";
            case 7U: return "EXTERNAL_CHARGE";
            default: break;
        }
    }

    if (warnings != 0U) return "DEVICE_WARNING";

    if ((masterData.valid && masterData.sourcePower_W > 0U) ||
        (clientData.valid && clientData.actualPower_W > 0U))
        return "ACTIVE";

    return "IDLE";
}

bool MasterValuesChanged()
{
    if (masterData.valid != previousMasterData.valid)
    {
        return true;
    }

    if (!masterData.valid)
    {
        return false;
    }

    return
        masterData.voltage_10mV != previousMasterData.voltage_10mV ||
        masterData.current_10mA != previousMasterData.current_10mA ||
        masterData.sourcePower_W != previousMasterData.sourcePower_W ||
        masterData.safePower_W != previousMasterData.safePower_W ||
        masterData.grantPower_W != previousMasterData.grantPower_W ||
        masterData.soc_percent != previousMasterData.soc_percent ||
        masterData.sourceMode != previousMasterData.sourceMode ||
        masterData.mpptState != previousMasterData.mpptState ||
        masterData.powerManagerState != previousMasterData.powerManagerState ||
        masterData.warningFlags != previousMasterData.warningFlags ||
        masterData.errorFlags != previousMasterData.errorFlags;
}

bool ClientValuesChanged()
{
    if (clientData.valid != previousClientData.valid)
    {
        return true;
    }

    if (!clientData.valid)
    {
        return false;
    }

    return
        clientData.voltage_10mV != previousClientData.voltage_10mV ||
        clientData.current_10mA != previousClientData.current_10mA ||
        clientData.requestedPower_W != previousClientData.requestedPower_W ||
        clientData.actualPower_W != previousClientData.actualPower_W ||
        clientData.soc_percent != previousClientData.soc_percent ||
        clientData.chargeState != previousClientData.chargeState ||
        clientData.operatingState != previousClientData.operatingState ||
        clientData.batteryCapacity_Ah != previousClientData.batteryCapacity_Ah ||
        clientData.batteryInfoCode != previousClientData.batteryInfoCode ||
        clientData.warningFlags != previousClientData.warningFlags ||
        clientData.errorFlags != previousClientData.errorFlags;
}

bool ErrorCountersChanged()
{
    return
        parser1.crcErrorCount != previousParser1CrcErrors ||
        parser1.formatErrorCount != previousParser1FormatErrors ||
        parser1.dataErrorCount != previousParser1DataErrors ||
        parser1.timeoutErrorCount != previousParser1TimeoutErrors ||
        parser3.crcErrorCount != previousParser3CrcErrors ||
        parser3.formatErrorCount != previousParser3FormatErrors ||
        parser3.dataErrorCount != previousParser3DataErrors ||
        parser3.timeoutErrorCount != previousParser3TimeoutErrors;
}

// ============================================================
// Protocol helper functions
// ============================================================

uint16_t ReadU16(const uint8_t *buffer, uint8_t *index)
{
    uint16_t value;

    value = (uint16_t)buffer[*index] << 8;
    (*index)++;

    value |= buffer[*index];
    (*index)++;

    return value;
}

uint16_t UpdateCrc16(uint16_t crc, uint8_t value)
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

// ============================================================
// Output helper functions
// ============================================================

void PrintColumnText(const char *text, uint8_t width)
{
    uint8_t length = (uint8_t)strlen(text);
    Serial2.print(text);
    while (length < width)
    {
        Serial2.print(' ');
        length++;
    }
}

void PrintColumnUnsigned(uint32_t value, uint8_t width)
{
    char buffer[12];
    snprintf(buffer, sizeof(buffer), "%lu", (unsigned long)value);
    PrintColumnText(buffer, width);
}

void PrintColumnVoltage(uint16_t value_10mV, uint8_t width)
{
    char buffer[12];
    snprintf(buffer, sizeof(buffer), "%u.%02u",
             (unsigned int)(value_10mV / 100U),
             (unsigned int)(value_10mV % 100U));
    PrintColumnText(buffer, width);
}

void PrintColumnCurrent(int16_t value_10mA, uint8_t width)
{
    char buffer[12];
    int32_t value = value_10mA;
    bool negative = false;

    if (value < 0)
    {
        negative = true;
        value = -value;
    }

    snprintf(buffer, sizeof(buffer), negative ? "-%ld.%02ld" : "%ld.%02ld",
             value / 100L, value % 100L);
    PrintColumnText(buffer, width);
}

void PrintColumnHex16(uint16_t value, uint8_t width)
{
    char buffer[5];
    snprintf(buffer, sizeof(buffer), "%04X", value);
    PrintColumnText(buffer, width);
}

void PrintVoltage(uint16_t value_10mV)
{
    uint8_t decimalPart;

    Serial2.print(value_10mV / 100U);
    Serial2.print('.');

    decimalPart = (uint8_t)(value_10mV % 100U);

    if (decimalPart < 10U)
    {
        Serial2.print('0');
    }

    Serial2.print(decimalPart);
}

void PrintCurrent(int16_t value_10mA)
{
    int32_t signedValue = value_10mA;
    uint8_t decimalPart;

    if (signedValue < 0L)
    {
        Serial2.print('-');
        signedValue = -signedValue;
    }

    Serial2.print(signedValue / 100L);
    Serial2.print('.');

    decimalPart = (uint8_t)(signedValue % 100L);

    if (decimalPart < 10U)
    {
        Serial2.print('0');
    }

    Serial2.print(decimalPart);
}

void PrintHex16(uint16_t value)
{
    const char hexDigits[] = "0123456789ABCDEF";

    Serial2.print(hexDigits[(value >> 12) & 0x0FU]);
    Serial2.print(hexDigits[(value >> 8) & 0x0FU]);
    Serial2.print(hexDigits[(value >> 4) & 0x0FU]);
    Serial2.print(hexDigits[value & 0x0FU]);
}

