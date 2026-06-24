#ifndef DYNESS_BMS_H
#define DYNESS_BMS_H

#include <Arduino.h>
#include <HardwareSerial.h>

// ================= CONFIG =================
#define DYNESS_MAX_BATTERIES    16
#define DYNESS_MAX_CELLS        24
#define DYNESS_MAX_TEMPS        8
#define DYNESS_FRAME_MAX        512
#define DYNESS_TIMEOUT_MS       2000
#define DYNESS_RETRY            3
#define DYNESS_BAUD_DEFAULT     115200

// ================= PROTOCOL CONSTANTS =================
#define DYNESS_SOI              0x7E
#define DYNESS_EOI              0x0D
#define DYNESS_CID1_BATTERY     0x46

// CID2 Commands
#define DYNESS_CMD_GET_VALUES   0x42
#define DYNESS_CMD_GET_ALARM    0x44
#define DYNESS_CMD_SYSPARAM     0x47
#define DYNESS_CMD_PROTOCOL     0x4F
#define DYNESS_CMD_MANUF        0x51
#define DYNESS_CMD_CHG_MGMT     0x92
#define DYNESS_CMD_SN           0x93
#define DYNESS_CMD_SET_CHG      0x94
#define DYNESS_CMD_TURNOFF      0x95
#define DYNESS_CMD_FW_VER       0x96

// ================= STATUS ENUM =================
enum DynessStatus {
    DYNESS_OK = 0,
    DYNESS_TIMEOUT,
    DYNESS_INVALID_FRAME,
    DYNESS_CHECKSUM_ERROR,
    DYNESS_DECODE_ERROR,
    DYNESS_RESPONSE_ERROR,
    DYNESS_NO_RESPONSE
};

// ================= BATTERY MODULE STRUCTURE =================
struct BatteryModule {
    uint8_t numberOfCells;
    float cellVoltages[DYNESS_MAX_CELLS];
    
    uint8_t numberOfTemperatures;
    float averageBMSTemperature;
    float groupedCellsTemperatures[DYNESS_MAX_TEMPS - 1];
    
    float current;
    float voltage;
    float power;
    float remainingCapacity;
    float totalCapacity;
    uint16_t cycleNumber;
    float stateOfCharge;
    
    BatteryModule() {
        memset(this, 0, sizeof(BatteryModule));
    }
};

// ================= BATTERY STATE STRUCTURE =================
struct BatteryState {
    uint8_t address;
    uint8_t groupAddress;
    uint8_t position;
    
    BatteryModule module;
    
    char serial[17];
    char deviceName[11];
    char manufacturerName[21];
    char softwareVersion[3];
    uint8_t protocolVersion;
    char firmwareVersion[10];
    
    // System parameters
    float cellHighVoltageLimit;
    float cellLowVoltageLimit;
    float cellUnderVoltageLimit;
    float chargeHighTemperatureLimit;
    float chargeLowTemperatureLimit;
    float chargeCurrentLimit;
    float moduleHighVoltageLimit;
    float moduleLowVoltageLimit;
    float moduleUnderVoltageLimit;
    float dischargeHighTemperatureLimit;
    float dischargeLowTemperatureLimit;
    float dischargeCurrentLimit;
    
    // Management info
    float chargeVoltageLimit;
    float dischargeVoltageLimit;
    float chargeCurrentMgmtLimit;
    float dischargeCurrentMgmtLimit;
    bool chargeEnable;
    bool dischargeEnable;
    bool chargeImmediately1;
    bool chargeImmediately2;
    bool fullChargeRequest;
    
    // Alarms
    bool alarmModuleUV;
    bool alarmChargeOT;
    bool alarmDischargeOT;
    bool alarmDOC;
    bool alarmCOC;
    bool alarmCellUV;
    bool alarmModuleOV;
    bool alarmCellOV;
    bool usingBatteryPower;
    bool dischargeMOS;
    bool chargeMOS;
    bool effectiveChargeCurrent;
    bool effectiveDischargeCurrent;
    bool heaterActive;
    bool fullyCharged;
    bool buzzerEnabled;
    bool cellError[DYNESS_MAX_CELLS];
    
    float soc;
    
    BatteryState() {
        memset(this, 0, sizeof(BatteryState));
    }
};

// ================= MAIN CLASS =================
class DynessBMS {
public:
    DynessBMS(HardwareSerial& port, uint8_t dePin);
    ~DynessBMS();
    
    void begin(uint32_t baud = DYNESS_BAUD_DEFAULT);
    
    // Main API methods
    DynessStatus getProtocolVersion(uint8_t& version);
    DynessStatus getManufacturerInfo(BatteryState& state);
    DynessStatus getSystemParameters(uint8_t devId, BatteryState& state);
    DynessStatus getManagementInfo(uint8_t devId, BatteryState& state);
    DynessStatus getModuleSerialNumber(uint8_t devId, BatteryState& state);
    DynessStatus getValues(BatteryState& state);
    DynessStatus getValuesSingle(uint8_t devId, BatteryState& state);
    
    // High level operations
    DynessStatus scanForBatteries(uint8_t start = 0x02, uint8_t end = 0x10);
    DynessStatus poll(uint8_t startAddr, uint8_t endAddr);
    
    // Getters
    uint8_t getBatteryCount() const;
    const BatteryState* getBattery(uint8_t i) const;
    
    // Address helpers
    static uint8_t calculateAddress(uint8_t group, uint8_t position) {
        return (group << 4) | position;
    }
    
    static uint8_t getGroupFromAddress(uint8_t addr) {
        return addr >> 4;
    }
    
    static uint8_t getPositionFromAddress(uint8_t addr) {
        return addr & 0x0F;
    }

private:
    HardwareSerial& _port;
    uint8_t _dePin;
    bool _initialized;
    
    BatteryState* _batteries;
    uint8_t _count;
    uint8_t _maxBatteries;
    
    // Core communication methods
    DynessStatus sendCmd(uint8_t address, uint8_t cid2, const uint8_t* info = nullptr, uint8_t infoLen = 0);
    DynessStatus readFrame(uint8_t* frame, uint16_t& frameLen);
    DynessStatus transact(uint8_t address, uint8_t cid2, const uint8_t* info, uint8_t infoLen,
                         uint8_t* responseInfo, uint16_t& responseInfoLen);
    
    // Frame encoding/decoding
    uint16_t getFrameChecksum(const uint8_t* frame, uint16_t len);
    uint16_t getInfoLength(uint16_t infoLen);
    uint8_t hexCharToValue(char c);
    uint8_t asciiHexToByte(char high, char low);
    
    // Parsers
    void parseManufacturerInfo(const uint8_t* info, uint16_t infoLen, BatteryState& state);
    void parseSystemParameters(const uint8_t* info, uint16_t infoLen, BatteryState& state);
    void parseManagementInfo(const uint8_t* info, uint16_t infoLen, BatteryState& state);
    void parseModuleSerialNumber(const uint8_t* info, uint16_t infoLen, BatteryState& state);
    void parseGetValuesSingle(const uint8_t* info, uint16_t infoLen, BatteryState& state);
};

#endif // DYNESS_BMS_H