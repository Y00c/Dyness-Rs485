#include "DynessBMS.h"

// ================= CONSTRUCTOR / DESTRUCTOR =================
DynessBMS::DynessBMS(HardwareSerial& port, uint8_t dePin)
    : _port(port), _dePin(dePin), _initialized(false), _count(0), _maxBatteries(DYNESS_MAX_BATTERIES) {
    _batteries = new BatteryState[_maxBatteries];
}

DynessBMS::~DynessBMS() {
    delete[] _batteries;
}

// ================= INITIALIZATION =================
void DynessBMS::begin(uint32_t baud) {
    pinMode(_dePin, OUTPUT);
    digitalWrite(_dePin, LOW);
    _port.begin(baud, SERIAL_8N1);
    _initialized = true;
    delay(100);
}

// ================= HEX UTILITIES =================
uint8_t DynessBMS::hexCharToValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

uint8_t DynessBMS::asciiHexToByte(char high, char low) {
    return (hexCharToValue(high) << 4) | hexCharToValue(low);
}

// ================= CHECKSUM =================
uint16_t DynessBMS::getFrameChecksum(const uint8_t* frame, uint16_t len) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += frame[i];
    }
    sum = ~sum;
    sum &= 0xFFFF;
    sum += 1;
    return sum & 0xFFFF;
}

uint16_t DynessBMS::getInfoLength(uint16_t infoLen) {
    if (infoLen == 0) return 0;
    
    uint16_t lenid = infoLen;
    uint8_t lenid_sum = (lenid & 0x0F) + ((lenid >> 4) & 0x0F) + ((lenid >> 8) & 0x0F);
    uint8_t lenid_modulo = lenid_sum % 16;
    uint8_t lenid_invert_plus_one = (0x0F - lenid_modulo) + 1;
    
    return (lenid_invert_plus_one << 12) | lenid;
}

// ================= FRAME ENCODING =================
DynessStatus DynessBMS::sendCmd(uint8_t address, uint8_t cid2, const uint8_t* info, uint8_t infoLen) {
    if (!_initialized) return DYNESS_NO_RESPONSE;
    
    uint8_t cid1 = DYNESS_CID1_BATTERY;
    uint16_t infoLength = getInfoLength(infoLen);
    
    // Build frame without SOI, CHKSUM, EOI
    uint8_t frame[256];
    uint16_t idx = 0;
    
    frame[idx++] = 0x20;  // VER
    frame[idx++] = address;
    frame[idx++] = cid1;
    frame[idx++] = cid2;
    frame[idx++] = (infoLength >> 8) & 0xFF;
    frame[idx++] = infoLength & 0xFF;
    
    if (infoLen > 0 && info != nullptr) {
        memcpy(&frame[idx], info, infoLen);
        idx += infoLen;
    }
    
    uint16_t chksum = getFrameChecksum(frame, idx);
    
    // Build complete frame with SOI, CHKSUM, EOI
    uint8_t wholeFrame[256];
    uint16_t wholeIdx = 0;
    
    wholeFrame[wholeIdx++] = DYNESS_SOI;
    memcpy(&wholeFrame[wholeIdx], frame, idx);
    wholeIdx += idx;
    
    char chksumStr[5];
    snprintf(chksumStr, sizeof(chksumStr), "%04X", chksum);
    for (int i = 0; i < 4; i++) {
        wholeFrame[wholeIdx++] = chksumStr[i];
    }
    
    wholeFrame[wholeIdx++] = DYNESS_EOI;
    
    // Send frame
    digitalWrite(_dePin, HIGH);
    delayMicroseconds(100);
    _port.write(wholeFrame, wholeIdx);
    _port.flush();
    delayMicroseconds(100);
    digitalWrite(_dePin, LOW);
    
    return DYNESS_OK;
}

// ================= FRAME RECEIVE =================
DynessStatus DynessBMS::readFrame(uint8_t* frame, uint16_t& frameLen) {
    uint32_t start = millis();
    frameLen = 0;
    bool inFrame = false;
    
    while (millis() - start < DYNESS_TIMEOUT_MS) {
        if (_port.available()) {
            uint8_t c = _port.read();
            
            if (!inFrame && c == DYNESS_SOI) {
                inFrame = true;
                frame[frameLen++] = c;
            } else if (inFrame) {
                if (frameLen < DYNESS_FRAME_MAX) {
                    frame[frameLen++] = c;
                }
                if (c == DYNESS_EOI && frameLen >= 10) {
                    return DYNESS_OK;
                }
            }
        }
        delay(1);
    }
    
    return DYNESS_TIMEOUT;
}

// ================= TRANSACTION =================
DynessStatus DynessBMS::transact(uint8_t address, uint8_t cid2, const uint8_t* info, uint8_t infoLen,
                                 uint8_t* responseInfo, uint16_t& responseInfoLen) {
    uint8_t rawFrame[DYNESS_FRAME_MAX];
    uint16_t rawLen;
    
    for (int attempt = 0; attempt < DYNESS_RETRY; attempt++) {
        DynessStatus status = sendCmd(address, cid2, info, infoLen);
        if (status != DYNESS_OK) continue;
        
        status = readFrame(rawFrame, rawLen);
        if (status != DYNESS_OK) continue;
        
        // Parse frame: ~ + frame_data(ascii hex) + chksum(4) + \r
        if (rawLen < 7) continue;
        
        // Extract frame_data (without SOI, CHKSUM, EOI)
        uint16_t dataLen = rawLen - 6;
        uint8_t frameData[256];
        memcpy(frameData, rawFrame + 1, dataLen);
        
        // Verify checksum
        char chksumStr[5] = {0};
        memcpy(chksumStr, rawFrame + 1 + dataLen, 4);
        uint16_t receivedChksum = strtol(chksumStr, nullptr, 16);
        uint16_t calculatedChksum = getFrameChecksum(frameData, dataLen);
        
        if (receivedChksum != calculatedChksum) continue;
        
        // Parse frame data (ASCII hex to bytes)
        if (dataLen < 12) continue;
        
        uint8_t ver = asciiHexToByte(frameData[0], frameData[1]);
        uint8_t adr = asciiHexToByte(frameData[2], frameData[3]);
        uint8_t cid1 = asciiHexToByte(frameData[4], frameData[5]);
        uint8_t cid2resp = asciiHexToByte(frameData[6], frameData[7]);
        
        // Verify response
        if (adr != address) continue;
        if ((cid2resp & 0x80) == 0) continue;
        
        // Parse info length
        uint8_t lenHigh = asciiHexToByte(frameData[8], frameData[9]);
        uint8_t lenLow = asciiHexToByte(frameData[10], frameData[11]);
        uint16_t respInfoLen = (lenHigh << 8) | lenLow;
        
        // Parse info
        uint16_t infoHexLen = dataLen - 12;
        uint16_t infoActualLen = infoHexLen / 2;
        
        if (infoActualLen > 0 && responseInfo != nullptr && infoActualLen <= responseInfoLen) {
            for (uint16_t i = 0; i < infoActualLen; i++) {
                responseInfo[i] = asciiHexToByte(frameData[12 + i * 2], frameData[12 + i * 2 + 1]);
            }
            responseInfoLen = infoActualLen;
        }
        
        return DYNESS_OK;
    }
    
    return DYNESS_TIMEOUT;
}

// ================= PARSERS =================
void DynessBMS::parseManufacturerInfo(const uint8_t* info, uint16_t infoLen, BatteryState& state) {
    if (infoLen < 12) return;
    
    memcpy(state.deviceName, info, 10);
    state.deviceName[10] = 0;
    
    snprintf(state.softwareVersion, sizeof(state.softwareVersion), "%c%c", info[10], info[11]);
    state.softwareVersion[2] = 0;
    
    uint16_t nameLen = infoLen - 12;
    if (nameLen > 20) nameLen = 20;
    memcpy(state.manufacturerName, info + 12, nameLen);
    state.manufacturerName[nameLen] = 0;
}

void DynessBMS::parseSystemParameters(const uint8_t* info, uint16_t infoLen, BatteryState& state) {
    if (infoLen < 24) return;
    
    uint16_t idx = 0;
    
    auto readInt16 = [&]() -> int16_t {
        int16_t val = (info[idx] << 8) | info[idx + 1];
        idx += 2;
        return val;
    };
    
    auto readUInt16 = [&]() -> uint16_t {
        uint16_t val = (info[idx] << 8) | info[idx + 1];
        idx += 2;
        return val;
    };
    
    state.cellHighVoltageLimit = readUInt16() / 1000.0f;
    state.cellLowVoltageLimit = readUInt16() / 1000.0f;
    state.cellUnderVoltageLimit = readInt16() / 1000.0f;
    state.chargeHighTemperatureLimit = (readInt16() - 2731) / 10.0f;
    state.chargeLowTemperatureLimit = (readInt16() - 2731) / 10.0f;
    state.chargeCurrentLimit = readInt16() / 10.0f;
    state.moduleHighVoltageLimit = readUInt16() / 1000.0f;
    state.moduleLowVoltageLimit = readUInt16() / 1000.0f;
    state.moduleUnderVoltageLimit = readUInt16() / 1000.0f;
    state.dischargeHighTemperatureLimit = (readInt16() - 2731) / 10.0f;
    state.dischargeLowTemperatureLimit = (readInt16() - 2731) / 10.0f;
    state.dischargeCurrentLimit = readInt16() / 10.0f;
}

void DynessBMS::parseManagementInfo(const uint8_t* info, uint16_t infoLen, BatteryState& state) {
    if (infoLen < 9) return;
    
    uint16_t idx = 0;
    
    auto readUInt16 = [&]() -> uint16_t {
        uint16_t val = (info[idx] << 8) | info[idx + 1];
        idx += 2;
        return val;
    };
    
    auto readInt16 = [&]() -> int16_t {
        int16_t val = (info[idx] << 8) | info[idx + 1];
        idx += 2;
        return val;
    };
    
    state.chargeVoltageLimit = readUInt16() / 1000.0f;
    state.dischargeVoltageLimit = readUInt16() / 1000.0f;
    state.chargeCurrentMgmtLimit = readInt16() / 10.0f;
    state.dischargeCurrentMgmtLimit = readInt16() / 10.0f;
    
    uint8_t status = info[idx];
    state.chargeEnable = (status & 0x80) != 0;
    state.dischargeEnable = (status & 0x40) != 0;
    state.chargeImmediately2 = (status & 0x20) != 0;
    state.chargeImmediately1 = (status & 0x10) != 0;
    state.fullChargeRequest = (status & 0x08) != 0;
}

void DynessBMS::parseModuleSerialNumber(const uint8_t* info, uint16_t infoLen, BatteryState& state) {
    if (infoLen < 17) return;
    memcpy(state.serial, info + 1, 16);
    state.serial[16] = 0;
}

void DynessBMS::parseGetValuesSingle(const uint8_t* info, uint16_t infoLen, BatteryState& state) {
    uint16_t idx = 0;
    
    if (idx >= infoLen) return;
    uint8_t numberOfModules = info[idx++];
    (void)numberOfModules;  // Unused for single module
    
    if (idx >= infoLen) return;
    state.module.numberOfCells = info[idx++];
    if (state.module.numberOfCells > DYNESS_MAX_CELLS) state.module.numberOfCells = DYNESS_MAX_CELLS;
    
    for (int c = 0; c < state.module.numberOfCells; c++) {
        if (idx + 1 >= infoLen) return;
        int16_t raw = (info[idx] << 8) | info[idx + 1];
        state.module.cellVoltages[c] = raw / 1000.0f;
        idx += 2;
    }
    
    if (idx >= infoLen) return;
    state.module.numberOfTemperatures = info[idx++];
    if (state.module.numberOfTemperatures > DYNESS_MAX_TEMPS) state.module.numberOfTemperatures = DYNESS_MAX_TEMPS;
    
    if (idx + 1 >= infoLen) return;
    int16_t tempRaw = (info[idx] << 8) | info[idx + 1];
    state.module.averageBMSTemperature = (tempRaw - 2731) / 10.0f;
    idx += 2;
    
    for (int t = 0; t < state.module.numberOfTemperatures - 1; t++) {
        if (idx + 1 >= infoLen) return;
        int16_t tempRaw = (info[idx] << 8) | info[idx + 1];
        state.module.groupedCellsTemperatures[t] = (tempRaw - 2731) / 10.0f;
        idx += 2;
    }
    
    if (idx + 1 >= infoLen) return;
    int16_t currentRaw = (info[idx] << 8) | info[idx + 1];
    state.module.current = currentRaw / 10.0f;
    idx += 2;
    
    if (idx + 1 >= infoLen) return;
    uint16_t voltageRaw = (info[idx] << 8) | info[idx + 1];
    state.module.voltage = voltageRaw / 1000.0f;
    idx += 2;
    
    state.module.power = state.module.current * state.module.voltage;
    
    if (idx + 1 >= infoLen) return;
    uint16_t remainingCap1 = (info[idx] << 8) | info[idx + 1];
    idx += 2;
    
    if (idx >= infoLen) return;
    uint8_t userDefinedItems = info[idx++];
    
    if (idx + 1 >= infoLen) return;
    uint16_t totalCap1 = (info[idx] << 8) | info[idx + 1];
    idx += 2;
    
    if (idx + 1 >= infoLen) return;
    state.module.cycleNumber = (info[idx] << 8) | info[idx + 1];
    idx += 2;
    
    if (userDefinedItems > 2 && idx + 2 < infoLen) {
        uint32_t remainingCap2 = (info[idx] << 16) | (info[idx + 1] << 8) | info[idx + 2];
        state.module.remainingCapacity = remainingCap2 / 1000.0f;
        idx += 3;
        
        if (idx + 2 < infoLen) {
            uint32_t totalCap2 = (info[idx] << 16) | (info[idx + 1] << 8) | info[idx + 2];
            state.module.totalCapacity = totalCap2 / 1000.0f;
        }
    } else {
        state.module.remainingCapacity = remainingCap1 / 1000.0f;
        state.module.totalCapacity = totalCap1 / 1000.0f;
    }
    
    if (state.module.totalCapacity > 0) {
        state.module.stateOfCharge = state.module.remainingCapacity / state.module.totalCapacity;
        state.soc = state.module.stateOfCharge * 100.0f;
    }
}

// ================= PUBLIC API METHODS =================
DynessStatus DynessBMS::getProtocolVersion(uint8_t& version) {
    uint8_t response[256];
    uint16_t respLen = sizeof(response);
    
    DynessStatus status = transact(0, DYNESS_CMD_PROTOCOL, nullptr, 0, response, respLen);
    if (status == DYNESS_OK && respLen >= 1) {
        version = response[0];
        return DYNESS_OK;
    }
    
    return status;
}

DynessStatus DynessBMS::getManufacturerInfo(BatteryState& state) {
    uint8_t response[256];
    uint16_t respLen = sizeof(response);
    
    DynessStatus status = transact(0, DYNESS_CMD_MANUF, nullptr, 0, response, respLen);
    if (status == DYNESS_OK) {
        parseManufacturerInfo(response, respLen, state);
        return DYNESS_OK;
    }
    
    return status;
}

DynessStatus DynessBMS::getSystemParameters(uint8_t devId, BatteryState& state) {
    uint8_t cmdInfo[1] = { devId };
    uint8_t response[256];
    uint16_t respLen = sizeof(response);
    
    DynessStatus status = transact(devId, DYNESS_CMD_SYSPARAM, cmdInfo, 1, response, respLen);
    if (status == DYNESS_OK) {
        parseSystemParameters(response, respLen, state);
        return DYNESS_OK;
    }
    
    return status;
}

DynessStatus DynessBMS::getManagementInfo(uint8_t devId, BatteryState& state) {
    uint8_t cmdInfo[1] = { devId };
    uint8_t response[256];
    uint16_t respLen = sizeof(response);
    
    DynessStatus status = transact(devId, DYNESS_CMD_CHG_MGMT, cmdInfo, 1, response, respLen);
    if (status == DYNESS_OK) {
        parseManagementInfo(response, respLen, state);
        return DYNESS_OK;
    }
    
    return status;
}

DynessStatus DynessBMS::getModuleSerialNumber(uint8_t devId, BatteryState& state) {
    uint8_t cmdInfo[1] = { devId };
    uint8_t response[256];
    uint16_t respLen = sizeof(response);
    
    DynessStatus status = transact(devId, DYNESS_CMD_SN, cmdInfo, 1, response, respLen);
    if (status == DYNESS_OK) {
        parseModuleSerialNumber(response, respLen, state);
        return DYNESS_OK;
    }
    
    return status;
}

DynessStatus DynessBMS::getValues(BatteryState& state) {
    uint8_t cmdInfo[2] = { 'F', 'F' };
    uint8_t response[512];
    uint16_t respLen = sizeof(response);
    
    DynessStatus status = transact(0x02, DYNESS_CMD_GET_VALUES, cmdInfo, 2, response, respLen);
    if (status == DYNESS_OK) {
        parseGetValuesSingle(response, respLen, state);
        return DYNESS_OK;
    }
    
    return status;
}

DynessStatus DynessBMS::getValuesSingle(uint8_t devId, BatteryState& state) {
    uint8_t cmdInfo[1] = { devId };
    uint8_t response[512];
    uint16_t respLen = sizeof(response);
    
    DynessStatus status = transact(devId, DYNESS_CMD_GET_VALUES, cmdInfo, 1, response, respLen);
    if (status == DYNESS_OK) {
        parseGetValuesSingle(response, respLen, state);
        return DYNESS_OK;
    }
    
    return status;
}

DynessStatus DynessBMS::scanForBatteries(uint8_t start, uint8_t end) {
    _count = 0;
    
    for (uint8_t adr = start; adr <= end; adr++) {
        BatteryState state;
        state.address = adr;
        
        DynessStatus status = getModuleSerialNumber(adr, state);
        if (status == DYNESS_OK && state.serial[0] != 0) {
            if (_count < _maxBatteries) {
                _batteries[_count] = state;
                _count++;
            }
            Serial.printf("Found battery at address 0x%02X with serial %s\n", adr, state.serial);
        }
        delay(50);
    }
    
    return DYNESS_OK;
}

DynessStatus DynessBMS::poll(uint8_t startAddr, uint8_t endAddr) {
    _count = 0;
    
    for (uint8_t adr = startAddr; adr <= endAddr; adr++) {
        BatteryState state;
        state.address = adr;
        
        DynessStatus status = getValuesSingle(adr, state);
        if (status != DYNESS_OK) continue;
        
        getModuleSerialNumber(adr, state);
        getManagementInfo(adr, state);
        
        if (_count < _maxBatteries) {
            _batteries[_count] = state;
            _count++;
        }
        delay(50);
    }
    
    return DYNESS_OK;
}

// ================= GETTERS =================
uint8_t DynessBMS::getBatteryCount() const {
    return _count;
}

const BatteryState* DynessBMS::getBattery(uint8_t i) const {
    if (i >= _count) return nullptr;
    return &_batteries[i];
}