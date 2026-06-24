#include "DynessBMS.h"

DynessBMS bms(Serial2, 17);  // Use Serial2, DE pin GPIO17

void setup() {
  Serial.begin(115200);
  bms.begin(115200);

  Serial.println("========================================");
  Serial.println("Pylontech/Dyness Battery Monitor");
  Serial.println("========================================");
  delay(2000);
}

void printCellVoltages(const BatteryState& bat) {
  Serial.println("\n--- Cell Voltages ---");
  for (int c = 0; c < bat.module.numberOfCells; c++) {
    Serial.printf("  Cell %2d: %.3f V", c + 1, bat.module.cellVoltages[c]);

    // Add visual bar
    float voltage = bat.module.cellVoltages[c];
    int barLength = (voltage - 2.8) * 50;  // 2.8V to 3.6V range
    if (barLength < 0) barLength = 0;
    if (barLength > 40) barLength = 40;

    Serial.print(" [");
    for (int i = 0; i < barLength; i++) Serial.print("=");
    for (int i = barLength; i < 40; i++) Serial.print(" ");
    Serial.print("]");

    // Show status
    if (voltage > 3.55) Serial.print(" HIGH!");
    else if (voltage < 3.0) Serial.print(" LOW!");
    else Serial.print(" OK");

    Serial.println();
  }
}

void printTemperatures(const BatteryState& bat) {
  Serial.println("\n--- Temperatures ---");
  Serial.printf("  BMS Board: %.1f°C\n", bat.module.averageBMSTemperature);

  for (int t = 0; t < bat.module.numberOfTemperatures - 1; t++) {
    Serial.printf("  Cell Group %d: %.1f°C\n", t + 1, bat.module.groupedCellsTemperatures[t]);
  }
}

void printAlarmStatus(const BatteryState& bat) {
  Serial.println("\n--- Alarm Status ---");
  Serial.printf("  Module Over Voltage:  %s\n", bat.alarmModuleOV ? "ALARM!" : "OK");
  Serial.printf("  Module Under Voltage: %s\n", bat.alarmModuleUV ? "ALARM!" : "OK");
  Serial.printf("  Cell Over Voltage:    %s\n", bat.alarmCellOV ? "ALARM!" : "OK");
  Serial.printf("  Cell Under Voltage:   %s\n", bat.alarmCellUV ? "ALARM!" : "OK");
  Serial.printf("  Charge Over Current:  %s\n", bat.alarmCOC ? "ALARM!" : "OK");
  Serial.printf("  Discharge Over Curr:  %s\n", bat.alarmDOC ? "ALARM!" : "OK");
  Serial.printf("  Charge Over Temp:     %s\n", bat.alarmChargeOT ? "ALARM!" : "OK");
  Serial.printf("  Discharge Over Temp:  %s\n", bat.alarmDischargeOT ? "ALARM!" : "OK");
}

void printCellErrors(const BatteryState& bat) {
  bool hasError = false;
  for (int c = 0; c < bat.module.numberOfCells; c++) {
    if (bat.cellError[c]) {
      if (!hasError) {
        Serial.println("\n--- Cell Errors ---");
        hasError = true;
      }
      Serial.printf("  Cell %d: ERROR!\n", c + 1);
    }
  }
  if (!hasError) {
    Serial.println("\n--- Cell Errors: None ---");
  }
}

void printManagementStatus(const BatteryState& bat) {
  Serial.println("\n--- Management Status ---");
  Serial.printf("  Charge Enabled:     %s\n", bat.chargeEnable ? "YES" : "NO");
  Serial.printf("  Discharge Enabled:  %s\n", bat.dischargeEnable ? "YES" : "NO");
  Serial.printf("  Charge MOSFET:      %s\n", bat.chargeMOS ? "ON" : "OFF");
  Serial.printf("  Discharge MOSFET:   %s\n", bat.dischargeMOS ? "ON" : "OFF");
  Serial.printf("  Using Batt Power:   %s\n", bat.usingBatteryPower ? "YES" : "NO");
  Serial.printf("  Fully Charged:      %s\n", bat.fullyCharged ? "YES" : "NO");
  Serial.printf("  Charge Immediately1: %s\n", bat.chargeImmediately1 ? "YES" : "NO");
  Serial.printf("  Charge Immediately2: %s\n", bat.chargeImmediately2 ? "YES" : "NO");
  Serial.printf("  Full Charge Req:    %s\n", bat.fullChargeRequest ? "YES" : "NO");
}

void printBatterySummary(const BatteryState& bat, int index) {
  Serial.println("\n========================================");
  Serial.printf("BATTERY %d (Address: 0x%02X)\n", index + 1, bat.address);
  Serial.println("========================================");

  // Basic info
  Serial.printf("\n--- Basic Information ---\n");
  Serial.printf("  Serial Number:      %s\n", bat.serial);
  Serial.printf("  Device Name:        %s\n", bat.deviceName);
  Serial.printf("  Manufacturer:       %s\n", bat.manufacturerName);
  Serial.printf("  Software Version:   %s\n", bat.softwareVersion);
  Serial.printf("  Number of Cells:    %d\n", bat.module.numberOfCells);

  // Electrical measurements
  Serial.println("\n--- Electrical Measurements ---");
  Serial.printf("  Total Voltage:      %.2f V\n", bat.module.voltage);
  Serial.printf("  Total Current:      %.2f A (%s)\n",
                abs(bat.module.current),
                bat.module.current >= 0 ? "CHARGING" : "DISCHARGING");
  Serial.printf("  Power:              %.2f W\n", bat.module.power);

  // Capacity
  Serial.println("\n--- Capacity Information ---");
  Serial.printf("  Remaining Capacity: %.1f Ah\n", bat.module.remainingCapacity);
  Serial.printf("  Total Capacity:     %.1f Ah\n", bat.module.totalCapacity);
  Serial.printf("  State of Charge:    %.1f %%\n", bat.soc);
  Serial.printf("  Cycle Count:        %d\n", bat.module.cycleNumber);

  // Cell voltages (all up to 16)
  printCellVoltages(bat);

  // Temperatures
  printTemperatures(bat);

  // Alarms
  printAlarmStatus(bat);

  // Cell errors
  printCellErrors(bat);

  // Management
  printManagementStatus(bat);

  // Limits
  Serial.println("\n--- System Limits ---");
  Serial.printf("  Cell High Voltage Limit:  %.3f V\n", bat.cellHighVoltageLimit);
  Serial.printf("  Cell Low Voltage Limit:   %.3f V\n", bat.cellLowVoltageLimit);
  Serial.printf("  Cell Under Voltage Limit: %.3f V\n", bat.cellUnderVoltageLimit);
  Serial.printf("  Charge Current Limit:     %.1f A\n", bat.chargeCurrentLimit);
  Serial.printf("  Discharge Current Limit:  %.1f A\n", bat.dischargeCurrentLimit);
  Serial.printf("  Charge Temp Limit:        %.1f °C\n", bat.chargeHighTemperatureLimit);
  Serial.printf("  Discharge Temp Limit:     %.1f °C\n", bat.dischargeHighTemperatureLimit);
}

void printCSVHeader() {
  Serial.println("\n--- CSV Format (for logging) ---");
  Serial.print("Time,Address,Voltage,Current,Power,SOC,RemainingCap,TotalCap,Cycles,BMS_Temp");
  for (int c = 0; c < 16; c++) {
    Serial.printf(",Cell%d_V", c + 1);
  }
  for (int t = 0; t < 4; t++) {
    Serial.printf(",TempGroup%d", t + 1);
  }
  Serial.println(",ChargeMOS,DischargeMOS");
}

void printCSVData(const BatteryState& bat, unsigned long time) {
  Serial.printf("%lu,0x%02X,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%d,%.1f",
                time, bat.address, bat.module.voltage, bat.module.current,
                bat.module.power, bat.soc,
                bat.module.remainingCapacity, bat.module.totalCapacity,
                bat.module.cycleNumber, bat.module.averageBMSTemperature);

  for (int c = 0; c < 16; c++) {
    if (c < bat.module.numberOfCells) {
      Serial.printf(",%.3f", bat.module.cellVoltages[c]);
    } else {
      Serial.print(",0");
    }
  }

  for (int t = 0; t < 4; t++) {
    if (t < bat.module.numberOfTemperatures - 1) {
      Serial.printf(",%.1f", bat.module.groupedCellsTemperatures[t]);
    } else {
      Serial.print(",0");
    }
  }

  Serial.printf(",%d,%d\n", bat.chargeMOS ? 1 : 0, bat.dischargeMOS ? 1 : 0);
}

void findMinMaxCells(const BatteryState& bat) {
  if (bat.module.numberOfCells == 0) return;

  int minCell = 0, maxCell = 0;
  float minVoltage = bat.module.cellVoltages[0];
  float maxVoltage = bat.module.cellVoltages[0];
  float sumVoltage = 0;

  for (int c = 0; c < bat.module.numberOfCells; c++) {
    float v = bat.module.cellVoltages[c];
    sumVoltage += v;
    if (v < minVoltage) {
      minVoltage = v;
      minCell = c;
    }
    if (v > maxVoltage) {
      maxVoltage = v;
      maxCell = c;
    }
  }

  float avgVoltage = sumVoltage / bat.module.numberOfCells;
  float delta = maxVoltage - minVoltage;

  Serial.println("\n--- Cell Balance Analysis ---");
  Serial.printf("  Average Cell Voltage: %.3f V\n", avgVoltage);
  Serial.printf("  Highest Cell: Cell %d at %.3f V\n", maxCell + 1, maxVoltage);
  Serial.printf("  Lowest Cell:  Cell %d at %.3f V\n", minCell + 1, minVoltage);
  Serial.printf("  Delta:        %.3f V\n", delta);

  if (delta > 0.1) {
    Serial.println("  WARNING: Cells are unbalanced! (>0.1V difference)");
  } else if (delta > 0.05) {
    Serial.println("  CAUTION: Cells showing some imbalance");
  } else {
    Serial.println("  GOOD: Cells are well balanced");
  }
}

void loop() {
  static uint32_t lastPoll = 0;
  static bool headerPrinted = false;

  // Poll every 5 seconds
  if (millis() - lastPoll > 5000) {
    Serial.println("\n\n========================================");
    Serial.println("POLLING BATTERIES...");
    Serial.println("========================================\n");

    DynessStatus status = bms.poll(0x02, 0x10);

    if (status == DYNESS_OK) {
      uint8_t count = bms.getBatteryCount();
      Serial.printf("Found %d battery(ies)\n", count);

      for (int i = 0; i < count; i++) {
        const BatteryState* bat = bms.getBattery(i);
        if (bat) {
          // Print complete details for each battery
          printBatterySummary(*bat, i);
          findMinMaxCells(*bat);

          // Print CSV format for data logging
          if (!headerPrinted) {
            printCSVHeader();
            headerPrinted = true;
          }
          printCSVData(*bat, millis());
        }
      }
    } else {
      Serial.println("Failed to poll batteries!");
      Serial.printf("Status: %d\n", status);
    }

    lastPoll = millis();
  }

  delay(100);
}