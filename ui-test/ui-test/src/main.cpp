#include <Arduino.h>
#include <Wire.h>

#define DEFAULT_I2C_ADDRESS 0xb1

static uint8_t i2cAddress = DEFAULT_I2C_ADDRESS;


static void sendI2C(uint8_t reg, int32_t value, bool verbose = true) {
  Wire.beginTransmission(i2cAddress);
  Wire.write(reg);
  Wire.write((value >> 24) & 0xFF);
  Wire.write((value >> 16) & 0xFF);
  Wire.write((value >>  8) & 0xFF);
  Wire.write( value        & 0xFF);
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    if (verbose) {
      Serial.print("OK  addr=0x");
      Serial.print(i2cAddress, HEX);
      Serial.print(" reg=");
      Serial.print(reg);
      Serial.print(" value=");
      Serial.println(value);
    }
  } else {
    Serial.print("ERR Wire.endTransmission returned ");
    Serial.println(err);
  }
}

static void printHelp() {
  Serial.println("Commands:");
  Serial.println("  <reg> <value>   send 32-bit value to register  e.g. \"0 1234\"");
  Serial.println("  addr <address>  set I2C target address (dec)   e.g. \"addr 32\"");
  Serial.println("  addr            show current I2C address");
  Serial.println("  vu [increment]  simulate VU meters on reg 4 (type \"stop\" to quit)");
}

static void runVuSimulation(uint16_t step = 1) {
  Serial.print("VU simulation started (increment=");
  Serial.print(step);
  Serial.println("). Type \"stop\" to quit.");

  uint16_t upper = 0;
  uint16_t lower = 0;
  int8_t upperDir = 1;
  int8_t lowerDir = 1;

  static const uint16_t kMax = 0xFFFF;
  const uint16_t kStep = step;

  while (true) {
    // Check for "stop" command
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.equalsIgnoreCase("stop")) {
        Serial.println("VU simulation stopped.");
        return;
      }
    }

    // Advance upper and lower 16-bit counters independently
    if (upperDir > 0) {
      upper = (upper + kStep >= kMax) ? kMax : upper + kStep;
      if (upper == kMax) upperDir = -1;
    } else {
      upper = (upper < kStep) ? 0 : upper - kStep;
      if (upper == 0) upperDir = 1;
    }

    if (lowerDir > 0) {
      lower = ((uint32_t)lower + kStep * 2 >= kMax) ? kMax : lower + kStep * 2;
      if (lower == kMax) lowerDir = -1;
    } else {
      lower = (lower < kStep * 2) ? 0 : lower - kStep * 2;
      if (lower == 0) lowerDir = 1;
    }

    int32_t value = ((int32_t)upper << 16) | lower;
    sendI2C(4, value, false);

    delay(20);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB CDC on Teensy */ }
  Wire.begin();
  Serial.println("I2C protocol tester ready.");
  printHelp();
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("vu") || line.startsWith("vu ")) {
    uint16_t increment = 1;
    if (line.startsWith("vu ")) {
      long arg = line.substring(3).toInt();
      if (arg > 0 && arg <= 0xFFFF) {
        increment = (uint16_t)arg;
      } else {
        Serial.println("ERR increment must be 1-65535");
        return;
      }
    }
    runVuSimulation(increment);
    return;
  }

  if (line.equalsIgnoreCase("addr")) {
    Serial.print("I2C address = 0x");
    Serial.print(i2cAddress, HEX);
    Serial.print(" (");
    Serial.print(i2cAddress);
    Serial.println(")");
    return;
  }

  if (line.startsWith("addr ")) {
    long addr = line.substring(5).toInt();
    if (addr < 1 || addr > 127) {
      Serial.println("ERR address must be 1-127");
    } else {
      i2cAddress = (uint8_t)addr;
      Serial.print("I2C address set to 0x");
      Serial.print(i2cAddress, HEX);
      Serial.print(" (");
      Serial.print(i2cAddress);
      Serial.println(")");
    }
    return;
  }

  int sep = line.indexOf(' ');
  if (sep < 0) {
    printHelp();
    return;
  }

  long reg = line.substring(0, sep).toInt();
  long value = line.substring(sep + 1).toInt();

  if (reg < 0 || reg > 255) {
    Serial.println("ERR register must be 0-255");
    return;
  }

  sendI2C((uint8_t)reg, (int32_t)value);
}