// ============================================================
//  I2C Scanner — temporary test program
//
//  Flash this instead of the full flight controller to debug
//  your wiring. It scans every possible I2C address and tells
//  you exactly what chips it can see.
//
//  Expected output when wired correctly:
//    Found device at address 0x68  ← MPU6050
//    Found device at address 0x76  ← BME280
//
//  To use:
//    1. Comment out your normal main() or setup()/loop()
//    2. Flash this file
//    3. Open serial monitor at 115200 baud
//    4. Fix wiring until both addresses appear
//    5. Swap back to your real code and reflash
// ============================================================

#ifdef ON_REAL_HARDWARE
#include <Arduino.h>
#include <Wire.h>

void setup() {
    Serial.begin(115200);
    delay(2000);  // Give serial monitor time to connect

    Serial.println("\n========================================");
    Serial.println("  I2C Scanner — looking for your chips");
    Serial.println("========================================\n");

    // Initialize I2C on GPIO 8 (SDA) and GPIO 9 (SCL)
    Wire.begin(15, 16);
    Wire.setClock(100000);  // Slow speed for debugging — more forgiving with bad connections
    delay(100);

    Serial.println("Scanning I2C bus (GPIO 8=SDA, GPIO 9=SCL)...\n");

    int found = 0;

    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("  ✓ Found device at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);

            // Tell the user what chip this likely is
            if (address == 0x68) Serial.print("  ← MPU6050 (your IMU) — AD0 tied to GND ✓");
            if (address == 0x69) Serial.print("  ← MPU6050 (your IMU) — but AD0 is HIGH, should be GND");
            if (address == 0x76) Serial.print("  ← BME280 (your barometer) — SDO tied to GND ✓");
            if (address == 0x77) Serial.print("  ← BME280 (your barometer) — but SDO is HIGH, should be GND");

            Serial.println();
            found++;
        }
    }

    Serial.println();

    if (found == 0) {
        Serial.println("✗ No I2C devices found at all.");
        Serial.println();
        Serial.println("Most likely causes:");
        Serial.println("  1. SDA and SCL wires swapped — try swapping GPIO 8 and GPIO 9");
        Serial.println("  2. VCC not connected — check red wire to 3.3V rail");
        Serial.println("  3. GND not connected — check black wire to GND rail");
        Serial.println("  4. Poor pin contact — press down on chip and check again");
        Serial.println("  5. Pins not soldered — temporary contact may be too unreliable");
    } else if (found == 1) {
        Wire.beginTransmission(0x68);
        if (Wire.endTransmission() != 0)
            Serial.println("  Missing: MPU6050 (expected at 0x68)");

        Wire.beginTransmission(0x76);
        if (Wire.endTransmission() != 0)
            Serial.println("  Missing: BME280 (expected at 0x76)");
    } else {
        Serial.println("All expected devices found! Wiring is correct.");
        Serial.println("You can now flash your real flight controller code.");
    }

    Serial.println("\n========================================");
    Serial.println("Scan complete. Rescanning in 5 seconds...");
    Serial.println("========================================");
}

void loop() {
    // Rescan every 5 seconds so you can fix wiring while watching
    delay(5000);

    Serial.println("\nRescanning...");
    int found = 0;

    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("  ✓ 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            if (address == 0x68) Serial.print(" MPU6050 ✓");
            if (address == 0x69) Serial.print(" MPU6050 (AD0 wrong — tie to GND)");
            if (address == 0x76) Serial.print(" BME280 ✓");
            if (address == 0x77) Serial.print(" BME280 (SDO wrong — tie to GND)");
            Serial.println();
            found++;
        }
    }

    if (found == 0) Serial.println("  ✗ Nothing found — check wiring");
    if (found == 2) Serial.println("  All good!");
}

#else

// Native build — this file does nothing on Mac
int main() { return 0; }

#endif