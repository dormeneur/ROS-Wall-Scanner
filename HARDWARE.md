# Wall Scanner Hardware Guide

This document provides comprehensive hardware specifications, assembly instructions, and integration guidelines for the ROS Wall Scanner project.

## Table of Contents
- [System Overview](#system-overview)
- [Hardware Requirements](#hardware-requirements)
- [Component Specifications](#component-specifications)
- [Circuit Design](#circuit-design)
- [Assembly Instructions](#assembly-instructions)
- [Calibration Procedures](#calibration-procedures)
- [Troubleshooting](#troubleshooting)
- [3D Printing Files](#3d-printing-files)

---

## System Overview

The Wall Scanner uses two complementary detection methods:
1. **Electromagnetic Detection** - For live electrical wires
2. **Capacitive Sensing** - For pipes, studs, and non-live objects

### Detection Capabilities

| Object Type | Detection Method | Range | Accuracy |
|-------------|------------------|--------|----------|
| Live Wires | Electromagnetic | 5-50cm | ±2cm |
| Water Pipes | Capacitive | 2-10cm | ±1cm |
| Wall Studs | Capacitive | 1-5cm | ±1cm |
| Non-live Wires | Capacitive (complex) | 2-8cm | ±2cm |

---

## Hardware Requirements

### Essential Components

#### 1. Electromagnetic Detection System
- **Inductive Coil:** 500-1000 turns, 10-20mm diameter
- **Ferrite Core:** Soft iron core, high permeability (μ > 2000)
- **Amplifier:** Low-noise operational amplifier (LM358 or better)
- **Frequency Filter:** 50/60 Hz bandpass for AC detection

#### 2. Capacitive Sensing Array
- **Capacitive Sensors:** 3x metal plates or custom PCB patterns
- **Capacitance-to-Digital Converter:** Texas Instruments FDC1004 or similar
- **Sensor Spacing:** 20mm between sensors for optimal coverage

#### 3. Core Processing Unit
- **Microcontroller:** Arduino Uno/Nano or ESP32
- **ADC Resolution:** Minimum 12-bit for accurate readings
- **Serial Interface:** USB or UART for ROS communication
- **Power Supply:** 5V/3.3V regulated supply

#### 4. Mechanical Components
- **Enclosure:** 3D printed housing (STL files provided)
- **Scanning Mechanism:** Manual sliding or motorized movement
- **Display:** Optional LCD for standalone operation

---

## Component Specifications

### Electromagnetic Sensor

#### Inductive Coil Specifications
```yaml
Wire Type: Enameled copper wire (28-32 AWG)
Turns: 800-1000 turns
Core Material: Ferrite (soft iron)
Core Dimensions: 
  - Length: 50-100mm
  - Diameter: 10-15mm
Inductance: 10-50 mH
Resistance: 50-200 Ω
Operating Frequency: 50/60 Hz (AC mains)
Sensitivity: 0.1 μT minimum detectable field
```

#### Amplifier Circuit
```yaml
Op-Amp: LM358 or LM324 (low noise)
Gain: 100-1000x (adjustable)
Bandwidth: 10 Hz - 1 kHz
Input Impedance: >1 MΩ
Output Range: 0-5V (Arduino compatible)
Power Supply: ±9V or single 9V supply
```

### Capacitive Sensor Array

#### FDC1004 Specifications
```yaml
Resolution: 24-bit
Channels: 4 (using 3 for sensor array)
Interface: I2C (400 kHz max)
Supply Voltage: 3.3V
Sampling Rate: Up to 13.3 kSPS
Capacitance Range: ±15 pF
Resolution: 0.5 fF
Operating Temperature: -40°C to +125°C
```

#### Sensor Plate Design
```yaml
Material: Copper PCB or metal plates
Dimensions: 15mm x 15mm per sensor
Thickness: 1.6mm (standard PCB)
Spacing: 20mm center-to-center
Guard Ring: Recommended for isolation
Connection: Coaxial cable or shielded wire
```

---

## Circuit Design

### Electromagnetic Sensor Circuit

```
                    +9V
                     |
                    R1 (10kΩ)
                     |
Inductive    C1     |     R2
Coil -------||----- +------ (100kΩ) ---- To Arduino A0
(L1)       100nF    |                |
                    |               C2
                   GND            100nF
                                   |
                                  GND

Components:
- L1: Inductive coil (800 turns, ferrite core)
- C1: AC coupling capacitor (100nF)
- R1: Pull-up resistor (10kΩ)
- R2: Input protection (100kΩ)
- C2: Anti-aliasing filter (100nF)
```

### Capacitive Sensor Circuit

```
FDC1004 Breakout Board:
VCC ---- 3.3V
GND ---- GND
SDA ---- Arduino A4 (I2C Data)
SCL ---- Arduino A5 (I2C Clock)
CIN1 --- Sensor Plate 1
CIN2 --- Sensor Plate 2  
CIN3 --- Sensor Plate 3
CIN4 --- (Unused)

Sensor Plates:
Material: Copper PCB
Size: 15mm x 15mm
Connection: Shielded wire to FDC1004
```

### Power Supply Circuit

```yaml
Input: 9V DC (wall adapter or battery)
Regulation:
  - 5V for Arduino: LM7805 voltage regulator
  - 3.3V for FDC1004: LM3.3V or Arduino 3.3V pin
Filtering: 
  - 1000μF electrolytic capacitor (input)
  - 100μF electrolytic capacitor (output)
  - 0.1μF ceramic capacitor (high frequency)
Current Rating: Minimum 500mA
```

---

## Assembly Instructions

### Step 1: Electromagnetic Sensor Assembly

1. **Wind the Inductive Coil**
   - Use 30 AWG enameled copper wire
   - Wind 800-1000 turns on ferrite core
   - Secure with tape or heat-shrink tubing
   - Test inductance: should be 20-40 mH

2. **Build Amplifier Circuit**
   - Assemble on breadboard or perfboard
   - Use LM358 op-amp in non-inverting configuration
   - Set gain to 100x initially (adjust during calibration)
   - Add input protection and filtering

3. **Connect to Arduino**
   - Coil output → Amplifier input
   - Amplifier output → Arduino A0
   - Power connections: +9V and GND

### Step 2: Capacitive Sensor Assembly

1. **Prepare Sensor Plates**
   - Cut 3x copper plates (15mm x 15mm)
   - Sand surface for good conductivity
   - Solder shielded wire to each plate
   - Mount on insulating substrate with 20mm spacing

2. **Connect FDC1004 Module**
   - Wire I2C connections (SDA, SCL)
   - Connect sensor plates to CIN1, CIN2, CIN3
   - Power with 3.3V regulated supply

3. **Integration with Arduino**
   - Connect I2C bus to Arduino
   - Upload test firmware to verify operation

### Step 3: Microcontroller Programming

#### Arduino Code Structure
```cpp
#include <Wire.h>
#include <FDC1004.h>

// Pin definitions
#define EM_SENSOR_PIN A0
#define STATUS_LED_PIN 13

// Sensor objects
FDC1004 capacitiveSensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  capacitiveSensor.begin();
  
  // Initialize sensors
  calibrateSensors();
}

void loop() {
  // Read EM sensor
  float emValue = readEMSensor();
  
  // Read capacitive sensors
  float cap1 = capacitiveSensor.getCapacitance(0);
  float cap2 = capacitiveSensor.getCapacitance(1);
  float cap3 = capacitiveSensor.getCapacitance(2);
  
  // Send data to ROS
  Serial.print("EM:");
  Serial.print(emValue);
  Serial.print(",CAP:");
  Serial.print(cap1);
  Serial.print(",");
  Serial.print(cap2);
  Serial.print(",");
  Serial.print(cap3);
  Serial.print(",POS:0.0");
  Serial.println();
  
  delay(100); // 10Hz sampling
}
```

### Step 4: Mechanical Assembly

1. **3D Print Enclosure**
   - Download STL files (see [3D Printing Files](#3d-printing-files))
   - Print with PLA or ABS plastic
   - Use 0.2mm layer height for good surface finish

2. **Mount Components**
   - Install Arduino in main compartment
   - Mount sensor assemblies in front panel
   - Route cables through designated channels
   - Secure with screws and standoffs

3. **Final Assembly**
   - Connect all electrical connections
   - Close enclosure and secure
   - Test all functions before use

---

## Calibration Procedures

### Electromagnetic Sensor Calibration

1. **Baseline Calibration**
   ```cpp
   // Place scanner away from any electrical sources
   // Record 100 samples for baseline
   float baseline = 0;
   for(int i = 0; i < 100; i++) {
     baseline += analogRead(EM_SENSOR_PIN);
     delay(10);
   }
   baseline = baseline / 100.0;
   ```

2. **Sensitivity Adjustment**
   - Test with known live wire (120V/240V AC)
   - Adjust amplifier gain for 5-50V output range
   - Verify detection range: should detect at 20-50cm

3. **Noise Filtering**
   - Implement digital low-pass filter
   - Remove 50/60 Hz interference if not detecting AC
   - Test in various electromagnetic environments

### Capacitive Sensor Calibration

1. **FDC1004 Initialization**
   ```cpp
   void calibrateCapacitive() {
     // Configure FDC1004 for maximum sensitivity
     capacitiveSensor.configureMeasurements(FDC1004_100HZ);
     
     // Record baseline readings
     for(int sensor = 0; sensor < 3; sensor++) {
       float baseline = 0;
       for(int i = 0; i < 50; i++) {
         baseline += capacitiveSensor.getCapacitance(sensor);
         delay(20);
       }
       baselineCapacitance[sensor] = baseline / 50.0;
     }
   }
   ```

2. **Object Detection Thresholds**
   - Test with known materials (wood, metal, plastic)
   - Set thresholds based on material properties:
     - **Wood studs:** 8-12 pF change
     - **Metal pipes:** 15-25 pF change
     - **Plastic pipes:** 5-10 pF change

3. **Environmental Compensation**
   - Account for temperature drift
   - Humidity compensation if needed
   - Periodic re-calibration during operation

---

## Troubleshooting

### Common Hardware Issues

#### Electromagnetic Sensor Problems

| Problem | Cause | Solution |
|---------|--------|----------|
| No EM detection | Broken coil | Check coil continuity with multimeter |
| False positives | EMI interference | Add ferrite beads, improve shielding |
| Low sensitivity | Wrong amplifier gain | Adjust gain potentiometer |
| Noisy readings | Poor grounding | Improve ground connections |

#### Capacitive Sensor Problems

| Problem | Cause | Solution |
|---------|--------|----------|
| No I2C communication | Wiring error | Check SDA/SCL connections |
| Erratic readings | Poor sensor coupling | Ensure good electrical contact |
| Low sensitivity | Wrong FDC1004 config | Adjust measurement settings |
| Temperature drift | No compensation | Implement temperature correction |

### Diagnostic Procedures

1. **Visual Inspection**
   - Check all connections
   - Look for damaged components
   - Verify power supply voltages

2. **Electrical Testing**
   - Measure supply voltages (5V, 3.3V)
   - Test continuity of all connections
   - Check I2C communication with oscilloscope

3. **Software Diagnostics**
   ```cpp
   void diagnosticMode() {
     Serial.println("=== Hardware Diagnostic ===");
     
     // Test EM sensor
     Serial.print("EM Sensor (raw ADC): ");
     Serial.println(analogRead(EM_SENSOR_PIN));
     
     // Test capacitive sensors
     for(int i = 0; i < 3; i++) {
       Serial.print("Cap Sensor ");
       Serial.print(i);
       Serial.print(": ");
       Serial.println(capacitiveSensor.getCapacitance(i));
     }
     
     // Test I2C communication
     Serial.print("I2C Status: ");
     if(capacitiveSensor.isConnected()) {
       Serial.println("OK");
     } else {
       Serial.println("FAILED");
     }
   }
   ```

---

## 3D Printing Files

### STL Files Available

Contact **Sedaxis** near C Block, Special Teams Garage for STL files:
- **WhatsApp:** +91-8925821153 
- **Hours:** 10 AM to 2 PM
- **Location:** Near C Block, Special Teams Garage

### Printable Components

#### Main Enclosure (`wall_scanner_body.stl`)
```yaml
Dimensions: 150mm x 80mm x 60mm
Material: PLA or ABS plastic
Layer Height: 0.2mm
Infill: 20%
Support: Required for overhangs
Print Time: ~4 hours
```

#### Sensor Housing (`sensor_array.stl`)
```yaml
Dimensions: 100mm x 40mm x 20mm
Purpose: Houses capacitive sensor array
Material: PLA (preferred for dimensional stability)
Layer Height: 0.15mm for precision
Infill: 15%
Support: Minimal
```

#### EM Coil Mount (`coil_mount.stl`)
```yaml
Dimensions: 60mm x 60mm x 30mm
Purpose: Secure mounting for inductive coil
Material: ABS (heat resistance)
Layer Height: 0.2mm
Infill: 30% for strength
```

#### Handle Assembly (`handle.stl`)
```yaml
Dimensions: 120mm x 25mm x 25mm
Purpose: Ergonomic grip for manual scanning
Material: PLA or PETG
Layer Height: 0.2mm
Infill: 25%
Post-processing: Light sanding for smooth finish
```

### Assembly Hardware

**Required Screws and Hardware:**
- 8x M3 x 12mm screws (enclosure assembly)
- 4x M2.5 x 8mm screws (PCB mounting)
- 6x M3 heat-set inserts (threaded inserts for plastic)
- 2x Cable glands (6mm diameter)
- 1x Power switch (6mm momentary)

### Print Settings Recommendations

```yaml
Printer Type: FDM (Fused Deposition Modeling)
Nozzle Diameter: 0.4mm
Layer Height: 0.15-0.2mm
Print Speed: 40-60 mm/s
Bed Temperature: 60°C (PLA), 80°C (ABS)
Extruder Temperature: 200°C (PLA), 240°C (ABS)
Cooling: Enabled for PLA, minimal for ABS
```

---

## Hardware Integration Checklist

### Pre-Assembly Verification
- [ ] All components received and inspected
- [ ] STL files downloaded and printed
- [ ] Arduino IDE installed with required libraries
- [ ] Multimeter available for testing
- [ ] Soldering equipment ready

### Assembly Checklist
- [ ] Inductive coil wound and tested
- [ ] Amplifier circuit built and verified
- [ ] Capacitive sensors mounted with proper spacing
- [ ] FDC1004 module connected and tested
- [ ] Arduino programmed with test firmware
- [ ] All connections secured and insulated
- [ ] Power supply tested and regulated
- [ ] Enclosure assembled with all components

### Testing Checklist
- [ ] Power-on test (check voltages)
- [ ] Serial communication with computer
- [ ] Electromagnetic sensor responds to AC sources
- [ ] Capacitive sensors detect objects
- [ ] ROS integration functional
- [ ] Calibration procedures completed
- [ ] Detection accuracy verified

### Final Integration
- [ ] ROS nodes communicate with hardware
- [ ] Real-time data streaming working
- [ ] Detection thresholds properly calibrated
- [ ] Visualization in RViz functional
- [ ] Documentation updated with actual specifications
- [ ] System ready for field testing

This hardware guide provides everything needed to build a functional wall scanner that integrates seamlessly with the ROS software stack.