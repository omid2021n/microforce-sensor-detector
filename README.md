# microforce-sensor-detector
I designed a MicroForce sensor detection and monitoring system. The software connects to the Arduino through a UART/serial port, automatically detects available COM ports, displays the sensor signal in real time, records the measurements to an Excel-compatible file, and can reset the Arduino board.
# MicroForce Sensor Detector

A PC-based force-sensor monitoring and data-logging system built around an **FMA MicroForce sensor** and an **Arduino Nano 33 IoT**.

The system reads the sensor data from the Arduino through a UART/serial connection and provides a Python desktop application for live monitoring, recording, and basic device control.

## Features

- Detects available COM/serial ports.
- Connects to the Arduino through a UART/USB serial connection.
- Reads live force-sensor measurements.
- Displays the sensor signal in real time.
- Shows device time and measured force.
- Records measurement data for later analysis in an Excel-compatible format.
- Provides Start, Stop, Clear, and Record controls.
- Provides a board-reset control.
- Displays the current measurement rate.
- Designed for an **FMA MicroForce sensor** with an **Arduino Nano 33 IoT**.

## System Architecture

```text
FMA MicroForce Sensor
        |
        | Sensor interface
        v
Arduino Nano 33 IoT
        |
        | USB / UART serial
        v
PC
        |
        v
Python GUI
  |          |
  |          +--> Live force plot
  |
  +----------> Data recording / Excel-compatible file
```

## Software

The PC application is written in **Python** and provides the graphical user interface, serial communication, live plotting, and data logging.

Suggested main Python file:

```text
python/microforce_sensor_gui.py
```

## Arduino Firmware

The Arduino firmware reads the FMA MicroForce sensor and sends the measured data to the PC through the serial interface.

Suggested firmware file:

```text
arduino/microforce_sensor.ino
```

> The actual Arduino firmware should be added to this repository. The filename above is only the recommended project name.

## PC Application

The Python application:

1. Detects available serial/COM ports.
2. Opens the selected serial port.
3. Receives sensor measurements from the Arduino.
4. Parses the incoming data.
5. Updates the live graph.
6. Calculates/displays the measurement rate.
7. Records the received measurements.
8. Provides controls for starting, stopping, clearing, and resetting the board.

Example serial configuration:

```text
Baud rate: 115200
```

## User Interface

![MicroForce Sensor Detector GUI](docs/images/Micro_force_sensor.png)

The GUI provides:

- **Port** – selected COM port.
- **Baud** – serial communication speed.
- **Refresh** – searches for available COM ports.
- **Start** – starts data acquisition.
- **Stop** – stops data acquisition.
- **Clear** – clears the displayed data.
- **Record** – records the received measurements.
- **Reset board** – resets the Arduino board.
- **Measured rate** – shows the received measurement rate.
- **Live plot** – displays force versus device time.

## Repository Structure

```text
microforce-sensor-detector/
│
├── README.md
│
├── arduino/
│   └── microforce_sensor.ino
│
├── python/
│   └── microforce_sensor_gui.py
│
└── docs/
    └── images/
        └── Micro_force_sensor.png
```

## Requirements

### Hardware

- FMA MicroForce sensor
- Arduino Nano 33 IoT
- USB connection between Arduino and PC
- PC running Windows/Linux as required by the Python application

### Python

Typical Python packages may include:

```text
pyserial
matplotlib
pandas
openpyxl
```

Install them with:

```bash
pip install pyserial matplotlib pandas openpyxl
```

Only install the packages actually imported by the final Python application.

## Running the Application

Clone the repository:

```bash
git clone https://github.com/<your-username>/microforce-sensor-detector.git
cd microforce-sensor-detector
```

Run the Python application:

```bash
python python/microforce_sensor_gui.py
```

Then:

1. Connect the Arduino Nano 33 IoT.
2. Click **Refresh**.
3. Select the Arduino COM port.
4. Select the correct baud rate.
5. Click **Start**.
6. Verify that force measurements are appearing on the graph.
7. Use **Record** to save the measurement data.

## Data Flow

The measurement path is:

```text
FMA MicroForce Sensor
        ↓
Arduino Nano 33 IoT
        ↓
Serial / UART
        ↓
Python application
        ↓
Data parsing
        ↓
Live plot + data logging
```

## Development Notes

The project is intended as a small embedded-to-PC measurement system. The Arduino is responsible for sensor acquisition and serial transmission, while the PC application handles visualization, recording, and user interaction.

For a production version, the serial data format should be documented explicitly. For example:

```text
<timestamp>,<force>
```

or:

```text
TIME=1.250,F=0.183
```

The exact format should match the Arduino firmware.

## Future Improvements

Possible improvements include:

- Sensor calibration and zero/tare function.
- Configurable sampling rate.
- Automatic file naming and timestamps.
- CSV and XLSX export options.
- Sensor calibration coefficients stored in a configuration file.
- Alarm limits for minimum/maximum force.
- Multiple-sensor support.
- Communication error detection.
- Automatic reconnection if the serial connection is lost.
- Measurement statistics such as minimum, maximum, average, and RMS force.

## Author

**Omid**

Electronics / Embedded Systems / FPGA Engineering

