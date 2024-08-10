# Home Automation Project

## Proximity-Based Control System

### Overview

This project uses an `ESP32 microcontroller` with an `ultrasonic sensor (HC-SR04)` and an `IR sensor` for home automation. It activates an LED when motion is detected and the ultrasonic sensor measures a distance less than `50cm`. The system provides real-time proximity-based control without relying on a relay module, enhancing efficiency and responsiveness in household applications.

### Components
- ESP32
- HC-SR04 Ultrasonic Sensor
- IR Sensor
- LED
- Connection Wires
### Connections
- **HC-SR04:**
  - VCC to 5V
  - GND to GND
  - Trig to GPIO `33`
  - Echo to GPIO `32`
- **IR Sensor:**
  - VCC to 3.3V
  - GND to GND
  - Signal to GPIO `26`
- **LED:**
  - Anode to GPIO `27`
  - Cathode to GND (with a current limiting resistor if necessary)

### Features
- Measures distance using the HC-SR04 ultrasonic sensor.
- Detects objects using the IR sensor.
- Controls an LED based on the proximity and IR detection.

### Prerequisites
- ESP-IDF development environment set up. Follow the [ESP-IDF Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/) to install and configure ESP-IDF.

### Installation
1. Clone this repository:
    ```sh
    git clone  https://github.com/Saeed-dev2/Home_Automation
    cd [repository_name]
    ```
2. Configure the project:
    ```sh
    idf.py menuconfig
    ```
3. Build the project:
    ```sh
    idf.py build
    ```
4. Flash the project to your ESP32:
    ```sh
    idf.py flash
    idf.py monitor
    ```

### Usage
1. Connect the components as described in the connections section.
2. Power on the ESP32.
3. The LED will turn on if the ultrasonic sensor detects an object within 50 cm or if the IR sensor detects an object.

`You can Change the Distance According your   Requirements`
### Author
`M.Saeed`

### Acknowledgements
Special thanks to the ESP-IDF and FreeRTOS communities for their excellent resources and support.
