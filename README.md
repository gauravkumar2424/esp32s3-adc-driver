ESP32-S3 ADC Driver for Voltage and Temperature Sensing
This project implements a robust, low-level Analog-to-Digital Converter (ADC) driver for the ESP32-S3 microcontroller using the ESP-IDF framework and FreeRTOS. It enables precise analog signal acquisition, voltage measurement, and temperature calculation using an NTC thermistor, making it suitable for IoT, sensor networks, and embedded systems applications. The driver is optimized for thread safety, error handling, and modularity, demonstrating advanced embedded programming skills.
Features

Low-Level ADC Control: Configures ESP32-S3 ADC1 with customizable channel, attenuation, and reference voltage.
Thread-Safe Operation: Uses FreeRTOS mutex for safe concurrent access in multi-task environments.
Voltage Measurement: Supports 0–3.3V range with configurable attenuation levels (0 dB to 11 dB).
Temperature Sensing: Calculates temperature using an NTC thermistor and the Steinhart-Hart model.
Robust Error Handling: Implements retries, timeouts, and detailed logging for reliable operation.
Modular Design: Separates ADC logic into reusable components for easy integration into larger systems.

Motivation
This project showcases expertise in embedded C, real-time systems, and hardware interfacing, targeting applications in IoT, environmental monitoring, and industrial automation. For PhD pursuits, it demonstrates a strong foundation in low-level hardware control and signal processing, ideal for research in embedded systems or sensor technology.
Hardware Requirements

ESP32-S3 development board (e.g., ESP32-S3-DevKitC-1)
NTC thermistor (10kΩ at 25°C, β=3950) paired with a 10kΩ resistor in a voltage divider
Jumper wires and breadboard for analog input on GPIO2
USB cable for programming and serial monitoring

Software Requirements

ESP-IDF (version 5.x recommended)
C compiler (included with ESP-IDF)
FreeRTOS (included with ESP-IDF)
Serial terminal (e.g., minicom or ESP-IDF’s idf.py monitor)

Installation

Clone the Repository:git clone https://github.com/<your-username>/esp32s3-adc-driver.git
cd esp32s3-adc-driver


Set Up ESP-IDF:source /path/to/esp-idf/export.sh


Build and Flash:idf.py build
idf.py -p /dev/ttyUSB0 flash monitor



Usage

The driver initializes ADC1 on GPIO2 (Channel 1) with 11 dB attenuation (0–3.3V range).
A FreeRTOS task continuously reads raw ADC values, converts them to voltage (mV), and calculates temperature (°C).
Output is logged to the serial monitor every second. Example:I (1234) MAIN: Raw: 2048, Voltage: 1650.23 mV, Temperature: 25.67 °C



Circuit Diagram
Connect the NTC thermistor in a voltage divider with a 10kΩ resistor to GPIO2:
Vref (3.3V) ---- 10kΩ Resistor ---- GPIO2 ---- NTC Thermistor ---- GND

 (Note: Add your own diagram to the docs/ folder and update this link.)
Project Structure
esp32s3-adc-driver/
├── .gitignore               # Ignores build artifacts
├── CMakeLists.txt           # Root build configuration
├── docs/
│   └── circuit.png          # Circuit diagram for hardware setup
├── main/
│   ├── adc_low_level.c      # ADC driver implementation
│   ├── adc_low_level.h      # ADC driver header
│   ├── CMakeLists.txt       # Component build configuration
│   └── main.c               # Main application with FreeRTOS task
├── tests/
│   └── test_adc.c           # Unit tests for ADC driver
├── README.md                # Project documentation
└── LICENSE                  # MIT License

Technical Highlights

Embedded C: Leverages direct register manipulation for precise ADC control on ESP32-S3.
FreeRTOS: Implements thread-safe ADC access using mutexes, ideal for real-time applications.
Signal Processing: Applies the Steinhart-Hart equation for accurate NTC thermistor-based temperature calculations.
Error Handling: Includes retries, timeouts, and logging for robust operation in production environments.
Modularity: Separates ADC logic into reusable functions, suitable for integration into larger IoT or embedded projects.

Future Improvements

Unit Testing: Expand test coverage using ESP-IDF’s Unity framework (initial test included in tests/test_adc.c).
Multi-Channel Support: Extend the driver to handle multiple ADC channels for multi-sensor applications.
Calibration: Add runtime calibration for improved ADC accuracy.
Configurability: Allow dynamic configuration of NTC thermistor parameters (e.g., β, R25) via a configuration file.


License
MIT License
Contact
Gaurav Kumar - gkumar20112000@gmail.com  www.linkedin.com/in/gaurav-kumar-b89570317
Feel free to reach out for collaboration, feedback, or opportunities!
