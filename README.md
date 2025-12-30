# ESP32-S3 Touch LCD 2.8" - Complete Development Platform

A comprehensive development platform for ESP32-S3 with a multi-touch display, audio, communication, and sensors.

Originally developed by [zonfacter](https://github.com/zonfacter), translated by Copilot and continued development by kneave

![ESP32-S3](https://img.shields.io/badge/ESP32--S3-240MHz-blue) ![Multi-Touch](https://img.shields.io/badge/Multi--Touch-5%20Finger-green) ![Audio](https://img.shields.io/badge/I2C-Audio-orange) ![Sensors](https://img.shields.io/badge/Sensors-I2C-purple) ![Communication](https://img.shields.io/badge/RS232%2F485-Serial-red)

## 🎯 Project Overview

This repository contains a **complete development platform** for the ESP32-S3 with a 2.8" touch LCD. The system offers professional hardware integration for:

- **Multi-touch interface** with gesture recognition
- **I2C audio system** for multimedia applications
- **Dual-bus communication** (RS232/RS485)
- **Sensor integration** (gyroscope, RTC)
- **Wireless connectivity** (WiFi, Bluetooth)
- **Power management** with battery support

## 🏗️ Hardware Architecture

### 📋 Main Components

| Component | Type | Interface | Status |
|-----------|------|-----------|--------|
| **Display** | ST7789 320x240 | SPI | ✅ Implemented |
| **Touch Controller** | CST328 (5-point) | I2C Wire1 | ✅ Multi-Touch |
| **Audio** | I2S/I2C Audio | I2C Wire | 🔄 Planned |
| **RTC** | DS3231/PCF8563 | I2C Wire | 🔄 Planned |
| **Gyroscope** | MPU6050/ICM20948 | I2C Wire | 🔄 Planned |
| **RS232/485** | MAX3232/MAX485 | UART | 🔄 Planned |
| **WiFi/BT** | ESP32-S3 integrated | - | 🔄 Planned |
| **Battery** | LiPo Management | ADC/GPIO | 🔄 Planned |
| **RGB LED** | WS2812/Neopixel | GPIO | 🔄 Planned |

### 🔌 Pinout

```cpp
```cpp
class SerialCommunication {
  private:
    HardwareSerial* serial_port;
    bool rs485_mode = false;
    
  public:
    void initRS232();
    void initRS485();
    void switchToRS485();
    void switchToRS232();
    
    // RS485 specific functions
    void setTransmitMode();
    void setReceiveMode();
    bool sendRS485Data(const uint8_t* data, size_t length);
    
    // Protocol handlers
    void handleModbusRTU();
    void handleCustomProtocol();
};

// Usage
SerialCommunication comm;

void setup() {
  comm.initRS232();
  // or: comm.initRS485();
}

void loop() {
  if (comm.dataAvailable()) {
    String received = comm.readData();
    processIncomingData(received);
  }
}
```

## ⏰ RTC & Time Management (🔄 Planned)

### Real-Time Clock Integration
```cpp
#include <RTClib.h>

class RTCManager {
  private:
    RTC_DS3231 rtc;  // or RTC_PCF8563
    bool rtc_found = false;
    
  public:
    bool initRTC();
    DateTime getCurrentTime();
    void setTime(DateTime dt);
    void setAlarm(DateTime alarm_time);
    bool isAlarmTriggered();
    
    // Touch interface for time setting
    void showTimeSetInterface();
    void handleTimeAdjustment(GestureEvent gesture);
};

// Integration with display
void drawClock() {
  DateTime now = rtc_manager.getCurrentTime();
  
  tft.setTextSize(3);
  tft.setCursor(50, 100);
  tft.printf("%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  
  tft.setTextSize(2);
  tft.setCursor(50, 140);
  tft.printf("%02d.%02d.%04d", now.day(), now.month(), now.year());
}
```

## 🔄 Gyroscope & Motion Sensor (🔄 Planned)

### IMU Integration (MPU6050/ICM20948)
```cpp
#include <MPU6050.h>

class MotionSensor {
  private:
    MPU6050 mpu;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float temperature;
    
  public:
    bool initIMU();
    void readSensorData();
    void calibrateSensor();
    
    // Motion detection
    bool detectShake();
    bool detectTilt();
    float getOrientation();
    
    // Integration with touch
    void enableMotionGestures();
    void handleMotionEvent();
};

// Use for display rotation
void handleAutoRotation() {
  float orientation = motion_sensor.getOrientation();
  
  if (abs(orientation) < 45) {
    tft.setRotation(1);  // Landscape
  } else if (orientation > 45) {
    tft.setRotation(2);  // Portrait
  }
}

// Shake-to-clear functionality
void checkShakeGestures() {
  if (motion_sensor.detectShake()) {
    clearDisplay();
    triggerHapticFeedback();
  }
}
```

## 🔋 Power Management (🔄 Planned)

### Battery Monitoring & Power Management
```cpp
class PowerManager {
  private:
    float battery_voltage = 0.0;
    uint8_t battery_percentage = 0;
    bool is_charging = false;
    bool low_power_mode = false;
    
  public:
    void initPowerSystem();
    float readBatteryVoltage();
    uint8_t calculateBatteryPercentage();
    bool isCharging();
    
    // Power modes
    void enterLowPowerMode();
    void exitLowPowerMode();
    void enterDeepSleep();
    void configureSleepWakeup();
    
    // Display integration
    void drawBatteryIndicator();
    void showPowerMenu();
};

// Auto-sleep on inactivity
void handlePowerManagement() {
  static unsigned long last_touch = millis();
  
  if (active_touch_count > 0) {
    last_touch = millis();
    power_manager.exitLowPowerMode();
  }
  
  if (millis() - last_touch > 30000) {  // 30s inactivity
    power_manager.enterLowPowerMode();
  }
  
  if (millis() - last_touch > 300000) { // 5min inactivity
    power_manager.enterDeepSleep();
  }
}
```

## 💡 RGB LED System (🔄 Planned)

### Addressable RGB LED Control
```cpp
#include <FastLED.h>

#define NUM_LEDS 1
#define LED_TYPE WS2812B

class RGBLEDManager {
  private:
    CRGB leds[NUM_LEDS];
    uint8_t brightness = 128;
    
  public:
    void initLEDs();
    void setColor(CRGB color);
    void setBrightness(uint8_t level);
    void rainbow();
    void breathe(CRGB color);
    
    // Status indicators
    void showTouchFeedback();
    void showBatteryStatus();
    void showConnectionStatus();
    void showErrorState();
};

// Touch feedback with LED
void handleTouchLED() {
  if (active_touch_count > 0) {
    rgb_led.setColor(CRGB::Green);
  } else if (last_gesture.type == GESTURE_DOUBLE_TAP) {
    rgb_led.setColor(CRGB::Blue);
  } else {
    rgb_led.setColor(CRGB::Black);
  }
}
```

## 📶 Wireless Connectivity (🔄 Planned)

### WiFi & Bluetooth Integration
```cpp
#include <WiFi.h>
#include <BluetoothSerial.h>

class WirelessManager {
  private:
    BluetoothSerial bt_serial;
    bool wifi_connected = false;
    bool bt_connected = false;
    
  public:
    // WiFi Management
    bool connectWiFi(const char* ssid, const char* password);
    void startWiFiAP(const char* ap_name);
    void handleWiFiEvents();
    
    // Bluetooth Management
    bool initBluetooth(const char* device_name);
    void handleBluetoothData();
    void sendBluetoothData(const String& data);
    
    // Web interface
    void startWebServer();
    void handleWebRequests();
    
    // OTA Updates
    void initOTA();
    void handleOTAUpdates();
};

// Touch interface for WiFi setup
void showWiFiSetupMenu() {
  tft.fillScreen(0x0000);
  tft.setTextColor(0xFFFF);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("WiFi Setup");
  
  // Scan and display WiFi networks
  wireless.scanWiFiNetworks();
  displayAvailableNetworks();
}

// Bluetooth data transfer
void handleBluetoothCommands() {
  if (bt_serial.available()) {
    String command = bt_serial.readString();
    
    if (command.startsWith("SET_TIME")) {
      // Set time via Bluetooth
      handleTimeCommand(command);
    } else if (command.startsWith("GET_SENSOR")) {
      // Send sensor data
      sendSensorData();
    }
  }
}
```

## 🗂️ Project Structure

```
ESP32-S3-Touch-LCD-2.8/
├── src/
│   ├── main.cpp                    # Main application
│   ├── multitouch/
│   │   ├── touch_system.h          # ✅ Touch controller
│   │   ├── touch_system.cpp        # ✅ Multi-touch logic
│   │   ├── gesture_recognition.h   # ✅ Gesture recognition
│   │   └── gesture_recognition.cpp # ✅ Gesture implementation
│   ├── audio/
│   │   ├── i2c_audio.h            # 🔄 Audio system
│   │   ├── i2c_audio.cpp          # 🔄 I2C audio codec
│   │   └── audio_effects.cpp      # 🔄 Audio effects
│   ├── communication/
│   │   ├── serial_comm.h          # 🔄 RS232/RS485
│   │   ├── serial_comm.cpp        # 🔄 Dual-mode serial
│   │   └── protocols.cpp          # 🔄 Modbus, custom
│   ├── sensors/
│   │   ├── rtc_manager.h          # 🔄 Real-time clock
│   │   ├── rtc_manager.cpp        # 🔄 Time management
│   │   ├── motion_sensor.h        # 🔄 Gyro/IMU
│   │   └── motion_sensor.cpp      # 🔄 Motion detection
│   ├── power/
│   │   ├── power_manager.h        # 🔄 Battery management
│   │   ├── power_manager.cpp      # 🔄 Sleep modes
│   │   └── battery_monitor.cpp    # 🔄 Battery monitoring
│   ├── wireless/
│   │   ├── wifi_manager.h         # 🔄 WiFi functions
│   │   ├── wifi_manager.cpp       # 🔄 WiFi management
│   │   ├── bluetooth_manager.h    # 🔄 Bluetooth system
│   │   └── bluetooth_manager.cpp  # 🔄 BT communication
│   ├── display/
│   │   ├── display_manager.h      # ✅ Display control
│   │   ├── display_manager.cpp    # ✅ ST7789 driver
│   │   └── ui_elements.cpp        # 🔄 UI framework
│   └── utils/
│       ├── config.h               # Hardware configuration
│       ├── hardware_hal.h         # Hardware abstraction
│       └── debug_utils.cpp        # Debug functions
├── examples/
│   ├── single_touch_demo/         # ✅ Single-touch demo
│   ├── multitouch_demo/           # ✅ Multi-touch demo
│   ├── audio_demo/                # 🔄 Audio test
│   ├── sensor_demo/               # 🔄 Sensor test
│   ├── communication_demo/        # 🔄 RS232/485 test
│   └── complete_system_demo/      # 🔄 Complete system
├── docs/
│   ├── hardware_guide.md          # Hardware documentation
│   ├── calibration_guide.md       # Calibration guide
│   ├── troubleshooting.md         # Troubleshooting
│   └── api_reference.md           # API documentation
├── tools/
│   ├── calibration_tool/          # Touch calibration
│   ├── config_generator/          # Hardware configurator
│   └── firmware_updater/          # OTA update tool
├── libraries/                     # External libraries
├── platformio.ini                 # PlatformIO configuration
└── LICENSE                        # MIT License
```

## 🛠️ Development Setup

### Requirements
```bash
# PlatformIO installation
pip install platformio

# Clone repository
git clone https://github.com/kneave/ESP32-S3-Touch-LCD-2.8
cd ESP32-S3-Touch-LCD-2.8

# Install dependencies
pio lib install
```

### Libraries
```ini
; platformio.ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino

lib_deps = 
    lovyangfx/LovyanGFX@^1.1.12
    adafruit/RTClib@^2.1.1
    electroniccats/MPU6050@^1.0.0
    fastled/FastLED@^3.6.0
    bblanchon/ArduinoJson@^6.21.3
    ottowinter/ESPAsyncWebServer@^3.0.0
```

## 📋 Development Roadmap

### Phase 1: Multi-Touch Foundation ✅
- [x] Single touch implementation
- [x] Multi-touch system (5 points)
- [x] Gesture recognition
- [x] Display integration
- [x] Performance optimization

### Phase 2: Audio & Communication 🔄
- [ ] I2C audio codec integration
- [ ] I2S audio pipeline
- [ ] RS232/RS485 dual-mode
- [ ] Modbus RTU protocol
- [ ] Audio–touch integration

### Phase 3: Sensors & Power 🔄
- [ ] RTC integration
- [ ] Gyro/IMU system
- [ ] Battery management
- [ ] Power modes implementation
- [ ] RGB LED control

### Phase 4: Wireless & Advanced 🔄
- [ ] WiFi management
- [ ] Bluetooth integration
- [ ] OTA updates
- [ ] Web interface
- [ ] Cloud connectivity

### Phase 5: System Integration 🔄
- [ ] Complete system demo
- [ ] Performance tuning
- [ ] Documentation
- [ ] Testing & validation
- [ ] Production release

## 🎯 Quick Start Guides

### 1. Multi-Touch Demo (✅ Available)
```cpp
#include "src/multitouch/touch_system.h"

void setup() {
  Serial.begin(115200);
  initMultiTouchSystem();
  Serial.println("Multi-Touch System ready!");
}

void loop() {
  updateMultiTouch();
  handleGestureEvents();
  delay(16); // ~60fps
}
```

### 2. Audio Demo (🔄 In Development)
```cpp
#include "src/audio/i2c_audio.h"

I2CAudioSystem audio;

void setup() {
  audio.initAudioCodec();
  audio.setVolume(50);
}

void loop() {
  if (last_gesture.type == GESTURE_TAP) {
    audio.playTone(440, 500); // A4, 500ms
  }
}
```

### 3. Sensor Demo (🔄 Planned)
```cpp
#include "src/sensors/rtc_manager.h"
#include "src/sensors/motion_sensor.h"

RTCManager rtc;
MotionSensor motion;

void setup() {
  rtc.initRTC();
  motion.initIMU();
}

void loop() {
  DateTime now = rtc.getCurrentTime();
  displayTime(now);
  
  if (motion.detectShake()) {
    clearDisplay();
  }
}
```

## 🔧 Configuration

### Hardware Variants
```cpp
// config.h - hardware-specific settings

// Display variants
#define DISPLAY_ST7789_320x240  1
#define DISPLAY_ILI9341_320x240 2
#define DISPLAY_TYPE DISPLAY_ST7789_320x240

// Touch controller variants
#define TOUCH_CST328_5POINT     1
#define TOUCH_GT911_10POINT     2
#define TOUCH_TYPE TOUCH_CST328_5POINT

// Audio codec variants
#define AUDIO_WM8960           1
#define AUDIO_ES8388           2
#define AUDIO_TYPE AUDIO_WM8960

// Communication modules
#define COMM_RS232_ONLY        1
#define COMM_RS485_ONLY        2  
#define COMM_DUAL_MODE         3
#define COMM_TYPE COMM_DUAL_MODE
```

## 📊 Performance & Benchmarks

### Multi-Touch Performance (Measured)
```
📊 FPS: 43.2 | Active Touches: 2 | Heap: 343KB
🎭 Gesture Latency: <50ms
📍 Touch Accuracy: ±2-3 pixels
🔄 Gesture Cooldown: 150ms
```

### Expected System Performance
| Component | Performance Target |
|-----------|--------------------|
| **Touch Update** | 40-60 FPS |
| **Audio Latency** | <10ms |
| **Sensor Rate** | 100Hz |
| **Serial Speed** | 115200-460800 baud |
| **WiFi Throughput** | 10-50 Mbps |
| **Battery Life** | 8-24h (depending on usage) |

## 🤝 Contributing

We welcome contributions! The following are especially welcome:

### Desired Contributions
- **Audio system implementation** (I2C codec integration)
- **RS485 Modbus protocol** (industrial communication)
- **RTC & alarm management** (real-time features)
- **IMU motion gestures** (advanced interaction)
- **Power optimization** (battery life improvements)
- **WiFi/BT examples** (connectivity demos)

### Development Guidelines
```bash
# Fork & development
git clone https://github.com/your-username/ESP32-S3-Touch-LCD-2.8
cd ESP32-S3-Touch-LCD-2.8

# Feature branch
git checkout -b feature/audio-integration

# Development & testing
pio run -t upload
pio test

# Pull request
git push origin feature/audio-integration
# Create PR with description
```

## 📞 Support & Community

- **GitHub Issues:** [Bug Reports & Feature Requests](https://github.com/kneave/ESP32-S3-Touch-LCD-2.8/issues)
- **Discussions:** [Community Forum](https://github.com/kneave/ESP32-S3-Touch-LCD-2.8/discussions)
- **Documentation:** [Wiki Pages](https://github.com/kneave/ESP32-S3-Touch-LCD-2.8/wiki)
- **Examples:** [Code Examples](https://github.com/kneave/ESP32-S3-Touch-LCD-2.8/tree/main/examples)

## 📄 License

MIT License - see LICENSE for details.

## 🙏 Acknowledgments

- **ESP32 Community** for hardware support
- **LovyanGFX Team** for the excellent display library
- **CST328 Developers** for touch controller documentation
- **Open Source Contributors** for inspiration and code examples

---

**⭐ If you find this project helpful, please give it a star!** ⭐

**🚀 Ready for the next development phase: I2C Audio Integration!** 🎵
