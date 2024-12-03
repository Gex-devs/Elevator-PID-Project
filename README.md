# **Elevator Control System**

This project involves designing and programming a four-floor elevator model using an STM32 NUCLEO embedded board. It focuses on smooth motion control using PID feedback, speed control, and optional IoT integration for data visualization and configuration.

## **Features**
- **PID Feedback Control**: Ensures precise positioning of the elevator for smooth floor transitions.
- **Speed Control**: Limits the elevator's speed during motion for safe operation.
- **Real-Time Monitoring**: Outputs current position and error via a serial interface.
- **IoT Dashboard (Optional)**:
  - Displays elevator position in real-time.
  - Configures system parameters such as maximum speed.
- **Multithreading Support**: Implements FreeRTOS (optional) to handle I/O and servo control in separate threads for efficiency.
- **Customizable Design**: Supports additional sensors and advanced functionality modes.

## **Technologies Used**
- **Microcontroller**: STM32 NUCLEO board.
- **Programming Language**: C/C++.
- **Hardware Components**:
  - 3D-printed elevator model.
  - Parallax Feedback 360° servo motor.
  - Break beam sensors for position detection.
  - Safety end-switch for top-floor limit.
- **IoT Integration (Optional)**: ESP32 for real-time data communication.
- **Software Tools**:
  - STM32CubeIDE.
  - FreeRTOS (optional).

## **Getting Started**
### **Prerequisites**
- STM32 NUCLEO board.
- 3D-printed elevator model.
- Break beam sensors and a safety switch.
- USB connection for serial communication.

### **Setup**
1. Clone the repository:
   ```bash
   git clone <repository-url>
   ```
2. Open the project in STM32CubeIDE.
3. Connect the hardware components as per the wiring diagram in the documentation.
4. Compile and upload the code to the STM32 board.
