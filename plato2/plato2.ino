//==================================================================================//

#include "Actuator.hpp"

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

actuator::Config config1 = {
    SERVO1_TX_ID,
    SERVO1_RX_ID,
    SERVO1_DXL_ID,
    2036,
    1,
    TORQUE_CONSTANT,
    GEAR_RATIO,
    1.222,
    -0.349
};

actuator::Config config2 = {
    SERVO2_TX_ID,
    SERVO2_RX_ID,
    SERVO2_DXL_ID,
    1020,
    1,
    TORQUE_CONSTANT,
    GEAR_RATIO,
    0.785,
    -0.785
};

can_interface::CANInterface can(CAN_BAUDRATE_1M);
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
actuator::Actuator* actuator1 = nullptr;
actuator::Actuator* actuator2 = nullptr;
//==================================================================================//

void setup(){
    // Give some delay
    delay(3000);
    // start serial for IDE output
    Serial.begin(115200);

    // set port baudrate for Dynamixel
    dxl.begin(1000000);
    // set Port Protocol Version
    dxl.setPortProtocolVersion(2.0);

    actuator1 = new actuator::Actuator(dxl, can, config1);
    actuator2 = new actuator::Actuator(dxl, can, config2);
   

    // Initialize the CAN bus
    if (can.init()){
    // Set the CAN filter
        can.set_can_filter(SERVO1_RX_ID, SERVO2_RX_ID);
    }
}

void loop() {
  while (CAN.parsePacket()) {
    uint32_t packetId = CAN.packetId();


    // Create and populate the CANMsg message struct
    CANMsg msg;
    msg.ID = packetId;
    msg.LEN = CAN.packetDlc(); // Get the length of the data

    // Populate the data buffer
    for (int i = 0; i < msg.LEN; i++) {
      msg.DATA[i] = CAN.read(); // Read each byte from the CAN packet
    }

    // Process the message based on ID
    if (packetId == 0x11) {
      actuator1->process_message(msg);
    } else if (packetId == 0x12) {
      actuator2->process_message(msg);
    }
  }
}

