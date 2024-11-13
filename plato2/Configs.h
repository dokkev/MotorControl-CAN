#ifndef MAIN__CONFIGS_H_
#define MAIN__CONFIGS_H_

#include <cstdint>
#include <cstring>
#include <cmath> 

constexpr uint32_t CAN_BAUDRATE_1M = 1000E3;

constexpr uint32_t SERVO1_TX_ID = 0x21;
constexpr uint32_t SERVO1_RX_ID = 0x11;
constexpr uint8_t  SERVO1_DXL_ID = 1;

constexpr uint32_t SERVO2_TX_ID = 0x22;
constexpr uint32_t SERVO2_RX_ID = 0x12;
constexpr uint8_t  SERVO2_DXL_ID = 2;

constexpr float GEAR_RATIO = 350.0f;
constexpr float TORQUE_CONSTANT = 1.17f;


struct CANMsg{
    uint32_t ID;
    uint8_t DATA[8];
    uint8_t LEN;
};


//// These Values need to match with the values in the motor driver. Don't Modify them
namespace CommandByte{
    // Control
    constexpr uint8_t START_MOTOR = 0x91;
    constexpr uint8_t STOP_MOTOR = 0x92;
    constexpr uint8_t TORQUE_CONTROL = 0x93;
    constexpr uint8_t SPEED_CONTROL = 0x94;
    constexpr uint8_t POSITION_CONTROL = 0x95;
    constexpr uint8_t STOP_CONTROL = 0x97;

    // Parameter
    constexpr uint8_t MODIFY_PARAMETER = 0xA1;
    constexpr uint8_t RETRIVE_PARAMETER = 0xA2;


} // namespace CommandByte

namespace ResultByte{
    constexpr uint8_t SUCCESS = 0x00;
    constexpr uint8_t FAILURE = 0x01;
} // namespace ResultByte


namespace ParamID{
    constexpr uint8_t KP_CURRENT = 0x00;
    constexpr uint8_t KI_CURRENT = 0x01;
    constexpr uint8_t KP_SPEED = 0x02;
    constexpr uint8_t KI_SPEED = 0x03;
    constexpr uint8_t KP_POSITION = 0x04;
    constexpr uint8_t KI_POSITION = 0x05;
    constexpr uint8_t KD_POSITION = 0x06;
}



#endif  // MAIN_CONFIGS_H_