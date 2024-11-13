#ifndef MAIN__CANPROTOCOL_HPP_
#define MAIN__CANPROTOCOL_HPP_

#include <cstdint>
#include <cstring>
#include <cmath> 

#include "Configs.h"

namespace  can_protocol{

/// @brief Runtime motor motion control command message. ControlMessage takes CANMsg message and encode the control command following the Steadywin
/// CAN Protocol for PCANInterface to send to the motor driver later
class MsgDecoder{

public:
    /// @brief Default constructor
    MsgDecoder(const float &gear_ratio, const float &torque_constant);

    /////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////// COMMAND MESSAGE //////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief Get the desired command value and duration from the received message
    /// @param torque desired torque value
    /// @param duration execution time in ms
    /// @param msg CANMsg reference to store the command message
    void get_command(const CANMsg &msg, float &value, uint32_t &duration);

    /////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////// GAIN MESSAGE /////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief Set the gain parameter ID and value to the message
    /// @param gain_val desired gain value
    void get_gain(CANMsg &msg, const uint32_t &gain_val, const uint8_t param_id);


private:
    /// @brief gear ratio of the motor initialized in the actuator constructor
    const float &gear_ratio_;

    /// @brief torque constant of the motor initialized in the actuator constructor
    const float &torque_constant_;

    /// @brief decode 32-bit unsigned integer value to the message buffer's BYTE4 to BYTE7 for the gain parameter
    /// @param msg reference  CANMsg to store the encoded value
    /// @param value desired 32-bit unsigned integer value of the gain parameter
    inline void decode_param_int_(const CANMsg msg, uint32_t value) const {
        // Copy the 32-bit unsigned integer value to the buffer using little-endian byte order
        value = msg.DATA[4] | (msg.DATA[5] << 8) | (msg.DATA[6] << 16) | (msg.DATA[7] << 24);
    }
};

/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// STATE MESSAGE ////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////


class MsgEncoder{
public:
    /// @brief constructor
    MsgEncoder(const float &gear_ratio, const float &torque_constant);


    /// @brief set the state of the motor from the received message
    /// @param msg received TPACNMsg message
    /// @param temperature reference to store the decoded temperature value
    /// @param position reference to store the decoded position value
    /// @param velocity reference to store the decoded velocity value
    /// @param torque reference to store the decoded torque value
    void set_states(CANMsg &msg, const uint8_t result, const uint8_t &temperature, const float &position, const float &velocity, const float &torque);

    /// @brief set the gain value from the received message
    void set_gain(const CANMsg &msg, uint32_t &gain_val) const;

    /// @brief set the motor response message for the start motor command
    /// @param msg message to store the response
    /// @param result result of the command
    void start_motor_response(CANMsg &msg, const uint8_t result);

    /// @brief set the motor response message for the stop motor command
    /// @param msg 
    /// @param result 
    void stop_motor_response(CANMsg &msg, const uint8_t result);

private:
    /// @brief gear ratio of the motor
    const float &gear_ratio_ ;

    /// @brief torque constant of the motor
    const float &torque_constant_; 

    /// @brief decode 32-bit unsigned integer value from the message buffer's BYTE4 to BYTE7
    /// @param msg recevied CANMsg message
    /// @param value reference to store the decoded 32-bit unsigned integer value of the gain parameter
    inline void encode_param_int_(const CANMsg &msg, uint32_t &value) const {
        value = msg.DATA[4] | (msg.DATA[5] << 8) | (msg.DATA[6] << 16) | (msg.DATA[7] << 24);
    }
};

} // namespace can_protocol
#endif  // MAIN__CANPROTOCOL_HPP_
