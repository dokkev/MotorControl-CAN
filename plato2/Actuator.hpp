#ifndef MAIN__ACTUATOR_HPP_
#define MAIN__ACTUATOR_HPP_


#include <cstdint>
#include <Dynamixel2Arduino.h>

#include "CANProtocol.hpp"
#include "CANInterface.hpp"



namespace actuator{

struct Config{
    const uint8_t can_tx_id;
    const uint8_t can_rx_id;
    const uint8_t dxl_id;
    const uint16_t encoder_tick;
    const char direction; // 0 for CW, 1 for CCW

    const float torque_constant;
    const float gear_ratio;

    const float joint_limit_max;
    const float joint_limit_min ;
};

/// @brief Actuator commands variables : position, torque
struct Commands{
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
    uint32_t duration = 0;
    uint32_t current = 0;
};

/// @brief Actuator states variables : position, velocity, torque
struct States{
    float position;
    float velocity;
    float torque;
    uint8_t temperature;
    
};

struct Gains{
    uint32_t kp_velocity;
    uint32_t ki_velocity;
    uint32_t kp_position;
    uint32_t ki_position;
    uint32_t kd_position;
};

enum class ControlMode{
    OFF,
    IDLE,
    TORQUE,
    VELOCITY,
    POSITION,

};

class Actuator{
    public:
        Actuator(Dynamixel2Arduino &dxl, can_interface::CANInterface &can_interface, Config &config);
        ~Actuator();

        States get_states(){ return states_; }

        void enable_motor();
        void disable_motor();
        void stop_control();
        void set_control_mode(const ControlMode &mode);
        void set_torque(const float &torque);
        void set_velocity(const float &velocity);
        void set_position(const float &position, const uint32_t &current);
        void apply_offset(const uint16_t &encoder_tick, const char &direction);
        
        void set_gains(const Gains &gains);

        void update_states();

        /// @brief Given the received message, identify the type of message and process it to store the data in the buffer
        /// @param msg 
        void process_message(const CANMsg &msg);


    private:

        Dynamixel2Arduino &dxl_;

        can_interface::CANInterface &can_interface_;

        Config &config_;

        can_protocol::MsgEncoder encoder_;
        can_protocol::MsgDecoder decoder_;

        Commands commands_;
        States states_;

        CANMsg state_msg_;
        CANMsg onoff_msg_;

        ControlMode current_mode_;
        
        inline CANMsg init_message_(){
            CANMsg msg;
            msg.ID = config_.can_tx_id;
            msg.LEN = 8;
            memset(msg.DATA, 0, 8);
            return msg;
        }

        inline float rad2deg(const float &rad){
            return rad * 180.0f / M_PI;
        }

        inline float deg2rad(const float &deg){
            return deg * M_PI / 180.0f;
        }
    

};

} // namespace actuator

#endif  // MAIN__ACTUATOR_HPP_