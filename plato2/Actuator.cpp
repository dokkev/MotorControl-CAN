#include "Actuator.hpp"
#define DXL_SERIAL   Serial1
using namespace ControlTableItem;

namespace actuator{

void canSender(uint32_t id) {
  Serial.print("Sending packets ... ");

  // Example packet with ID 0x01 and 8 bytes of data
  CAN.beginPacket(id);
  CAN.write(0x91);
  for (int i = 0; i < 7; i++) {
    CAN.write(0x00);
  }
  CAN.endPacket();

  Serial.print("Packets sent ... ");
  Serial.println(" done");
}


Actuator::Actuator(Dynamixel2Arduino &dxl, can_interface::CANInterface &can_interface, Config &config)
    :   dxl_(dxl),
        can_interface_(can_interface), 
        config_(config),
        encoder_(config.gear_ratio, config.torque_constant),
        decoder_(config.gear_ratio, config.torque_constant){

    // initilze messages
    state_msg_ = init_message_();
    onoff_msg_ = init_message_();
    
    Serial.println("Actuator Constructor");
    // Get DYNAMIXEL information
    dxl_.ping(config.dxl_id);
  
    // Apply the offset
    apply_offset(config.encoder_tick, config.direction);

    // Set the control mode
    // set the initial current limit
      // Turn off the torque
    dxl_.torqueOff(config.dxl_id);
}

Actuator::~Actuator(){
    // Turn off the torque
    

    if (dxl_.torqueOff(config_.dxl_id)){
        encoder_.stop_motor_response(state_msg_, ResultByte::SUCCESS);
        can_interface_.send_cb(state_msg_);
        Serial.println("Motor Disabled");
    }
    else{
        encoder_.stop_motor_response(state_msg_, ResultByte::FAILURE);
        Serial.println("Motor Disable Failed");
    }
}

void Actuator::enable_motor(){
    // Turn on the torque
    dxl_.torqueOff(config_.dxl_id);
    dxl_.setOperatingMode(config_.dxl_id, OP_CURRENT_BASED_POSITION);
    if (dxl_.torqueOn(config_.dxl_id)){
        encoder_.start_motor_response(onoff_msg_, ResultByte::SUCCESS);
        can_interface_.send_cb(onoff_msg_);
        Serial.println("Motor Enabled");
      
    }
    else{
        encoder_.start_motor_response(onoff_msg_, ResultByte::FAILURE);
        can_interface_.send_cb(onoff_msg_);
        Serial.println("Motor Enable Failed");
    }
}

void Actuator::disable_motor(){
    // Turn off the torque

    if (dxl_.torqueOff(config_.dxl_id)){
        encoder_.stop_motor_response(state_msg_, ResultByte::SUCCESS);
        Serial.println("Motor Disabled");
    }
    else{
        encoder_.stop_motor_response(state_msg_, ResultByte::FAILURE);
        Serial.println("Motor Disable Failed");
    }
}


void Actuator::set_torque(const float &torque){
    float current = torque / config_.torque_constant * 1000;
    // set the torque
    dxl_.setGoalCurrent(config_.dxl_id, torque);
}

void Actuator::set_velocity(const float &velocity){
    // set the velocity
    dxl_.setGoalVelocity(config_.dxl_id, velocity);
}

void Actuator::set_position(const float &position, const uint32_t &current){

    // set the current
    dxl_.setGoalCurrent(config_.dxl_id, current, UNIT_MILLI_AMPERE);
    float cmd;
    // check the position is within the limit and clamp it
    if (position > config_.joint_limit_max){
        cmd = rad2deg(config_.joint_limit_max);
        return;
    }
    else if (position < config_.joint_limit_min){
        cmd = rad2deg(config_.joint_limit_min);
        return;
    }

    // set the position
    cmd = rad2deg(position);
    // Serial.println(cmd);

    dxl_.setGoalPosition(config_.dxl_id, cmd, UNIT_DEGREE);
}

void Actuator::apply_offset(const uint16_t &encoder_tick, const char &direction){
    // set the zero position
    dxl_.writeControlTableItem(HOMING_OFFSET, config_.dxl_id, encoder_tick);
    dxl_.writeControlTableItem(DRIVE_MODE, config_.dxl_id, direction);

}

void Actuator::update_states(){
    // get the states
    states_.position = deg2rad(dxl_.getPresentPosition(config_.dxl_id, UNIT_DEGREE));
    states_.velocity = dxl_.getPresentVelocity(config_.dxl_id, UNIT_RPM);
    states_.torque = dxl_.getPresentCurrent(config_.dxl_id, UNIT_MILLI_AMPERE) / 1000 * config_.torque_constant;
    // states_.temperature = dxl_.readControlTableItem(PRESENT_TEMPERATURE, config_.dxl_id);

    // make a state message and send it
    encoder_.set_states(state_msg_, ResultByte::SUCCESS, states_.temperature, states_.position, states_.velocity, states_.torque);
    can_interface_.send_cb(state_msg_);

}

void Actuator::process_message(const CANMsg &msg){
    switch (msg.DATA[0]){

        case CommandByte::START_MOTOR:
            enable_motor();
            return;

        case CommandByte::STOP_MOTOR:
        case CommandByte::STOP_CONTROL:
            disable_motor();
            return;

        case CommandByte::TORQUE_CONTROL:
        case CommandByte::POSITION_CONTROL:
            decoder_.get_command(msg, commands_.position, commands_.current);
            set_position(commands_.position, commands_.current);
            update_states(); 
     
            break;
    }


    
}




} // namespace actuator