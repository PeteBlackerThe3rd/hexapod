#ifndef __ROBOT_HW_INTERFACE_HPP__
#define __ROBOT_HW_INTERFACE_HPP__
#include "buffer.hpp"

class RobotHWInterface
{
public:
  
  enum class MotorState { On = 1, Off = 0 };

  float min_endstop_angles[3];
  float max_endstop_angles[3];

  uint16_t joint_feedback_mins[18];
  uint16_t joint_feedback_maxs[18];
  bool first_feedback_cal;
  
  RobotHWInterface();

  void readJointStates();
  void setMotorState(MotorState newMotorState);

  JointState jointStates[18];
  MotorState motorState;

};

#endif // __ROBOT+HW_INTERFACE_HPP__
