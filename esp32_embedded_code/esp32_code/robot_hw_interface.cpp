#include <arduino.h>
#include "robot_hw_interface.hpp"

  
RobotHWInterface::RobotHWInterface()
{
  motorState == MotorState::Off;

  for (unsigned int j=0; j<18; ++j) {
    jointStates[j].driveDutyCycle = 32767; // 50% of servo PWM range for now
    jointStates[j].feedbackAngle_mrads = 0;
  }

  first_feedback_cal = true;
  
  min_endstop_angles[0] = 0.0f;
  min_endstop_angles[1] = 100.0f;
  min_endstop_angles[2] = 160.0f;
  max_endstop_angles[0] = 0.0f;
  max_endstop_angles[1] = -55.0f;
  max_endstop_angles[2] = 10.0f;
};

void RobotHWInterface::readJointStates()
{
  float feedbackAngleCoeff = (3.141592654 * 1000.0 ) / 2920.0;

  jointStates[0].rawFeedback = analogRead(34);
  jointStates[1].rawFeedback = analogRead(39);
  jointStates[2].rawFeedback = analogRead(36);

  // temp perform joint feedback calibration
  if (first_feedback_cal) {
    for (int i=0; i<3; ++i) {
      joint_feedback_mins[i] = jointStates[i].rawFeedback;
      joint_feedback_maxs[i] = jointStates[i].rawFeedback;
    }
    first_feedback_cal = false;
  }
  else {
    for (int i=0; i<3; ++i) {
      if (jointStates[i].rawFeedback < joint_feedback_mins[i])
        joint_feedback_mins[i] = jointStates[i].rawFeedback;
      else if (jointStates[i].rawFeedback > joint_feedback_maxs[i])
        joint_feedback_maxs[i] = jointStates[i].rawFeedback;
    }
  }

  /*for (int i=0; i<3; ++i) {
    Serial.print("Joint [");
    Serial.print(i);
    Serial.print("] feedback range [");
    Serial.print(joint_feedback_mins[i]);
    Serial.print(" - ");
    Serial.print(joint_feedback_maxs[i]);
    Serial.println("]");
  }*/

  
  jointStates[0].feedbackAngle_mrads = (-jointStates[0].rawFeedback * feedbackAngleCoeff) + 1570;
  
  for (int i=1; i<3; ++i) {
    if (joint_feedback_mins[i] == joint_feedback_maxs[i])
      jointStates[1].feedbackAngle_mrads = 0;
    else {
      float ratio = (jointStates[i].rawFeedback - joint_feedback_mins[i]) / (float)(joint_feedback_maxs[i] - joint_feedback_mins[i]);
      // Serial.print("Joint [");
      // Serial.print(i);
      // Serial.print("] ratio: ");
      // Serial.print(ratio);
      // Serial.print(" angle:");
      float angle = ((max_endstop_angles[i] - min_endstop_angles[i]) * ratio) + min_endstop_angles[i];
      //Serial.println(angle);
      // jointStates[i].feedbackAngle_mrads = angle * 17.45329f;
      //jointStates[1].feedbackAngle_mrads = -jointStates[1].rawFeedback * feedbackAngleCoeff;
    }
  }
  // jointStates[2].feedbackAngle_mrads = -jointStates[2].rawFeedback * feedbackAngleCoeff;
};

void RobotHWInterface::setMotorState(MotorState newMotorState) 
{  
  motorState = newMotorState;
};
