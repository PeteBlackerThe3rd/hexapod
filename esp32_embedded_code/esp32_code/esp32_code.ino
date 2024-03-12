#include "comms_thread.hpp"

class RobotInterface
{
public:
  
  enum class MotorState { On = 1, Off = 0 };

  float min_endstop_angles[3];
  float max_endstop_angles[3];

  uint16_t joint_feedback_mins[18];
  uint16_t joint_feedback_maxs[18];
  bool first_feedback_cal;

  /*class JointState {
  public:
    uint16_t rawFeedback;
    int16_t feedbackAngle_mrads;
    uint16_t current_mA;
    uint16_t drivePosition;
    uint16_t goalAngle_mrads;
  };*/
  
  RobotInterface() {
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

  void readJointStates() {

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

    for (int i=0; i<3; ++i) {
      Serial.print("Joint [");
      Serial.print(i);
      Serial.print("] feedback range [");
      Serial.print(joint_feedback_mins[i]);
      Serial.print(" - ");
      Serial.print(joint_feedback_maxs[i]);
      Serial.println("]");
    }

    
    jointStates[0].feedbackAngle_mrads = (-jointStates[0].rawFeedback * feedbackAngleCoeff) + 1570;
    
    for (int i=1; i<3; ++i) {
      if (joint_feedback_mins[i] == joint_feedback_maxs[i])
        jointStates[1].feedbackAngle_mrads = 0;
      else {
        float ratio = (jointStates[i].rawFeedback - joint_feedback_mins[i]) / (float)(joint_feedback_maxs[i] - joint_feedback_mins[i]);
        Serial.print("Joint [");
        Serial.print(i);
        Serial.print("] ratio: ");
        Serial.print(ratio);
        Serial.print(" angle:");
        float angle = ((max_endstop_angles[i] - min_endstop_angles[i]) * ratio) + min_endstop_angles[i];
        Serial.println(angle);
        jointStates[i].feedbackAngle_mrads = angle * 17.45329f;
        //jointStates[1].feedbackAngle_mrads = -jointStates[1].rawFeedback * feedbackAngleCoeff;
      }
    }
    // jointStates[2].feedbackAngle_mrads = -jointStates[2].rawFeedback * feedbackAngleCoeff;
  };

  void setMotorState(MotorState newMotorState) {
    
    motorState = newMotorState;
  };

  JointState jointStates[18];
  MotorState motorState;

};

CommsThread* commsThread;
RobotInterface* robotInterface;

uint32_t lastHKTMTime = 0;
uint16_t lastHKSeqId = 0;

uint32_t lastRSTMTime = 0;
uint16_t lastRSSeaId = 0;
//uint32_t lastMotorTMTime = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("\n[*] Creating AP");
  commsThread = new CommsThread;
  robotInterface = new RobotInterface;
}

void loop()
{
  //Serial.println("Something to stop core#1 watchdog triggering");
  delay(10);

  while (commsThread->getPacketRxQueueSize() > 0) {
    BasePacketPtr packet = commsThread->popPacket();
    Serial.print("Handling: ");
    Serial.println(packet->toString().c_str());

    if (packet->getId() == TCManualJointPositionPacket::packetId){
      auto manualJointPositionPacket = std::static_pointer_cast<TCManualJointPositionPacket>(packet);
      Serial.println("Handling manual joint position packet");
      Serial.println(manualJointPositionPacket->toString().c_str());
    }
  }

  // if the socket is connected then check if telemetry needs sending
  if (commsThread->socketConected()) {
    
    uint32_t timeNow = millis();
    
    // if more than 1 second has passed since the last 
    if (timeNow - lastHKTMTime > 1000) {
      lastHKTMTime = timeNow;

      //robotInterface->readJointStates();

      auto hkPacket = new TMHouseKeepingPacket(++lastHKSeqId);
      hkPacket->socketConnectionCount = 5;
      hkPacket->motorsOn = static_cast<uint8_t>(robotInterface->motorState);
      hkPacket->batteryVoltage100thsVolt = robotInterface->jointStates[0].rawFeedback; 

      commsThread->sendPacket(BasePacketPtr(hkPacket));
    }

    if (timeNow - lastRSTMTime > 100) {
      lastRSTMTime = timeNow;

      robotInterface->readJointStates();

      auto robotStatePacket = new TMRobotStatePacket(robotInterface->jointStates);
      commsThread->sendPacket(BasePacketPtr(robotStatePacket));
    }
  }
}
