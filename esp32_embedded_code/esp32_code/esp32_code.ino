#include "comms_thread.hpp"

class RobotInterface
{
public:
  
  enum class MotorState { On = 1, Off = 0 };

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
  };

  void readJointStates() {

    float feedbackAngleCoeff = (3.141592654 * 1000.0 ) / 2920.0;

    jointStates[2].rawFeedback = analogRead(36);
    jointStates[2].feedbackAngle_mrads = jointStates[2].rawFeedback * feedbackAngleCoeff;
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
