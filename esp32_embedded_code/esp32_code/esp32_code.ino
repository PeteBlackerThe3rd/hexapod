#include "comms_thread.hpp"

class RobotInterface
{
public:
  
  enum class MotorState { On = 1, Off = 0 };

  class JointState {
  public:
    uint16_t rawFeedback;
    uint16_t current;
    uint16_t drivePosition;
    uint16_t goalFeedbackPos;
  };
  
  RobotInterface() {
    motorState == MotorState::Off;

    for (unsigned int j=0; j<18; ++j) {
      jointStates[j].drivePosition = 32767; // 50% of servo PWM range for now
    }
  };

  void readJointStates() {

    jointStates[0].rawFeedback = analogRead(36);
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
uint32_t lastMotorTMTime = 0;

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
    if (timeNow - lastHKTMTime > 250) {
      lastHKTMTime = timeNow;

      robotInterface->readJointStates();

      auto hkPacket = new TMHouseKeepingPacket(++lastHKSeqId);
      hkPacket->socketConnectionCount = 5;
      hkPacket->motorsOn = static_cast<uint8_t>(robotInterface->motorState);
      hkPacket->batteryVoltage100thsVolt = robotInterface->jointStates[0].rawFeedback; 

      commsThread->sendPacket(BasePacketPtr(hkPacket));
    }
  }
}
