#include "comms_thread.hpp"
#include "robot_hw_interface.hpp"
#include "configuration.hpp"

CommsThread* commsThread;
RobotHWInterface* robotHWInterface;
RobotConfig* robotConfig;

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
  robotHWInterface = new RobotHWInterface;
  robotConfig = new RobotConfig(robotHWInterface);
}

void loop()
{
  //Serial.println("Something to stop core#1 watchdog triggering");
  delay(10);

  while (commsThread->getPacketRxQueueSize() > 0) {
    BasePacketPtr packet = commsThread->popPacket();
    Serial.print("Handling: ");
    Serial.println(packet->toString().c_str());

    if (packet->getId() == TCManualJointPositionPacket::packetId) {
      auto manualJointPositionPacket = std::static_pointer_cast<TCManualJointPositionPacket>(packet);
      Serial.println("Handling manual joint position packet");
      Serial.println(manualJointPositionPacket->toString().c_str());
    }

    if (packet->getId() == TCWriteRobotConfig::packetId) {
      auto writeRobotConfig = std::static_pointer_cast<TCWriteRobotConfig>(packet);
      Serial.println("Handling TCWriteRobotConfig packet");
      robotConfig->setFromPacket(writeRobotConfig);
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

      hkPacket->flags = 0;
      hkPacket->flags |= static_cast<uint32_t>(robotHWInterface->motorState);
      if (robotConfig->getNVSFlashInitialised())
        hkPacket->flags |= 0x2;
      if (robotConfig->getConfigurationLoaded())
        hkPacket->flags |= 0x4;

      hkPacket->batteryVoltage100thsVolt = robotHWInterface->jointStates[0].rawFeedback; 

      commsThread->sendPacket(BasePacketPtr(hkPacket));
    }

    if (timeNow - lastRSTMTime > 100) {
      lastRSTMTime = timeNow;

      robotHWInterface->readJointStates();

      auto robotStatePacket = new TMRobotStatePacket(robotHWInterface->jointStates);
      commsThread->sendPacket(BasePacketPtr(robotStatePacket));
    }
  }
}
