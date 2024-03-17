/*
    Configuration Module
  ------------------------

  Handles loading and saving robot config to ESP32 non-volatile memory, as
  well as reading and writing it over the packet protocol.

*/
#ifndef __CONFIGURATION_HPP__
#define __CONFIGURATION_HPP__
#include "nvs_flash.h"
#include "robot_hw_interface.hpp"
#include "packet_defs.hpp"

class RobotConfig
{
public:
  RobotConfig(RobotHWInterface* robotInterface) : robotInterface(robotInterface) 
  {
    nvsFlashInitialised = false;
    configurationLoaded = false;
    esp_err_t initRes = nvs_flash_init();

    strcpy(description, "Robot description new value.");

    if (initRes == ESP_OK) {
      Serial.println("nvs_flash_init returned okay.");
      nvsFlashInitialised = true;
    }
    else if (initRes == ESP_ERR_NVS_NO_FREE_PAGES) {
      Serial.println("nvs_flash_init failed with ESP_ERR_NVS_NO_FREE_PAGES.");
      return;
    }
    else if (initRes == ESP_ERR_NOT_FOUND) {
      Serial.println("nvs_flash_init failed with ESP_ERR_NOT_FOUND .");
      return;
    }
    else if (initRes == ESP_ERR_NO_MEM) {
      Serial.println("nvs_flash_init failed with ESP_ERR_NO_MEM.");
      return;
    }
    else {
      Serial.print("nvs_flash_init failed with unrecognised error [");
      Serial.print(initRes);
      Serial.println("].");
      return;
    }
  };

  bool readStoredConfig()
  {
    // return failure if NVM flash was not setup
    if (!nvsFlashInitialised)
      return false;

    // attempt to open the default vns partition for reading
    nvs_handle_t nvsHandle;
    esp_err_t openRes = nvs_open("Hexapod", NVS_READONLY, &nvsHandle);
    if (openRes != ESP_OK) {
      Serial.print("nvs_open failed with error code [");
      Serial.print(openRes);
      Serial.print(" - ");
      switch(openRes) {
        case ESP_FAIL: Serial.print("ESP_FAIL"); break;
        case ESP_ERR_NVS_NOT_INITIALIZED: Serial.print("ESP_ERR_NVS_NOT_INITIALIZED"); break;
        case ESP_ERR_NVS_PART_NOT_FOUND: Serial.print("ESP_ERR_NVS_PART_NOT_FOUND"); break;
        case ESP_ERR_NVS_NOT_FOUND: Serial.print("ESP_ERR_NVS_NOT_FOUND"); break;
        case ESP_ERR_NVS_INVALID_NAME: Serial.print("ESP_ERR_NVS_INVALID_NAME"); break;
        case ESP_ERR_NO_MEM: Serial.print("ESP_ERR_NO_MEM"); break;
        case ESP_ERR_NVS_NOT_ENOUGH_SPACE: Serial.print("ESP_ERR_NVS_NOT_ENOUGH_SPACE"); break;
        //case ESP_ERR_NOT_ALLOWED: Serial.print("ESP_ERR_NOT_ALLOWED"); break;
        case ESP_ERR_INVALID_ARG: Serial.print("ESP_ERR_INVALID_ARG"); break;
      }
      Serial.println("].");
      return false;
    }

    bool readOkay = true;

    // temp values to hold configuration while it was loaded.
    // Final locations are only written once the full config has been successfully read from nvs
    char tempDescription[descriptionLength];

    // attempt to read robot description 
    size_t length = descriptionLength;
    esp_err_t labelReadRes = nvs_get_str(nvsHandle, "Description", tempDescription, &length);
    if (labelReadRes != ESP_OK) {
      Serial.print("reading robot description string failed with error code [");
      Serial.print(labelReadRes);
      Serial.println("].");
      readOkay = false;
    }
    else {
      Serial.print("Read description from nvs: ");
      Serial.print(tempDescription);
      Serial.println("");
    }

    void nvs_close(nvs_handle_t nvsHandle);

    if (readOkay) {
      strncpy(description, tempDescription, descriptionLength);
      configurationLoaded = true;
    }

    return readOkay;
  }

  bool writeStoredConfig()
  {
    // return failure if NVM flash was not setup
    if (!nvsFlashInitialised)
      return false;

    // attempt to open the default vns partition for reading and writing
    nvs_handle_t nvsHandle;
    esp_err_t openRes = nvs_open("Hexapod", NVS_READWRITE, &nvsHandle);
    if (openRes != ESP_OK) {
      Serial.print("nvs_open failed with error code [");
      Serial.print(openRes);
      Serial.println("].");
      return false;
    }

    bool writeOkay = true;

    // attempt to write robot description 
    esp_err_t labelWriteRes = nvs_set_str(nvsHandle, "Description", description);
    if (labelWriteRes != ESP_OK) {
      Serial.print("writing robot description string failed with error code [");
      Serial.print(labelWriteRes);
      Serial.println("].");
      writeOkay = false;
    }

    void nvs_close(nvs_handle_t nvsHandle);

    return writeOkay;
  }

  void setFromPacket(std::shared_ptr<TCWriteRobotConfig> configPacket) {
    strncpy(description, configPacket->description, 30);
    for (int i=0; i<18; ++i) {
      robotInterface->joint_feedback_mins[i] = configPacket->jointFeedbackMins[i];
      robotInterface->joint_feedback_maxs[i] = configPacket->jointFeedbackMaxs[i];
    }
  }

  bool getNVSFlashInitialised() { return nvsFlashInitialised; };
  bool getConfigurationLoaded() { return configurationLoaded; };

private:

  static constexpr size_t descriptionLength = 30;
  char description[descriptionLength];

  bool nvsFlashInitialised;
  bool configurationLoaded;

  RobotHWInterface* robotInterface;
};


#endif // __CONFIGURATION_HPP__
