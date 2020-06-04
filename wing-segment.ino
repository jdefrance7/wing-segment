//------------------------------------------------------------------------------
// File Information



//------------------------------------------------------------------------------
// Arduino Support Library

#include <Arduino.h>

//------------------------------------------------------------------------------
// UAVCAN Driver and Data Types

#include <ast-uavcan.h>

//------------------------------------------------------------------------------
// IMU Device Libraries

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//------------------------------------------------------------------------------
// Configuration File

#include "config.h"

//------------------------------------------------------------------------------
// IMU Class Instance

Adafruit_BNO055 sensor;

//------------------------------------------------------------------------------
// UAVCAN Node Instance

UAVCAN_Node node;

//------------------------------------------------------------------------------
// UAVCAN Driver Instance

UAVCAN_Driver uavcan;

//------------------------------------------------------------------------------
// Canard Callback - On Transfer Received

static void onTransferReceived(
  CanardInstance* ins,
  CanardRxTransfer* transfer)
{
  canardGetLocalNodeID(ins);

  if(transfer->transfer_type == CanardTransferTypeResponse)
  {
    // Add response handlers here...
  }
  else if(transfer->transfer_type == CanardTransferTypeRequest)
  {
    // Add request handlers here...     
  
    if(transfer->data_type_id == GET_NODE_INFO_DATA_TYPE_ID)
    {
      uavcan.service(&node, CanardResponse, transfer->source_node_id);
      canardReleaseRxTransferPayload(ins, transfer);
    }
  }
  else if(transfer->transfer_type == CanardTransferTypeBroadcast)
  {
    // Add braodcast handlers here...
  }
}

//------------------------------------------------------------------------------
// Canard Callback - Should Accept Transfer

static bool shouldAcceptTransfer(
  const CanardInstance* ins,
  uint64_t* out_data_type_signature,
  uint16_t data_type_id,
  CanardTransferType transfer_type,
  uint8_t source_node_id)
{
  (void)source_node_id;

  canardGetLocalNodeID(ins);

  if(transfer_type == CanardTransferTypeResponse)
  {
    // Add response handlers here...
  }
  else if(transfer_type == CanardTransferTypeRequest)
  {
    // Add request handlers here...     
    
    if(data_type_id == GET_NODE_INFO_DATA_TYPE_ID)
    {
      *out_data_type_signature = GET_NODE_INFO_DATA_TYPE_SIGNATURE;
      return true;
    }
  }
  else if(transfer_type == CanardTransferTypeBroadcast)
  {
    // Add broadcast handlers here...
  }

  return false;
}


//------------------------------------------------------------------------------
// Setup Function

void setup()
{
  sensor.begin();
  delay(1000);
  sensor.setExtCrystalUse(true);
  
  node.status.uptime_sec = millis();
  node.status.health = HEALTH_OK;
  node.status.mode = MODE_INITIALIZATION;
  node.status.sub_mode = SUB_MODE;
  node.status.vendor_specific_status_code = VENDOR_SPECIFIC_STATUS_CODE;

  node.software_version.major = SOFTWARE_VERSION_MAJOR;
  node.software_version.minor = SOFTWARE_VERSION_MINOR;
  node.software_version.optional_field_flags = OPTIONAL_FIELD_FLAGS;
  node.software_version.vcs_commit = VCS_COMMIT;
  node.software_version.image_crc = IMAGE_CRC;

  node.hardware_version.major = HARDWARE_VERSION_MAJOR;
  node.hardware_version.minor = HARDWARE_VERSION_MINOR;
  memset(node.hardware_version.unique_id, 0, 16);
  memcpy(node.hardware_version.unique_id, UNIQUE_ID, sizeof(UNIQUE_ID));
  memset(node.hardware_version.certificate_of_authenticity, 0, 255);
  memcpy(node.hardware_version.certificate_of_authenticity, CERTIFICATE, sizeof(CERTIFICATE));

  uavcan.setId(CAN_ID);
  uavcan.setBitrate(CAN_BITRATE);
  uavcan.setCallbacks(onTransferReceived, shouldAcceptTransfer);
  uavcan.begin();

  Serial.begin(SERIAL_BAUDRATE);

  node.status.mode = MODE_OPERATIONAL;

  Serial.println("Initialization Complete!");
}

//------------------------------------------------------------------------------
// Main Loop

void loop()
{
  //----------------------------------------------------------------------------
  // Task 1 - Send Node Status

  static uint64_t send_node_status_time = millis();

  if(millis() - send_node_status_time > SEND_NODE_STATUS_PERIOD)
  {
    node.status.uptime_sec = millis()/1000;

    Serial.println("\nNodeStatus");
    Serial.print("  uptime_sec: "); Serial.println(node.status.uptime_sec);
    Serial.print("  health: "); Serial.println(node.status.health);
    Serial.print("  mode: "); Serial.println(node.status.mode);
    Serial.print("  sub_mode: "); Serial.println(node.status.sub_mode);
    Serial.print("  vendor: "); Serial.println(node.status.vendor_specific_status_code);

    Serial.print("\nBroadcast: "); Serial.println(uavcan.broadcast(&node.status));

    send_node_status_time = millis();
  }

  //----------------------------------------------------------------------------
  // Task 2 - Send Orientation

  static uint64_t send_orientation_time = millis();

  if(millis() - send_orientation_time > SEND_ORIENTATION_PERIOD)
  {
    imu::Quaternion quat = sensor.getQuat();

    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    sensor.getCalibration(&system, &gyro, &accel, &mag);

    AngularCommand angular_command;

    angular_command.gimbal_id = SEGMENT_NUMBER;

    angular_command.mode = system;

    angular_command.quaternion_xyzw[0] = quat.x();
    angular_command.quaternion_xyzw[1] = quat.y();
    angular_command.quaternion_xyzw[2] = quat.z();
    angular_command.quaternion_xyzw[3] = quat.w();

    Serial.println("\nAngularCommand");
    Serial.print("  gimbal_id: "); Serial.println(angular_command.gimbal_id);
    Serial.print("  mode: "); Serial.println(angular_command.mode);
    Serial.print("  x: "); Serial.println(angular_command.quaternion_xyzw[0], 3);
    Serial.print("  y: "); Serial.println(angular_command.quaternion_xyzw[1], 3);
    Serial.print("  z: "); Serial.println(angular_command.quaternion_xyzw[2], 3);
    Serial.print("  w: "); Serial.println(angular_command.quaternion_xyzw[3], 3);

    Serial.print("\nBroadcast: "); Serial.println(uavcan.broadcast(&angular_command));

    send_orientation_time = millis();
  }


  //----------------------------------------------------------------------------
  // Task 3 - Free Memory Pool

  static uint64_t free_memory_pool_time = millis();

  if(millis() - free_memory_pool_time > FREE_MEMORY_POOL_PERIOD)
  {
    uavcan.clean(1000*millis());
    
    free_memory_pool_time = millis();
  }

  //----------------------------------------------------------------------------
}
