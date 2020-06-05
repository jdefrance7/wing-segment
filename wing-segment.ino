//##############################################################################
// File Information
//##############################################################################



//##############################################################################
// Arduino Support Library
//##############################################################################

#include <Arduino.h>

//##############################################################################
// UAVCAN Driver and Data Types
//##############################################################################

#include <ast-uavcan.h>

//##############################################################################
// IMU Device Libraries
//##############################################################################

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//##############################################################################
// Configuration File
//##############################################################################

#include "config.h"

//##############################################################################
// IMU Class Instance
//##############################################################################

Adafruit_BNO055 sensor;

//##############################################################################
// UAVCAN Node Instance
//##############################################################################

UAVCAN_Node node;

//##############################################################################
// UAVCAN Driver Instance
//##############################################################################

UAVCAN_Driver uavcan;

//##############################################################################
// Canard Callback - On Transfer Received
//##############################################################################

static void onTransferReceived(
  CanardInstance* ins,
  CanardRxTransfer* transfer)
{
  canardGetLocalNodeID(ins);
  
  //----------------------------------------------------------------------------
  // Response Handlers

  if(transfer->transfer_type == CanardTransferTypeResponse)
  {
    // Add response handlers here...
  }

  //----------------------------------------------------------------------------
  // Request Handlers

  else if(transfer->transfer_type == CanardTransferTypeRequest)
  {
    // Add request handlers here...

    if(transfer->data_type_id == GET_NODE_INFO_DATA_TYPE_ID)
    {
      uavcan.service(&node, CanardResponse, transfer->source_node_id);
      canardReleaseRxTransferPayload(ins, transfer);
    }
  }

  //----------------------------------------------------------------------------
  // Broadcast Handlers

  else if(transfer->transfer_type == CanardTransferTypeBroadcast)
  {
    // Add braodcast handlers here...
  }

  //----------------------------------------------------------------------------
}

//##############################################################################
// Canard Callback - Should Accept Transfer
//##############################################################################

static bool shouldAcceptTransfer(
  const CanardInstance* ins,
  uint64_t* out_data_type_signature,
  uint16_t data_type_id,
  CanardTransferType transfer_type,
  uint8_t source_node_id)
{
   (void)source_node_id;

  canardGetLocalNodeID(ins);

  //----------------------------------------------------------------------------
  // Response Handlers

  if(transfer_type == CanardTransferTypeResponse)
  {
    // Add response handlers here...
  }

  //----------------------------------------------------------------------------
  // Request Handlers

  else if(transfer_type == CanardTransferTypeRequest)
  {
    // Add request handlers here...

    if(data_type_id == GET_NODE_INFO_DATA_TYPE_ID)
    {
      *out_data_type_signature = GET_NODE_INFO_DATA_TYPE_SIGNATURE;
      return true;
    }
  }

  //----------------------------------------------------------------------------
  // Broadcast Handlers

  else if(transfer_type == CanardTransferTypeBroadcast)
  {
    // Add broadcast handlers here...
  }

  //----------------------------------------------------------------------------
  // No Handlers

  return false;

  //----------------------------------------------------------------------------
}

//##############################################################################
// Setup Function
//##############################################################################

void setup()
{
  //----------------------------------------------------------------------------
  // IMU Initialization

  sensor.begin();
  delay(1000);
  sensor.setExtCrystalUse(true);

  //----------------------------------------------------------------------------
  // UAVCAN Node Initialization

  // Node Status
  node.status.uptime_sec                                  = millis()/1000;
  node.status.health                                      = HEALTH_OK;
  node.status.mode                                        = MODE_INITIALIZATION;
  node.status.sub_mode                                    = SUB_MODE;
  node.status.vendor_specific_status_code                 = VENDOR_SPECIFIC_STATUS_CODE;

  // Node Software Version
  node.software_version.major                             = SOFTWARE_VERSION_MAJOR;
  node.software_version.minor                             = SOFTWARE_VERSION_MINOR;
  node.software_version.optional_field_flags              = OPTIONAL_FIELD_FLAGS;
  node.software_version.vcs_commit                        = VCS_COMMIT;
  node.software_version.image_crc                         = IMAGE_CRC;

  // Node Hardware Version
  node.hardware_version.major                             = HARDWARE_VERSION_MAJOR;
  node.hardware_version.minor                             = HARDWARE_VERSION_MINOR;
  memcpy(node.hardware_version.unique_id,                   UNIQUE_ID,
    sizeof(UNIQUE_ID));
  memcpy(node.hardware_version.certificate_of_authenticity, CERTIFICATE,
    sizeof(CERTIFICATE));

  //----------------------------------------------------------------------------
  // UAVCAN Driver Initialization

  uavcan.setId(CAN_ID);
  uavcan.setBitrate(CAN_BITRATE);
  uavcan.setCallbacks(onTransferReceived, shouldAcceptTransfer);
  uavcan.begin();

  //----------------------------------------------------------------------------
  // Serial Module Initialization

  #ifdef SERIAL_ENABLED
  Serial.begin(SERIAL_BAUDRATE);
  while(!Serial);
  Serial.println("\nInitialization Complete!");
  #endif

  //----------------------------------------------------------------------------
}

//##############################################################################
// Main Loop
//##############################################################################

void loop()
{
  //----------------------------------------------------------------------------
  // Task 1 - Send Node Status
  // Runtime: ~450us

  // Last execution counter
  static uint64_t send_node_status_time = millis();

  // Check time for execution
  if(millis() - send_node_status_time > SEND_NODE_STATUS_PERIOD)
  {
    // Update node uptime
    node.status.uptime_sec = millis()/1000;

    // Send NodeStatus broadcast
    int frames = uavcan.broadcast(&node.status);

    #ifdef SERIAL_ENABLED
    Serial.println("\nNodeStatus");
    Serial.print("  uptime_sec: ");  Serial.println(node.status.uptime_sec);
    Serial.print("  health: ");      Serial.println(node.status.health);
    Serial.print("  mode: ");        Serial.println(node.status.mode);
    Serial.print("  sub_mode: ");    Serial.println(node.status.sub_mode);
    Serial.print("  vendor: ");      Serial.println(node.status.vendor_specific_status_code);
    Serial.print("\nBroadcast: ");   Serial.println(frames);
    #endif

    // Update execution counter if successful
    if(frames > 0)
    {
      send_node_status_time = millis();      
    }
  }

  //----------------------------------------------------------------------------
  // Task 2 - Send Orientation
  // Runtime: ~4000us

  // Last execution counter
  static uint64_t send_orientation_time = millis();

  // Check time for execution
  if(millis() - send_orientation_time > SEND_ORIENTATION_PERIOD)
  {
    // Get quaternion from IMU
    imu::Quaternion quat = sensor.getQuat();

    // Retrieve callibration status of IMU
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    sensor.getCalibration(&system, &gyro, &accel, &mag);

    // Set node to operational if callibrated
    if(system == 3)
    {
      node.status.mode = MODE_OPERATIONAL;
    }

    // Set node to initialization if not callibrated
    else
    {
      node.status.mode = MODE_INITIALIZATION;
    }

    // Creat AngularCommand structure
    AngularCommand angular_command;

    // Set gimbal_id as wing segment identification number
    angular_command.gimbal_id = SEGMENT_NUMBER;

    // Set mode as callibration status
    angular_command.mode = system;

    // Populate the quaternion
    angular_command.quaternion_xyzw[0] = quat.x();
    angular_command.quaternion_xyzw[1] = quat.y();
    angular_command.quaternion_xyzw[2] = quat.z();
    angular_command.quaternion_xyzw[3] = quat.w();

    // Send AngularCommand broadcast
    int frames = uavcan.broadcast(&angular_command);

    #ifdef SERIAL_ENABLED
    Serial.println("\nAngularCommand");
    Serial.print("  gimbal_id: ");   Serial.println(angular_command.gimbal_id);
    Serial.print("  mode: ");        Serial.println(angular_command.mode);
    Serial.print("  x: ");           Serial.println(angular_command.quaternion_xyzw[0], 3);
    Serial.print("  y: ");           Serial.println(angular_command.quaternion_xyzw[1], 3);
    Serial.print("  z: ");           Serial.println(angular_command.quaternion_xyzw[2], 3);
    Serial.print("  w: ");           Serial.println(angular_command.quaternion_xyzw[3], 3);
    Serial.print("\nBroadcast: ");   Serial.println(frames);
    #endif

    // Update execution counter if successful
    if(frames > 0)
    {
      send_orientation_time = millis();      
    }
  }


  //----------------------------------------------------------------------------
  // Task 3 - Free Memory Pool
  // Runtime: 12us

  // Last execution counter
  static uint64_t free_memory_pool_time = millis();

  // Check time for execution
  if(millis() - free_memory_pool_time > FREE_MEMORY_POOL_PERIOD)
  {
    #ifdef SERIAL_ENABLED
    CanardPoolAllocatorStatistics memory;
    uavcan.stats(&memory);
    Serial.println("\nMemory Statistics");
    Serial.print("  Capacity: ");       Serial.println(memory.capacity_blocks);
    Serial.print("  Current Usage: ");  Serial.println(memory.current_usage_blocks);
    Serial.print("  Peak Usage: ");     Serial.println(memory.peak_usage_blocks);
    #endif

    // Cleanup stale transfers
    uavcan.clean(1000*millis());

    // Update execution counter
    free_memory_pool_time = millis();
  }

  //----------------------------------------------------------------------------
}

//##############################################################################
// End of File
//##############################################################################
