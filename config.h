//##############################################################################
// File Information
//##############################################################################



//##############################################################################
// Multiple Inclusion Guard
//##############################################################################

#ifndef CONFIG_H
#define CONFIG_H

//##############################################################################
// General Information
//##############################################################################

#define NODE_NAME                     ("1.segment.wing")

#define SEGMENT_NUMBER                (1)

//##############################################################################
// CAN Driver Information
//##############################################################################

#define CAN_ID                        (SEGMENT_NUMBER+20)

#define CAN_BITRATE                   (1000000)

//##############################################################################
// Serial Module Information
//##############################################################################

#define SERIAL_ENABLED                // Uncomment to Enable, Comment to Disable

#define SERIAL_BAUDRATE               (115200)

//##############################################################################
// Task Periods
//##############################################################################

#define SEND_NODE_STATUS_PERIOD       (500)

#define SEND_ORIENTATION_PERIOD       (100)

#define FREE_MEMORY_POOL_PERIOD       (1000)

//##############################################################################
// Node Status Information
//##############################################################################

#define SUB_MODE                      (0)

#define VENDOR_SPECIFIC_STATUS_CODE   (0)

//##############################################################################
// Node Software Version Information
//##############################################################################

#define SOFTWARE_VERSION_MAJOR        (1)
#define SOFTWARE_VERSION_MINOR        (0)

#define OPTIONAL_FIELD_FLAGS          (0)

#define VCS_COMMIT                    (0)

#define IMAGE_CRC                     (0)

//##############################################################################
// Node Hardware Version Information
//##############################################################################

#define HARDWARE_VERSION_MAJOR        (0)
#define HARDWARE_VERSION_MINOR        (0)

#define UNIQUE_ID                     (NODE_NAME)

#define CERTIFICATE                   ("Certificate of Authenticity")

//##############################################################################
// Multiple Inclusion Guard
//##############################################################################

#endif // CONFIG_H

//##############################################################################
// End of File
//##############################################################################
