#pragma once
#include <stdint.h>
#include <math.h>
#include <vector>
#pragma pack(push, 1)
#pragma GCC diagnostic ignored "-Wpedantic"

#define ENDPOINT_HOST_IN    	0x81 //BULK         // camera frame read
#define ENDPOINT_HOST_OUT		0x01 //BULK         // firmware write
#define ENDPOINT_HOST_MSGS_IN	0x82 //BULK         // general communication
#define ENDPOINT_HOST_MSGS_OUT	0x02 //BULK         // general communication
#define ENDPOINT_HOST_INT_IN	0x83 //INTERRUPT    // 6DOF data read
#define ENDPOINT_HOST_INT_OUT	0x03 //INTERRUPT

typedef enum {
    SUCCESS = 0X0000,
    UNKNOWN_MESSAGE_ID = 0x0001,
    INVALID_REQUEST_LEN = 0x0002,
    INVALID_PARAMETER = 0x0003,
    INTERNAL_ERROR = 0x0004,
    UNSUPPORTED = 0x0005,
    LIST_TOO_BIG = 0x0006,
    MORE_DATA_AVAILABLE = 0x0007,
    DEVICE_BUSY = 0x0008,         /* Indicates that this command is not supported in the current device state, e.g.trying to change configuration after START */
    TIMEOUT = 0x0009,
    TABLE_NOT_EXIST = 0x000A,     /* The requested configuration table does not exists on the EEPROM */
    TABLE_LOCKED = 0x000B,        /* The configuration table is locked for writing or permanently locked from unlocking */
    DEVICE_STOPPED = 0x000C,      /* Used with DEV_STATUS messages to mark the last message over an endpoint after a DEV_STOP command */
    TEMPERATURE_WARNING = 0x0010, /* The device temperature reached 10 % from its threshold */
    TEMPERATURE_STOP = 0x0011,    /* The device temperature reached its threshold, and the device stopped tracking */
    CRC_ERROR = 0x0012,           /* CRC Error in firmware update */
    INCOMPATIBLE = 0x0013,        /* Controller version is incompatible with TM2 version */
    AUTH_ERROR = 0x0014,          /* Authentication error in firmware update */
    DEVICE_RESET = 0x0015,        /* A device reset has occurred. The user may read the FW log for additional detail */
    SLAM_NO_DICTIONARY = 0x9001,  /* No relocalization dictionary was loaded */
} MESSAGE_STATUS;

typedef enum {
    DEV_GET_DEVICE_INFO = 0x0001,
    DEV_GET_SUPPORTED_RAW_STREAMS = 0x0004,
    DEV_RAW_STREAMS_CONTROL = 0x0005,
    DEV_SAMPLE = 0x0011,
    DEV_START = 0x0012,
    DEV_STATUS = 0x0014,
    DEV_GET_POSE = 0x0015,
    DEV_SET_LOW_POWER_MODE = 0x0025,

    SLAM_6DOF_CONTROL = 0x1006,
    SLAM_GET_LOCALIZATION_DATA_STREAM = 0x1009,
} BULK_MESSAGE_ID;

enum SensorType
{
    Color = 0,
    Depth = 1,
    IR = 2,
    Fisheye = 3,
    Gyro = 4,
    Accelerometer = 5,
    Controller = 6,
    Rssi = 7,
    Velocimeter = 8,
    Stereo = 9,
    Pose = 10,
    ControllerProperty = 11,
    Mask = 12,
    Max
};

typedef struct {
        uint32_t dwLength;   /**< Message length in bytes */
        uint16_t wMessageID; /**< ID of message           */
    } bulk_message_request_header;

typedef struct {
        uint32_t dwLength;   /**< Message length in bytes            */
        uint16_t wMessageID; /**< ID of message                      */
        uint16_t wStatus;    /**< Status of request (MESSAGE_STATUS) */
    } bulk_message_response_header;

typedef struct {
    uint8_t bDeviceType;                    /**< Device identifier: 0x1 = T250                                                      */
    uint8_t bHwVersion;                     /**< ASIC Board version: 0x00 = ES0, 0x01 = ES1, 0x02 = ES2, 0x03 = ES3, 0xFF = Unknown */
    uint8_t bStatus;                        /**< Bits 0-3: device status: 0x0 = device functional, 0x1 = error, Bits 4-7: Reserved  */
    uint8_t bEepromDataMajor;               /**< Major part of the EEPROM data version                                              */
    uint8_t bEepromDataMinor;               /**< Minor part of the EEPROM data version                                              */
    uint8_t bFWVersionMajor;                /**< Major part of the Myriad firmware version                                          */
    uint8_t bFWVersionMinor;                /**< Minor part of the Myriad firmware version                                          */
    uint8_t bFWVersionPatch;                /**< Patch part of the Myriad firmware version                                          */
    uint32_t dwFWVersionBuild;              /**< Build part of the Myriad firmware version                                          */
    uint32_t dwRomVersion;                  /**< Myriad ROM version                                                                 */
    uint32_t dwStatusCode;                  /**< Status Code: S_OK = 0, E_FAIL = 1, E_NO_CALIBRATION_DATA = 1000                    */
    uint32_t dwExtendedStatus;              /**< Extended status information (details TBD)                                          */
    uint64_t llSerialNumber;                /**< Device serial number                                                               */
    uint8_t bCentralProtocolVersion;        /**< Central BLE Protocol Version                                                       */
    uint8_t bCentralAppVersionMajor;        /**< Major part of the Central firmware version                                         */
    uint8_t bCentralAppVersionMinor;        /**< Minor part of the Minor Central firmware version                                   */
    uint8_t bCentralAppVersionPatch;        /**< Patch part of the Patch Central firmware version                                   */
    uint8_t bCentralSoftdeviceVersion;      /**< Central BLE Softdevice Version                                                     */
    uint8_t bCentralBootloaderVersionMajor; /**< Major part of the Central firmware version                                         */
    uint8_t bCentralBootloaderVersionMinor; /**< Minor part of the Minor Central firmware version                                   */
    uint8_t bCentralBootloaderVersionPatch; /**< Patch part of the Patch Central firmware version                                   */
    uint32_t dwCentralAppVersionBuild;      /**< Build part of the Build Central firmware version                                   */
    uint8_t bEepromLocked;                  /**< 0x0 - The EEPROM is fully writeable                                                */
                                            /**< 0x1 - The upper quarter of the EEPROM memory is write-protected                    */
                                            /**< 0x3 - The upper quarter of the EEPROM memory is permanently write-protected        */
    uint8_t bSKUInfo;                       /**< 1 for T260 - with ble, 0 for T265 - w/o ble                                        */
} device_info_libtm_message;

typedef struct {
    uint8_t bSensorID;         /**< Bits 0-4: Type of sensor, supported values are: Color = 0, Depth = 1, IR = 2, Fisheye = 3, Gyro = 4, Accelerometer = 5 */
                               /**< Bits 5-7: Sensor index - Zero based index of sensor with the same type within device. For example if the device supports two fisheye cameras, */
                               /**<                          The first will use index 0 (bSensorID = 0x03) and the second will use index 1 (bSensorID = 0x23)                     */
    uint8_t bReserved;         /**< Reserved = 0                                                                                                                                  */
    uint16_t wWidth;           /**< Supported width (in pixels) of first stream, 0 for non-camera streams                                                                         */
    uint16_t wHeight;          /**< Supported height (in pixels) or first stream, 0 for non-camera streams                                                                        */
    uint8_t bPixelFormat;      /**< Pixel format of the stream, according to enum PixelFormat                                                                                     */
    union {                    /**<                                                                                                                                               */
        uint8_t bOutputMode;   /**< 0x0 - Send sensor outputs to the internal middlewares only, 0x1 - Send this sensor outputs also to the host over the USB interface.           */
        uint8_t bReserved2;    /**< Reserved and always 0. Sent from device to host.                                                                                              */
    };                         /**<                                                                                                                                               */
    uint16_t wStride;          /**< Length in bytes of each line in the image (including padding). 0 for non-camera streams.                                                      */
    uint16_t wFramesPerSecond; /**< Supported FPS for first stream to be enabled                                                                                                  */
} supported_raw_stream_libtm_message;

// LOW POWER MODE
typedef struct {
        bulk_message_request_header header; /**< Message request header: dwLength = 8, wMessageID = DEV_SET_LOW_POWER_MODE    */
        uint8_t bEnabled;                   /**< 1 to enable low power mode, 0 to disable                                   */
        uint8_t bReserved;                  /**< Reserved = 0                                                               */
    } bulk_message_request_set_low_power_mode;

typedef struct {
        bulk_message_response_header header; /**< Message response header: dwLendth = 8, wMessageID = DEV_SET_LOW_POWER_MODE  */
    } bulk_message_response_set_low_power_mode;

// DEVICE INFORMATION
typedef struct {
    bulk_message_request_header header; /**< Message request header: dwLength = 6 bytes, wMessageID = DEV_GET_DEVICE_INFO */
} bulk_message_request_get_device_info;

typedef struct {
    bulk_message_response_header header; /**< Message response header: dwLength = 49 or 8 bytes, wMessageID = DEV_GET_DEVICE_INFO */
    device_info_libtm_message message;   /**< Device info                                                                         */
} bulk_message_response_get_device_info;

// RAW STREAM CONTROL
typedef struct {
    bulk_message_request_header header;          /**< Message request header: wMessageID = DEV_RAW_STREAMS_CONTROL */
    uint16_t wNumEnabledStreams;                 /**< Number of enabled streams in list below                      */
    supported_raw_stream_libtm_message stream[2]; /**< Supported stream info variable sized array                   */
} bulk_message_request_raw_streams_control;

typedef struct {
    bulk_message_response_header header; /**< Message response header: dwLength = 8 bytes, wMessageID = DEV_RAW_STREAMS_CONTROL */
} bulk_message_response_raw_streams_control;

// 6DOF CONTROL
typedef struct {
    bulk_message_request_header header; /**< Message request header: dwLength = 9 bytes, wMessageID = SLAM_6DOF_CONTROL                       */
    uint8_t bEnable;                    /**< 0x00 - Disable 6DoF, 0x01 - Enable 6DoF                                                          */
    uint8_t bMode;                      /**< 0x00 - Normal Mode, 0x01 - Fast Playback, 0x02 - Mapping Enabled , 0x04 - Relocalization Enabled */
} bulk_message_request_6dof_control;

typedef struct {
    bulk_message_response_header header; /**< Message response header: dwLength = 8 bytes, wMessageID = SLAM_6DOF_CONTROL */
} bulk_message_response_6dof_control;

// GET SUPPORTED RAW STREAMS
typedef struct {
    bulk_message_request_header header; /**< Message request header: dwLength = 6 bytes, wMessageID = DEV_GET_SUPPORTED_RAW_STREAMS */
} bulk_message_request_get_supported_raw_streams;

typedef struct {
    bulk_message_response_header header;         /**< Message response header: wMessageID = DEV_GET_SUPPORTED_RAW_STREAMS */
    uint16_t wNumSupportedStreams;               /**< Number of supported streams in list below                           */
    uint16_t wReserved;                          /**< Reserved = 0                                                        */
    supported_raw_stream_libtm_message stream[]; /**< Supported stream info variable sized array                          */
} bulk_message_response_get_supported_raw_streams;

// START DEVICE
typedef struct {
    bulk_message_request_header header; /**< Message request header: dwLength = 6 bytes, wMessageID = DEV_START */
} bulk_message_request_start;

typedef struct {
    bulk_message_response_header header; /**< Message request header: dwLength = 8 bytes, wMessageID = DEV_START */
} bulk_message_response_start;

//VIDEO STREAM PACKET
typedef struct {
    bulk_message_request_header header; /**< Message request header: dwLength = 28 + dwMetadataLength + dwFrameLength bytes, wMessageID = DEV_SAMPLE                                       */
    uint8_t bSensorID;                  /**< Bits 0-4: Type of sensor, supported values are: Color = 0, Depth = 1, IR = 2, Fisheye = 3, Gyro = 4, Accelerometer = 5 */
                                        /**< Bits 5-7: Sensor index - Zero based index of sensor with the same type within device. For example if the device supports two fisheye cameras, */
                                        /**<                          The first will use index 0 (bSensorID = 0x03) and the second will use index 1 (bSensorID = 0x23)                     */
    uint8_t bReserved;                  /**< Reserved = 0                                                                                                                                  */
    uint64_t llNanoseconds;             /**< Frame integration timestamp, as measured in nanoseconds since device initialization                                                           */
    uint64_t llArrivalNanoseconds;      /**< Frame arrival timestamp, as measured in nanoseconds since device initialization                                                               */
    uint32_t dwFrameId;                 /**< A running index of frames from every unique sensor. Starting from 0.                                                                          */
} bulk_message_raw_stream_header;

typedef struct
{
    uint32_t dwMetadataLength; /**< Metadata length in bytes (8 bytes)                                       */
    uint32_t dwExposuretime;   /**< Exposure time of this frame in microseconds                              */
    float_t fGain;             /**< Gain multiplier of this frame                                            */
    uint32_t dwFrameLength;    /**< Length of frame below, in bytes, shall be equal to Stride X Height X BPP */
    uint8_t bFrameData[800*848];      /**< Frame data variable sized array                                          */
} bulk_message_video_stream_metadata;

typedef struct
{
    bulk_message_raw_stream_header rawStreamHeader;
    bulk_message_video_stream_metadata metadata;
} bulk_message_video_stream;

//6DOF INTERRUPT 
typedef struct {
        uint32_t dwLength;    /**< Message length in bytes */
        uint16_t wMessageID;  /**< ID of message           */
    } interrupt_message_header;

typedef struct {
    float_t flX;                  /**< X value of translation, in meters (relative to initial position)                              */
    float_t flY;                  /**< Y value of translation, in meters (relative to initial position)                              */
    float_t flZ;                  /**< Z value of translation, in meters (relative to initial position)                              */
    float_t flQi;                 /**< Qi component of rotation as represented in quaternion rotation (relative to initial position) */
    float_t flQj;                 /**< Qj component of rotation as represented in quaternion rotation (relative to initial position) */
    float_t flQk;                 /**< Qk component of rotation as represented in quaternion rotation (relative to initial position) */
    float_t flQr;                 /**< Qr component of rotation as represented in quaternion rotation (relative to initial position) */
    float_t flVx;                 /**< X value of velocity, in meter/sec                                                             */
    float_t flVy;                 /**< Y value of velocity, in meter/sec                                                             */
    float_t flVz;                 /**< Z value of velocity, in meter/sec                                                             */
    float_t flVAX;                /**< X value of angular velocity, in RAD/sec                                                       */
    float_t flVAY;                /**< Y value of angular velocity, in RAD/sec                                                       */
    float_t flVAZ;                /**< Z value of angular velocity, in RAD/sec                                                       */
    float_t flAx;                 /**< X value of acceleration, in meter/sec^2                                                       */
    float_t flAy;                 /**< Y value of acceleration, in meter/sec^2                                                       */
    float_t flAz;                 /**< Z value of acceleration, in meter/sec^2                                                       */
    float_t flAAX;                /**< X value of angular acceleration, in RAD/sec^2                                                 */
    float_t flAAY;                /**< Y value of angular acceleration, in RAD/sec^2                                                 */
    float_t flAAZ;                /**< Z value of angular acceleration, in RAD/sec^2                                                 */
    uint64_t llNanoseconds;       /**< Timestamp of pose, measured in nanoseconds since device system initialization                 */
    uint32_t dwTrackerConfidence; /**< pose data confidence 0x0 - Failed, 0x1 - Low, 0x2 - Medium, 0x3 - High                        */
    uint32_t dwMapperConfidence;  /**< Bits 0-1: 0x0 - Failed, 0x1 - Low, 0x2 - Medium, 0x3 - High, Bits 2-31: Reserved              */
    uint32_t dwTrackerState;      /**< tracker state 0x0 - Inactive, 0x3 Active 3DOF, 0x4 Active 6DOF, 0x7 Inertial only 3DOF        */
} pose_data;

typedef struct {
    interrupt_message_header header; /**< Interrupt message header: dwLength = 92 bytes, wMessageID = DEV_GET_POSE       */
    uint8_t bIndex;                  /**< Index of HMD - 0x0 = HMD */
    uint8_t wReserved;               /**< Reserved = 0                                                                   */
    pose_data pose;                  /**< Short low-latency data (namely 6DoF pose data)                                 */
} interrupt_message_get_pose;

std::vector<supported_raw_stream_libtm_message> _active_raw_streams;

template<typename Request, typename Response> int bulk_request_response(const Request &request, Response &response, size_t max_response_size = 0, bool assert_success = true);
