#include <opencv2/opencv.hpp>
#include <libusb-1.0/libusb.h>
#include "krampf.hpp"
#include <iostream>
#include <memory.h>
#include <thread>


#define SENSOR_TYPE_POS (0)
#define SENSOR_TYPE_MASK (0x001F << SENSOR_TYPE_POS)
#define SENSOR_INDEX_POS (5)
#define SENSOR_INDEX_MASK (0x0007 << SENSOR_INDEX_POS)
#define GET_SENSOR_INDEX(_sensorID) ((_sensorID & SENSOR_INDEX_MASK) >> SENSOR_INDEX_POS)
#define GET_SENSOR_TYPE(_sensorID) ((_sensorID & SENSOR_TYPE_MASK) >> SENSOR_TYPE_POS)

static struct libusb_context *usb_context;
static struct libusb_device_handle *device_handle;

static const int BUFFER_SIZE = 1024; // Max size for control transfers
static const int MAX_TRANSFER_SIZE = sizeof(bulk_message_video_stream);
static const int POSE_DATA_SIZE = sizeof(interrupt_message_get_pose);
static const int USB_TIMEOUT = 10000; // in ms

static unsigned char image_buffer[MAX_TRANSFER_SIZE];
static unsigned char pose_buffer[POSE_DATA_SIZE];

uint8_t left_frame[800*848];
uint8_t right_frame[800*848];

template<typename Request, typename Response>
int bulk_request_response(const Request &request, Response &response, size_t max_response_size, bool assert_success) {
    // request
    uint32_t length = request.header.dwLength;
    uint16_t message_id = request.header.wMessageID;
    std::cout << "Sending message with length: " << length << "\n";
    int error;
    int transferred = 0;
    error = libusb_bulk_transfer(device_handle, ENDPOINT_HOST_MSGS_OUT, (unsigned char*)&request, sizeof(request), &transferred, 0);
    if (error != LIBUSB_SUCCESS) {
		std::cerr << "[ERROR] Bulk request error\n";
        return error;
    }
    if (transferred != length) {
        std::cerr << "[ERROR] sent " << transferred << " not " << length << "\n";
        // return platform::RS2_USB_STATUS_OTHER
        return -13;
    }

    // response
    if(max_response_size == 0)
        max_response_size = sizeof(response);
    std::cout << "Receiving message with max_response_size " << max_response_size << "\n";

    transferred = 0;
    error = libusb_bulk_transfer(device_handle, ENDPOINT_HOST_MSGS_IN, (unsigned char*)&response, max_response_size, &transferred, 0);
    if (error != LIBUSB_SUCCESS) {
		std::cerr << "[ERROR] Bulk response error: " << libusb_error_name(error) << "\n";
        return error;
    }
    if (transferred != response.header.dwLength) {
        std::cerr << "[ERROR] Received " << transferred << " but header was " << response.header.dwLength << " bytes (max_response_size was " << max_response_size << ")\n";
        // return platform::RS2_USB_STATUS_OTHER
        return -13;
    }

    if (assert_success && MESSAGE_STATUS(response.header.wStatus) != MESSAGE_STATUS::SUCCESS) {
        std::cerr << "[ERROR] Received " << response.header.dwLength << "bytes but got non-zero status";
        //LOG_ERROR("Received " << message_name(response.header) << " with length " << response.header.dwLength << " but got non-zero status of " << status_name(response.header));
    }
    //LOG_DEBUG("Received " << message_name(response.header) << " with length " << response.header.dwLength);
    std::cout << "[SUCCESS] Received " << response.header.dwLength << " bytes\n";

    return error;
}

void LIBUSB_CALL stream_callback(struct libusb_transfer *transfer) {
    auto video_data = (bulk_message_video_stream *)transfer->buffer;
    std::cout << "frame at " << video_data->rawStreamHeader.llNanoseconds/1e9 << "s after boot\n";

    // left fisheye
    if (video_data->rawStreamHeader.bSensorID == 0x03) {
        memcpy(left_frame, video_data->metadata.bFrameData, sizeof(left_frame));
    // right fisheye
    } else if (video_data->rawStreamHeader.bSensorID == 0x23) {
        memcpy(right_frame, video_data->metadata.bFrameData, sizeof(right_frame));
    }

    libusb_submit_transfer(transfer);
}

void LIBUSB_CALL interrupt_callback(struct libusb_transfer *transfer) {
    auto pose_data = (interrupt_message_get_pose *)transfer->buffer;
    std::cout << "pose at " << pose_data->pose.llNanoseconds/1e9 << "s after boot\n";
    
    libusb_submit_transfer(transfer);
}

void show_image_stream()
{
    cv::Mat merged_image;
    while (true) {
        cv::Mat image_left(cv::Size(848, 800), CV_8UC1, left_frame, cv::Mat::AUTO_STEP);
        cv::Mat image_right(cv::Size(848, 800), CV_8UC1, right_frame, cv::Mat::AUTO_STEP);
/*
        cv::fisheye::undistortImage(image_distorted_left, image_undistorted_left, K_left, D_left, P_left, stereo_size);
        cv::fisheye::undistortImage(image_distorted_right, image_undistorted_right, K_right, D_right, P_right, stereo_size);
*/
        cv::hconcat(image_left, image_right, merged_image);

        cv::imshow("window", merged_image);
        cv::waitKey(1);
    }
}

int main(int argc, char * argv[]) {

    // init
    libusb_init(&usb_context);
    libusb_set_option(usb_context, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
    device_handle = libusb_open_device_with_vid_pid(usb_context, 0x8087, 0x0B37);
    libusb_claim_interface(device_handle, 0);

    // disable low power mode
    bulk_message_request_set_low_power_mode power_request = {{ sizeof(power_request), DEV_SET_LOW_POWER_MODE }};
    bulk_message_response_set_low_power_mode power_response = {};
    power_request.bEnabled = 0; // disable low power mode
    auto response = bulk_request_response(power_request, power_response);

    // get supported streams
    bulk_message_request_get_supported_raw_streams supported_raw_streams_request = {{ sizeof(supported_raw_streams_request), DEV_GET_SUPPORTED_RAW_STREAMS }};
    char supported_raw_streams_buffer[BUFFER_SIZE];
    auto supported_raw_streams_response = (bulk_message_response_get_supported_raw_streams *)supported_raw_streams_buffer;
    bulk_request_response(supported_raw_streams_request, *supported_raw_streams_response, BUFFER_SIZE);
    _active_raw_streams.clear();
    for(int i = 0; i < supported_raw_streams_response->wNumSupportedStreams; i++) {
            auto tm_stream = supported_raw_streams_response->stream[i];

            auto sensor_type = GET_SENSOR_TYPE(tm_stream.bSensorID);
            auto sensor_id   = GET_SENSOR_INDEX(tm_stream.bSensorID);
            if(sensor_type == SensorType::Fisheye) {
                tm_stream.bOutputMode = 1;
                _active_raw_streams.push_back(tm_stream);
            }
    }

    // enable pose
    bulk_message_request_6dof_control control_request = {{ sizeof(control_request), SLAM_6DOF_CONTROL }};
    control_request.bEnable = true;
    control_request.bMode = 0x06; // TODO: figure out what that is
    bulk_message_response_6dof_control control_response = {};
    bulk_request_response(control_request, control_response, sizeof(control_response), false);

    // enable fisheye streams
    char streams_buffer[BUFFER_SIZE];
    auto request = (bulk_message_request_raw_streams_control *)streams_buffer;
    request->header.wMessageID = DEV_RAW_STREAMS_CONTROL;
    request->wNumEnabledStreams = 2; // pose stream doesn't count here // maybe gyro and accel for pose?
    memcpy(request->stream, _active_raw_streams.data(), request->wNumEnabledStreams*sizeof(supported_raw_stream_libtm_message));
    request->header.dwLength = request->wNumEnabledStreams * sizeof(supported_raw_stream_libtm_message) + sizeof(request->header) + sizeof(request->wNumEnabledStreams);
    bulk_message_response_raw_streams_control raw_stream_response;
    bulk_request_response(*request, raw_stream_response, sizeof(raw_stream_response), false);
    
    // request pose data
    struct libusb_transfer *pose_interrupt_transfer = libusb_alloc_transfer(0);
    libusb_fill_interrupt_transfer(pose_interrupt_transfer, device_handle, ENDPOINT_HOST_INT_IN, pose_buffer, sizeof(pose_buffer), interrupt_callback, NULL, 0);
    libusb_submit_transfer(pose_interrupt_transfer);

    // request camera streams
    struct libusb_transfer *video_stream_transfer = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(video_stream_transfer, device_handle, ENDPOINT_HOST_IN, image_buffer, sizeof(image_buffer), stream_callback, NULL, 0);
    libusb_submit_transfer(video_stream_transfer);
    
    // start the device
    bulk_message_request_start dev_start_request = {{ sizeof(dev_start_request), DEV_START }};
    bulk_message_response_start dev_start_response = {};
    bulk_request_response(dev_start_request, dev_start_response, sizeof(dev_start_response), false);

    // thread to show image
    std::thread image_stream_thread(show_image_stream);

    // handle the transfer submission
    while(true) {
        libusb_handle_events(usb_context);
    }

    // clean everything up
    libusb_release_interface(device_handle, 0);
    libusb_close(device_handle);
    libusb_exit(usb_context);
    
    return 0;
}
