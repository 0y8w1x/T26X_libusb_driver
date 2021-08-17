#include <libusb-1.0/libusb.h>
#include <iostream>
#include "krampf.hpp"

// current firmware https://librealsense.intel.com/Releases/TM2/FW/target/0.2.0.951/target-0.2.0.951.mvcmd

static const int USB_TIMEOUT = 10000; // in ms

int write_firmware(libusb_device* device, unsigned char* firmware_buffer, unsigned int size) {
	libusb_device_handle* device_handle = NULL;
	if (libusb_open(device, &device_handle)) {
		printf("COULDN'T GET THE DEVICE HANDLE\n");
		return -1;
	}
	if (libusb_claim_interface(device_handle, 0)) {
		printf("COULDN'T CLAIM THE DEVICE HANDLE\n");
		return -1;
	}
	int transferred = 0;
	int error = libusb_bulk_transfer(device_handle, ENDPOINT_HOST_OUT, firmware_buffer, size, &transferred, USB_TIMEOUT);
	printf("[FIRMWARE]\t\ttransferred: %d bytes\n", transferred);
	if (error || transferred != size) {
		printf("ERROR: Couldn't write firmware\n");
		return -1;
	}
	libusb_release_interface(device_handle, 0);
	libusb_close(device_handle);
	return 0;
}


long get_filesize(FILE* file_pointer) {
	long filesize;
	
	if (fseek(file_pointer, 0, SEEK_END) != 0) {
		exit(EXIT_FAILURE);
	}
	filesize = ftell(file_pointer);
	rewind(file_pointer);

	return filesize;
}

int main(int argc, char *argv[]) {
    libusb_init(NULL);

    libusb_device* device;
    struct libusb_device_descriptor device_descriptor;
    libusb_device **device_list = NULL;
    int count = libusb_get_device_list(NULL, &device_list);

    device = device_list[6];
    libusb_get_device_descriptor(device, &device_descriptor);
    printf("[DEVICE]\t\t0x%x:0x%x\n", device_descriptor.idVendor, device_descriptor.idProduct);

    long filesize;
	unsigned char* firmware_buffer;
    FILE* file_pointer;
    file_pointer = fopen("../firmware/target-0.2.0.951.mvcmd", "rb");

    if (file_pointer == NULL) {
		printf("FILE NOT FOUND\n");
		return -1;
	}

    // get the firmware size
	filesize = get_filesize(file_pointer);
	if (filesize < 1) exit(EXIT_FAILURE);

    // allocate memory buffer
	firmware_buffer = (unsigned char*)malloc(filesize * sizeof(unsigned char));
	if (firmware_buffer == NULL) {
		exit(EXIT_FAILURE);
	}

    // write firmware to memory buffer
	if ((fread(firmware_buffer, sizeof(unsigned char), filesize, file_pointer)) != filesize) {
		exit(EXIT_FAILURE);
	}
    write_firmware(device, firmware_buffer, filesize);
	fclose(file_pointer);
	free(firmware_buffer);

    return 0;
}