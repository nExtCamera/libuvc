
#include <unistd.h>
#include "doctest.h"
#include "libuvc/libuvc.h"
#include "libuvc/libuvc_internal.h"
#include "mock_uvc.h"

struct libusb_config_descriptor mock_config_desc = {
    .bLength = sizeof(struct libusb_config_descriptor)
};

uvc_streaming_interface_t stream_if = {
    .bEndpointAddress = 1,
    .bTerminalLink = 0,
    .bStillCaptureMethod = 0
};

uvc_device_info_t mock_dev_info = {
    .config = &mock_config_desc,
    .ctrl_if = {},
    .stream_ifs = &stream_if
};

static uvc_device_handle_t* mock_device_handle() {
    uvc_device_handle_t *devh = static_cast<uvc_device_handle_t*>(calloc(1, sizeof(uvc_device_handle_t)));
    devh->info = &mock_dev_info;
    return devh;
}

TEST_CASE("opening a stream") {
    // given
    uvc_device_handle_t *devh = mock_device_handle();

    // when
    uvc_stream_ctrl_t ctrl;
    uvc_stream_handle_t *strmh;
    REQUIRE(uvc_stream_open_ctrl(devh, &strmh, &ctrl) == UVC_SUCCESS);
}
