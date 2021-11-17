#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "libuvc/libuvc.h"
#include "libuvc/libuvc_internal.h"

TEST_CASE("uvc initialization") {
    uvc_context_t *ctx;

    CHECK(uvc_init(&ctx, NULL) == UVC_SUCCESS);

    REQUIRE(ctx->own_usb_ctx == 1);
    REQUIRE(ctx->usb_ctx != nullptr);

    uvc_exit(ctx);
}

TEST_CASE("uvc initialization - with custom libusb") {
    uvc_context_t *ctx;
    libusb_context *libusb = reinterpret_cast<libusb_context*>(0x1234);

    CHECK(uvc_init(&ctx, libusb) == UVC_SUCCESS);

    REQUIRE(ctx->own_usb_ctx == 0);
    REQUIRE(ctx->usb_ctx == libusb);

    uvc_exit(ctx);
}

