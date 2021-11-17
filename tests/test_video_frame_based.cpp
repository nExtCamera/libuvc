
#include "doctest.h"
#include "libuvc/libuvc.h"
#include "libuvc/libuvc_internal.h"
#include "mock_uvc.h"
#include <unistd.h>

static uint8_t guid_yuy2[16] = {'Y', 'U', 'Y', '2', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71};
extern struct uvc_video_handler frame_vid_handler;

static uint8_t* prepare_payload(size_t *size) {
    uint8_t* payload = new uint8_t[2 + 4];
    payload[0] = 2;
    payload[1] = UVC_STREAM_EOH | UVC_STREAM_EOF;
    payload[2] = 0xFF;
    payload[3] = 0xD8; // SOI
    payload[4] = 0xFF;
    payload[5] = 0xD9; // EOI
    *size = 2 + 4;
    return payload;
}

TEST_CASE("frame-based format handler") {
    uvc_context_t *ctx;

    CHECK(uvc_init(&ctx, NULL) == UVC_SUCCESS);

    REQUIRE(uvc_get_video_handler(ctx, guid_yuy2) != nullptr);

    uvc_exit(ctx);
}

TEST_CASE("frame-based video processing") {
    // given
    uvc_stream_handle_t *strmh = mock_stream_open();
    uvc_enqueue_frame(strmh, uvc_allocate_frame(10));
    uvc_enqueue_frame(strmh, uvc_allocate_frame(10));

    // when
    uvc_video_payload_context_t* pctx = frame_vid_handler.init(strmh);
    REQUIRE(pctx != nullptr);

    size_t size;
    uint8_t* payload = prepare_payload(&size);
    frame_vid_handler.process_payload(strmh, pctx, payload, size);

    uvc_frame_t* frame = nullptr;
    uvc_error_t result = uvc_stream_get_frame(strmh, &frame, 100);
    REQUIRE(result == UVC_SUCCESS);
    CHECK(frame->data_bytes == 4);
    CHECK(frame->sequence == 1);
    CHECK(memcmp(frame->data, payload+2, 4) == 0);

    SUBCASE("2nd frame") {
        payload[1] ^= UVC_STREAM_FID;
        result = frame_vid_handler.process_payload(strmh, pctx, payload, size);
        CHECK(result == UVC_SUCCESS);
        CHECK(uvc_stream_get_frame(strmh, &frame, 0) == UVC_SUCCESS);
        REQUIRE(frame != nullptr);
        CHECK(frame->sequence == 2);
    }
    
    // after
    frame_vid_handler.release(pctx);
    free(strmh);
}
