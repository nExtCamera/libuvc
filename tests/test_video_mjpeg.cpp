
#include "doctest.h"
#include "libuvc/libuvc.h"
#include "libuvc/libuvc_internal.h"
#include "mock_uvc.h"
#include <unistd.h>
#include <sys/stat.h>

static uint8_t guid_mjpg[16] = {'M','J','P','G',0,};
extern struct uvc_video_handler mjpeg_vid_handler;

static uint8_t* prepare_payload(size_t *size) {
    FILE* f = fopen("testorig.jpg", "r");
    REQUIRE(f != nullptr);
    fseek(f, 0, SEEK_END);
    long filelen = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t* payload = new uint8_t[2 + filelen];
    payload[0] = 2;
    payload[1] = UVC_STREAM_EOH | UVC_STREAM_EOF;
    fread(payload+2, filelen, 1, f);
    fclose(f);
    *size = 2 + filelen;
    return payload;
}

TEST_CASE("mjpeg format handler") {
    uvc_context_t *ctx;

    CHECK(uvc_init(&ctx, NULL) == UVC_SUCCESS);

    REQUIRE(uvc_get_video_handler(ctx, guid_mjpg) != nullptr);

    uvc_exit(ctx);
}

TEST_CASE("mjpeg initialization") {
    // given
    uvc_stream_handle_t *strmh = static_cast<uvc_stream_handle_t*>(calloc(1, sizeof(uvc_stream_handle_t)));
    strmh->cur_ctrl.dwMaxVideoFrameSize = 1;
    uvc_frame_t* frame = uvc_allocate_frame(10);
    uvc_enqueue_frame(strmh, frame);

    // when
    uvc_video_payload_context_t* pctx = mjpeg_vid_handler.init(strmh);
    mjpeg_vid_handler.release(pctx);

    // then
    free(strmh);
}

TEST_CASE("mjpeg payload processing") {
    // given
    uvc_stream_handle_t *strmh = mock_stream_open();
    uvc_enqueue_frame(strmh, uvc_allocate_frame(4*227*149));
    uvc_enqueue_frame(strmh, uvc_allocate_frame(4*227*149));
    uvc_video_payload_context_t* pctx = mjpeg_vid_handler.init(strmh);

    // when
    size_t size;
    uint8_t* payload = prepare_payload(&size);
    MESSAGE("process payload " << size);
    CHECK(mjpeg_vid_handler.process_payload(strmh, pctx, payload, size) == UVC_SUCCESS);
    
    // then
    uvc_frame_t* frame = nullptr;
    CHECK(uvc_stream_get_frame(strmh, &frame, 0) == UVC_SUCCESS);
    REQUIRE(frame != nullptr);
    CHECK(frame->width == 227);
    CHECK(frame->height == 149);
    CHECK(frame->data_bytes == frame->capacity);
    CHECK(frame->frame_format == UVC_COLOR_FORMAT_RGB);

    SUBCASE("2nd frame") {
        CHECK(mjpeg_vid_handler.process_payload(strmh, pctx, payload, size) == UVC_SUCCESS);
        CHECK(uvc_stream_get_frame(strmh, &frame, 0) == UVC_SUCCESS);
        REQUIRE(frame != nullptr);
        CHECK(frame->width == 227);
        CHECK(frame->height == 149);
        CHECK(frame->data_bytes == frame->capacity);
        CHECK(frame->frame_format == UVC_COLOR_FORMAT_RGB);
    }

    // cleanup
    mjpeg_vid_handler.release(pctx);
    free(strmh);
}


TEST_CASE("mjpeg block writer with slow consumer") {
    // given
    uvc_stream_handle_t *strmh = mock_stream_open();
    uvc_enqueue_frame(strmh, uvc_allocate_frame(4*227*149));
    uvc_enqueue_frame(strmh, uvc_allocate_frame(4*227*149));
    uvc_video_payload_context_t* pctx = mjpeg_vid_handler.init(strmh);

    // when
    MESSAGE("test blocking writer");
    size_t size;
    uint8_t* payload = prepare_payload(&size);
    CHECK(mjpeg_vid_handler.process_payload(strmh, pctx, payload, size) == UVC_SUCCESS);
    payload[1] ^= UVC_STREAM_FID;
    CHECK(mjpeg_vid_handler.process_payload(strmh, pctx, payload, size) == UVC_ERROR_IO);
    pctx->frame_status = UVC_FRAME_INVALID; // todo maybe process_payload should update it

    // then
    uvc_frame_t* frame = nullptr;
    CHECK(uvc_stream_get_frame(strmh, &frame, 0) == UVC_SUCCESS);
    REQUIRE(frame != nullptr);
    CHECK(frame->sequence == 1);

    SUBCASE("2nd frame skipped") {
        CHECK(uvc_stream_get_frame(strmh, &frame, -1) != UVC_SUCCESS);
    }

    SUBCASE("3rd frame after skipped second") {
        payload[1] ^= UVC_STREAM_FID;
        CHECK(mjpeg_vid_handler.process_payload(strmh, pctx, payload, size) == UVC_SUCCESS);
        CHECK(uvc_stream_get_frame(strmh, &frame, 0) == UVC_SUCCESS);
        REQUIRE(frame != nullptr);
        CHECK(frame->sequence == 3);
    }

    // after
    mjpeg_vid_handler.release(pctx);
    free(strmh);
}
