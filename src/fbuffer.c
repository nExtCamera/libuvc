
#include "libuvc/libuvc.h"
#include "libuvc/libuvc_internal.h"

uvc_error_t uvc_enqueue_frame(uvc_stream_handle_t *strmh, uvc_frame_t *frame) {
    if (frame->frame_format != UVC_FRAME_FORMAT_ANY && frame->frame_format != strmh->frame_format) {
        return UVC_ERROR_INVALID_PARAM;
    }
    struct uvc_framebuffer *fb = calloc(1, sizeof(struct uvc_framebuffer));
    frame->data_bytes = 0;
    frame->frame_format = strmh->frame_format;
    fb->frame = frame;
    fb->meta_buffer = malloc( LIBUVC_XFER_META_BUF_SIZE );
    pthread_mutex_lock(&strmh->cb_mutex);
    DL_APPEND(strmh->backbuffers, fb);
    pthread_mutex_unlock(&strmh->cb_mutex);
    return UVC_SUCCESS;
}

uvc_error_t uvc_dequeue_frame(uvc_stream_handle_t *strmh, uvc_frame_t** frame) {
    struct uvc_framebuffer *fb;
    uvc_error_t result = UVC_ERROR_NOT_FOUND;
    pthread_mutex_lock(&strmh->cb_mutex);
    fb = strmh->frontbuffers;
    if (fb != NULL) {
        *frame = fb->frame;
        DL_DELETE(strmh->frontbuffers, fb);
        free(fb);
        result = UVC_SUCCESS;
    }
    pthread_mutex_unlock(&strmh->cb_mutex);
    return result;
}
