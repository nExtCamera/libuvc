
#include "libuvc/libuvc.h"
#include "libuvc/libuvc_internal.h"

static uvc_video_payload_context_t* init(uvc_stream_handle_t* strmh) {
    uvc_video_payload_context_t* pCtx = calloc(1, sizeof(uvc_video_payload_context_t));
    pCtx->strmh = strmh;
    return pCtx;
}

static uvc_error_t release(uvc_video_payload_context_t* pCtx) {
    free(pCtx);
    return UVC_SUCCESS;
}

static void finalize_frame(uvc_stream_handle_t *strmh) {
    struct uvc_framebuffer* back_fb = strmh->backbuffers;
    uvc_frame_t* frame = back_fb->frame;
    frame->frame_format = strmh->frame_format;
    if (!frame->isStillImage) {
        frame->width = strmh->width;
        frame->height = strmh->height;
    }
    _uvc_swap_buffers(strmh);
}

static uvc_error_t process_payload_frame_based(uvc_stream_handle_t *strmh, uvc_video_payload_context_t *pctx, uint8_t *payload, size_t length) {
    size_t header_len;
    uint8_t header_info;
    size_t data_len;
    struct uvc_framebuffer* back_fb = strmh->backbuffers;

    header_len = payload[0];
    header_info = payload[1];
    data_len = length - header_len;
    uint hdr_fid = (header_info & UVC_STREAM_FID);
    uint8_t hdr_still_image = (header_info & UVC_STREAM_STI) != 0;

    // hackfix for devices that do not update FID when a still image is received
    if (pctx->still_image != hdr_still_image && pctx->fid == hdr_fid) {
        pctx->fid = !pctx->fid;
    }

    if (pctx->fid != hdr_fid) {
        /* The frame ID bit was flipped, but we have image data sitting
           around from prior transfers. This means the camera didn't send
           an EOF for the last transfer of the previous frame. */
        if (back_fb) {
            back_fb->status = pctx->frame_status;
            finalize_frame(strmh);
        }
        pctx->frame_status = UVC_FRAME_VALID;
        pctx->still_image = hdr_still_image;
        if (pctx->still_image && strmh->stillbuffers) {
            UVC_DEBUG("Prepend frame from stillbuffer..");
            pthread_mutex_lock(&strmh->cb_mutex);
            struct uvc_framebuffer* elem = strmh->stillbuffers;
            DL_DELETE(strmh->stillbuffers, elem);
            DL_PREPEND(strmh->backbuffers, elem);
            pthread_mutex_unlock(&strmh->cb_mutex);
        }
        back_fb = strmh->backbuffers;
    }

    pctx->fid = hdr_fid;

    if (!back_fb) {
        return UVC_ERROR_NOT_FOUND;
    }
    back_fb->status = pctx->frame_status;
    if (pctx->frame_status != UVC_FRAME_VALID) {
        return UVC_ERROR_IO;
    }

    size_t variable_offset = 2;
    if (header_info & UVC_STREAM_PTS) {
        pctx->pts = DW_TO_INT(payload + variable_offset);
        variable_offset += 4;
    }

    if (header_info & UVC_STREAM_SCR) {
        /** @todo read the SOF token counter */
        pctx->scr = DW_TO_INT(payload + variable_offset);
        variable_offset += 6;
    }

//    if (header_len > variable_offset)
//    {
//      // Metadata is attached to header
//      memcpy(back_fb->meta_buffer + back_fb->meta_got_bytes, payload + variable_offset, header_len - variable_offset);
//      back_fb->meta_got_bytes += header_len - variable_offset;
//    }

    if (data_len > 0) {
        uvc_frame_t* frame = back_fb->frame;
        frame->isStillImage = (header_info & UVC_STREAM_STI) != 0;
        if (frame->data_bytes + data_len > frame->capacity) {
            UVC_DEBUG("buffer overflow: %zd > %zd", frame->data_bytes + data_len, frame->capacity);
        } else {
            memcpy(frame->data + frame->data_bytes, payload + header_len, data_len);
            frame->data_bytes += data_len;
        }

    }
    if (header_info & UVC_STREAM_EOF) {
        /* The EOF bit is set, so publish the complete frame */
        finalize_frame(strmh);
    }
    return UVC_SUCCESS;
}

static size_t get_frame_buffer_size(uvc_video_payload_context_t* pCtx) {
    return pCtx->strmh->maxVideoFrameBufferSize;
}

struct uvc_video_handler frame_vid_handler = {
    .init = &init,
    .process_payload = &process_payload_frame_based,
    .release = &release,
    .get_frame_buffer_size = get_frame_buffer_size
};
