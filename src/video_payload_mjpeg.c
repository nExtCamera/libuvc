
#include "libuvc/libuvc.h"
#include "libuvc/libuvc_internal.h"
#include <errno.h>
#include <jpeglib.h>
#include <setjmp.h>

typedef enum mjpeg_decode_state {
    STATE_START,
    STATE_IN_HEADER,
    STATE_PRE_SCAN,
    STATE_SCANLINE
} mjpeg_decode_state_t;
struct error_mgr {
    struct jpeg_error_mgr super;
    jmp_buf jmp;
};

typedef struct mjpeg_payload_context {
    struct jpeg_decompress_struct dinfo;
    struct error_mgr jerr;
    struct jpeg_source_mgr srcmgr;
    mjpeg_decode_state_t state;
} mjpeg_payload_context_t;

static void _error_exit(j_common_ptr dinfo) {
    struct error_mgr *myerr = (struct error_mgr *)dinfo->err;
    (*dinfo->err->output_message)(dinfo);
    longjmp(myerr->jmp, 1);
}

static void mjpeg_decode(mjpeg_payload_context_t *pCtx, struct uvc_framebuffer *backfb, uint8_t *buffer, size_t len) {
    struct jpeg_decompress_struct* dinfo = &pCtx->dinfo;
    //dinfo->out_color_space = JCS_EXT_RGB;
    //dinfo->dct_method = JDCT_FASTEST;
    if (setjmp(pCtx->jerr.jmp)) {
        /* If we get here, the JPEG code has signaled an error.
         * We need to clean up the JPEG object, close the input file, and return.
         */
        goto fail;
    }

    // setup data source
    dinfo->src->next_input_byte = buffer;
    dinfo->src->bytes_in_buffer = len;

    // decode
    int retcode = -1;
    while (retcode != JPEG_SUSPENDED) {
        switch (pCtx->state) {
            case STATE_START:
            case STATE_IN_HEADER:
                retcode = jpeg_read_header(dinfo, TRUE);
                if (retcode == JPEG_HEADER_OK) {
                    pCtx->state = STATE_PRE_SCAN;
                    continue;
                }
                break;
            case STATE_PRE_SCAN:
                if (jpeg_start_decompress(dinfo)) {
                    pCtx->state = STATE_SCANLINE;
                    continue;
                }
            case STATE_SCANLINE:
                int lines_read = 0;
                while (dinfo->output_scanline < dinfo->output_height) {
                    uint8_t *buffer[1] = {( unsigned char*) out->data + lines_read * out->step };
                    int num_scanlines;

                    num_scanlines = jpeg_read_scanlines(dinfo, buffer, 1);
                    lines_read += num_scanlines;
                }

                jpeg_finish_decompress(dinfo);
            default:
                break;
        }
    }

fail:
    return;
}



uvc_error_t init(mjpeg_payload_context_t* pCtx) {
    struct jpeg_source_mgr *src = &pCtx->srcmgr;
    src->init_source = init_mem_source;
    src->fill_input_buffer = fill_mem_input_buffer;
    src->skip_input_data = skip_input_data;
    src->resync_to_restart = jpeg_resync_to_restart; /* use default method */
    src->term_source = term_source;
    src->bytes_in_buffer = (size_t)insize;
    src->next_input_byte = (const JOCTET *)inbuffer;

    struct jpeg_decompress_struct* dinfo = &pCtx->dinfo;
    struct error_mgr* jerr = &pCtx->jerr;

    dinfo->err = jpeg_std_error(&jerr->super);
    dinfo->err->error_exit = _error_exit;

    if (setjmp(jerr->jmp)) {
        /* If we get here, the JPEG code has signaled an error.
         * We need to clean up the JPEG object, close the input file, and return.
         */
        goto fail;
    }

    jpeg_create_decompress(dinfo);
    dinfo->src = src;
    pCtx->state = STATE_START;

    return UVC_SUCCESS;
fail:
    jpeg_destroy_decompress(dinfo);
    return UVC_ERROR_OTHER;
}

uvc_error_t release(mjpeg_payload_context_t* pCtx) {
    return UVC_SUCCESS;
}

uvc_error_t process_payload_mjpeg(uvc_stream_handle_t *strmh, mjpeg_payload_context_t *pCtx, uint8_t *payload, size_t length) {
    size_t header_len;
    uint8_t header_info;
    size_t data_len;
    struct uvc_framebuffer* back_fb = strmh->backbuffers;

    header_len = payload[0];
    header_info = payload[1];
    data_len = length - header_len;

    if (strmh->fid != (header_info & UVC_STREAM_FID)) {
        /* The frame ID bit was flipped, but we have image data sitting
           around from prior transfers. This means the camera didn't send
           an EOF for the last transfer of the previous frame. */
        _uvc_swap_buffers(strmh);
        back_fb = strmh->backbuffers;
    }

    strmh->fid = (uint8_t) (header_info & UVC_STREAM_FID);

    size_t variable_offset = 2;
    if (header_info & UVC_STREAM_PTS) {
        back_fb->pts = DW_TO_INT(payload + variable_offset);
        variable_offset += 4;
    }

    if (header_info & UVC_STREAM_SCR) {
        /** @todo read the SOF token counter */
        back_fb->scr = DW_TO_INT(payload + variable_offset);
        variable_offset += 6;
    }

    if (header_len > variable_offset)
    {
        // Metadata is attached to header
        memcpy(back_fb->meta_buffer + back_fb->meta_got_bytes, payload + variable_offset, header_len - variable_offset);
        back_fb->meta_got_bytes += header_len - variable_offset;
    }

    if (data_len > 0) {
        if (back_fb->got_bytes + data_len > back_fb->buffer_size) {
            UVC_DEBUG("buffer overflow: %zd > %zd", back_fb->got_bytes + data_len,
                      back_fb->buffer_size);
        } else {
            mjpeg_decode(pCtx, back_fb, payload + header_len, data_len);
        }

    }
    if (header_info & UVC_STREAM_EOF) {
        /* The EOF bit is set, so publish the complete frame */
        _uvc_swap_buffers(strmh);
    }
    return UVC_SUCCESS;
}
