/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2010-2012 Ken Tossell
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/**
 * @defgroup streaming Streaming control functions
 * @brief Tools for creating, managing and consuming video streams
 */

#include "libuvc/libuvc.h"
#include "libuvc/libuvc_internal.h"
#include "errno.h"

#ifdef _MSC_VER

#define DELTA_EPOCH_IN_MICROSECS  116444736000000000Ui64

// gettimeofday - get time of day for Windows;
// A gettimeofday implementation for Microsoft Windows;
// Public domain code, author "ponnada";
int gettimeofday(struct timeval *tv, struct timezone *tz)
{
    FILETIME ft;
    unsigned __int64 tmpres = 0;
    static int tzflag = 0;
    if (NULL != tv)
    {
        GetSystemTimeAsFileTime(&ft);
        tmpres |= ft.dwHighDateTime;
        tmpres <<= 32;
        tmpres |= ft.dwLowDateTime;
        tmpres /= 10;
        tmpres -= DELTA_EPOCH_IN_MICROSECS;
        tv->tv_sec = (long)(tmpres / 1000000UL);
        tv->tv_usec = (long)(tmpres % 1000000UL);
    }
    return 0;
}
#endif // _MSC_VER
uvc_frame_desc_t *uvc_find_frame_desc_stream(uvc_stream_handle_t *strmh,
    uint16_t format_id, uint16_t frame_id);
uvc_frame_desc_t *uvc_find_frame_desc(uvc_device_handle_t *devh,
    uint16_t format_id, uint16_t frame_id);
void *_uvc_user_caller(void *arg);

static uvc_streaming_interface_t *_uvc_get_stream_if(uvc_device_handle_t *devh, int interface_idx);
static uvc_stream_handle_t *_uvc_get_stream_by_interface(uvc_device_handle_t *devh, int interface_idx);

struct format_table_entry {
  enum uvc_frame_format format;
  uint8_t abstract_fmt;
  uint8_t guid[16];
  int children_count;
  enum uvc_frame_format *children;
};

struct format_table_entry *_get_format_entry(enum uvc_frame_format format) {
  #define ABS_FMT(_fmt, _num, ...) \
    case _fmt: { \
    static enum uvc_frame_format _fmt##_children[] = __VA_ARGS__; \
    static struct format_table_entry _fmt##_entry = { \
      _fmt, 0, {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, _num, _fmt##_children }; \
    return &_fmt##_entry; }

  #define FMT(_fmt, ...) \
    case _fmt: { \
    static struct format_table_entry _fmt##_entry = { \
      _fmt, 0, __VA_ARGS__, 0, NULL }; \
    return &_fmt##_entry; }

  switch(format) {
    /* Define new formats here */
    ABS_FMT(UVC_FRAME_FORMAT_ANY, 2,
      {UVC_FRAME_FORMAT_UNCOMPRESSED, UVC_FRAME_FORMAT_COMPRESSED})

    ABS_FMT(UVC_FRAME_FORMAT_UNCOMPRESSED, 6,
      {UVC_FRAME_FORMAT_YUYV, UVC_FRAME_FORMAT_UYVY, UVC_FRAME_FORMAT_GRAY8,
      UVC_FRAME_FORMAT_GRAY16, UVC_FRAME_FORMAT_NV12, UVC_FRAME_FORMAT_BGR})
    FMT(UVC_FRAME_FORMAT_YUYV,
      {'Y',  'U',  'Y',  '2', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71})
    FMT(UVC_FRAME_FORMAT_UYVY,
      {'U',  'Y',  'V',  'Y', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71})
    FMT(UVC_FRAME_FORMAT_GRAY8,
      {'Y',  '8',  '0',  '0', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71})
    FMT(UVC_FRAME_FORMAT_GRAY16,
      {'Y',  '1',  '6',  ' ', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71})
    FMT(UVC_FRAME_FORMAT_NV12,
      {'N',  'V',  '1',  '2', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71})
    FMT(UVC_FRAME_FORMAT_BGR,
      {0x7d, 0xeb, 0x36, 0xe4, 0x4f, 0x52, 0xce, 0x11, 0x9f, 0x53, 0x00, 0x20, 0xaf, 0x0b, 0xa7, 0x70})
    FMT(UVC_FRAME_FORMAT_BY8,
      {'B',  'Y',  '8',  ' ', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71})
    FMT(UVC_FRAME_FORMAT_BA81,
      {'B',  'A',  '8',  '1', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71})
    FMT(UVC_FRAME_FORMAT_SGRBG8,
      {'G',  'R',  'B',  'G', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71})
    FMT(UVC_FRAME_FORMAT_SGBRG8,
      {'G',  'B',  'R',  'G', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71})
    FMT(UVC_FRAME_FORMAT_SRGGB8,
      {'R',  'G',  'G',  'B', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71})
    FMT(UVC_FRAME_FORMAT_SBGGR8,
      {'B',  'G',  'G',  'R', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71})
    ABS_FMT(UVC_FRAME_FORMAT_COMPRESSED, 2,
      {UVC_FRAME_FORMAT_MJPEG, UVC_FRAME_FORMAT_H264})
    FMT(UVC_FRAME_FORMAT_MJPEG,
      {'M',  'J',  'P',  'G'})
    FMT(UVC_FRAME_FORMAT_H264,
      {'H',  '2',  '6',  '4', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71})

    default:
      return NULL;
  }

  #undef ABS_FMT
  #undef FMT
}

static uint8_t _uvc_frame_format_matches_guid(enum uvc_frame_format fmt, uint8_t guid[16]) {
  struct format_table_entry *format;
  int child_idx;

  format = _get_format_entry(fmt);
  if (!format)
    return 0;

  if (!format->abstract_fmt && !memcmp(guid, format->guid, 16))
    return 1;

  for (child_idx = 0; child_idx < format->children_count; child_idx++) {
    if (_uvc_frame_format_matches_guid(format->children[child_idx], guid))
      return 1;
  }

  return 0;
}

static enum uvc_frame_format uvc_frame_format_for_guid(uint8_t guid[16]) {
  struct format_table_entry *format;
  enum uvc_frame_format fmt;

  for (fmt = 0; fmt < UVC_FRAME_FORMAT_COUNT; ++fmt) {
    format = _get_format_entry(fmt);
    if (!format || format->abstract_fmt)
      continue;
    if (!memcmp(format->guid, guid, 16))
      return format->format;
  }

  return UVC_FRAME_FORMAT_UNKNOWN;
}

/** @internal
 * Run a streaming control query
 * @param[in] devh UVC device
 * @param[in,out] ctrl Control block
 * @param[in] probe Whether this is a probe query or a commit query
 * @param[in] req Query type
 */
uvc_error_t uvc_query_stream_ctrl(
    uvc_device_handle_t *devh,
    uvc_stream_ctrl_t *ctrl,
    uint8_t probe,
    enum uvc_req_code req) {
  uint8_t buf[34];
  size_t len;
  uvc_error_t err;

  memset(buf, 0, sizeof(buf));

  if (devh->info->ctrl_if.bcdUVC >= 0x0110)
    len = 34;
  else
    len = 26;

  /* prepare for a SET transfer */
  if (req == UVC_SET_CUR) {
    SHORT_TO_SW(ctrl->bmHint, buf);
    buf[2] = ctrl->bFormatIndex;
    buf[3] = ctrl->bFrameIndex;
    INT_TO_DW(ctrl->dwFrameInterval, buf + 4);
    SHORT_TO_SW(ctrl->wKeyFrameRate, buf + 8);
    SHORT_TO_SW(ctrl->wPFrameRate, buf + 10);
    SHORT_TO_SW(ctrl->wCompQuality, buf + 12);
    SHORT_TO_SW(ctrl->wCompWindowSize, buf + 14);
    SHORT_TO_SW(ctrl->wDelay, buf + 16);
    INT_TO_DW(ctrl->dwMaxVideoFrameSize, buf + 18);
    INT_TO_DW(ctrl->dwMaxPayloadTransferSize, buf + 22);

    if (len == 34) {
      INT_TO_DW ( ctrl->dwClockFrequency, buf + 26 );
      buf[30] = ctrl->bmFramingInfo;
      buf[31] = ctrl->bPreferredVersion;
      buf[32] = ctrl->bMinVersion;
      buf[33] = ctrl->bMaxVersion;
      /** @todo support UVC 1.1 */
    }
  }

  /* do the transfer */
  err = libusb_control_transfer(
      devh->usb_devh,
      req == UVC_SET_CUR ? 0x21 : 0xA1,
      req,
      probe ? (UVC_VS_PROBE_CONTROL << 8) : (UVC_VS_COMMIT_CONTROL << 8),
      ctrl->bInterfaceNumber,
      buf, len, 0
  );

  if (err <= 0) {
    return err;
  }

  /* now decode following a GET transfer */
  if (req != UVC_SET_CUR) {
    ctrl->bmHint = SW_TO_SHORT(buf);
    ctrl->bFormatIndex = buf[2];
    ctrl->bFrameIndex = buf[3];
    ctrl->dwFrameInterval = DW_TO_INT(buf + 4);
    ctrl->wKeyFrameRate = SW_TO_SHORT(buf + 8);
    ctrl->wPFrameRate = SW_TO_SHORT(buf + 10);
    ctrl->wCompQuality = SW_TO_SHORT(buf + 12);
    ctrl->wCompWindowSize = SW_TO_SHORT(buf + 14);
    ctrl->wDelay = SW_TO_SHORT(buf + 16);
    ctrl->dwMaxVideoFrameSize = DW_TO_INT(buf + 18);
    ctrl->dwMaxPayloadTransferSize = DW_TO_INT(buf + 22);

    if (len == 34) {
      ctrl->dwClockFrequency = DW_TO_INT ( buf + 26 );
      ctrl->bmFramingInfo = buf[30];
      ctrl->bPreferredVersion = buf[31];
      ctrl->bMinVersion = buf[32];
      ctrl->bMaxVersion = buf[33];
      /** @todo support UVC 1.1 */
    }
    else
      ctrl->dwClockFrequency = devh->info->ctrl_if.dwClockFrequency;

    /* fix up block for cameras that fail to set dwMax* */
    if (ctrl->dwMaxVideoFrameSize == 0) {
      uvc_frame_desc_t *frame = uvc_find_frame_desc(devh, ctrl->bFormatIndex, ctrl->bFrameIndex);

      if (frame) {
        ctrl->dwMaxVideoFrameSize = frame->dwMaxVideoFrameBufferSize;
      }
    }
  }

  return UVC_SUCCESS;
}

/** @internal
 * Run a streaming control query
 * @param[in] devh UVC device
 * @param[in,out] ctrl Control block
 * @param[in] probe Whether this is a probe query or a commit query
 * @param[in] req Query type
 */
uvc_error_t uvc_query_still_ctrl(
  uvc_device_handle_t *devh,
  uvc_still_ctrl_t *still_ctrl,
  uint8_t probe,
  enum uvc_req_code req) {

  uint8_t buf[11];
  const size_t len = 11;
  uvc_error_t err;

  memset(buf, 0, sizeof(buf));

  if (req == UVC_SET_CUR) {
    /* prepare for a SET transfer */
    buf[0] = still_ctrl->bFormatIndex;
    buf[1] = still_ctrl->bFrameIndex;
    buf[2] = still_ctrl->bCompressionIndex;
    INT_TO_DW(still_ctrl->dwMaxVideoFrameSize, buf + 3);
    INT_TO_DW(still_ctrl->dwMaxPayloadTransferSize, buf + 7);
  }

  /* do the transfer */
  err = libusb_control_transfer(
      devh->usb_devh,
      req == UVC_SET_CUR ? 0x21 : 0xA1,
      req,
      probe ? (UVC_VS_STILL_PROBE_CONTROL << 8) : (UVC_VS_STILL_COMMIT_CONTROL << 8),
      still_ctrl->bInterfaceNumber,
      buf, len, 0
  );

  if (err <= 0) {
    return err;
  }

  /* now decode following a GET transfer */
  if (req != UVC_SET_CUR) {
    still_ctrl->bFormatIndex = buf[0];
    still_ctrl->bFrameIndex = buf[1];
    still_ctrl->bCompressionIndex = buf[2];
    still_ctrl->dwMaxVideoFrameSize = DW_TO_INT(buf + 3);
    still_ctrl->dwMaxPayloadTransferSize = DW_TO_INT(buf + 7);
  }

  return UVC_SUCCESS;
}

/** Initiate a method 2 (in stream) still capture
 * @ingroup streaming
 *
 * @param[in] devh Device handle
 * @param[in] still_ctrl Still capture control block
 */
uvc_error_t uvc_trigger_still(
    uvc_device_handle_t *devh,
    uvc_still_ctrl_t *still_ctrl) {
  uvc_stream_handle_t* stream;
  uvc_streaming_interface_t* stream_if;
  uint8_t buf;
  uvc_error_t err;

  /* Stream must be running for method 2 to work */
  stream = _uvc_get_stream_by_interface(devh, still_ctrl->bInterfaceNumber);
  if (!stream || !stream->running)
    return UVC_ERROR_NOT_SUPPORTED;

  /* Only method 2 is supported */
  stream_if = _uvc_get_stream_if(devh, still_ctrl->bInterfaceNumber);
  if(!stream_if || stream_if->bStillCaptureMethod != 2)
      return UVC_ERROR_NOT_SUPPORTED;

  /* prepare for a SET transfer */
  buf = 1;

  /* do the transfer */
  err = libusb_control_transfer(
      devh->usb_devh,
      0x21, //type set
      UVC_SET_CUR,
      (UVC_VS_STILL_IMAGE_TRIGGER_CONTROL << 8),
      still_ctrl->bInterfaceNumber,
      &buf, 1, 0);

  if (err <= 0) {
    return err;
  }

  return UVC_SUCCESS;
}

static void free_transfer(struct libusb_transfer *transfer) {
    if (transfer->buffer != NULL) {
        free(transfer->buffer);
        transfer->buffer = NULL;
    }
    libusb_free_transfer(transfer);
}

/** @brief Reconfigure stream with a new stream format.
 * @ingroup streaming
 *
 * This may be executed whether or not the stream is running.
 *
 * @param[in] strmh Stream handle
 * @param[in] ctrl Control block, processed using {uvc_probe_stream_ctrl} or
 *             {uvc_get_stream_ctrl_format_size}
 */
uvc_error_t uvc_stream_ctrl(uvc_stream_handle_t *strmh, uvc_stream_ctrl_t *ctrl) {
  uvc_error_t ret;

  if (strmh->stream_if->bInterfaceNumber != ctrl->bInterfaceNumber)
    return UVC_ERROR_INVALID_PARAM;

  /* @todo Allow the stream to be modified without restarting the stream */
  if (strmh->running)
    return UVC_ERROR_BUSY;

  ret = uvc_query_stream_ctrl(strmh->devh, ctrl, 0, UVC_SET_CUR);
  if (ret != UVC_SUCCESS)
    return ret;

  strmh->cur_ctrl = *ctrl;
  return UVC_SUCCESS;
}

/** @brief Gets current stream control block
 * @ingroup streaming
 *
 * This may be executed whether or not the stream is running.
 *
 * @param[in] strmh Stream handle
 * @param[out] ctrl Current control block
 */
uvc_error_t uvc_stream_get_current_ctrl(uvc_stream_handle_t *strmh, uvc_stream_ctrl_t *ctrl) {

    *ctrl = strmh->cur_ctrl;
    return UVC_SUCCESS;
}

/** @internal
 * @brief Find the descriptor for a specific frame configuration
 * @param stream_if Stream interface
 * @param format_id Index of format class descriptor
 * @param frame_id Index of frame descriptor
 */
static uvc_frame_desc_t *_uvc_find_frame_desc_stream_if(uvc_streaming_interface_t *stream_if,
    uint16_t format_id, uint16_t frame_id) {
 
  uvc_format_desc_t *format = NULL;
  uvc_frame_desc_t *frame = NULL;

  DL_FOREACH(stream_if->format_descs, format) {
    if (format->bFormatIndex == format_id) {
      DL_FOREACH(format->frame_descs, frame) {
        if (frame->bFrameIndex == frame_id)
          return frame;
      }
    }
  }

  return NULL;
}

uvc_frame_desc_t *uvc_find_frame_desc_stream(uvc_stream_handle_t *strmh,
    uint16_t format_id, uint16_t frame_id) {
  return _uvc_find_frame_desc_stream_if(strmh->stream_if, format_id, frame_id);
}

uvc_frame_desc_t* uvc_stream_get_current_frame_desc(uvc_stream_handle_t *strmh) {
    return uvc_find_frame_desc_stream(strmh, strmh->cur_ctrl.bFormatIndex, strmh->cur_ctrl.bFrameIndex);
}

/** @internal
 * @brief Find the descriptor for a specific frame configuration
 * @param devh UVC device
 * @param format_id Index of format class descriptor
 * @param frame_id Index of frame descriptor
 */
uvc_frame_desc_t *uvc_find_frame_desc(uvc_device_handle_t *devh,
    uint16_t format_id, uint16_t frame_id) {
 
  uvc_streaming_interface_t *stream_if;
  uvc_frame_desc_t *frame;

  DL_FOREACH(devh->info->stream_ifs, stream_if) {
    frame = _uvc_find_frame_desc_stream_if(stream_if, format_id, frame_id);
    if (frame)
      return frame;
  }

  return NULL;
}

/** Get a negotiated streaming control block for some common parameters.
 * @ingroup streaming
 *
 * @param[in] devh Device handle
 * @param[in,out] ctrl Control block
 * @param[in] format_class Type of streaming format
 * @param[in] width Desired frame width
 * @param[in] height Desired frame height
 * @param[in] fps Frame rate, frames per second
 */
uvc_error_t uvc_get_stream_ctrl_format_size(
    uvc_device_handle_t *devh,
    uvc_stream_ctrl_t *ctrl,
    enum uvc_frame_format cf,
    int width, int height,
    int fps) {
  uvc_streaming_interface_t *stream_if;

  /* find a matching frame descriptor and interval */
  DL_FOREACH(devh->info->stream_ifs, stream_if) {
    uvc_format_desc_t *format;

    DL_FOREACH(stream_if->format_descs, format) {
      uvc_frame_desc_t *frame;

      if (!_uvc_frame_format_matches_guid(cf, format->guidFormat))
        continue;

      DL_FOREACH(format->frame_descs, frame) {
        if (frame->wWidth != width || frame->wHeight != height)
          continue;

        uint32_t *interval;

        ctrl->bInterfaceNumber = stream_if->bInterfaceNumber;
        UVC_DEBUG("claiming streaming interface %d", stream_if->bInterfaceNumber );
        uvc_claim_if(devh, ctrl->bInterfaceNumber);

        if (frame->intervals) {
          for (interval = frame->intervals; *interval; ++interval) {
            // allow a fps rate of zero to mean "accept first rate available"
            if (10000000 / *interval == (unsigned int) fps || fps == 0) {

              ctrl->bmHint = (1 << 0); /* don't negotiate interval */
              ctrl->bFormatIndex = format->bFormatIndex;
              ctrl->bFrameIndex = frame->bFrameIndex;
              ctrl->dwFrameInterval = *interval;

              goto found;
            }
          }
        } else {
          uint32_t interval_100ns = 10000000 / fps;
          uint32_t interval_offset = interval_100ns - frame->dwMinFrameInterval;

          if (interval_100ns >= frame->dwMinFrameInterval
              && interval_100ns <= frame->dwMaxFrameInterval
              && !(interval_offset
                   && (interval_offset % frame->dwFrameIntervalStep))) {

            ctrl->bmHint = (1 << 0);
            ctrl->bFormatIndex = format->bFormatIndex;
            ctrl->bFrameIndex = frame->bFrameIndex;
            ctrl->dwFrameInterval = interval_100ns;

            goto found;
          }
        }
      }
    }
  }

  return UVC_ERROR_INVALID_MODE;

found:
  return uvc_probe_stream_ctrl(devh, ctrl);
}

/** Get a streaming control block for some common parameters.
 * @ingroup streaming
 *
 * @param[in] devh Device handle
 * @param[in,out] ctrl Control block
 * @param[in] formatIndex
 * @param[in] frameIndex
 * @param[in] interval Frame interval rate (in 100ns)
 */
uvc_error_t uvc_get_stream_ctrl(
        uvc_device_handle_t *devh,
        uvc_stream_ctrl_t *ctrl,
        uint8_t formatIndex, uint8_t frameIndex, uint32_t interval) {
  uvc_error_t err;
  const uvc_format_desc_t *format_descs = uvc_get_format_descs(devh);

  const uvc_format_desc_t *format;
  DL_SEARCH_SCALAR(format_descs, format, bFormatIndex, formatIndex);

  if (format) {
    uvc_frame_desc_t *frame;
    DL_SEARCH_SCALAR(format->frame_descs, frame, bFrameIndex, frameIndex);

    if (frame) {
      ctrl->bInterfaceNumber = format->parent->bInterfaceNumber;

      UVC_DEBUG("claiming streaming interface %d", ctrl->bInterfaceNumber );
      err = uvc_claim_if(devh, ctrl->bInterfaceNumber);
      if (err != UVC_SUCCESS) {
        UVC_DEBUG("failed to claim usb interface: error_code=%d", err);
        return err;
      }

      ctrl->bmHint = (1 << 0); /* don't negotiate interval */
      ctrl->bFormatIndex = format->bFormatIndex;
      ctrl->bFrameIndex = frame->bFrameIndex;
      ctrl->dwFrameInterval = interval;

      return uvc_probe_stream_ctrl(devh, ctrl);
    }
  }
  return UVC_ERROR_INVALID_MODE;
}

/** Get a negotiated still control block for some common parameters.
 * @ingroup streaming
 *
 * @param[in] devh Device handle
 * @param[in] ctrl Control block
 * @param[in, out] still_ctrl Still capture control block
 * @param[in] width Desired frame width
 * @param[in] height Desired frame height
 */
uvc_error_t uvc_get_still_ctrl_format_size(
    uvc_device_handle_t *devh,
    uvc_stream_ctrl_t *ctrl,
    uvc_still_ctrl_t *still_ctrl,
    int width, int height) {
  uvc_streaming_interface_t *stream_if;
  uvc_still_frame_desc_t *still;
  uvc_format_desc_t *format;
  uvc_still_frame_res_t *sizePattern;

  stream_if = _uvc_get_stream_if(devh, ctrl->bInterfaceNumber);

  /* Only method 2 is supported */
  if(!stream_if || stream_if->bStillCaptureMethod != 2)
    return UVC_ERROR_NOT_SUPPORTED;

  DL_FOREACH(stream_if->format_descs, format) {

    if (ctrl->bFormatIndex != format->bFormatIndex)
      continue;

    /* get the max values */
    uvc_query_still_ctrl(devh, still_ctrl, 1, UVC_GET_MAX);

    //look for still format
    DL_FOREACH(format->still_frame_desc, still) {
      DL_FOREACH(still->imageSizePatterns, sizePattern) {

        if (sizePattern->wWidth != width || sizePattern->wHeight != height)
          continue;

        still_ctrl->bInterfaceNumber = ctrl->bInterfaceNumber;
        still_ctrl->bFormatIndex = format->bFormatIndex;
        still_ctrl->bFrameIndex = sizePattern->bResolutionIndex;
        still_ctrl->bCompressionIndex = 0; //TODO support compression index
        goto found;
      }
    }
  }

  return UVC_ERROR_INVALID_MODE;

  found:
    return uvc_probe_still_ctrl(devh, still_ctrl);
}

static int _uvc_stream_params_negotiated(
  uvc_stream_ctrl_t *required,
  uvc_stream_ctrl_t *actual) {
    return required->bFormatIndex == actual->bFormatIndex &&
    required->bFrameIndex == actual->bFrameIndex &&
    required->dwMaxPayloadTransferSize == actual->dwMaxPayloadTransferSize;
}

/** @internal
 * Negotiate streaming parameters with the device
 *
 * @param[in] devh UVC device
 * @param[in,out] ctrl Control block
 */
uvc_error_t uvc_probe_stream_ctrl(
    uvc_device_handle_t *devh,
    uvc_stream_ctrl_t *ctrl) {
  uvc_stream_ctrl_t required_ctrl = *ctrl;

  uvc_query_stream_ctrl( devh, ctrl, 1, UVC_SET_CUR );
  uvc_query_stream_ctrl( devh, ctrl, 1, UVC_GET_CUR );

  if(!_uvc_stream_params_negotiated(&required_ctrl, ctrl)) {
    UVC_DEBUG("Unable to negotiate streaming format");
    return UVC_ERROR_INVALID_MODE;
  }

  return UVC_SUCCESS;
}

/** @internal
 * Negotiate still parameters with the device
 *
 * @param[in] devh UVC device
 * @param[in,out] still_ctrl Still capture control block
 */
uvc_error_t uvc_probe_still_ctrl(
    uvc_device_handle_t *devh,
    uvc_still_ctrl_t *still_ctrl) {

  int res = uvc_query_still_ctrl(
    devh, still_ctrl, 1, UVC_SET_CUR
  );

  if(res == UVC_SUCCESS) {
    res = uvc_query_still_ctrl(
      devh, still_ctrl, 1, UVC_GET_CUR
    );

    if(res == UVC_SUCCESS) {
      res = uvc_query_still_ctrl(
        devh, still_ctrl, 0, UVC_SET_CUR
      );
    }
  }

  return res;
}

/** @internal
 * @brief Swap the working buffer with the presented buffer and notify consumers
 */
void _uvc_swap_buffers(uvc_stream_handle_t *strmh) {
  struct uvc_framebuffer *current = strmh->backbuffers;
  if (current->status == UVC_FRAME_INVALID || current->frame->data_bytes == 0) {
      // drop invalid frame and abort swap
      current->status = UVC_FRAME_VALID;
      current->frame->data_bytes = 0;
      current->meta_got_bytes = 0;
      return;
  }
  (void)clock_gettime(CLOCK_MONOTONIC, &current->capture_time_finished);

  pthread_mutex_lock(&strmh->cb_mutex);

  /* swap the buffers */
  current->frame->sequence = strmh->seq++;
  DL_DELETE(strmh->backbuffers, current);
  DL_APPEND(strmh->frontbuffers, current);

  if (strmh->backbuffers == NULL && strmh->frontbuffers != current) {
    // steal some buffer from the front, only if it was not just taken from the back
    current = strmh->frontbuffers;
    DL_DELETE(strmh->frontbuffers, current);
    DL_APPEND(strmh->backbuffers, current);
  }

  if (strmh->backbuffers != NULL) {
    strmh->backbuffers->frame->data_bytes = 0;
    strmh->backbuffers->meta_got_bytes = 0;
  }

  pthread_cond_broadcast(&strmh->cb_cond);
  pthread_mutex_unlock(&strmh->cb_mutex);
}

/** @internal
 * @brief Process a payload transfer
 * 
 * Processes stream, places frames into buffer, signals listeners
 * (such as user callback thread and any polling thread) on new frame
 *
 * @param payload Contents of the payload transfer, either a packet (isochronous) or a full
 * transfer (bulk mode)
 * @param payload_len Length of the payload transfer
 */
uvc_error_t _uvc_process_payload(uvc_stream_handle_t *strmh, uint8_t *payload, size_t payload_len) {
  size_t header_len;
  uint8_t header_info;
  uvc_video_payload_context_t* pctx = strmh->payload_ctx;

  /* magic numbers for identifying header packets from some iSight cameras */
  static uint8_t isight_tag[] = {
    0x11, 0x22, 0x33, 0x44,
    0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xfa, 0xce
  };

  /* ignore empty payload transfers */
  if (payload_len == 0)
    return UVC_SUCCESS;

  /* Certain iSight cameras have strange behavior: They send header
   * information in a packet with no image data, and then the following
   * packets have only image data, with no more headers until the next frame.
   *
   * The iSight header: len(1), flags(1 or 2), 0x11223344(4),
   * 0xdeadbeefdeadface(8), ??(16)
   */

  if (strmh->devh->is_isight &&
      (payload_len < 14 || memcmp(isight_tag, payload + 2, sizeof(isight_tag))) &&
      (payload_len < 15 || memcmp(isight_tag, payload + 3, sizeof(isight_tag)))) {
    /* The payload transfer doesn't have any iSight magic, so it's all image data */
    header_len = 0;
  } else {
    header_len = payload[0];

    if (header_len > payload_len || header_len == 0) {
      UVC_DEBUG("bogus packet: actual_len=%zd, header_len=%zd\n", payload_len, header_len);
      pctx->frame_status = UVC_FRAME_INVALID;
      return UVC_ERROR_IO;
    }
  }

  if (header_len > 1) {
    header_info = payload[1];
    if (header_info & UVC_STREAM_ERR) {
      UVC_DEBUG("bad packet: error bit set");
      pctx->frame_status = UVC_FRAME_INVALID;
    }
  }

  uvc_error_t res = strmh->videoHandler->process_payload(strmh, strmh->payload_ctx, payload, payload_len);
  if (res != UVC_SUCCESS) {
    pctx->frame_status = UVC_FRAME_INVALID;
  }
  return res;
}

/** @internal
 * @brief Stream transfer callback
 *
 * Processes stream, places frames into buffer, signals listeners
 * (such as user callback thread and any polling thread) on new frame
 *
 * @param transfer Active transfer
 */
void LIBUSB_CALL _uvc_stream_callback(struct libusb_transfer *transfer) {
  uvc_stream_handle_t *strmh = transfer->user_data;

  int resubmit = 1;

  switch (transfer->status) {
  case LIBUSB_TRANSFER_COMPLETED:
    if (!strmh->running) break;
    if (transfer->type != LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) {
      /* This is a bulk mode transfer, so it just has one payload transfer */
      _uvc_process_payload(strmh, transfer->buffer, (size_t) transfer->actual_length);
    } else {
      /* This is an isochronous mode transfer, so each packet has a payload transfer */
      unsigned int packet_id;

      int error_packets = 0;
      for (packet_id = 0; packet_id < transfer->num_iso_packets; ++packet_id) {
        uint8_t *pktbuf;
        struct libusb_iso_packet_descriptor *pkt;

        pkt = transfer->iso_packet_desc + packet_id;

        if (pkt->status != LIBUSB_TRANSFER_COMPLETED) {
          UVC_DEBUG("bad packet (isochronous transfer); pkt_id=%d status: %s(%d), actual_length=%d", packet_id, libusb_error_name(pkt->status), pkt->status, pkt->actual_length);
          strmh->payload_ctx->frame_status = UVC_FRAME_INVALID;
          continue;
        }

        pktbuf = libusb_get_iso_packet_buffer(transfer, packet_id);

        uvc_error_t result = _uvc_process_payload(strmh, pktbuf, pkt->actual_length);
        if (result != UVC_SUCCESS) {
          ++error_packets;
        }
      }
#ifndef NDEBUG
      if (error_packets > 0) {
        int i = 0;
        for (i = 0; i < LIBUVC_NUM_TRANSFER_BUFS; ++i) {
          if (strmh->transfers[i] == transfer) break;
        }
        UVC_ERROR("Detected erroneous packets %d/%d in transfer %d", error_packets, transfer->num_iso_packets, i);
      }
#endif //NDEBUG
    }
    break;
  case LIBUSB_TRANSFER_CANCELLED: 
  case LIBUSB_TRANSFER_NO_DEVICE: {
    UVC_DEBUG("not retrying transfer, status = %s", libusb_error_name(transfer->status));
    resubmit = 0;
    break;
  }
  case LIBUSB_TRANSFER_ERROR:
  case LIBUSB_TRANSFER_TIMED_OUT:
  case LIBUSB_TRANSFER_STALL:
  case LIBUSB_TRANSFER_OVERFLOW:
    UVC_DEBUG("retrying transfer, status = %s", libusb_error_name(transfer->status));
    break;
  }

    if (resubmit && strmh->running) {
        int libusbRet;
        int retry = 1000;
        do {
          libusbRet = libusb_submit_transfer(transfer);
        } while (libusbRet != LIBUSB_SUCCESS && --retry > 0);
        if (libusbRet != LIBUSB_SUCCESS) {
            int i;
            UVC_ERROR("Resubmit transfer failed after %d retries (%p)", 1000-retry, transfer);
            pthread_mutex_lock(&strmh->cb_mutex);

            /* Mark transfer as deleted. */
            for (i = 0; i < LIBUVC_NUM_TRANSFER_BUFS; ++i) {
                if (strmh->transfers[i] == transfer) {
                    UVC_DEBUG("Freeing failed transfer %d (%p)", i, transfer);
                    free_transfer(transfer);
                    strmh->transfers[i] = NULL;
                    break;
                }
            }
            if (i == LIBUVC_NUM_TRANSFER_BUFS) {
                UVC_DEBUG("failed transfer %p not found; not freeing!", transfer);
            }

            pthread_cond_broadcast(&strmh->cb_cond);
            pthread_mutex_unlock(&strmh->cb_mutex);
        }
    } else {
        int i;
        /* Mark transfer as deleted. */
        for (i = 0; i < LIBUVC_NUM_TRANSFER_BUFS; ++i) {
            if (strmh->transfers[i] == transfer) {
                UVC_DEBUG("Freeing orphan transfer %d (%p)", i, transfer);
                free_transfer(transfer);
                strmh->transfers[i] = NULL;
                break;
            }
        }
        if (i == LIBUVC_NUM_TRANSFER_BUFS) {
            UVC_ERROR("orphan transfer %p not found; not freeing!", transfer);
        }
    }
}

/** Begin streaming video from the camera into the callback function.
 * @ingroup streaming
 *
 * @param devh UVC device
 * @param ctrl Control block, processed using {uvc_probe_stream_ctrl} or
 *             {uvc_get_stream_ctrl_format_size}
 * @param cb   User callback function. See {uvc_frame_callback_t} for restrictions.
 * @param flags Stream setup flags, currently undefined. Set this to zero. The lower bit
 * is reserved for backward compatibility.
 */
uvc_error_t uvc_start_streaming(
    uvc_device_handle_t *devh,
    uvc_stream_ctrl_t *ctrl,
    uvc_frame_callback_t *cb,
    void *user_ptr,
    uint8_t flags
) {
  uvc_error_t ret;
  uvc_stream_handle_t *strmh;

  ret = uvc_stream_open_ctrl(devh, &strmh, ctrl);
  if (ret != UVC_SUCCESS)
    return ret;

  ret = uvc_stream_start(strmh, cb, user_ptr, flags);
  if (ret != UVC_SUCCESS) {
    uvc_stream_close(strmh);
    return ret;
  }

  return UVC_SUCCESS;
}

/** Begin streaming video from the camera into the callback function.
 * @ingroup streaming
 *
 * @deprecated The stream type (bulk vs. isochronous) will be determined by the
 * type of interface associated with the uvc_stream_ctrl_t parameter, regardless
 * of whether the caller requests isochronous streaming. Please switch to
 * uvc_start_streaming().
 *
 * @param devh UVC device
 * @param ctrl Control block, processed using {uvc_probe_stream_ctrl} or
 *             {uvc_get_stream_ctrl_format_size}
 * @param cb   User callback function. See {uvc_frame_callback_t} for restrictions.
 */
uvc_error_t uvc_start_iso_streaming(
    uvc_device_handle_t *devh,
    uvc_stream_ctrl_t *ctrl,
    uvc_frame_callback_t *cb,
    void *user_ptr
) {
  return uvc_start_streaming(devh, ctrl, cb, user_ptr, 0);
}

static uvc_stream_handle_t *_uvc_get_stream_by_interface(uvc_device_handle_t *devh, int interface_idx) {
  uvc_stream_handle_t *strmh;

  DL_FOREACH(devh->streams, strmh) {
    if (strmh->stream_if->bInterfaceNumber == interface_idx)
      return strmh;
  }

  return NULL;
}

static uvc_streaming_interface_t *_uvc_get_stream_if(uvc_device_handle_t *devh, int interface_idx) {
  uvc_streaming_interface_t *stream_if;

  DL_FOREACH(devh->info->stream_ifs, stream_if) {
    if (stream_if->bInterfaceNumber == interface_idx)
      return stream_if;
  }
  
  return NULL;
}

/** Open a new video stream.
 * @ingroup streaming
 *
 * @param devh UVC device
 * @param ctrl Control block, processed using {uvc_probe_stream_ctrl} or
 *             {uvc_get_stream_ctrl_format_size}
 */
uvc_error_t uvc_stream_open_ctrl(uvc_device_handle_t *devh, uvc_stream_handle_t **strmhp, uvc_stream_ctrl_t *ctrl) {
  /* Chosen frame and format descriptors */
  uvc_stream_handle_t *strmh = NULL;
  uvc_streaming_interface_t *stream_if;
  uvc_error_t ret;

  UVC_ENTER();

  if (_uvc_get_stream_by_interface(devh, ctrl->bInterfaceNumber) != NULL) {
    ret = UVC_ERROR_BUSY; /* Stream is already opened */
    goto fail;
  }

  stream_if = _uvc_get_stream_if(devh, ctrl->bInterfaceNumber);
  if (!stream_if) {
    ret = UVC_ERROR_INVALID_PARAM;
    goto fail;
  }

  strmh = calloc(1, sizeof(*strmh));
  if (!strmh) {
    ret = UVC_ERROR_NO_MEM;
    goto fail;
  }
  strmh->devh = devh;
  strmh->stream_if = stream_if;
  strmh->max_packets_per_transfer = 64;

  ret = uvc_claim_if(strmh->devh, strmh->stream_if->bInterfaceNumber);
  if (ret != UVC_SUCCESS)
    goto fail;

  ret = uvc_stream_ctrl(strmh, ctrl);
  if (ret != UVC_SUCCESS)
    goto fail;

  // Set up the streaming status and data space
  strmh->running = 0;

  uvc_frame_desc_t *frame_desc;
  uvc_format_desc_t *format_desc;
  frame_desc = uvc_find_frame_desc_stream(strmh, ctrl->bFormatIndex, ctrl->bFrameIndex);
  if (!frame_desc) {
    ret = UVC_ERROR_INVALID_PARAM;
    goto fail;
  }
  format_desc = frame_desc->parent;

  strmh->frame_format = uvc_frame_format_for_guid(format_desc->guidFormat);
  if (strmh->frame_format == UVC_FRAME_FORMAT_UNKNOWN) {
    ret = UVC_ERROR_NOT_SUPPORTED;
    goto fail;
  }

  strmh->videoHandler = uvc_get_video_handler(devh->dev->ctx, format_desc->guidFormat);
  if (strmh->videoHandler == NULL) {
    goto fail;
  }
  strmh->width = frame_desc->wWidth;
  strmh->height = frame_desc->wHeight;
  strmh->maxVideoFrameBufferSize = frame_desc->dwMaxVideoFrameBufferSize;

  strmh->payload_ctx = strmh->videoHandler->init(strmh);
  if (strmh->payload_ctx == NULL) {
    goto fail;
  }

  pthread_mutex_init(&strmh->cb_mutex, NULL);
  pthread_cond_init(&strmh->cb_cond, NULL);

  DL_APPEND(devh->streams, strmh);

  *strmhp = strmh;

  UVC_EXIT(0);
  return UVC_SUCCESS;

fail:
  if(strmh)
    free(strmh);
  UVC_EXIT(ret);
  return ret;
}

/** Begin streaming video from the stream into the callback function.
 * @ingroup streaming  strmh->running = 1;
  strmh->seq = 1;
  strmh->fid = 0;vc_frame_callback_t} for restrictions.
 * @param flags Stream setup flags, currently undefined. Set this to zero. The lower bit
 * is reserved for backward compatibility.
 */
uvc_error_t uvc_stream_start(
    uvc_stream_handle_t *strmh,
    uvc_frame_callback_t *cb,
    void *user_ptr,
    uint8_t flags
) {
  /* USB interface we'll be using */
  const struct libusb_interface *interface;
  int interface_id;
  char isochronous;

  uvc_stream_ctrl_t *ctrl;
  uvc_error_t ret;
  /* Total amount of data per transfer */
  size_t total_transfer_size = 0;
  struct libusb_transfer *transfer;
  int transfer_id;

  ctrl = &strmh->cur_ctrl;

  UVC_ENTER();

  if (strmh->running) {
    UVC_EXIT(UVC_ERROR_BUSY);
    return UVC_ERROR_BUSY;
  }

  strmh->running = 1;
  strmh->seq = 1;

  // Get the interface that provides the chosen format and frame configuration
  interface_id = strmh->stream_if->bInterfaceNumber;
  interface = &strmh->devh->info->config->interface[interface_id];

  /* A VS interface uses isochronous transfers iff it has multiple altsettings.
   * (UVC 1.5: 2.4.3. VideoStreaming Interface) */
  isochronous = interface->num_altsetting > 1;

  if (isochronous) {
    UVC_DEBUG("isochronous transfer mode:  num_altsetting=%d", interface->num_altsetting);
    /* For isochronous streaming, we choose an appropriate altsetting for the endpoint
     * and set up several transfers */
    const struct libusb_interface_descriptor *altsetting = 0;
    const struct libusb_endpoint_descriptor *endpoint = 0;
    /* The greatest number of bytes that the device might provide, per packet, in this
     * configuration */
    size_t config_bytes_per_packet;
    /* Number of packets per transfer */
    size_t packets_per_transfer = 0;
    /* Size of packet transferable from the chosen endpoint */
    size_t endpoint_bytes_per_packet = 0;
    /* Index of the altsetting */
    int alt_idx, ep_idx;
    
    config_bytes_per_packet = strmh->cur_ctrl.dwMaxPayloadTransferSize;

    /* Go through the altsettings and find one whose packets are at least
     * as big as our format's maximum per-packet usage. Assume that the
     * packet sizes are increasing. */
    for (alt_idx = 0; alt_idx < interface->num_altsetting; alt_idx++) {
      altsetting = interface->altsetting + alt_idx;
      endpoint_bytes_per_packet = 0;

      /* Find the endpoint with the number specified in the VS header */
      for (ep_idx = 0; ep_idx < altsetting->bNumEndpoints; ep_idx++) {
        endpoint = altsetting->endpoint + ep_idx;

        struct libusb_ss_endpoint_companion_descriptor *ep_comp = 0;
        libusb_get_ss_endpoint_companion_descriptor(NULL, endpoint, &ep_comp);
        if (ep_comp)
        {
          endpoint_bytes_per_packet = ep_comp->wBytesPerInterval;
          libusb_free_ss_endpoint_companion_descriptor(ep_comp);
          break;
        }
        else
        {
          if (endpoint->bEndpointAddress == strmh->stream_if->bEndpointAddress) {
              endpoint_bytes_per_packet = endpoint->wMaxPacketSize;
            // wMaxPacketSize: [unused:2 (multiplier-1):3 size:11]
            endpoint_bytes_per_packet = (endpoint_bytes_per_packet & 0x07ff) *
              (((endpoint_bytes_per_packet >> 11) & 3) + 1);
            break;
          }
        }
      }

      if (endpoint_bytes_per_packet >= config_bytes_per_packet) {
        /* Transfers will be at most one frame long: Divide the maximum frame size
         * by the size of the endpoint and round up */
        packets_per_transfer = (ctrl->dwMaxVideoFrameSize +
                                endpoint_bytes_per_packet - 1) / endpoint_bytes_per_packet;

        /* But keep a reasonable limit: Otherwise we start dropping data */
        if (packets_per_transfer > strmh->max_packets_per_transfer) packets_per_transfer = strmh->max_packets_per_transfer;
        
        total_transfer_size = packets_per_transfer * endpoint_bytes_per_packet;
        break;
      }
    }

    /* If we searched through all the altsettings and found nothing usable */
    if (alt_idx == interface->num_altsetting) {
      ret = UVC_ERROR_INVALID_MODE;
      goto fail;
    }

    /* Select the altsetting */
    ret = libusb_set_interface_alt_setting(strmh->devh->usb_devh,
                                           altsetting->bInterfaceNumber,
                                           altsetting->bAlternateSetting);
    if (ret != UVC_SUCCESS) {
      UVC_DEBUG("libusb_set_interface_alt_setting failed");
      goto fail;
    }

  UVC_DEBUG("endpoint_bytes_per_packet=%zd, packets_per_transfer=%zd", endpoint_bytes_per_packet, packets_per_transfer);

  /* Set up the transfers */
  for (transfer_id = 0; transfer_id < LIBUVC_NUM_TRANSFER_BUFS; ++transfer_id) {
      transfer = libusb_alloc_transfer(packets_per_transfer);
      strmh->transfers[transfer_id] = transfer;
      uint8_t *buffer = malloc(total_transfer_size);
      libusb_fill_iso_transfer(
        transfer, strmh->devh->usb_devh, strmh->stream_if->bEndpointAddress,
        buffer, total_transfer_size, packets_per_transfer, _uvc_stream_callback, (void*) strmh, 5000);

      libusb_set_iso_packet_lengths(transfer, endpoint_bytes_per_packet);
    }
  } else { // bulk transfer
    for (transfer_id = 0; transfer_id < LIBUVC_NUM_TRANSFER_BUFS; ++transfer_id) {
      transfer = libusb_alloc_transfer(0);
      strmh->transfers[transfer_id] = transfer;
      uint8_t *buffer = malloc(strmh->cur_ctrl.dwMaxPayloadTransferSize);
      libusb_fill_bulk_transfer( transfer, strmh->devh->usb_devh,
          strmh->stream_if->bEndpointAddress,
          buffer, strmh->cur_ctrl.dwMaxPayloadTransferSize, _uvc_stream_callback,
          ( void* ) strmh, 5000 );
    }
  }

  strmh->user_cb = cb;
  strmh->user_ptr = user_ptr;

  /* If the user wants it, set up a thread that calls the user's function
   * with the contents of each frame.
   */
  if (cb) {
    pthread_create(&strmh->cb_thread, NULL, _uvc_user_caller, (void*) strmh);
  }

  for (transfer_id = 0; transfer_id < LIBUVC_NUM_TRANSFER_BUFS; ++transfer_id) {
    ret = libusb_submit_transfer(strmh->transfers[transfer_id]);
    if (ret != LIBUSB_SUCCESS) {
      UVC_ERROR("libusb_submit_transfer (%d) failed: %d", transfer_id, ret);
      if (transfer_id >= 2) {
          // require at least 2 transfers, so they can be used interchangeably
          ret = UVC_SUCCESS;
      }
      break;
    }
  }

  // clear all other transfers
  for (int i = transfer_id ; i < LIBUVC_NUM_TRANSFER_BUFS; ++i) {
    free_transfer(strmh->transfers[i]);
    strmh->transfers[i] = NULL;
  }
  if (ret != LIBUSB_SUCCESS) {
      for (int i = 0; i < transfer_id; ++i) {
          libusb_cancel_transfer(strmh->transfers[i]);
      }
      goto fail;
  }

  UVC_EXIT(ret);
  return ret;
fail:
  strmh->running = 0;
  UVC_EXIT(ret);
  return ret;
}

/** Begin streaming video from the stream into the callback function.
 * @ingroup streaming
 *
 * @deprecated The stream type (bulk vs. isochronous) will be determined by the
 * type of interface associated with the uvc_stream_ctrl_t parameter, regardless
 * of whether the caller requests isochronous streaming. Please switch to
 * uvc_stream_start().
 *
 * @param strmh UVC stream
 * @param cb   User callback function. See {uvc_frame_callback_t} for restrictions.
 */
uvc_error_t uvc_stream_start_iso(
    uvc_stream_handle_t *strmh,
    uvc_frame_callback_t *cb,
    void *user_ptr
) {
  return uvc_stream_start(strmh, cb, user_ptr, 0);
}

/** @internal
 * @brief User callback runner thread
 * @note There should be at most one of these per currently streaming device
 * @param arg Device handle
 */
void *_uvc_user_caller(void *arg) {
  uvc_stream_handle_t *strmh = (uvc_stream_handle_t *) arg;
  uvc_frame_t *frame = NULL;
    for (int i = 0; i < LIBUVC_NUM_FRAMEBUFFERS; ++i) {
        frame = uvc_allocate_frame(4 * strmh->width * strmh->height);
        frame->frame_format = UVC_FRAME_FORMAT_RGB;
        frame->width = strmh->width;
        frame->height = strmh->height;
        frame->data_bytes = 0;
        uvc_enqueue_frame(strmh, frame);
    }
  frame = uvc_allocate_frame(4 * strmh->width * strmh->height);
  frame->frame_format = UVC_FRAME_FORMAT_RGB;
  frame->width = strmh->width;
  frame->height = strmh->height;
  frame->data_bytes = 0;
  while (strmh->running) {
    if (UVC_SUCCESS == uvc_stream_get_frame(strmh, &frame, 0)) {
      strmh->user_cb(frame, strmh->user_ptr);
    }
  }
  return NULL; // return value ignored
}

struct timespec _get_abs_timeout(int32_t timeout_us) {
    time_t add_secs;
    time_t add_nsecs;
    struct timespec ts;
    struct timeval tv;

    add_secs = timeout_us / 1000000;
    add_nsecs = (timeout_us % 1000000) * 1000;
    ts.tv_sec = 0;
    ts.tv_nsec = 0;

#if _POSIX_TIMERS > 0
    clock_gettime(CLOCK_REALTIME, &ts);
#else
    gettimeofday(&tv, NULL);
      ts.tv_sec = tv.tv_sec;
      ts.tv_nsec = tv.tv_usec * 1000;
#endif

    ts.tv_sec += add_secs;
    ts.tv_nsec += add_nsecs;

    /* pthread_cond_timedwait FAILS with EINVAL if ts.tv_nsec > 1000000000 (1 billion)
     * Since we are just adding values to the timespec, we have to increment the seconds if nanoseconds is greater than 1 billion,
     * and then re-adjust the nanoseconds in the correct range.
     * */
    ts.tv_sec += ts.tv_nsec / 1000000000;
    ts.tv_nsec = ts.tv_nsec % 1000000000;
    return ts;
}

/** Poll for a frame
 * @ingroup streaming
 *
 * @param devh UVC device
 * @param[in|out] frame Location to store pointer to captured frame
 * @param timeout_us >0: Wait at most N microseconds; 0: Wait indefinitely; -1: return immediately
 */
uvc_error_t uvc_stream_get_frame(uvc_stream_handle_t *strmh,
			  uvc_frame_t **frame,
			  int32_t timeout_us) {
  struct timespec ts;
  if (!strmh->running || frame == NULL) {
    return UVC_ERROR_OTHER;
  }

  pthread_mutex_lock(&strmh->cb_mutex);

  if (strmh->backbuffers == NULL && strmh->frontbuffers == NULL) {
      UVC_ERROR("No framebuffers in the queue. Call uvc_enqueue_frame() before this call.");
      pthread_mutex_unlock(&strmh->cb_mutex);
      return UVC_ERROR_OTHER;
  }
  
  // wait indefinitely for buffer availability
  if (timeout_us == 0) {
      int err = 0;
      while (strmh->frontbuffers == NULL
             && strmh->running
             && err == 0) {
          err = pthread_cond_wait(&strmh->cb_cond, &strmh->cb_mutex);
      }
  } else if (timeout_us > 0) {
      int err = 0;
      ts = _get_abs_timeout(timeout_us);
      while (strmh->frontbuffers == NULL
             && strmh->running
             && err == 0) {
          err = pthread_cond_timedwait(&strmh->cb_cond, &strmh->cb_mutex, &ts);
      }
  }

  struct uvc_framebuffer *fb = strmh->frontbuffers;
  if (fb) {
      DL_DELETE(strmh->frontbuffers, fb);
      uvc_frame_t* next_frame = *frame;
      *frame = fb->frame;
      if (next_frame) {
          // push next frame
          next_frame->data_bytes = 0;
          next_frame->sequence = 0;
          fb->frame = next_frame;
          fb->status = UVC_FRAME_VALID;
          DL_APPEND(strmh->backbuffers, fb);
      } else {
          free(fb);
      }
  } else {
      *frame = NULL;
  }
  pthread_mutex_unlock(&strmh->cb_mutex);

  if (*frame == NULL) return UVC_ERROR_OTHER;
  else return UVC_SUCCESS;
}

/** @brief Stop streaming video
 * @ingroup streaming
 *
 * Closes all streams, ends threads and cancels pollers
 *
 * @param devh UVC device
 */
void uvc_stop_streaming(uvc_device_handle_t *devh) {
  uvc_stream_handle_t *strmh, *strmh_tmp;

  DL_FOREACH_SAFE(devh->streams, strmh, strmh_tmp) {
    uvc_stream_close(strmh);
  }
}

/** @brief Stop stream.
 * @ingroup streaming
 *
 * Stops stream, ends threads and cancels pollers
 *
 * @param devh UVC device
 */
uvc_error_t uvc_stream_stop(uvc_stream_handle_t *strmh) {
  UVC_ENTER();
  int i;

  if (!strmh->running)
    return UVC_ERROR_INVALID_PARAM;

  strmh->running = 0;

  for(i=0; i < LIBUVC_NUM_TRANSFER_BUFS; i++) {
    if(strmh->transfers[i] != NULL) {
      libusb_cancel_transfer(strmh->transfers[i]);
    }
  }

  // Kick the user thread awake
  pthread_mutex_lock(&strmh->cb_mutex);
  pthread_cond_broadcast(&strmh->cb_cond);
  pthread_mutex_unlock(&strmh->cb_mutex);

  /** @todo stop the actual stream, camera side? */

  if (strmh->user_cb) {
    /* wait for the thread to stop (triggered by
     * LIBUSB_TRANSFER_CANCELLED transfer) */
    pthread_join(strmh->cb_thread, NULL);
  }

  return UVC_SUCCESS;
}

/** @brief Close stream.
 * @ingroup streaming
 *
 * Closes stream, frees handle and all streaming resources.
 *
 * @param strmh UVC stream handle
 */
void uvc_stream_close(uvc_stream_handle_t *strmh) {
  UVC_ENTER();
  if (strmh->running)
    uvc_stream_stop(strmh);

  uvc_release_if(strmh->devh, strmh->stream_if->bInterfaceNumber);

  /* Wait for transfers to complete/cancel */
  do {
    int i;
    for(i=0; i < LIBUVC_NUM_TRANSFER_BUFS; i++) {
      if(strmh->transfers[i] != NULL)
        break;
    }
    if(i == LIBUVC_NUM_TRANSFER_BUFS )
      break;
    UVC_DEBUG("Waiting for transfers to complete: %d", i);
  } while(1);

  strmh->videoHandler->release(strmh->payload_ctx);

  struct uvc_framebuffer *fb, *tmp1;
  DL_FOREACH_SAFE(strmh->backbuffers, fb, tmp1) {
    DL_DELETE(strmh->backbuffers, fb);
    uvc_free_frame(fb->frame);
    free(fb->meta_buffer);
    free(fb);
  }
  DL_FOREACH_SAFE(strmh->frontbuffers, fb, tmp1) {
    DL_DELETE(strmh->frontbuffers, fb);
    uvc_free_frame(fb->frame);
    free(fb->meta_buffer);
    free(fb);
  }

  pthread_cond_destroy(&strmh->cb_cond);
  pthread_mutex_destroy(&strmh->cb_mutex);

  DL_DELETE(strmh->devh->streams, strmh);
  free(strmh);
}

void uvc_stream_set_max_packets_per_transfer(uvc_stream_handle_t *strmh, const size_t maxPpt) {
    if (strmh)
        strmh->max_packets_per_transfer = maxPpt;
}

size_t uvc_stream_get_max_packets_per_transfer(uvc_stream_handle_t *strmh) {
    return strmh->max_packets_per_transfer;
}

size_t uvc_stream_get_max_buffer_size(uvc_stream_handle_t *strmh) {
  return strmh->videoHandler->get_frame_buffer_size(strmh->payload_ctx);
}