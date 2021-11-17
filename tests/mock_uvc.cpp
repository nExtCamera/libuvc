

#include "mock_uvc.h"

uvc_stream_handle_t* mock_stream_open() {
    uvc_stream_handle_t *strmh = static_cast<uvc_stream_handle_t*>(calloc(1, sizeof(uvc_stream_handle_t)));
    strmh->cur_ctrl.dwMaxVideoFrameSize = 10000;
    strmh->seq = 1;
    strmh->running = true;
    pthread_mutex_init(&strmh->cb_mutex, nullptr);
    pthread_cond_init(&strmh->cb_cond, nullptr);
    return strmh;
}
