
#include "SDL.h"
#include "libuvc/libuvc.h"
#include <time.h>

static SDL_Window *window;
static SDL_Renderer *renderer;
static SDL_Texture *texture;

static void setupSDL()
{
    SDL_Init(SDL_INIT_VIDEO);

    window = SDL_CreateWindow(
        "SDL2Test",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        640,
        480,
        SDL_WINDOW_SHOWN);

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);

    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB888, SDL_TEXTUREACCESS_STREAMING, 640, 480);
}
static void cleanupSDL()
{
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
static void eventLoop()
{
    SDL_Event e;
    bool quit = false;
    while (!quit)
    {
        while (SDL_PollEvent(&e))
        {
            if (e.type == SDL_QUIT)
            {
                quit = true;
            }
            if (e.type == SDL_KEYDOWN)
            {
                quit = true;
            }
            if (e.type == SDL_MOUSEBUTTONDOWN)
            {
                quit = true;
            }
        }
        //First clear the renderer
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        //Update the screen
        SDL_RenderPresent(renderer);
    }
}
static void cb(uvc_frame_t *frame, void *ptr);

int main()
{
    setupSDL();
    uvc_error_t ret;
    uvc_context_t *ctx;
    if ((ret = uvc_init(&ctx, nullptr)) != UVC_SUCCESS) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "init err %s", uvc_strerror(ret));
        return ret;
    }

    uvc_device_t *dev;
    ret = uvc_find_device(ctx, &dev, 0, 0, nullptr);
    if (UVC_SUCCESS != ret) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "uvc_find_device err %s", uvc_strerror(ret));
        return ret;
    }

    uvc_device_descriptor_t *desc;
    uvc_get_device_descriptor(dev, &desc);
    if (desc->product)
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "found device %s", desc->product);
    else
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "found device (unknown)");
    uvc_free_device_descriptor(desc);

    // open device
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "open device...");
    uvc_device_handle_t *devh;

    ret = uvc_open(dev, &devh);
    if (UVC_SUCCESS != ret) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "uvc_open err %s", uvc_strerror(ret));
        return ret;
    }
    uvc_print_diag(devh, stderr);

    uvc_stream_ctrl_t ctrl;
    ret = uvc_get_stream_ctrl_format_size(
        devh, &ctrl, UVC_FRAME_FORMAT_MJPEG, 640, 480, 30);
    if (UVC_SUCCESS != ret) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "uvc_get_stream_ctrl_format_size err %s", uvc_strerror(ret));
        return ret;
    }
    uvc_print_stream_ctrl(&ctrl, stderr);

    ret = uvc_start_streaming(devh, &ctrl, cb, 0, 0);
    if (UVC_SUCCESS != ret) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "uvc_start_streaming err %s", uvc_strerror(ret));
        return ret;
    }
    eventLoop();

    uvc_stop_streaming(devh);
    uvc_close(devh);

    // cleanup
    if (ret == UVC_SUCCESS)
    {
        uvc_unref_device(dev);
    }

    uvc_exit(ctx);
    cleanupSDL();
}

struct timespec lastTime, current;

int fps = 0;
static bool firstImg = false;
void cb(uvc_frame_t *frame, void *ptr)
{
    uvc_frame_t rgb;

    clock_gettime(CLOCK_MONOTONIC_RAW, &current);
    ++fps;
    double diff = (current.tv_nsec - lastTime.tv_nsec) / 1000000000.0 +
    (current.tv_sec  - lastTime.tv_sec);
    if (diff >= 1.0) {
        printf("stream_thread_func: fps=%d avg_ftime=%d\n", fps, 1000/fps);
        lastTime = current;
        fps = 0;
    }

    uint8_t *d = (uint8_t *)frame->data;

    int pitch;
    if (!SDL_LockTexture(texture, nullptr, &rgb.data, &pitch)) {
        rgb.data_bytes = pitch * 480;
        auto ret = uvc_any2rgb(frame, &rgb);
        if (ret)
        {
            uvc_perror(ret, "uvc_mjpeg2rgb");
            SDL_UnlockTexture(texture);
            return;
        }

        SDL_UnlockTexture(texture);
    }
}
