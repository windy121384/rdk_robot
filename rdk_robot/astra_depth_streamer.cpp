#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <jpeglib.h>

#include <algorithm>
#include <vector>

#include <astra/capi/astra.h>
#include <astra_core/capi/astra_types.h>

static volatile int g_running = 1;
static const int JPEG_QUALITY = 85;
static const int CAPTURE_INTERVAL_US = 250000;
static const int STREAM_INTERVAL_US = 250000;
static const int DOWNSCALE = 2;

enum StreamMode { STREAM_DEPTH, STREAM_RGB };

struct CachedJpeg {
    pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
    std::vector<unsigned char> jpeg;
    uint64_t seq = 0;
};

struct AppState {
    astra_reader_t reader;
    CachedJpeg depth;
    CachedJpeg rgb;
};

struct ClientArgs {
    int client;
    CachedJpeg *cache;
};

static void on_signal(int sig) {
    (void)sig;
    g_running = 0;
}

static int write_all(int fd, const void *buf, size_t len) {
    const uint8_t *p = static_cast<const uint8_t *>(buf);
    while (len > 0) {
        ssize_t n = write(fd, p, len);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (n == 0) return -1;
        p += n;
        len -= static_cast<size_t>(n);
    }
    return 0;
}

static int make_server(int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }
    if (listen(fd, 8) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static void send_page(int client) {
    const char *page =
        "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
        "<!doctype html><html><head><title>Astra Streams</title>"
        "<style>body{font-family:sans-serif;background:#111;color:#eee;text-align:center}"
        ".wrap{display:flex;gap:16px;justify-content:center;flex-wrap:wrap}.card{width:48%;min-width:360px}"
        "img{width:100%;border:1px solid #555}.hint{color:#aaa}</style>"
        "</head><body><h2>Astra Mini RGB + Depth Streams</h2>"
        "<p class=\"hint\">Single capture thread + cached MJPEG. Depth: red/yellow = near, blue = far.</p>"
        "<div class=\"wrap\"><div class=\"card\"><h3>RGB</h3><img src=\"/rgb.mjpg\"></div>"
        "<div class=\"card\"><h3>Color Depth</h3><img src=\"/depth.mjpg\"></div></div>"
        "</body></html>";
    write_all(client, page, strlen(page));
}

static bool jpeg_encode_rgb(const uint8_t *rgb, int width, int height, int quality, std::vector<unsigned char> &jpeg) {
    jpeg_compress_struct cinfo{};
    jpeg_error_mgr jerr{};
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    unsigned char *out_buf = nullptr;
    unsigned long out_size = 0;
    jpeg_mem_dest(&cinfo, &out_buf, &out_size);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    while (cinfo.next_scanline < cinfo.image_height) {
        JSAMPROW row = const_cast<JSAMPROW>(&rgb[cinfo.next_scanline * width * 3]);
        jpeg_write_scanlines(&cinfo, &row, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg.assign(out_buf, out_buf + out_size);
    jpeg_destroy_compress(&cinfo);
    free(out_buf);
    return !jpeg.empty();
}

static void colorize_depth(uint16_t depth_mm, uint8_t *rgb) {
    if (depth_mm == 0) {
        rgb[0] = rgb[1] = rgb[2] = 0;
        return;
    }
    const uint16_t min_mm = 300;
    const uint16_t max_mm = 4000;
    uint16_t d = std::max(min_mm, std::min(depth_mm, max_mm));
    float t = static_cast<float>(d - min_mm) / static_cast<float>(max_mm - min_mm);
    float x = (1.0f - t) * 4.0f;
    float r = std::min(1.0f, std::max(0.0f, x - 1.5f));
    float g = std::min(1.0f, std::max(0.0f, 1.5f - std::abs(x - 2.0f)));
    float b = std::min(1.0f, std::max(0.0f, 1.5f - x));
    rgb[0] = static_cast<uint8_t>(r * 255.0f);
    rgb[1] = static_cast<uint8_t>(g * 255.0f);
    rgb[2] = static_cast<uint8_t>(b * 255.0f);
}

static bool encode_depth_jpeg(astra_depthframe_t depth_frame, std::vector<unsigned char> &jpeg) {
    astra_image_metadata_t meta{};
    uint32_t data_len = 0;
    astra_depthframe_get_metadata(depth_frame, &meta);
    astra_depthframe_get_data_byte_length(depth_frame, &data_len);
    const int width = meta.width;
    const int height = meta.height;
    if (width <= 0 || height <= 0 || data_len == 0) return false;

    std::vector<uint16_t> depth(static_cast<size_t>(data_len) / sizeof(uint16_t));
    astra_depthframe_copy_data(depth_frame, reinterpret_cast<int16_t *>(depth.data()));

    const int out_width = width / DOWNSCALE;
    const int out_height = height / DOWNSCALE;
    std::vector<uint8_t> rgb(static_cast<size_t>(out_width) * static_cast<size_t>(out_height) * 3);
    for (int y = 0; y < out_height; ++y) {
        for (int x = 0; x < out_width; ++x) {
            int src_y = y * DOWNSCALE;
            int src_x = width - 1 - (x * DOWNSCALE);
            int src_index = src_y * width + src_x;
            int dst_index = y * out_width + x;
            uint16_t v = depth[static_cast<size_t>(src_index)];
            colorize_depth(v, &rgb[static_cast<size_t>(dst_index) * 3]);
        }
    }
    return jpeg_encode_rgb(rgb.data(), out_width, out_height, JPEG_QUALITY, jpeg);
}

static bool encode_rgb_jpeg(astra_colorframe_t color_frame, std::vector<unsigned char> &jpeg) {
    astra_image_metadata_t meta{};
    astra_rgb_pixel_t *src = nullptr;
    uint32_t byte_len = 0;
    astra_colorframe_get_metadata(color_frame, &meta);
    astra_colorframe_get_data_rgb_ptr(color_frame, &src, &byte_len);
    const int width = meta.width;
    const int height = meta.height;
    if (width <= 0 || height <= 0 || !src || byte_len == 0) return false;

    const int out_width = width / DOWNSCALE;
    const int out_height = height / DOWNSCALE;
    std::vector<uint8_t> rgb(static_cast<size_t>(out_width) * static_cast<size_t>(out_height) * 3);
    for (int y = 0; y < out_height; ++y) {
        for (int x = 0; x < out_width; ++x) {
            int src_y = y * DOWNSCALE;
            int src_x = width - 1 - (x * DOWNSCALE);
            int src_index = src_y * width + src_x;
            int dst_index = (y * out_width + x) * 3;
            rgb[static_cast<size_t>(dst_index + 0)] = src[src_index].r;
            rgb[static_cast<size_t>(dst_index + 1)] = src[src_index].g;
            rgb[static_cast<size_t>(dst_index + 2)] = src[src_index].b;
        }
    }
    return jpeg_encode_rgb(rgb.data(), out_width, out_height, JPEG_QUALITY, jpeg);
}

static void update_cache(CachedJpeg *cache, const std::vector<unsigned char> &jpeg) {
    pthread_mutex_lock(&cache->lock);
    cache->jpeg = jpeg;
    cache->seq++;
    pthread_cond_broadcast(&cache->cond);
    pthread_mutex_unlock(&cache->lock);
}

static void *capture_thread(void *arg) {
    AppState *state = static_cast<AppState *>(arg);
    astra_frame_index_t last_depth = -1;
    astra_frame_index_t last_rgb = -1;

    while (g_running) {
        astra_update();
        astra_reader_frame_t frame;
        astra_status_t rc = astra_reader_open_frame(state->reader, 50, &frame);
        if (rc != ASTRA_STATUS_SUCCESS) {
            usleep(10000);
            continue;
        }

        astra_depthframe_t depth_frame;
        astra_frame_get_depthframe(frame, &depth_frame);
        astra_frame_index_t depth_index = -1;
        astra_depthframe_get_frameindex(depth_frame, &depth_index);
        if (depth_index != last_depth) {
            std::vector<unsigned char> jpeg;
            if (encode_depth_jpeg(depth_frame, jpeg)) {
                update_cache(&state->depth, jpeg);
                last_depth = depth_index;
            }
        }

        astra_colorframe_t color_frame;
        astra_frame_get_colorframe(frame, &color_frame);
        astra_frame_index_t rgb_index = -1;
        astra_colorframe_get_frameindex(color_frame, &rgb_index);
        if (rgb_index != last_rgb) {
            std::vector<unsigned char> jpeg;
            if (encode_rgb_jpeg(color_frame, jpeg)) {
                update_cache(&state->rgb, jpeg);
                last_rgb = rgb_index;
            }
        }

        astra_reader_close_frame(&frame);
        usleep(CAPTURE_INTERVAL_US);
    }
    return nullptr;
}

static bool wait_cached_jpeg(CachedJpeg *cache, uint64_t *last_seq, std::vector<unsigned char> &jpeg) {
    pthread_mutex_lock(&cache->lock);
    while (g_running && cache->seq == *last_seq) {
        pthread_cond_wait(&cache->cond, &cache->lock);
    }
    if (!g_running) {
        pthread_mutex_unlock(&cache->lock);
        return false;
    }
    jpeg = cache->jpeg;
    *last_seq = cache->seq;
    pthread_mutex_unlock(&cache->lock);
    return !jpeg.empty();
}

static void serve_mjpeg(int client, CachedJpeg *cache) {
    const char *hdr =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n";
    if (write_all(client, hdr, strlen(hdr)) < 0) return;

    uint64_t last_seq = 0;
    while (g_running) {
        std::vector<unsigned char> jpeg;
        if (!wait_cached_jpeg(cache, &last_seq, jpeg)) continue;
        char part_header[160];
        int part_len = snprintf(part_header, sizeof(part_header),
                                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                                jpeg.size());
        if (write_all(client, part_header, static_cast<size_t>(part_len)) < 0 ||
            write_all(client, jpeg.data(), jpeg.size()) < 0 ||
            write_all(client, "\r\n", 2) < 0) {
            break;
        }
        usleep(STREAM_INTERVAL_US);
    }
}

static void *stream_thread(void *arg) {
    ClientArgs *ctx = static_cast<ClientArgs *>(arg);
    serve_mjpeg(ctx->client, ctx->cache);
    close(ctx->client);
    delete ctx;
    return nullptr;
}

static bool spawn_stream(int client, CachedJpeg *cache) {
    ClientArgs *ctx = new ClientArgs{client, cache};
    pthread_t tid;
    if (pthread_create(&tid, nullptr, stream_thread, ctx) == 0) {
        pthread_detach(tid);
        return true;
    }
    delete ctx;
    return false;
}

static bool handle_client(int client, AppState *state) {
    char req[1024];
    ssize_t n = read(client, req, sizeof(req) - 1);
    if (n <= 0) return false;
    req[n] = '\0';
    if (strncmp(req, "GET /depth.mjpg", 15) == 0 || strncmp(req, "GET /depth.pgm", 14) == 0) {
        return spawn_stream(client, &state->depth);
    }
    if (strncmp(req, "GET /rgb.mjpg", 13) == 0) {
        return spawn_stream(client, &state->rgb);
    }
    send_page(client);
    return false;
}

int main(int argc, char **argv) {
    int port = 8090;
    if (argc > 1) port = atoi(argv[1]);
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    astra_initialize();

    astra_streamsetconnection_t sensor;
    astra_depthstream_t depth_stream;
    astra_colorstream_t color_stream;
    AppState state{};

    astra_streamset_open("device/default", &sensor);
    astra_reader_create(sensor, &state.reader);
    astra_reader_get_depthstream(state.reader, &depth_stream);
    astra_reader_get_colorstream(state.reader, &color_stream);
    astra_stream_start(depth_stream);
    astra_stream_start(color_stream);

    float hfov = 0.0f, vfov = 0.0f;
    char serial[128] = {0};
    astra_depthstream_get_hfov(depth_stream, &hfov);
    astra_depthstream_get_vfov(depth_stream, &vfov);
    astra_depthstream_get_serialnumber(depth_stream, serial, sizeof(serial));
    fprintf(stderr, "Astra cached RGB+Depth MJPEG starting: serial=%s hfov=%f vfov=%f port=%d\n", serial, hfov, vfov, port);

    pthread_t cap_tid;
    if (pthread_create(&cap_tid, nullptr, capture_thread, &state) != 0) {
        fprintf(stderr, "failed to start capture thread\n");
        return 1;
    }

    int server = make_server(port);
    if (server < 0) {
        perror("listen");
        return 1;
    }
    fprintf(stderr, "Open http://<board-ip>:%d/\n", port);

    while (g_running) {
        sockaddr_in peer{};
        socklen_t peer_len = sizeof(peer);
        int client = accept(server, reinterpret_cast<sockaddr *>(&peer), &peer_len);
        if (client < 0) {
            if (errno == EINTR) continue;
            perror("accept");
            break;
        }
        bool handed_to_worker = handle_client(client, &state);
        if (!handed_to_worker) close(client);
    }

    close(server);
    pthread_join(cap_tid, nullptr);
    astra_reader_destroy(&state.reader);
    astra_streamset_close(&sensor);
    astra_terminate();
    return 0;
}
