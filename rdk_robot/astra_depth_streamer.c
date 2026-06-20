#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <astra/capi/astra.h>
#include <astra_core/capi/astra_types.h>

static volatile int g_running = 1;

static void on_signal(int sig) {
    (void)sig;
    g_running = 0;
}

static int write_all(int fd, const void *buf, size_t len) {
    const uint8_t *p = (const uint8_t *)buf;
    while (len > 0) {
        ssize_t n = write(fd, p, len);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (n == 0) return -1;
        p += n;
        len -= (size_t)n;
    }
    return 0;
}

static int make_server(int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }
    if (listen(fd, 4) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static void send_page(int client) {
    const char *page =
        "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
        "<!doctype html><html><head><title>Astra Depth</title>"
        "<style>body{font-family:sans-serif;background:#111;color:#eee;text-align:center}img{max-width:96vw;border:1px solid #555}</style>"
        "</head><body><h2>Astra Mini Depth Stream</h2><img src=\"/depth.pgm\"></body></html>";
    write_all(client, page, strlen(page));
}

static void serve_depth_pgm(int client, astra_reader_t reader) {
    const char *hdr =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n";
    if (write_all(client, hdr, strlen(hdr)) < 0) return;

    astra_frame_index_t last_index = -1;
    while (g_running) {
        astra_update();
        astra_reader_frame_t frame;
        astra_status_t rc = astra_reader_open_frame(reader, 50, &frame);
        if (rc != ASTRA_STATUS_SUCCESS) {
            usleep(10000);
            continue;
        }

        astra_depthframe_t depth_frame;
        astra_frame_get_depthframe(frame, &depth_frame);

        astra_frame_index_t frame_index;
        astra_depthframe_get_frameindex(depth_frame, &frame_index);
        if (frame_index == last_index) {
            astra_reader_close_frame(&frame);
            continue;
        }
        last_index = frame_index;

        astra_image_metadata_t meta;
        uint32_t data_len = 0;
        astra_depthframe_get_metadata(depth_frame, &meta);
        astra_depthframe_get_data_byte_length(depth_frame, &data_len);
        int width = meta.width;
        int height = meta.height;
        if (width <= 0 || height <= 0 || data_len == 0) {
            astra_reader_close_frame(&frame);
            continue;
        }

        uint16_t *depth = (uint16_t *)malloc(data_len);
        uint8_t *gray = (uint8_t *)malloc((size_t)width * (size_t)height);
        if (!depth || !gray) {
            free(depth);
            free(gray);
            astra_reader_close_frame(&frame);
            break;
        }
        astra_depthframe_copy_data(depth_frame, (int16_t *)depth);

        uint16_t max_mm = 4000;
        for (int i = 0; i < width * height; ++i) {
            uint16_t v = depth[i];
            gray[i] = (v == 0) ? 0 : (uint8_t)(255 - ((v > max_mm ? max_mm : v) * 255 / max_mm));
        }

        char pgm_header[64];
        int pgm_header_len = snprintf(pgm_header, sizeof(pgm_header), "P5\n%d %d\n255\n", width, height);
        size_t pgm_len = (size_t)pgm_header_len + (size_t)width * (size_t)height;
        char part_header[160];
        int part_len = snprintf(part_header, sizeof(part_header),
                                "--frame\r\nContent-Type: image/x-portable-graymap\r\nContent-Length: %zu\r\n\r\n",
                                pgm_len);
        if (write_all(client, part_header, (size_t)part_len) < 0 ||
            write_all(client, pgm_header, (size_t)pgm_header_len) < 0 ||
            write_all(client, gray, (size_t)width * (size_t)height) < 0 ||
            write_all(client, "\r\n", 2) < 0) {
            free(depth);
            free(gray);
            astra_reader_close_frame(&frame);
            break;
        }

        free(depth);
        free(gray);
        astra_reader_close_frame(&frame);
        usleep(33000);
    }
}

static void handle_client(int client, astra_reader_t reader) {
    char req[1024];
    ssize_t n = read(client, req, sizeof(req) - 1);
    if (n <= 0) return;
    req[n] = '\0';
    if (strncmp(req, "GET /depth.pgm", 14) == 0) {
        serve_depth_pgm(client, reader);
    } else {
        send_page(client);
    }
}

int main(int argc, char **argv) {
    int port = 8090;
    if (argc > 1) port = atoi(argv[1]);
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    astra_initialize();

    astra_streamsetconnection_t sensor;
    astra_reader_t reader;
    astra_depthstream_t depth_stream;

    astra_streamset_open("device/default", &sensor);
    astra_reader_create(sensor, &reader);
    astra_reader_get_depthstream(reader, &depth_stream);
    astra_stream_start(depth_stream);

    float hfov = 0.0f, vfov = 0.0f;
    char serial[128] = {0};
    astra_depthstream_get_hfov(depth_stream, &hfov);
    astra_depthstream_get_vfov(depth_stream, &vfov);
    astra_depthstream_get_serialnumber(depth_stream, serial, sizeof(serial));
    fprintf(stderr, "Astra depth stream starting: serial=%s hfov=%f vfov=%f port=%d\n", serial, hfov, vfov, port);

    int server = make_server(port);
    if (server < 0) {
        perror("listen");
        return 1;
    }
    fprintf(stderr, "Open http://<board-ip>:%d/\n", port);

    while (g_running) {
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);
        int client = accept(server, (struct sockaddr *)&peer, &peer_len);
        if (client < 0) {
            if (errno == EINTR) continue;
            perror("accept");
            break;
        }
        handle_client(client, reader);
        close(client);
    }

    close(server);
    astra_reader_destroy(&reader);
    astra_streamset_close(&sensor);
    astra_terminate();
    return 0;
}
