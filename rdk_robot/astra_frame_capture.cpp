#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <string>

#include <algorithm>
#include <vector>

#include <jpeglib.h>

#include <astra/capi/astra.h>
#include <astra_core/capi/astra_types.h>

static volatile int g_running = 1;

static void on_signal(int sig) {
    (void)sig;
    g_running = 0;
}

static bool write_file_atomic(const char *path, const void *data, size_t len) {
    std::string tmp = std::string(path) + ".tmp";
    FILE *fp = fopen(tmp.c_str(), "wb");
    if (!fp) return false;
    bool ok = fwrite(data, 1, len, fp) == len;
    fclose(fp);
    if (ok) ok = rename(tmp.c_str(), path) == 0;
    return ok;
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

static bool save_rgb(astra_colorframe_t color_frame, const char *path) {
    astra_image_metadata_t meta{};
    astra_rgb_pixel_t *src = nullptr;
    uint32_t byte_len = 0;
    astra_colorframe_get_metadata(color_frame, &meta);
    astra_colorframe_get_data_rgb_ptr(color_frame, &src, &byte_len);
    int width = meta.width;
    int height = meta.height;
    if (width <= 0 || height <= 0 || !src || byte_len == 0) return false;

    std::vector<uint8_t> rgb(static_cast<size_t>(width) * height * 3);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int src_index = y * width + (width - 1 - x);
            int dst_index = (y * width + x) * 3;
            rgb[dst_index + 0] = src[src_index].r;
            rgb[dst_index + 1] = src[src_index].g;
            rgb[dst_index + 2] = src[src_index].b;
        }
    }

    std::vector<unsigned char> jpeg;
    if (!jpeg_encode_rgb(rgb.data(), width, height, 85, jpeg)) return false;
    return write_file_atomic(path, jpeg.data(), jpeg.size());
}

static bool save_depth_pgm(astra_depthframe_t depth_frame, const char *path) {
    astra_image_metadata_t meta{};
    uint32_t data_len = 0;
    astra_depthframe_get_metadata(depth_frame, &meta);
    astra_depthframe_get_data_byte_length(depth_frame, &data_len);
    int width = meta.width;
    int height = meta.height;
    if (width <= 0 || height <= 0 || data_len == 0) return false;

    std::vector<uint16_t> depth(static_cast<size_t>(data_len) / sizeof(uint16_t));
    astra_depthframe_copy_data(depth_frame, reinterpret_cast<int16_t *>(depth.data()));

    std::vector<uint16_t> mirrored(static_cast<size_t>(width) * height);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            mirrored[static_cast<size_t>(y) * width + x] = depth[static_cast<size_t>(y) * width + (width - 1 - x)];
        }
    }

    std::string tmp = std::string(path) + ".tmp";
    FILE *fp = fopen(tmp.c_str(), "wb");
    if (!fp) return false;
    fprintf(fp, "P5\n%d %d\n65535\n", width, height);
    bool ok = fwrite(mirrored.data(), sizeof(uint16_t), mirrored.size(), fp) == mirrored.size();
    fclose(fp);
    if (ok) ok = rename(tmp.c_str(), path) == 0;
    return ok;
}

int main(int argc, char **argv) {
    const char *out_dir = argc > 1 ? argv[1] : "/tmp/rdk_robot_frames";
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    char cmd[512];
    snprintf(cmd, sizeof(cmd), "mkdir -p %s", out_dir);
    system(cmd);

    char rgb_path[512], depth_path[512], stamp_path[512];
    snprintf(rgb_path, sizeof(rgb_path), "%s/latest_rgb.jpg", out_dir);
    snprintf(depth_path, sizeof(depth_path), "%s/latest_depth16.pgm", out_dir);
    snprintf(stamp_path, sizeof(stamp_path), "%s/latest_stamp.txt", out_dir);

    astra_initialize();
    astra_streamsetconnection_t sensor;
    astra_reader_t reader;
    astra_depthstream_t depth_stream;
    astra_colorstream_t color_stream;

    astra_streamset_open("device/default", &sensor);
    astra_reader_create(sensor, &reader);
    astra_reader_get_depthstream(reader, &depth_stream);
    astra_reader_get_colorstream(reader, &color_stream);
    astra_stream_start(depth_stream);
    astra_stream_start(color_stream);

    fprintf(stderr, "Astra frame capture writing to %s\n", out_dir);
    astra_frame_index_t last_depth = -1;
    astra_frame_index_t last_rgb = -1;
    while (g_running) {
        astra_update();
        astra_reader_frame_t frame;
        astra_status_t rc = astra_reader_open_frame(reader, 50, &frame);
        if (rc != ASTRA_STATUS_SUCCESS) {
            usleep(10000);
            continue;
        }

        bool wrote = false;
        astra_depthframe_t depth_frame;
        astra_frame_get_depthframe(frame, &depth_frame);
        astra_frame_index_t depth_index = -1;
        astra_depthframe_get_frameindex(depth_frame, &depth_index);
        if (depth_index != last_depth && save_depth_pgm(depth_frame, depth_path)) {
            last_depth = depth_index;
            wrote = true;
        }

        astra_colorframe_t color_frame;
        astra_frame_get_colorframe(frame, &color_frame);
        astra_frame_index_t rgb_index = -1;
        astra_colorframe_get_frameindex(color_frame, &rgb_index);
        if (rgb_index != last_rgb && save_rgb(color_frame, rgb_path)) {
            last_rgb = rgb_index;
            wrote = true;
        }

        astra_reader_close_frame(&frame);
        if (wrote) {
            FILE *fp = fopen(stamp_path, "w");
            if (fp) {
                fprintf(fp, "%ld depth=%d rgb=%d\n", (long)time(NULL), depth_index, rgb_index);
                fclose(fp);
            }
        }
        usleep(100000);
    }

    astra_reader_destroy(&reader);
    astra_streamset_close(&sensor);
    astra_terminate();
    return 0;
}
