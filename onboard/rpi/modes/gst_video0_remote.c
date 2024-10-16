#include "../camera_streaming/gst_camera_stream.h"
#include <gst/gst.h>

int main (int argc, char *argv[]) {
    gst_stream_video0_localhost(argc, argv);
    return 0;
}