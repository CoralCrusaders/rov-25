#include <gst/gst.h>
#include "gst_camera_stream.h"

void gst_stream_video0_localhost(int argc, char* argv[]) {
    GstElement *pipeline;
    GstBus *bus;
    GstMessage *msg;
    GError *error = NULL;
    
    gst_init(&argc, &argv);

    pipeline = 
        gst_parse_lanch("v4l2src device=/dev/video0 ! video/x-h264 ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=6000", &error);
    
    if (!pipeline) {
        g_printerr("Failed to create pipeline: %s\n", error->message);
        g_clear_error(&error);
        return -1;
    }

    // Start playing the pipeline
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Wait until error or EOS (End of Stream)
    bus = gst_element_get_bus(pipeline);
    msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE, GST_MESSAGE_ERROR | GST_MESSAGE_EOS);

    // Handle errors or end of stream
    if (msg != NULL) {
        GError *err;
        gchar *debug_info;

        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR:
                gst_message_parse_error(msg, &err, &debug_info);
                g_printerr("Error received from element %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
                g_printerr("Debugging information: %s\n", debug_info ? debug_info : "none");
                g_clear_error(&err);
                g_free(debug_info);
                break;
            case GST_MESSAGE_EOS:
                g_print("End-Of-Stream reached.\n");
                break;
            default:
                g_printerr("Unexpected message received.\n");
                break;
        }
        gst_message_unref(msg);
    }

    // Free resources
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
}
/*
void gst_stream_video_remote(int argc, char* argv[], char* video, char* ip, int port) {
    GstElement *pipeline;
    GstBus *bus;
    GstMessage *msg;
    GError *error = NULL;
    
    gst_init(&argc, &argv);

    
    char str_gst_parse[255];
    sprintf(str_gst_parse, "v4l2src device=%s ! video/x-h264 ! h264parse ! rtph264pay ! udpsink host=%s port=%d", video, ip, port);

    pipeline = 
        gst_parse_lanch((*gchar)(&str_gst_parse), &error);
    
    if (!pipeline) {
        g_printerr("Failed to create pipeline: %s\n", error->message);
        g_clear_error(&error);
        return -1;
    }

    // Start playing the pipeline
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Wait until error or EOS (End of Stream)
    bus = gst_element_get_bus(pipeline);
    msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE, GST_MESSAGE_ERROR | GST_MESSAGE_EOS);

    // Handle errors or end of stream
    if (msg != NULL) {
        GError *err;
        gchar *debug_info;

        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR:
                gst_message_parse_error(msg, &err, &debug_info);
                g_printerr("Error received from element %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
                g_printerr("Debugging information: %s\n", debug_info ? debug_info : "none");
                g_clear_error(&err);
                g_free(debug_info);
                break;
            case GST_MESSAGE_EOS:
                g_print("End-Of-Stream reached.\n");
                break;
            default:
                g_printerr("Unexpected message received.\n");
                break;
        }
        gst_message_unref(msg);
    }

    // Free resources
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
}
*/