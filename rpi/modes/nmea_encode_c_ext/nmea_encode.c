#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <Python.h>

#define NMEA_HEADER "$RPCTL"
#define NMEA_HEADER_LEN 6
#define NMEA_NUM_ARGS 18
#define NMEA_FOOTER_LEN 3
#define NMEA_LEN (NMEA_HEADER_LEN + sizeof(gamepad_data_t) + NMEA_FOOTER_LEN)
#define NMEA_MAX_LEN 256

#ifdef __GNUC__
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK( __Declaration__ ) __pragma(pack(push, 1)) __Declaration__ __pragma(pack(pop))
#endif

// packed so i can memcpy to nmea buffer
// should have no performance impact, all i64 
PACK(struct gamepad_data_t {
    int64_t ABS_LX;
    int64_t ABS_LY;
    int64_t ABS_RX;
    int64_t ABS_RY;
    int64_t BTN_THUMBL;
    int64_t BTN_THUMBR;
    int64_t ABS_HAT0X;
    int64_t ABS_HAT0Y;
    int64_t BTN_SOUTH;
    int64_t BTN_EAST;
    int64_t BTN_NORTH;
    int64_t BTN_WEST;
    int64_t BTN_LB;
    int64_t BTN_RB;
    int64_t ABS_LT;
    int64_t ABS_RT;
    int64_t BTN_START;
    int64_t BTN_SELECT;
});

typedef struct gamepad_data_t gamepad_data_t;

static uint8_t nmea_checksum(const char* nmea, size_t len) {
    uint8_t* nmea_ptr = (uint8_t*)nmea;
    uint8_t checksum = 0;
    // skip '$'
    for(uint32_t i = 1; i < len; i++) {
        checksum ^= nmea_ptr[i];
    }
    return checksum;
}

// python method to encode gamepad data to NMEA-formatted bytes
// takes in 18 python ints in the order of the gamepad_data struct
// returns a RAW (not ascii) byte buffer ready to be transmitted
static PyObject* nmea_encode_from_gamepad(PyObject* self, PyObject* args) {
    gamepad_data_t gamepad_data = {0};
    char nmea[NMEA_MAX_LEN];
    if (!PyArg_ParseTuple(args, "(llllllllllllllllll)", 
    &gamepad_data.ABS_LX, &gamepad_data.ABS_LY, &gamepad_data.ABS_RX, &gamepad_data.ABS_RY, 
    &gamepad_data.BTN_THUMBL, &gamepad_data.BTN_THUMBR, &gamepad_data.ABS_HAT0X, &gamepad_data.ABS_HAT0Y, 
    &gamepad_data.BTN_SOUTH, &gamepad_data.BTN_EAST, &gamepad_data.BTN_NORTH, &gamepad_data.BTN_WEST, 
    &gamepad_data.BTN_LB, &gamepad_data.BTN_RB, &gamepad_data.ABS_LT, &gamepad_data.ABS_RT, 
    &gamepad_data.BTN_START, &gamepad_data.BTN_SELECT)) {
        return NULL;
    }
    //printf("ABS_LX: %d ", (int32_t)gamepad_data.ABS_LX);
    //printf("ABS_LY: %d ", (int32_t)gamepad_data.ABS_LY);
    //printf("ABS_RX: %d ", (int32_t)gamepad_data.ABS_RX);
    //printf("ABS_RY: %d ", (int32_t)gamepad_data.ABS_RY);
    //printf("\n");
    // keep track of the index of the nmea buffer
    uint32_t nmea_index = 0;
    // add header to beginning of nmea buffer
    memcpy(&nmea[nmea_index], NMEA_HEADER, NMEA_HEADER_LEN);
    nmea_index += NMEA_HEADER_LEN;
    // copy PACKED (no padding!) gamepad data to nmea buffer
    // these are raw bytes, not ascii, so recipient can memcpy directly from transmission
    memcpy(&nmea[nmea_index], &gamepad_data, sizeof(gamepad_data));
    nmea_index += sizeof(gamepad_data);
    // add checksum to end of nmea buffer, formatted as 2 hex ascii characters
    snprintf(&nmea[nmea_index], NMEA_FOOTER_LEN + 1, "*%02X", nmea_checksum(nmea, nmea_index));
    // 1 extra for null terminator
    nmea_index += NMEA_FOOTER_LEN + 1;
    // print nmea buffer for debugging (will not be readable ascii, will term early)
    PyObject* res_bytes;
    res_bytes = PyBytes_FromStringAndSize(nmea, nmea_index);
    return res_bytes;
}

// python method table
static PyMethodDef nmea_methods[] = {
    {"nmea_encode", nmea_encode_from_gamepad, METH_VARARGS, "Encode gamepad data to NMEA-formatted bytes"},
    {NULL, NULL, 0, NULL}
};

// python module definition
static struct PyModuleDef nmea_module = {
    PyModuleDef_HEAD_INIT,
    "nmea_encode",
    "NMEA encoding module",
    -1,
    nmea_methods
};

// python module init
PyMODINIT_FUNC PyInit_nmea_encode(void) {
    return PyModule_Create(&nmea_module);
}