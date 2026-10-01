#pragma once
#include <stdint.h>
#include <gui/smgui.h>
#include <dsp/types.h>

#define SERVER_MAX_PACKET_SIZE  (STREAM_BUFFER_SIZE * sizeof(dsp::complex_t) * 2)

#define SERVER_MIN_FFT_BINS     64
#define SERVER_DEF_FFT_BINS     1024
#define SERVER_MAX_FFT_BINS     16384
#define SERVER_MIN_FFT_RATE     1.0
#define SERVER_DEF_FFT_RATE     20.0
#define SERVER_MAX_FFT_RATE     60.0

// Bins are quantized to a byte, so the range they're quantized over decides
// the resolution. Empty bins can sit hundreds of dB down, which would waste
// most of the range, so the span is capped: 150 dB over 255 steps is 0.59 dB.
#define SERVER_MAX_FFT_DB_SPAN  150.0f

namespace server {
    enum PacketType {
        // Client to Server
        PACKET_TYPE_COMMAND,
        PACKET_TYPE_COMMAND_ACK,
        PACKET_TYPE_BASEBAND,
        PACKET_TYPE_BASEBAND_COMPRESSED,
        PACKET_TYPE_VFO,
        PACKET_TYPE_FFT,
        PACKET_TYPE_ERROR
    };

    enum Command {
        // Client to Server
        COMMAND_GET_UI = 0x00,
        COMMAND_UI_ACTION,
        COMMAND_START,
        COMMAND_STOP,
        COMMAND_SET_FREQUENCY,
        COMMAND_GET_SAMPLERATE,
        COMMAND_SET_SAMPLE_TYPE,
        COMMAND_SET_COMPRESSION,
        COMMAND_SET_STREAM_MODE,
        COMMAND_SET_FFT_PARAMS,

        // Server to client
        COMMAND_SET_SAMPLERATE = 0x80,
        COMMAND_DISCONNECT
    };

    // Which streams the server is asked to send. Clients that never send
    // COMMAND_SET_STREAM_MODE keep getting STREAM_MODE_IQ, which is the
    // behavior of every server build before this existed.
    enum StreamMode {
        STREAM_MODE_IQ  = (1 << 0),
        STREAM_MODE_FFT = (1 << 1)
    };

    enum Error {
        ERROR_NONE = 0x00,
        ERROR_INVALID_PACKET,
        ERROR_INVALID_COMMAND,
        ERROR_INVALID_ARGUMENT
    };
    
#pragma pack(push, 1)
    struct PacketHeader {
        uint32_t type;
        uint32_t size;
    };

    struct CommandHeader {
        uint32_t cmd;
    };

    // Payload of COMMAND_SET_FFT_PARAMS
    struct FFTParams {
        uint32_t binCount;
        float rate;
    };

    // Precedes the bins of a PACKET_TYPE_FFT packet. Bins are quantized
    // linearly between minDb and maxDb, so a spectrum frame costs one byte per
    // bin rather than four.
    struct FFTHeader {
        uint32_t binCount;
        float minDb;
        float maxDb;
    };
#pragma pack(pop)
}