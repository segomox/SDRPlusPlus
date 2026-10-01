#include "server.h"
#include "core.h"
#include <utils/flog.h>
#include <version.h>
#include <config.h>
#include <filesystem>
#include <dsp/types.h>
#include <signal_path/signal_path.h>
#include <gui/smgui.h>
#include <utils/optionlist.h>
#include "dsp/compression/sample_stream_compressor.h"
#include "dsp/sink/handler_sink.h"
#include "dsp/routing/splitter.h"
#include "dsp/buffer/reshaper.h"
#include "dsp/window/nuttall.h"
#include <volk/volk.h>
#include <fftw3.h>
#include <algorithm>
#include <cmath>
#include <zstd.h>

namespace server {
    dsp::stream<dsp::complex_t> dummyInput;
    dsp::routing::Splitter<dsp::complex_t> split;

    // Baseband branch
    dsp::stream<dsp::complex_t> iqStream;
    dsp::compression::SampleStreamCompressor comp;
    dsp::sink::Handler<uint8_t> hnd;

    // Spectrum branch
    dsp::stream<dsp::complex_t> fftStream;
    dsp::buffer::Reshaper<dsp::complex_t> reshape;
    dsp::sink::Handler<dsp::complex_t> fftSink;
    fftwf_complex* fftInBuf = NULL;
    fftwf_complex* fftOutBuf = NULL;
    fftwf_plan fftPlan;
    float* fftWindow = NULL;
    float* fftAmps = NULL;
    int fftBins = SERVER_DEF_FFT_BINS;
    double fftRate = SERVER_DEF_FFT_RATE;
    int nzFFTSize = SERVER_DEF_FFT_BINS;
    std::mutex fftMtx;

    int streamMode = STREAM_MODE_IQ;

    net::Conn client;
    std::mutex sendMtx;
    uint8_t* rbuf = NULL;
    uint8_t* sbuf = NULL;
    uint8_t* bbuf = NULL;
    uint8_t* fbuf = NULL;

    PacketHeader* r_pkt_hdr = NULL;
    uint8_t* r_pkt_data = NULL;
    CommandHeader* r_cmd_hdr = NULL;
    uint8_t* r_cmd_data = NULL;

    PacketHeader* s_pkt_hdr = NULL;
    uint8_t* s_pkt_data = NULL;
    CommandHeader* s_cmd_hdr = NULL;
    uint8_t* s_cmd_data = NULL;

    PacketHeader* bb_pkt_hdr = NULL;
    uint8_t* bb_pkt_data = NULL;

    PacketHeader* f_pkt_hdr = NULL;
    uint8_t* f_pkt_data = NULL;

    SmGui::DrawListElem dummyElem;

    ZSTD_CCtx* cctx;

    net::Listener listener;

    OptionList<std::string, std::string> sourceList;
    int sourceId = 0;
    bool running = false;
    bool compression = false;
    double sampleRate = 1000000.0;

    int main() {
        flog::info("=====| SERVER MODE |=====");

        // Init DSP
        split.init(&dummyInput);
        split.bindStream(&iqStream);
        comp.init(&iqStream, dsp::compression::PCM_TYPE_I8);
        hnd.init(&comp.out, _testServerHandler, NULL);
        reshape.init(&fftStream, SERVER_DEF_FFT_BINS, 0);
        fftSink.init(&reshape.out, _fftHandler, NULL);
        rbuf = new uint8_t[SERVER_MAX_PACKET_SIZE];
        sbuf = new uint8_t[SERVER_MAX_PACKET_SIZE];
        bbuf = new uint8_t[SERVER_MAX_PACKET_SIZE];
        fbuf = new uint8_t[sizeof(PacketHeader) + sizeof(FFTHeader) + SERVER_MAX_FFT_BINS];
        split.start();
        comp.start();
        hnd.start();

        // Initialize headers
        r_pkt_hdr = (PacketHeader*)rbuf;
        r_pkt_data = &rbuf[sizeof(PacketHeader)];
        r_cmd_hdr = (CommandHeader*)r_pkt_data;
        r_cmd_data = &rbuf[sizeof(PacketHeader) + sizeof(CommandHeader)];

        s_pkt_hdr = (PacketHeader*)sbuf;
        s_pkt_data = &sbuf[sizeof(PacketHeader)];
        s_cmd_hdr = (CommandHeader*)s_pkt_data;
        s_cmd_data = &sbuf[sizeof(PacketHeader) + sizeof(CommandHeader)];

        bb_pkt_hdr = (PacketHeader*)bbuf;
        bb_pkt_data = &bbuf[sizeof(PacketHeader)];

        f_pkt_hdr = (PacketHeader*)fbuf;
        f_pkt_data = &fbuf[sizeof(PacketHeader)];

        // Build the FFT path for the default parameters
        updateFFTPath();

        // Initialize compressor
        cctx = ZSTD_createCCtx();

        // Load config
        core::configManager.acquire();
        std::string modulesDir = core::configManager.conf["modulesDirectory"];
        std::vector<std::string> modules = core::configManager.conf["modules"];
        auto modList = core::configManager.conf["moduleInstances"].items();
        std::string sourceName = core::configManager.conf["source"];
        core::configManager.release();
        modulesDir = std::filesystem::absolute(modulesDir).string();

        // Initialize SmGui in server mode
        SmGui::init(true);

        flog::info("Loading modules");
        // Load modules and check type to only load sources ( TODO: Have a proper type parameter int the info )
        // TODO LATER: Add whitelist/blacklist stuff
        if (std::filesystem::is_directory(modulesDir)) {
            for (const auto& file : std::filesystem::directory_iterator(modulesDir)) {
                std::string path = file.path().generic_string();
                std::string fn = file.path().filename().string();
                if (file.path().extension().generic_string() != SDRPP_MOD_EXTENTSION) {
                    continue;
                }
                if (!file.is_regular_file()) { continue; }
                if (fn.find("source") == std::string::npos) { continue; }

                flog::info("Loading {0}", path);
                core::moduleManager.loadModule(path);
            }
        }
        else {
            flog::warn("Module directory {0} does not exist, not loading modules from directory", modulesDir);
        }

        // Load additional modules through the config ( TODO: Have a proper type parameter int the info )
        // TODO LATER: Add whitelist/blacklist stuff
        for (auto const& apath : modules) {
            std::filesystem::path file = std::filesystem::absolute(apath);
            std::string path = file.generic_string();
            std::string fn = file.filename().string();
            if (file.extension().generic_string() != SDRPP_MOD_EXTENTSION) {
                continue;
            }
            if (!std::filesystem::is_regular_file(file)) { continue; }
            if (fn.find("source") == std::string::npos) { continue; }

            flog::info("Loading {0}", path);
            core::moduleManager.loadModule(path);
        }

        // Create module instances
        for (auto const& [name, _module] : modList) {
            std::string mod = _module["module"];
            bool enabled = _module["enabled"];
            if (core::moduleManager.modules.find(mod) == core::moduleManager.modules.end()) { continue; }
            flog::info("Initializing {0} ({1})", name, mod);
            core::moduleManager.createInstance(name, mod);
            if (!enabled) { core::moduleManager.disableInstance(name); }
        }

        // Do post-init
        core::moduleManager.doPostInitAll();

        // Generate source list
        auto list = sigpath::sourceManager.getSourceNames();
        for (auto& name : list) {
            sourceList.define(name, name);
        }

        // Load sourceId from config
        sourceId = 0;
        if (sourceList.keyExists(sourceName)) { sourceId = sourceList.keyId(sourceName); }
        sigpath::sourceManager.selectSource(sourceList[sourceId]);

        // TODO: Use command line option
        std::string host = (std::string)core::args["addr"];
        int port = (int)core::args["port"];
        listener = net::listen(host, port);
        listener->acceptAsync(_clientHandler, NULL);

        flog::info("Ready, listening on {0}:{1}", host, port);
        while(1) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

        return 0;
    }

    void _clientHandler(net::Conn conn, void* ctx) {
        // Reject if someone else is already connected
        if (client && client->isOpen()) {
            flog::info("REJECTED Connection from {0}:{1}, another client is already connected.", "TODO", "TODO");
            
            // Issue a disconnect command to the client
            uint8_t buf[sizeof(PacketHeader) + sizeof(CommandHeader)];
            PacketHeader* tmp_phdr = (PacketHeader*)buf;
            CommandHeader* tmp_chdr = (CommandHeader*)&buf[sizeof(PacketHeader)];
            tmp_phdr->size = sizeof(PacketHeader) + sizeof(CommandHeader);
            tmp_phdr->type = PACKET_TYPE_COMMAND;
            tmp_chdr->cmd = COMMAND_DISCONNECT;
            conn->write(tmp_phdr->size, buf);

            // TODO: Find something cleaner
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            conn->close();
            
            // Start another async accept
            listener->acceptAsync(_clientHandler, NULL);
            return;
        }

        flog::info("Connection from {0}:{1}", "TODO", "TODO");
        client = std::move(conn);
        client->readAsync(sizeof(PacketHeader), rbuf, _packetHandler, NULL);

        // Perform settings reset
        sigpath::sourceManager.stop();
        comp.setPCMType(dsp::compression::PCM_TYPE_I16);
        compression = false;
        fftBins = SERVER_DEF_FFT_BINS;
        fftRate = SERVER_DEF_FFT_RATE;
        updateFFTPath();
        setStreamMode(STREAM_MODE_IQ);

        sendSampleRate(sampleRate);

        // TODO: Wait otherwise someone else could connect

        listener->acceptAsync(_clientHandler, NULL);
    }

    void _packetHandler(int count, uint8_t* buf, void* ctx) {
        PacketHeader* hdr = (PacketHeader*)buf;

        // Read the rest of the data (TODO: CHECK SIZE OR SHIT WILL BE FUCKED + ADD TIMEOUT)
        int len = 0;
        int read = 0;
        int goal = hdr->size - sizeof(PacketHeader);
        while (len < goal) {
            read = client->read(goal - len, &buf[sizeof(PacketHeader) + len]);
            if (read < 0) { return; };
            len += read;
        }

        // Parse and process
        if (hdr->type == PACKET_TYPE_COMMAND && hdr->size >= sizeof(PacketHeader) + sizeof(CommandHeader)) {
            CommandHeader* chdr = (CommandHeader*)&buf[sizeof(PacketHeader)];
            commandHandler((Command)chdr->cmd, &buf[sizeof(PacketHeader) + sizeof(CommandHeader)], hdr->size - sizeof(PacketHeader) - sizeof(CommandHeader));
        }
        else {
            sendError(ERROR_INVALID_PACKET);
        }

        // Start another async read
        client->readAsync(sizeof(PacketHeader), rbuf, _packetHandler, NULL);
    }

    void _testServerHandler(uint8_t* data, int count, void* ctx) {
        // Compress data if needed and fill out header fields
        if (compression) {
            bb_pkt_hdr->type = PACKET_TYPE_BASEBAND_COMPRESSED;
            bb_pkt_hdr->size = sizeof(PacketHeader) + (uint32_t)ZSTD_compressCCtx(cctx, &bbuf[sizeof(PacketHeader)], SERVER_MAX_PACKET_SIZE-sizeof(PacketHeader), data, count, 1);
        }
        else {
            bb_pkt_hdr->type = PACKET_TYPE_BASEBAND;
            bb_pkt_hdr->size = sizeof(PacketHeader) + count;
            memcpy(&bbuf[sizeof(PacketHeader)], data, count);
        }

        // Write to network
        if (client && client->isOpen()) {
            std::lock_guard<std::mutex> lck(sendMtx);
            client->write(bb_pkt_hdr->size, bbuf);
        }
    }

    void _fftHandler(dsp::complex_t* data, int count, void* ctx) {
        std::lock_guard<std::mutex> lck(fftMtx);
        if (!(streamMode & STREAM_MODE_FFT)) { return; }
        if (!client || !client->isOpen()) { return; }

        // Window and transform. The window is pre-multiplied by (-1)^n so the
        // output comes out centered, ie. bin 0 is the lowest frequency.
        volk_32fc_32f_multiply_32fc((lv_32fc_t*)fftInBuf, (lv_32fc_t*)data, fftWindow, nzFFTSize);
        fftwf_execute(fftPlan);

        // Convert to dB amplitude
        volk_32fc_s32f_power_spectrum_32f(fftAmps, (lv_32fc_t*)fftOutBuf, fftBins, fftBins);

        // Find the range to quantize over
        float minDb = INFINITY;
        float maxDb = -INFINITY;
        for (int i = 0; i < fftBins; i++) {
            if (fftAmps[i] < minDb) { minDb = fftAmps[i]; }
            if (fftAmps[i] > maxDb) { maxDb = fftAmps[i]; }
        }
        if (!std::isfinite(maxDb)) { return; }

        // Don't waste the 8 bits on bins that are hundreds of dB down
        if (!std::isfinite(minDb) || (maxDb - minDb) > SERVER_MAX_FFT_DB_SPAN) {
            minDb = maxDb - SERVER_MAX_FFT_DB_SPAN;
        }
        if ((maxDb - minDb) < 1.0f) { minDb = maxDb - 1.0f; }

        // Quantize to one byte per bin
        FFTHeader* fhdr = (FFTHeader*)f_pkt_data;
        fhdr->binCount = fftBins;
        fhdr->minDb = minDb;
        fhdr->maxDb = maxDb;
        uint8_t* bins = &f_pkt_data[sizeof(FFTHeader)];
        float scale = 255.0f / (maxDb - minDb);
        for (int i = 0; i < fftBins; i++) {
            bins[i] = (uint8_t)(std::clamp<float>)((fftAmps[i] - minDb) * scale, 0.0f, 255.0f);
        }

        // Write to network
        f_pkt_hdr->type = PACKET_TYPE_FFT;
        f_pkt_hdr->size = sizeof(PacketHeader) + sizeof(FFTHeader) + fftBins;
        std::lock_guard<std::mutex> slck(sendMtx);
        client->write(f_pkt_hdr->size, fbuf);
    }

    void updateFFTPath() {
        // Drain the branch before touching its buffers
        reshape.tempStop();
        fftSink.tempStop();

        {
            std::lock_guard<std::mutex> lck(fftMtx);

            // Take fftBins samples every (sampleRate / fftRate) samples
            int interval = (std::max<int>)(1, (int)round(sampleRate / fftRate));
            nzFFTSize = (std::min<int>)(interval, fftBins);
            reshape.setKeep(nzFFTSize);
            reshape.setSkip(interval - nzFFTSize);

            // Rebuild the window
            if (fftWindow) { dsp::buffer::free(fftWindow); }
            fftWindow = dsp::buffer::alloc<float>(nzFFTSize);
            for (int i = 0; i < nzFFTSize; i++) {
                fftWindow[i] = dsp::window::nuttall(i, nzFFTSize) * ((i % 2) ? -1.0f : 1.0f);
            }

            // Rebuild the plan
            if (fftInBuf) { fftwf_free(fftInBuf); }
            if (fftOutBuf) { fftwf_free(fftOutBuf); }
            fftInBuf = (fftwf_complex*)fftwf_malloc(fftBins * sizeof(fftwf_complex));
            fftOutBuf = (fftwf_complex*)fftwf_malloc(fftBins * sizeof(fftwf_complex));
            fftPlan = fftwf_plan_dft_1d(fftBins, fftInBuf, fftOutBuf, FFTW_FORWARD, FFTW_ESTIMATE);

            // Zero pad the rest of the input
            dsp::buffer::clear(fftInBuf, fftBins - nzFFTSize, nzFFTSize);

            if (fftAmps) { dsp::buffer::free(fftAmps); }
            fftAmps = dsp::buffer::alloc<float>(fftBins);
        }

        reshape.tempStart();
        fftSink.tempStart();
    }

    void setStreamMode(int mode) {
        // Never leave the client with nothing
        if (!(mode & (STREAM_MODE_IQ | STREAM_MODE_FFT))) { mode = STREAM_MODE_IQ; }

        bool wantIQ = mode & STREAM_MODE_IQ;
        bool wantFFT = mode & STREAM_MODE_FFT;
        bool hadIQ = streamMode & STREAM_MODE_IQ;
        bool hadFFT = streamMode & STREAM_MODE_FFT;
        streamMode = mode;

        // Only run the branches that are actually being streamed
        if (wantIQ != hadIQ) {
            if (wantIQ) { split.bindStream(&iqStream); }
            else { split.unbindStream(&iqStream); }
        }
        if (wantFFT != hadFFT) {
            if (wantFFT) {
                split.bindStream(&fftStream);
                reshape.start();
                fftSink.start();
            }
            else {
                split.unbindStream(&fftStream);
                reshape.stop();
                fftSink.stop();
            }
        }

        flog::info("Stream mode: {0}{1}", wantIQ ? "IQ " : "", wantFFT ? "FFT" : "");
    }

    void setInput(dsp::stream<dsp::complex_t>* stream) {
        split.setInput(stream);
    }

    void commandHandler(Command cmd, uint8_t* data, int len) {
        if (cmd == COMMAND_GET_UI) {
            sendUI(COMMAND_GET_UI, "", dummyElem);
        }
        else if (cmd == COMMAND_UI_ACTION && len >= 3) {
            // Check if sending back data is needed
            int i = 0;
            bool sendback = data[i++];
            len--;
            
            // Load id
            SmGui::DrawListElem diffId;
            int count = SmGui::DrawList::loadItem(diffId, &data[i], len);
            if (count < 0) { sendError(ERROR_INVALID_ARGUMENT); return; }
            if (diffId.type != SmGui::DRAW_LIST_ELEM_TYPE_STRING) { sendError(ERROR_INVALID_ARGUMENT); return; } 
            i += count;
            len -= count;

            // Load value
            SmGui::DrawListElem diffValue;
            count = SmGui::DrawList::loadItem(diffValue, &data[i], len);
            if (count < 0) { sendError(ERROR_INVALID_ARGUMENT); return; }
            i += count;
            len -= count;

            // Render and send back
            if (sendback) {
                sendUI(COMMAND_UI_ACTION, diffId.str, diffValue);
            }
            else {
                renderUI(NULL, diffId.str, diffValue);
            }
        }
        else if (cmd == COMMAND_START) {
            sigpath::sourceManager.start();
            running = true;
        }
        else if (cmd == COMMAND_STOP) {
            sigpath::sourceManager.stop();
            running = false;
        }
        else if (cmd == COMMAND_SET_FREQUENCY && len == 8) {
            sigpath::sourceManager.tune(*(double*)data);
            sendCommandAck(COMMAND_SET_FREQUENCY, 0);
        }
        else if (cmd == COMMAND_SET_SAMPLE_TYPE && len == 1) {
            dsp::compression::PCMType type = (dsp::compression::PCMType)*(uint8_t*)data;
            comp.setPCMType(type);
        }
        else if (cmd == COMMAND_SET_COMPRESSION && len == 1) {
            compression = *(uint8_t*)data;
        }
        else if (cmd == COMMAND_SET_STREAM_MODE && len == 1) {
            setStreamMode(*(uint8_t*)data);

            // The ack is what tells the client this server knows about stream
            // modes at all. Older servers answer ERROR_INVALID_COMMAND instead.
            sendCommandAck(COMMAND_SET_STREAM_MODE, 0);
        }
        else if (cmd == COMMAND_SET_FFT_PARAMS && len == sizeof(FFTParams)) {
            FFTParams* params = (FFTParams*)data;
            fftBins = (std::clamp<int>)(params->binCount, SERVER_MIN_FFT_BINS, SERVER_MAX_FFT_BINS);
            fftRate = (std::clamp<double>)(params->rate, SERVER_MIN_FFT_RATE, SERVER_MAX_FFT_RATE);
            updateFFTPath();

            // Report back what was actually accepted
            FFTParams* accepted = (FFTParams*)s_cmd_data;
            accepted->binCount = fftBins;
            accepted->rate = fftRate;
            sendCommandAck(COMMAND_SET_FFT_PARAMS, sizeof(FFTParams));
        }
        else {
            flog::error("Invalid Command: {0} (len = {1})", (int)cmd, len);
            sendError(ERROR_INVALID_COMMAND);
        }
    }

    void drawMenu() {
        if (running) { SmGui::BeginDisabled(); }
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Combo("##sdrpp_server_src_sel", &sourceId, sourceList.txt)) {
            sigpath::sourceManager.selectSource(sourceList[sourceId]);
            core::configManager.acquire();
            core::configManager.conf["source"] = sourceList.key(sourceId);
            core::configManager.release(true);
        }
        if (running) { SmGui::EndDisabled(); }

        sigpath::sourceManager.showSelectedMenu();
    }

    void renderUI(SmGui::DrawList* dl, std::string diffId, SmGui::DrawListElem diffValue) {
        // If we're recording and there's an action, render once with the action and record without

        if (dl && !diffId.empty()) {
            SmGui::setDiff(diffId, diffValue);
            drawMenu();

            SmGui::setDiff("", dummyElem);
            SmGui::startRecord(dl);
            drawMenu();
            SmGui::stopRecord();
        }
        else {
            SmGui::setDiff(diffId, diffValue);
            SmGui::startRecord(dl);
            drawMenu();
            SmGui::stopRecord();
        }
    }

    void sendUI(Command originCmd, std::string diffId, SmGui::DrawListElem diffValue) {
        // Render UI
        SmGui::DrawList dl;
        renderUI(&dl, diffId, diffValue);

        // Create response
        int size = dl.getSize();
        dl.store(s_cmd_data, size);

        // Send to network
        sendCommandAck(originCmd, size);
    }

    void sendError(Error err) {
        PacketHeader* hdr = (PacketHeader*)sbuf;
        s_pkt_data[0] = err;
        sendPacket(PACKET_TYPE_ERROR, 1);
    }

    void sendSampleRate(double sampleRate) {
        *(double*)s_cmd_data = sampleRate;
        sendCommand(COMMAND_SET_SAMPLERATE, sizeof(double));
    }

    void setInputSampleRate(double samplerate) {
        sampleRate = samplerate;

        // The reshaper interval depends on the sample rate
        updateFFTPath();

        if (!client || !client->isOpen()) { return; }
        sendSampleRate(sampleRate);
    }

    void sendPacket(PacketType type, int len) {
        s_pkt_hdr->type = type;
        s_pkt_hdr->size = sizeof(PacketHeader) + len;
        std::lock_guard<std::mutex> lck(sendMtx);
        client->write(s_pkt_hdr->size, sbuf);
    }

    void sendCommand(Command cmd, int len) {
        s_cmd_hdr->cmd = cmd;
        sendPacket(PACKET_TYPE_COMMAND, sizeof(CommandHeader) + len);
    }

    void sendCommandAck(Command cmd, int len) {
        s_cmd_hdr->cmd = cmd;
        sendPacket(PACKET_TYPE_COMMAND_ACK, sizeof(CommandHeader) + len);
    }
}
