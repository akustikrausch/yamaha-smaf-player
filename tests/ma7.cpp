// SPDX-License-Identifier: Apache-2.0
#include "ma_player.h"
#include "smaf_voice.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

using namespace fxchain::smaf;

static void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

static std::vector<float> render(int format, const std::vector<uint8_t>& sequence, bool rhythm = false) {
    SmafFile file;
    TrackChunk track;
    track.formatType = format;
    track.trackNumber = format == 3 ? 7 : 0;
    track.durationTimeBase = track.gateTimeBase = 2;
    track.channelStatus.resize(format == 3 ? 32 : 16);
    if (rhythm) track.channelStatus[0] = 3;
    track.sequenceData = sequence;
    file.tracks.push_back(track);
    MaPlayer player;
    require(player.init(file, 8000), "score rejected");
    std::vector<float> audio;
    float buffer[512];
    for (int frames = 0; frames < 32000;) {
        int got = player.render(buffer, 256);
        if (!got) break;
        audio.insert(audio.end(), buffer, buffer + got * 2);
        frames += got;
    }
    return audio;
}

static void events() {
    // different programs, volumes and running velocities on channels 0 and 16.
    // the equivalent mobile score uses channels 0 and 1.
    const std::vector<uint8_t> native = {
        0, 0xf0, 5, 0x43, 0x10, 0x30, 0x90, 0xf7,
        0, 0x40, 16, 0, 0xc0, 40,
        0, 0x30, 7, 80, 0, 0xb0, 7, 110,
        0, 0x30, 10, 32, 0, 0xb0, 10, 96,
        0, 0xe0, 0, 72,
        0, 0x10, 60, 30, 0x81, 2,
        0, 0x90, 67, 110, 0x81, 2,
        0x81, 2, 0x00, 64, 0x81, 2,
        0, 0x80, 71, 0x81, 2,
        0, 0xff, 0,
        0, 0xff, 0x2f, 0,
        0, 0x10, 20, 127, 1,
    };
    const std::vector<uint8_t> mobile = {
        0, 0xf0, 5, 0x43, 0x10, 0x30, 0x90, 0xf7,
        0, 0xc0, 16, 0, 0xc1, 40,
        0, 0xb0, 7, 80, 0, 0xb1, 7, 110,
        0, 0xb0, 10, 32, 0, 0xb1, 10, 96,
        0, 0xe1, 0, 72,
        0, 0x90, 60, 30, 0x81, 2,
        0, 0x91, 67, 110, 0x81, 2,
        0x81, 2, 0x80, 64, 0x81, 2,
        0, 0x81, 71, 0x81, 2,
        0, 0xff, 0,
        0, 0xff, 0x2f, 0,
    };
    auto actual = render(3, native);
    auto expected = render(2, mobile);
    require(actual == expected, "MA-7 timing, channel state or event decoding differs");
    require(std::any_of(actual.begin(), actual.end(), [](float v) {
        return std::isfinite(v) && std::fabs(v) > 0.001f;
    }), "render is silent");
}

static void percussion() {
    auto actual = render(3, {0, 0x30, 0, 0x7d, 0, 0x10, 40, 100, 125});
    auto expected = render(2, {0, 0xb0, 0, 0x7d, 0, 0x90, 40, 100, 125}, true);
    auto melodic = render(2, {0, 0x90, 40, 100, 125});
    require(actual == expected && actual != melodic,
            "MA-7 percussion bank did not use percussion fallback");
}

static void voices() {
    for (int alg : {0, 1, 2, 7}) {
        int ops = alg < 2 ? 2 : 4;
        std::vector<uint8_t> native = {0x43, 0x79, 8, 0x7f, 0x21,
                                     0x7c, 2, 57, 0, 0, 0, 0, 0x79, uint8_t(alg)};
        std::vector<uint8_t> mobile = {0x43, 0x79, 7, 0x7f, 1,
                                     0x7c, 2, 57, 0, 0, 0, 0x79, uint8_t(alg)};
        for (int i = 0; i < ops; ++i) {
            std::vector<uint8_t> op = {2, 0x73, 0xf4, uint8_t(i * 12), 1, 0x10, 0};
            native.insert(native.end(), op.begin(), op.end());
            native.insert(native.end(), {0x0e, 0xca, 0x10});
            mobile.insert(mobile.end(), op.begin(), op.end());
        }
        auto a = parseVoiceExclusive(native.data(), native.size());
        auto b = parseVoiceExclusive(mobile.data(), mobile.size());
        require(a.valid && b.valid && a.key == b.key, "MA-7 FM voice rejected");
        FmVoice va, vb;
        va.setSampleRate(8000); vb.setSampleRate(8000);
        va.noteOn(a.patch, 440, 0.5); vb.noteOn(b.patch, 440, 0.5);
        for (int i = 0; i < 800; ++i)
            require(va.tick() == vb.tick(), "MA-7 operator layout differs");
        for (size_t n = 0; n < native.size(); ++n)
            require(!parseVoiceExclusive(native.data(), n).valid, "truncated voice accepted");
        native[10] = 3;
        require(!parseVoiceExclusive(native.data(), native.size()).valid,
                "unsupported PCM voice interpreted as FM");
    }
}

int main() {
    try {
        events();
        voices();
        percussion();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 1;
    }
}
