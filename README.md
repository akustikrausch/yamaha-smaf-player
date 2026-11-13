# Yamaha SMAF Player (C++)

**by Akustikrausch (Andreas Wendorf)**

<p>
  <a href="https://github.com/akustikrausch/yamaha-smaf-player/actions"><img src="https://img.shields.io/badge/build-passing-3da639" alt="build"></a>
  <img src="https://img.shields.io/badge/license-Apache%202.0-3da639" alt="license Apache 2.0">
  <img src="https://img.shields.io/badge/C%2B%2B-20-00599c" alt="C++20">
  <img src="https://img.shields.io/badge/deps-none-blue" alt="no dependencies">
  <img src="https://img.shields.io/badge/chip-Yamaha%20MA--3%20%2F%20MA--5-e10098" alt="Yamaha MA-3/MA-5">
  <img src="https://img.shields.io/badge/synthesis-FM-f0a23b" alt="FM synthesis">
  <img src="https://img.shields.io/badge/ROM-free-4ade80" alt="ROM-free">
  <a href="https://github.com/akustikrausch/FXChainPlayer-Releases"><img src="https://img.shields.io/badge/proven%20in-FXChainPlayer-6c7bff" alt="proven in FXChainPlayer"></a>
</p>

a small, dependency-free **player for Yamaha SMAF ringtones** (`.mmf`) in c++.
you hand it one of those files that made your samsung, sharp, or panasonic go
*deedle-deedle-dee* in 2004, and it hands you clean stereo audio. no phone, no
firmware, no sample rom. just the FM chip, rebuilt in software.

this is the sound of an entire decade that almost nobody bothered to preserve in
open code. there are millions of these files still floating around, and **nobody
ever finished an open player for them**: the attempts stalled in 2004 or route
around the synth entirely. so here is a c++ one, written from the format up, with
its own FM synth core.

> 🎧 **this code ships in a real product: [FXChainPlayer](https://github.com/akustikrausch/FXChainPlayer-Releases).**
> a native windows audio player that plays your old ringtones next to your FLACs,
> SIDs, and Amiga modules. that is where this engine runs every day, against a
> corpus of hundreds of real handset tones. grab the player to just listen, or
> read on for how the chip works.

## why this exists

the **Yamaha MA series** (YMU762 = MA-3, YMU765 = MA-5) was the sound chip inside
a huge slice of early-2000s phones. it is a proper FM synthesizer, a close cousin
of the OPL/OPN chips from the adlib and sega-genesis era, shrunk down for a
handset. the files it played, **SMAF** (`.mmf`), carry an FM score plus optional
adpcm samples. the polyphonic ringtone was born here.

then smartphones arrived, mp3 ringtones took over, and the whole format quietly
died. yamaha took its sdk offline. and unlike the amiga or c64 scenes, almost
nobody wrote an emulator. what exists:

- **MAME** has no working MA-3 core.
- **yamaha's own tools** were closed-source dlls, now gone.
- **mmfplay** (c, 2004) got the furthest and then stopped: it maps MA-3 scores
  onto an OPL3 emulator "as a proof of concept", chokes on SMF-converted files,
  carries no license, and by its own readme "sounds really bad". last touched two
  decades ago.
- **`umjammer/vavi-sound`** (java) parses and plays smaf, and it is a brilliant
  behavioural reference, but it does not do FM synthesis at all: it re-wraps the
  tones as general-midi and lets java's synth approximate them. and it is java.
- **ffmpeg** parses only the adpcm side and rejects the FM sequencer outright.

so a native, permissively-licensed, **rom-free** player of the actual FM chip did
not exist. this is that. every byte of the parser and every operator of the synth
was written from the public SMAF format description plus a lot of staring at real
files in a hex editor, never by copying another player.

## how it works

the engine is five small pieces, each one testable on its own:

```
src/
  smaf_file      the MMMD container: big-endian TLV chunks (CNTI/OPDA/MTR/ATR),
                 two score generations (HandyPhone for MA-1/2, Mobile for MA-3/5)
  ma_player      the two event decoders (+ okumura huffman for the packed
                 variant), a scheduler that runs in milliseconds like the chip
                 did, and the render loop that drives a pool of FM voices
  ma_fm_core     the FM DSP: 2-op and 4-op operators, exponential ADSR envelopes,
                 the OPL-style connection algorithms, and its own general-midi
                 approximation patch bank
  smaf_voice     decodes the yamaha voice sysex blobs (VM35 for MA-3/5, VMA for
                 MA-1/2) into FM patches, including the MA-3 bit-packed form
  yamaha_adpcm   the YM2608-family 4-bit adpcm codec, for the sampled voices
```

the timing model is the nice surprise: SMAF has no tempo events, time is purely
metric (event time = sum of durations times a timebase, in milliseconds), so one
tick maps straight to one millisecond and then to samples. the FM core runs at
your output rate directly, so there is no resampling on the synth path.

## the rom question (read this)

the MA chips also carried a small **general-midi sample rom** so a ringtone could
say "play program 5" and get a preset piano. that rom is yamaha's, it was never
publicly dumped, and this project **does not ship it, does not want it, and will
never bundle it**. that is a hard line.

instead, `ma_fm_core` carries **its own hand-built FM patch bank** that stands in
for the rom: a file that asks for "program 5" gets a plausible FM voice of the
right family. it will not be bit-identical to a 2003 handset, and that is the
honest trade. what you get is the real FM score, the real note timing, the real
adsr, played by a synth of its own. recognised, and it plays.

files that carry their *own* custom voices (a lot of them do) sound exactly as
authored, because those voices live in the file, not the rom.

## quick start

```sh
cmake -B build && cmake --build build
./build/smaf-render your_ringtone.mmf out.wav 30
```

or wire the library into your own code:

```cpp
#include "smaf_file.h"
#include "ma_player.h"
using namespace fxchain::smaf;

SmafFile file;
file.parse(bytes.data(), bytes.size());       // MMMD container

MaPlayer player;
player.init(file, 44100);                      // build voices + decode the score

float buf[1024 * 2];                           // interleaved stereo
int frames = player.render(buf, 1024);         // pump the synth
```

## status

- **works today:** the FM path (all 8 MA/YMF825 connection algorithms, the linear
  MULTI, per-operator feedback, a tamed modulation index and a gentle analog-lite
  output filter so it sounds warm like the chip, not shrill), HandyPhone (MA-1/2)
  and Mobile (MA-3/5) scores, the huffman-packed variant, custom voices, multi-
  track, pitch bend, volume, pan. **plus embedded PCM voices**: the yamaha-adpcm
  wave bank is decoded and sampled drum / instrument voices play alongside the FM.
- **ma-7 (format 0x03):** native score events and basic 2-op/4-op fm definitions
  play. advanced filtered or sampled voices use approximate instruments;
  embedded ma-7 sample banks and effects are not decoded.
- **next:** the bulk voice-wave block (`43 79 06 7F 03`) that a few pure drum-kit
  files carry their samples in; and the SD-1-specific higher waveshapes. see
  [ROADMAP.md](ROADMAP.md).
- **out of scope:** the rom timbres (see above), the graphics chunks, MA-7
  streaming, and the MFi (`.mld`) sibling format (a cousin; see
  [ROADMAP.md](ROADMAP.md) m3).

## security

this parser is meant to eat untrusted files, so it is written defensively: every
chunk length is clamped to the real buffer, the huffman inflate has node + depth
guards and an output ceiling, and the event decoders carry iteration + total-event
caps. it degrades to "not a valid SMAF file" rather than crashing. details and
reporting in [SECURITY.md](SECURITY.md).

## what's in the box

`src/` the engine (four small translation units plus a header-only codec, no
deps). `example/smaf-render.cpp` the wav CLI in under 100 lines. `CMakeLists.txt`
builds a static `smaf` lib plus the example. that is the whole thing.

## license

Apache License 2.0 (see [LICENSE](LICENSE)). the parser and synth are original
work; no third-party code is vendored, and no yamaha rom or sdk material is
included. the format knowledge came from the public SMAF spec and observation.

## disclaimer

not affiliated with, authorized by, or endorsed by Yamaha. "SMAF" and the MA
chip names are Yamaha's. this is interoperability work: an independent player of
an open file format, built without any yamaha code or the protected sample rom.

---

*greetz to everyone who still remembers what their ringtone sounded like.*
