/*
    Copyright 2016-2026 melonDS team

    This file is part of melonDS.

    melonDS is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    melonDS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with melonDS. If not, see http://www.gnu.org/licenses/.
*/

#ifndef AUDIOSPEED_H
#define AUDIOSPEED_H

#include <algorithm>
#include <cmath>

// The DS's true framerate; a skew of 1.0 emits at the hardware's rate.
constexpr double INTERNAL_FRAME_RATE = 59.8260982880808;

// Setting value meaning "do not filter at all". An explicit sentinel: without
// it a 24000 reference would still filter at 12 kHz at 2x.
constexpr int kSpeedUpLowPassOff = 24000;

// Gain of the FIFO-fill correction on the stretch ratio.
constexpr double kStretchTrimGain = 0.25;

// Below 1 the stretcher expands (slow-mo), above it compresses (fast-forward).
constexpr double kMinStretchRatio = 0.25;
constexpr double kMaxStretchRatio = 32.0;

// Floor on what we hand blip_buf, and it is load-bearing rather than cosmetic.
// SPU::BufferAudio flushes every 512*128 SPU cycles, which accumulates
// 65536 * 48000 / (16756991 * skew) ~= 188/skew output samples, and blip_new(512)
// asserts avail <= 512. Below ~0.37 that overflows the allocation.
constexpr double kMinOutputSkew = 0.4;

// Emulated speed relative to normal.
inline double audioSpeedRatio(double curFPS, double targetFPS)
{
    if (targetFPS <= 0.0) return 1.0;
    return curFPS / targetFPS;
}

inline bool audioIsOffSpeed(double curFPS, double targetFPS)
{
    return std::fabs(curFPS - targetFPS) > 0.01;
}

// Deliberately independent of emulation speed: off-speed audio just arrives
// faster or slower, and fitting it to the output is the stretcher's job.
// Deriving this from curFPS instead would resample, shifting pitch.
inline double audioComputeOutputSkew(double targetFPS)
{
    return std::max(targetFPS / INTERNAL_FRAME_RATE, kMinOutputSkew);
}

// How much input each output frame consumes.
//
// Driven by measured arrival rather than the requested speed. The SPU FIFO is
// polled once per callback and holds 2048 frames, so supply is capped near 4x
// however fast the emulator runs; asking for the requested ratio above that
// starves the stretcher, and the trim has nowhere near the authority to close
// a gap that large. In steady state consumption must equal arrival, so that is
// the ratio. The FIFO-fill term only steers the level: a larger ratio drains
// faster, so a full FIFO must raise it.
inline double audioComputeStretchRatio(double arrivalPerCallback, int outputPerCallback,
                                       int inputFill, int targetFill)
{
    if (outputPerCallback <= 0) return 1.0;

    double ratio = arrivalPerCallback / (double)outputPerCallback;

    if (targetFill > 0)
    {
        double err = (inputFill - (double)targetFill) / (double)targetFill;
        err = std::clamp(err, -1.0, 1.0);
        ratio *= 1.0 + (kStretchTrimGain * err);
    }

    return std::clamp(ratio, kMinStretchRatio, kMaxStretchRatio);
}

// Cutoff for the low-pass, in Hz. wideOpen means transparent; the filter only
// engages above normal speed, and gets duller the faster the emulator runs.
inline double audioComputeLowPassCutoff(double curFPS, double targetFPS,
                                        int reference, double wideOpen)
{
    if (wideOpen <= 200.0) return wideOpen;

    double speed = audioSpeedRatio(curFPS, targetFPS);
    if (speed <= 1.0) return wideOpen;
    if (reference >= kSpeedUpLowPassOff) return wideOpen;

    return std::clamp(reference / speed, 200.0, wideOpen);
}

#endif // AUDIOSPEED_H
