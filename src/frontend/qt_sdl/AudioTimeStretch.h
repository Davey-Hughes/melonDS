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

#ifndef AUDIOTIMESTRETCH_H
#define AUDIOTIMESTRETCH_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

// WSOLA time-stretcher: changes playback rate while preserving pitch, by
// overlap-adding windowed frames picked for waveform similarity so consecutive
// frames splice on matching phase.
//
// kFrameSize and kSearchRadius are tuned, not guessed. The artifact they trade
// against is amplitude pumping at the synthesis hop rate on noisy material,
// measured at ratio 3 with the search on versus forced off:
//
//   frame 1024   0.139 on / 0.180 off    search buys 1.3x
//   frame  512   0.071 on / 0.175 off    search buys 2.5x
//   frame  256   0.036 on / 0.169 off    search buys 4.7x
//
// Shorter frames let the search find better matches; off, the figure is flat.
// Below 256 the window holds barely one cycle of a bass note, so this stops.
class AudioTimeStretch
{
public:
    static constexpr int kFrameSize = 256;                // analysis window, frames
    static constexpr int kSynthesisHop = kFrameSize / 2;  // periodic Hann at 50% sums to unity
    static constexpr int kSearchRadius = 1024;            // frames either side of nominal
    static constexpr int kCoarseStride = 4;
    static constexpr int kFineRadius = 3;
    static constexpr int kInputCapacity = 32768;          // must be a power of two
    static constexpr int kOutputCapacity = 8192;          // must be a power of two

    // Input the ratio control aims to keep buffered. Absolute, not a multiple
    // of the window: it absorbs lumpy arrival from the emu thread.
    static constexpr int kTargetInputFill = 4096;

    AudioTimeStretch()
    {
        for (int i = 0; i < kFrameSize; i++)
            Window[i] = (float)(0.5 * (1.0 - std::cos((2.0 * M_PI * i) / kFrameSize)));
        Reset();
    }

    void Reset()
    {
        WritePos = 0;
        AnalysisPos = 0;
        NaturalPos = 0;
        OutReadPos = 0;
        OutWritePos = 0;
        Primed = false;
        std::memset(AccL, 0, sizeof(AccL));
        std::memset(AccR, 0, sizeof(AccR));

        // Belt and braces: the search is already bounded to written frames, but
        // this makes any future bound slip degrade to silence rather than noise.
        std::memset(InL, 0, sizeof(InL));
        std::memset(InR, 0, sizeof(InR));
        std::memset(InMono, 0, sizeof(InMono));
    }

    int InputFill() const
    {
        int64_t pending = WritePos - AnalysisPos;
        if (pending < 0) return 0;
        if (pending > kInputCapacity) return kInputCapacity;
        return (int)pending;
    }

    int OutputFill() const { return (int)(OutWritePos - OutReadPos); }

    // Append interleaved stereo frames.
    int Write(const int16_t* samples, int numFrames)
    {
        for (int i = 0; i < numFrames; i++)
        {
            int idx = (int)(WritePos & (kInputCapacity - 1));
            int16_t l = samples[(i*2)+0];
            int16_t r = samples[(i*2)+1];
            InL[idx] = l;
            InR[idx] = r;
            InMono[idx] = 0.5f * ((float)l + (float)r);
            WritePos++;
        }

        // Producer outran us and overwrote frames we still wanted: skip forward
        // rather than read whatever landed on top of them.
        int64_t floor = (WritePos - kInputCapacity) + kSearchRadius + kFrameSize;
        if (AnalysisPos < floor)
        {
            AnalysisPos = floor;
            NaturalPos = floor;
            Primed = false;
        }

        return numFrames;
    }

    // Emit up to numFrames of interleaved stereo, synthesising as needed.
    int Read(int16_t* samples, int numFrames, double ratio)
    {
        while ((OutputFill() < numFrames) && CanSynthesise())
            SynthesiseHop(ratio);

        int n = std::min(numFrames, OutputFill());
        for (int i = 0; i < n; i++)
        {
            int idx = (int)(OutReadPos & (kOutputCapacity - 1));
            samples[(i*2)+0] = OutL[idx];
            samples[(i*2)+1] = OutR[idx];
            OutReadPos++;
        }
        return n;
    }

private:
    static int16_t Saturate(float v)
    {
        long s = std::lround(v);
        if (s > 32767) s = 32767;
        if (s < -32768) s = -32768;
        return (int16_t)s;
    }

    bool CanSynthesise() const
    {
        if ((kOutputCapacity - OutputFill()) < kSynthesisHop) return false;

        // Only the frame itself and the natural-continuation reference need to
        // be present; the search clamps to whatever else is available. Demanding
        // the full radius here would emit no output at all on a short FIFO.
        int64_t frameEnd = AnalysisPos + kFrameSize;
        int64_t naturalEnd = NaturalPos + kSynthesisHop;
        return (WritePos >= frameEnd) && (WritePos >= naturalEnd);
    }

    void SynthesiseHop(double ratio)
    {
        int hop = (int)std::lround(kSynthesisHop * ratio);
        if (hop < 1) hop = 1;

        int64_t chosen = Primed ? FindBestOffset() : AnalysisPos;
        Primed = true;

        for (int i = 0; i < kFrameSize; i++)
        {
            int idx = (int)((chosen + i) & (kInputCapacity - 1));
            float w = Window[i];
            AccL[i] += w * (float)InL[idx];
            AccR[i] += w * (float)InR[idx];
        }

        for (int i = 0; i < kSynthesisHop; i++)
        {
            int idx = (int)(OutWritePos & (kOutputCapacity - 1));
            OutL[idx] = Saturate(AccL[i]);
            OutR[idx] = Saturate(AccR[i]);
            OutWritePos++;
        }

        std::memmove(AccL, AccL + kSynthesisHop, kSynthesisHop * sizeof(float));
        std::memmove(AccR, AccR + kSynthesisHop, kSynthesisHop * sizeof(float));
        std::memset(AccL + kSynthesisHop, 0, kSynthesisHop * sizeof(float));
        std::memset(AccR + kSynthesisHop, 0, kSynthesisHop * sizeof(float));

        // The nominal pointer advances by hop alone; the search only picks which
        // frame to window. Advancing from chosen instead would make consumption
        // hop + E[bestK], which the caller's ratio control cannot see.
        NaturalPos = chosen + kSynthesisHop;
        AnalysisPos += hop;
    }

    int64_t FindBestOffset() const
    {
        int64_t oldest = std::max<int64_t>(0, WritePos - kInputCapacity);

        // NaturalPos trails AnalysisPos by up to hop + kSearchRadius - kSynthesisHop,
        // which at a high ratio on a near-full ring can fall off the back. The
        // reference would then be overwritten frames, so search nothing instead.
        if (NaturalPos < oldest) return AnalysisPos;

        double refEnergy = Energy(NaturalPos);

        // Full radius regardless of hop: expansion needs the reach, since the
        // natural continuation sits |kSynthesisHop - hop| ahead of the nominal.
        const int radius = kSearchRadius;

        // Masking a negative position wraps it into frames we never wrote.
        int lowestK = -radius;
        if ((AnalysisPos + lowestK) < oldest)
            lowestK = (int)(oldest - AnalysisPos);

        // Clamp to what has arrived, so a short FIFO narrows the search.
        int highestK = radius;
        int64_t latest = WritePos - kFrameSize;
        if ((AnalysisPos + highestK) > latest)
            highestK = (int)(latest - AnalysisPos);
        if (highestK < lowestK) highestK = lowestK;

        int bestK = std::min(std::max(lowestK, 0), highestK);
        double bestScore = -1.0e30;

        for (int k = lowestK; k <= highestK; k += kCoarseStride)
        {
            double s = Score(AnalysisPos + k, refEnergy);
            if (s > bestScore) { bestScore = s; bestK = k; }
        }

        int lo = std::max(lowestK, bestK - kFineRadius);
        int hi = std::min(highestK, bestK + kFineRadius);
        for (int k = lo; k <= hi; k++)
        {
            double s = Score(AnalysisPos + k, refEnergy);
            if (s > bestScore) { bestScore = s; bestK = k; }
        }

        return AnalysisPos + bestK;
    }

    double Energy(int64_t pos) const
    {
        double e = 0.0;
        for (int i = 0; i < kSynthesisHop; i++)
        {
            double v = InMono[(int)((pos + i) & (kInputCapacity - 1))];
            e += v * v;
        }
        return e;
    }

    // Normalised so the search doesn't just latch onto the loudest candidate.
    double Score(int64_t pos, double refEnergy) const
    {
        double dot = 0.0;
        double energy = 0.0;
        for (int i = 0; i < kSynthesisHop; i++)
        {
            double a = InMono[(int)((pos + i) & (kInputCapacity - 1))];
            double b = InMono[(int)((NaturalPos + i) & (kInputCapacity - 1))];
            dot += a * b;
            energy += a * a;
        }
        return dot / std::sqrt((energy * refEnergy) + 1.0e-9);
    }

    float Window[kFrameSize];

    int16_t InL[kInputCapacity];
    int16_t InR[kInputCapacity];
    float InMono[kInputCapacity];
    int64_t WritePos = 0;
    int64_t AnalysisPos = 0;
    int64_t NaturalPos = 0;
    bool Primed = false;

    float AccL[kFrameSize];
    float AccR[kFrameSize];

    int16_t OutL[kOutputCapacity];
    int16_t OutR[kOutputCapacity];
    int64_t OutReadPos = 0;
    int64_t OutWritePos = 0;
};

#endif // AUDIOTIMESTRETCH_H
