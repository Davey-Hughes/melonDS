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

#ifndef AUDIOLOWPASS_H
#define AUDIOLOWPASS_H

#include <algorithm>
#include <cmath>
#include <cstdint>

// fourth-order Butterworth low-pass: two cascaded RBJ biquads, stereo with
// independent state per channel. the cutoff is smoothed rather than jumped,
// since stepping the coefficients would click. wide open leaves the output alone.
class AudioLowPass
{
public:
    static constexpr double kSmoothingTau = 0.05;      // cutoff smoother, seconds
    static constexpr double kBypassThreshold = 0.995;  // fraction of wide-open
    // section Q's for a fourth-order Butterworth cascade
    static constexpr double kSectionQ[2] = {0.54119610014619698, 1.3065629648763766};
    static constexpr double kMinCutoff = 20.0;

    void Init(double sampleRate)
    {
        SampleRate = sampleRate;
        WideOpen = std::max(0.45 * sampleRate, kMinCutoff);
        for (int s = 0; s < 2; s++)
        {
            Stages[s].z1[0] = Stages[s].z1[1] = 0.0;
            Stages[s].z2[0] = Stages[s].z2[1] = 0.0;
        }
        SetCutoffNow(WideOpen);
    }

    double WideOpenCutoff() const { return WideOpen; }
    double Cutoff() const { return CurCutoff; }
    bool Bypassed() const { return CurCutoff >= (WideOpen * kBypassThreshold); }

    // advance the smoothed cutoff by one block, then filter in place
    void Process(int16_t* samples, int numFrames, double targetHz, double blockSeconds)
    {
        Smooth(targetHz, blockSeconds);
        bool bypass = Bypassed();

        for (int i = 0; i < numFrames; i++)
        {
            for (int ch = 0; ch < 2; ch++)
            {
                // runs even when bypassed: the state must stay in step with
                // the signal, or re-engaging would click
                double y = ProcessSample(samples[(i*2)+ch], ch);
                if (!bypass) samples[(i*2)+ch] = Saturate(y);
            }
        }
    }

    // advance the cutoff and the filter state over silence, writing nothing.
    // used while muted, so unmuting neither steps the coefficients nor dumps
    // whatever was still ringing in the biquads into a buffer meant to be quiet.
    void ProcessMuted(int numFrames, double targetHz, double blockSeconds)
    {
        Smooth(targetHz, blockSeconds);
        for (int i = 0; i < numFrames; i++)
            for (int ch = 0; ch < 2; ch++)
                ProcessSample(0.0, ch);
    }

    void Smooth(double targetHz, double blockSeconds)
    {
        targetHz = std::clamp(targetHz, kMinCutoff, WideOpen);
        double a = 1.0 - std::exp(-blockSeconds / kSmoothingTau);
        SetCutoffNow(CurCutoff + ((targetHz - CurCutoff) * a));
    }

    void SetCutoffNow(double cutoffHz)
    {
        CurCutoff = std::clamp(cutoffHz, kMinCutoff, WideOpen);
        for (int s = 0; s < 2; s++)
            Stages[s].Design(CurCutoff, SampleRate, kSectionQ[s]);
    }

    double ProcessSample(double x, int ch)
    {
        double y = x;
        for (int s = 0; s < 2; s++)
            y = Stages[s].Run(y, ch);
        return y;
    }

private:
    static int16_t Saturate(double y)
    {
        long v = std::lround(y);
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
        return (int16_t)v;
    }

    struct Biquad
    {
        double b0 = 1.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0;
        double z1[2] = {0.0, 0.0};
        double z2[2] = {0.0, 0.0};

        void Design(double cutoffHz, double sampleRate, double q)
        {
            double w0 = 2.0 * M_PI * (cutoffHz / sampleRate);
            double cw = std::cos(w0);
            double alpha = std::sin(w0) / (2.0 * q);
            double a0 = 1.0 + alpha;

            b0 = ((1.0 - cw) * 0.5) / a0;
            b1 = (1.0 - cw) / a0;
            b2 = b0;
            a1 = (-2.0 * cw) / a0;
            a2 = (1.0 - alpha) / a0;
        }

        // transposed direct form II
        double Run(double x, int ch)
        {
            double y = (b0 * x) + z1[ch];
            z1[ch] = (b1 * x) - (a1 * y) + z2[ch];
            z2[ch] = (b2 * x) - (a2 * y);
            return y;
        }
    };

    double SampleRate = 48000.0;
    double WideOpen = 21600.0;
    double CurCutoff = 21600.0;
    Biquad Stages[2];
};

#endif // AUDIOLOWPASS_H
