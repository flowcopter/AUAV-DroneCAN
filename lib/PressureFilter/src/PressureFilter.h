#ifndef PRESSURE_FILTER_H
#define PRESSURE_FILTER_H

// Butterworth low-pass for the static pressure channel.
//
// Runs at the full sensor rate, before decimation, so the blade-passing tone
// is removed before the signal leaves the node (ArduPilot only averages what
// it receives and cannot undo aliasing).
//
// Plain C++ with no Arduino dependency so it can be unit tested on the host
// (pio test -e native). Double precision: absolute pressure is ~1e5 Pa and
// the Cortex-M4 has a single-precision FPU only, but at ~200 Hz the software
// doubles cost nothing worth measuring.

#include <math.h>

// One second-order section, transposed direct form II.
class Biquad
{
public:
    // Low-pass section by bilinear transform with frequency prewarping, so the
    // digital filter has exactly the analogue response at cutoff_hz.
    void set_lowpass(double sample_hz, double cutoff_hz, double q)
    {
        const double k = tan(M_PI * cutoff_hz / sample_hz);
        const double norm = 1.0 / (1.0 + k / q + k * k);
        b0_ = k * k * norm;
        b1_ = 2.0 * b0_;
        b2_ = b0_;
        a1_ = 2.0 * (k * k - 1.0) * norm;
        a2_ = (1.0 - k / q + k * k) * norm;
    }

    // Set the state as if the input had been constant at x forever, so the
    // output starts at x instead of ringing up from zero.
    void reset(double x)
    {
        // DC gain is 1, so the steady-state output is x.
        z2_ = (b2_ - a2_) * x;
        z1_ = (b1_ - a1_) * x + z2_;
    }

    double apply(double x)
    {
        const double y = b0_ * x + z1_;
        z1_ = b1_ * x - a1_ * y + z2_;
        z2_ = b2_ * x - a2_ * y;
        return y;
    }

private:
    double b0_ = 1.0, b1_ = 0.0, b2_ = 0.0, a1_ = 0.0, a2_ = 0.0;
    double z1_ = 0.0, z2_ = 0.0;
};

// Butterworth low-pass of order 2 or 4 (one or two cascaded sections).
// A cutoff of 0, or one too close to Nyquist to be meaningful, disables the
// filter and the input passes through unchanged.
class PressureLowPass
{
public:
    static constexpr int MAX_SECTIONS = 2;

    // Returns false (and leaves the filter disabled) if the settings are not
    // usable; the caller should report that.
    bool configure(double sample_hz, double cutoff_hz, int order)
    {
        enabled_ = false;
        sections_ = 0;
        group_delay_s_ = 0.0;
        if (!(sample_hz > 0.0) || cutoff_hz <= 0.0) {
            return cutoff_hz <= 0.0 && sample_hz > 0.0; // 0 = deliberately off
        }
        if (cutoff_hz >= 0.4 * sample_hz) {
            return false;
        }

        // Butterworth section Q values.
        static const double q2[] = {0.70710678118654752};
        static const double q4[] = {0.54119610014619698, 1.3065629648763766};
        const double *q;
        if (order == 2) {
            q = q2;
            sections_ = 1;
        } else if (order == 4) {
            q = q4;
            sections_ = 2;
        } else {
            return false;
        }

        // Low-frequency group delay of the analogue prototype: each section
        // w^2 / (s^2 + (w/Q) s + w^2) contributes 1 / (Q w).
        const double w = 2.0 * M_PI * cutoff_hz;
        for (int i = 0; i < sections_; i++) {
            section_[i].set_lowpass(sample_hz, cutoff_hz, q[i]);
            group_delay_s_ += 1.0 / (q[i] * w);
        }
        enabled_ = true;
        primed_ = false;
        return true;
    }

    // Forget the history; the next sample re-primes the filter at its value.
    void reset() { primed_ = false; }

    double apply(double x)
    {
        if (!enabled_) {
            return x;
        }
        if (!primed_) {
            for (int i = 0; i < sections_; i++) {
                section_[i].reset(x);
            }
            primed_ = true;
        }
        for (int i = 0; i < sections_; i++) {
            x = section_[i].apply(x);
        }
        return x;
    }

    bool enabled() const { return enabled_; }

    // Delay of slowly varying signals through the filter, in seconds. Add the
    // other delays (conversion, publishing, ArduPilot's 10 Hz average) to get
    // the value for EK3_HGT_DELAY.
    double group_delay_s() const { return group_delay_s_; }

private:
    Biquad section_[MAX_SECTIONS];
    int sections_ = 0;
    bool enabled_ = false;
    bool primed_ = false;
    double group_delay_s_ = 0.0;
};

#endif // PRESSURE_FILTER_H
