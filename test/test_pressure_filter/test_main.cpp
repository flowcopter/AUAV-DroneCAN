// Host tests for the static pressure low-pass: pio test -e native

#include <math.h>
#include <unity.h>

#include <PressureFilter.h>

static const double FS = 180.0;        // nominal node sample rate
static const double BLADE_HZ = 52.6;   // hover blade-passing frequency from the flight log
static const double P0 = 101325.0;

void setUp() {}
void tearDown() {}

// Steady-state gain for a sine at freq_hz riding on P0.
static double sine_gain(PressureLowPass &lp, double freq_hz, double seconds = 20.0)
{
    const int n = (int)(seconds * FS);
    const int settle = n / 2;
    double peak = 0.0;
    lp.reset();
    for (int i = 0; i < n; i++) {
        const double x = P0 + 100.0 * sin(2.0 * M_PI * freq_hz * i / FS);
        const double y = lp.apply(x) - P0;
        if (i >= settle && fabs(y) > peak) {
            peak = fabs(y);
        }
    }
    return peak / 100.0;
}

void test_starts_at_first_sample_without_transient()
{
    PressureLowPass lp;
    TEST_ASSERT_TRUE(lp.configure(FS, 5.0, 2));
    for (int i = 0; i < 1000; i++) {
        TEST_ASSERT_DOUBLE_WITHIN(1e-6, P0, lp.apply(P0));
    }
}

void test_cutoff_is_minus_3db()
{
    PressureLowPass lp;
    TEST_ASSERT_TRUE(lp.configure(FS, 5.0, 2));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, M_SQRT1_2, sine_gain(lp, 5.0));
}

void test_passband_is_flat()
{
    PressureLowPass lp;
    TEST_ASSERT_TRUE(lp.configure(FS, 5.0, 2));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 1.0, sine_gain(lp, 0.5, 40.0));
}

void test_blade_tone_attenuation()
{
    // 2nd order: analogue (5/52.6)^2 ~ -41 dB; bilinear adds a little more.
    PressureLowPass lp2;
    TEST_ASSERT_TRUE(lp2.configure(FS, 5.0, 2));
    TEST_ASSERT_LESS_THAN_DOUBLE(0.01, sine_gain(lp2, BLADE_HZ));

    // 4th order: better than -80 dB.
    PressureLowPass lp4;
    TEST_ASSERT_TRUE(lp4.configure(FS, 5.0, 4));
    TEST_ASSERT_LESS_THAN_DOUBLE(1e-4, sine_gain(lp4, BLADE_HZ));
}

// A ramp comes out delayed by the low-frequency group delay.
static double measured_ramp_delay(PressureLowPass &lp)
{
    const double slope = 12.0; // Pa/s, about 1 m/s climb
    double y = 0.0;
    const int n = (int)(10.0 * FS);
    lp.reset();
    for (int i = 0; i < n; i++) {
        y = lp.apply(P0 + slope * i / FS);
    }
    const double x_last = P0 + slope * (n - 1) / FS;
    return (x_last - y) / slope;
}

void test_group_delay_matches_reported_value()
{
    PressureLowPass lp2;
    TEST_ASSERT_TRUE(lp2.configure(FS, 5.0, 2));
    TEST_ASSERT_DOUBLE_WITHIN(0.0005, 0.0450, lp2.group_delay_s());
    TEST_ASSERT_DOUBLE_WITHIN(0.002, lp2.group_delay_s(), measured_ramp_delay(lp2));

    PressureLowPass lp4;
    TEST_ASSERT_TRUE(lp4.configure(FS, 5.0, 4));
    TEST_ASSERT_DOUBLE_WITHIN(0.0005, 0.0832, lp4.group_delay_s());
    TEST_ASSERT_DOUBLE_WITHIN(0.002, lp4.group_delay_s(), measured_ramp_delay(lp4));
}

void test_zero_cutoff_passes_through()
{
    PressureLowPass lp;
    TEST_ASSERT_TRUE(lp.configure(FS, 0.0, 2));
    TEST_ASSERT_FALSE(lp.enabled());
    TEST_ASSERT_EQUAL_DOUBLE(P0 + 3.0, lp.apply(P0 + 3.0));
    TEST_ASSERT_EQUAL_DOUBLE(0.0, lp.group_delay_s());
}

void test_rejects_bad_settings()
{
    PressureLowPass lp;
    TEST_ASSERT_FALSE(lp.configure(FS, 80.0, 2)); // too close to Nyquist
    TEST_ASSERT_FALSE(lp.enabled());
    TEST_ASSERT_FALSE(lp.configure(FS, 5.0, 3));  // unsupported order
    TEST_ASSERT_FALSE(lp.enabled());
    TEST_ASSERT_FALSE(lp.configure(0.0, 5.0, 2));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_starts_at_first_sample_without_transient);
    RUN_TEST(test_cutoff_is_minus_3db);
    RUN_TEST(test_passband_is_flat);
    RUN_TEST(test_blade_tone_attenuation);
    RUN_TEST(test_group_delay_matches_reported_value);
    RUN_TEST(test_zero_cutoff_passes_through);
    RUN_TEST(test_rejects_bad_settings);
    return UNITY_END();
}
