#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/printk.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(als, 4);

#if CONFIG_PROSPECTOR_IDLE_TIMEOUT_S > 0
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#endif

static const struct device *pwm_leds_dev = DEVICE_DT_GET_ONE(pwm_leds);
#define DISP_BL DT_NODE_CHILD_IDX(DT_NODELABEL(disp_bl))

// --- Idle-timeout shared state --------------------------------------------
// Tracks the brightness the display should show while awake, independent of
// whether the ambient sensor or fixed brightness is active, so the idle
// timeout can dim to 0 and restore the correct value on the next keypress.
static uint8_t prospector_last_brightness = 100;
#if CONFIG_PROSPECTOR_IDLE_TIMEOUT_S > 0
static bool prospector_screen_awake = true;
#endif

// Applies `value` to the backlight. While idle-dimmed, the value is recorded
// but not written to the LED, so ambient-light updates that arrive while
// asleep don't wake the screen; they just update what gets restored on wake.
static void prospector_apply_brightness(uint8_t value) {
    prospector_last_brightness = value;
#if CONFIG_PROSPECTOR_IDLE_TIMEOUT_S > 0
    if (!prospector_screen_awake) {
        return;
    }
#endif
    if (led_set_brightness(pwm_leds_dev, DISP_BL, value)) {
        LOG_ERR("Failed to set brightness");
    }
}

#ifdef CONFIG_PROSPECTOR_USE_AMBIENT_LIGHT_SENSOR

static uint8_t current_brightness = 100;

#define SENSOR_MIN      0       // Minimum sensor reading
#define SENSOR_MAX      100   // Maximum sensor reading
#define PWM_MIN         1       // Minimum PWM duty cycle (%) - keep display visible
#define PWM_MAX         100     // Maximum PWM duty cycle (%)

#define FADE_STEP                        1
#define FADE_SLEEP_BRIGHTEN_MS           3
#define FADE_SLEEP_DARKEN_MS             10
#define FADE_THRESHOLD                   10

#define NORMAL_SAMPLE_SLEEP_MS           100

#define BURST_SAMPLE_SLEEP_MS            30
#define BURST_SAMPLE_TIMEOUT             10
#define BURST_SAMPLE_CONSECUTIVE         3

uint8_t map_light_to_pwm(int32_t sensor_reading) {
    // Handle invalid/error readings
    if (sensor_reading < SENSOR_MIN) {
        return PWM_MIN;  // Default to minimum brightness on error
    }

    // Clamp to maximum
    if (sensor_reading > SENSOR_MAX) {
        sensor_reading = SENSOR_MAX;
    }

    // Linear mapping
    uint8_t pwm_value = (uint8_t)(
        PWM_MIN + ((PWM_MAX - PWM_MIN) *
        (sensor_reading - SENSOR_MIN)) / (SENSOR_MAX - SENSOR_MIN)
    );

    return pwm_value;
}

uint8_t bl_fade(uint8_t source, uint8_t target) {
    bool increasing = target > source;

    while ((increasing && current_brightness < target) ||
           (!increasing && current_brightness > target)) {

        prospector_apply_brightness(current_brightness);

        current_brightness += increasing ? FADE_STEP : -FADE_STEP;

        // Ensure we don't overshoot bounds
        if (current_brightness > 100) {
            current_brightness = 100;
        } else if (current_brightness < 0) {
            current_brightness = 0;
        }

        k_msleep(increasing ? FADE_SLEEP_BRIGHTEN_MS : FADE_SLEEP_DARKEN_MS);
    }

    return 0;
}

extern void als_thread(void *d0, void *d1, void *d2) {
    ARG_UNUSED(d0);
    ARG_UNUSED(d1);
    ARG_UNUSED(d2);

    const struct device *dev;
    struct sensor_value intensity;
    uint8_t mapped_brightness;

    dev = DEVICE_DT_GET_ONE(avago_apds9960);
    if (!device_is_ready(dev)) {
        printk("sensor: device not ready.\n");
    }

    // led_set_brightness(pwm_leds_dev, DISP_BL, 100);

    while (1) {

        k_msleep(NORMAL_SAMPLE_SLEEP_MS);


        if (sensor_sample_fetch(dev)) {
            LOG_ERR("sensor_sample fetch failed\n");
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &intensity)) {
            LOG_ERR("Cannot read ALS data.\n");
        }

        // LOG_INF("ambient light intensity %d", intensity.val1);

        mapped_brightness = map_light_to_pwm(intensity.val1);
        // LOG_INF("NORMAL: mapped PWM duty cycle %d\n", mapped_brightness);

        if (abs(mapped_brightness - current_brightness) > FADE_THRESHOLD) {
            uint8_t integrator = 0;

            for (int i = 0; i < BURST_SAMPLE_TIMEOUT; i++) {
                k_msleep(BURST_SAMPLE_SLEEP_MS);

                if (sensor_sample_fetch(dev)) {
                    LOG_ERR("sensor_sample fetch failed\n");
                }
                if (sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &intensity)) {
                    LOG_ERR("Cannot read ALS data.\n");
                }

                mapped_brightness = map_light_to_pwm(intensity.val1);
                // LOG_INF("BURST: mapped PWM duty cycle %d\n", mapped_brightness);

                if (abs(mapped_brightness - current_brightness) > FADE_THRESHOLD) {
                    integrator++;
                    // printk("integrator at: %d", integrator);
                    if (integrator >= BURST_SAMPLE_CONSECUTIVE) {
                        bl_fade(current_brightness, mapped_brightness);
                        current_brightness = mapped_brightness;
                        // LOG_INF("SETTING NEW BRIGHTNESS: %d", mapped_brightness);
                        break;
                    }
                }
            }
        }
        // led_set_brightness(pwm_leds_dev, DISP_BL, map_light_to_pwm(intensity.val1));
    }
}

K_THREAD_DEFINE(als_tid, 1024, als_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0,
                0);

#else

static int init_fixed_brightness(void) {
    prospector_apply_brightness(CONFIG_PROSPECTOR_FIXED_BRIGHTNESS);

    return 0;
}

SYS_INIT(init_fixed_brightness, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif

#if CONFIG_PROSPECTOR_IDLE_TIMEOUT_S > 0

// --- Idle timeout ----------------------------------------------------------
// Turns the display off after CONFIG_PROSPECTOR_IDLE_TIMEOUT_S seconds
// without a keypress or layer change, and restores the last brightness
// (ambient-sensor or fixed) on the next keypress/layer change.

#define PROSPECTOR_IDLE_TIMEOUT_MS (CONFIG_PROSPECTOR_IDLE_TIMEOUT_S * 1000)

static int64_t prospector_last_activity;

// Idle fade-off state. prospector_fading_off is true while the screen is
// smoothly dimming to 0; during this time the screen is logically still
// "awake" (so the idle timer keeps being reset by activity), but a wake
// event also aborts the fade and restores full brightness.
static volatile bool prospector_fading_off;
static volatile bool prospector_fade_abort;

// Fade-out step interval. ~15ms per step gives roughly 750ms total
// fade from a brightness of 50 -- long enough to read as a deliberate
// fade rather than a flicker, short enough that a wake during the fade
// returns to full brightness within a perceptible beat.
#define PROSPECTOR_FADE_STEP_MS 15

static void prospector_screen_set_awake(bool awake) {
    if (awake) {
        // Wake path: cancel any in-progress fade and restore brightness.
        if (prospector_fading_off) {
            prospector_fade_abort = true;
            prospector_fading_off = false;
            if (led_set_brightness(pwm_leds_dev, DISP_BL, prospector_last_brightness)) {
                LOG_ERR("Failed to set brightness");
            }
            LOG_INF("Screen woken (fade aborted)");
        } else if (!prospector_screen_awake) {
            prospector_screen_awake = true;
            if (led_set_brightness(pwm_leds_dev, DISP_BL, prospector_last_brightness)) {
                LOG_ERR("Failed to set brightness");
            }
            LOG_INF("Screen woken from idle timeout");
        }
    } else {
        // Sleep path: smoothly fade from last brightness to 0.
        if (!prospector_screen_awake) {
            return; // already asleep
        }
        prospector_fading_off = true;
        // prospector_screen_awake stays true so the activity listener
        // keeps resetting the idle timer during the fade.

        int b = prospector_last_brightness;
        while (b > 0) {
            if (prospector_fade_abort) {
                prospector_fade_abort = false;
                LOG_INF("Fade-off aborted by activity");
                return;
            }
            if (led_set_brightness(pwm_leds_dev, DISP_BL, b)) {
                LOG_ERR("Failed to set brightness");
            }
            b--;
            k_msleep(PROSPECTOR_FADE_STEP_MS);
        }
        if (led_set_brightness(pwm_leds_dev, DISP_BL, 0)) {
            LOG_ERR("Failed to set brightness");
        }
        prospector_fading_off = false;
        prospector_screen_awake = false;
        LOG_INF("Idle timeout reached, screen faded off");
    }
}

static void prospector_idle_thread(void) {
    prospector_last_activity = k_uptime_get();

    while (1) {
        if (prospector_screen_awake) {
            int64_t elapsed = k_uptime_get() - prospector_last_activity;
            int64_t remaining = PROSPECTOR_IDLE_TIMEOUT_MS - elapsed;

            if (remaining <= 0) {
                prospector_screen_set_awake(false);
                k_sleep(K_FOREVER); // woken by prospector_activity_listener via k_wakeup()
            } else {
                k_sleep(K_MSEC(remaining));
            }
        } else {
            k_sleep(K_FOREVER);
        }
    }
}

K_THREAD_DEFINE(prospector_idle_tid, 512, prospector_idle_thread, NULL, NULL, NULL, 7, 0, 0);

static int prospector_activity_listener(const zmk_event_t *eh) {
    prospector_last_activity = k_uptime_get();
    if (!prospector_screen_awake || prospector_fading_off) {
        prospector_screen_set_awake(true);
        k_wakeup(prospector_idle_tid);
    }
    return 0;
}

ZMK_LISTENER(prospector_idle, prospector_activity_listener);
ZMK_SUBSCRIPTION(prospector_idle, zmk_keycode_state_changed);
ZMK_SUBSCRIPTION(prospector_idle, zmk_layer_state_changed);

#endif // CONFIG_PROSPECTOR_IDLE_TIMEOUT_S > 0