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
uint8_t prospector_last_brightness = 100;
// Actual backlight level currently applied to the LED (vs. last_brightness,
// the level the display should hold while awake). A wake that interrupts a
// fade-off resumes from this level so the fade-in mirrors the fade-out.
static uint8_t prospector_led_level = 100;
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
    prospector_led_level = value;
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

// Screen lifecycle, driven by prospector_idle_thread (a dedicated worker) so
// neither a fade-out nor a fade-in ever blocks ZMK's event thread: a fade is
// ~750ms of k_msleep and must not stall key processing.
//
// prospector_screen_awake is true whenever the display is showing content,
// including mid-fade; it drops to false only once a fade-out reaches 0.
// prospector_fading_off marks the worker's downward ramp -- during it the
// screen still counts as awake so activity keeps resetting the idle timer,
// and a wake aborts the ramp and fades back in. prospector_wake_pending is
// set by the activity listener when the screen is off or fading off; the
// worker consumes it to run the symmetric fade-in from the current level.
static volatile bool prospector_fading_off;
static volatile bool prospector_wake_pending;

// Parks the worker while the screen is off. It is given (along with
// wake_pending) on activity, so a wake can never be lost to a race with the
// worker parking -- a token given before the worker blocks is simply consumed
// the moment it does.
K_SEM_DEFINE(prospector_wake_sem, 0, 1);

// Fade step interval. ~15ms per step gives roughly 750ms total fade from a
// brightness of 50 -- long enough to read as a deliberate fade rather than a
// flicker, short enough that a wake lands at full brightness within a beat.
#define PROSPECTOR_FADE_STEP_MS 15

// Writes a backlight level and records it, so a wake that interrupts a
// fade-off resumes from the actual level rather than snapping to 0 then up.
static void prospector_write_led(uint8_t level) {
    prospector_led_level = level;
    if (led_set_brightness(pwm_leds_dev, DISP_BL, level)) {
        LOG_ERR("Failed to set brightness");
    }
}

// Downward ramp: last brightness -> 0, worker thread. Aborts (leaving the
// LED where it is) if a wake is queued mid-ramp; the caller then fades back in.
static void prospector_fade_out(void) {
    int b = prospector_last_brightness;
    while (b > 0) {
        if (prospector_wake_pending) {
            return; // a fade-in is queued; resume from the current level
        }
        prospector_write_led(b);
        b--;
        k_msleep(PROSPECTOR_FADE_STEP_MS);
    }
    prospector_write_led(0);
    prospector_screen_awake = false;
    LOG_INF("Idle timeout reached, screen faded off");
}

// Upward ramp: current level -> last brightness, worker thread. Mirror of the
// fade-out so waking the display looks like the reverse of the idle dim. Runs
// on the worker, never ZMK's event thread.
static void prospector_fade_in(void) {
    prospector_screen_awake = true;
    int b = prospector_led_level;
    int target = prospector_last_brightness;
    while (b < target) {
        b++;
        prospector_write_led(b);
        k_msleep(PROSPECTOR_FADE_STEP_MS);
    }
    if (prospector_led_level != target) {
        prospector_write_led(target); // already at target: ensure it is applied
    }
    LOG_INF("Screen woken, faded back in");
}

static void prospector_idle_thread(void) {
    prospector_last_activity = k_uptime_get();

    while (1) {
        // Service a queued wake: the screen is off (or fading off) and
        // activity arrived -- run the fade-in on this worker thread.
        if (prospector_wake_pending) {
            prospector_wake_pending = false;
            prospector_fading_off = false;
            prospector_fade_in();
            prospector_last_activity = k_uptime_get();
            continue;
        }

        if (!prospector_screen_awake) {
            // Fully faded off: park until activity signals a fade-in.
            k_sem_take(&prospector_wake_sem, K_FOREVER);
            continue;
        }

        // Screen on: count down the idle timeout.
        int64_t elapsed = k_uptime_get() - prospector_last_activity;
        int64_t remaining = PROSPECTOR_IDLE_TIMEOUT_MS - elapsed;
        if (remaining > 0) {
            k_sleep(K_MSEC(remaining));
            continue;
        }

        // Timeout elapsed: fade off. On an early abort (wake mid-ramp)
        // screen_awake is still true and the top of the loop runs the queued
        // fade-in; otherwise it parks at 0 until the next activity.
        prospector_fading_off = true;
        prospector_fade_out();
        prospector_fading_off = false;
    }
}

K_THREAD_DEFINE(prospector_idle_tid, 512, prospector_idle_thread, NULL, NULL, NULL, 7, 0, 0);

static int prospector_activity_listener(const zmk_event_t *eh) {
    prospector_last_activity = k_uptime_get();
    if (!prospector_screen_awake || prospector_fading_off) {
        // Screen off or fading off: request a fade-in and unpark the worker.
        // The worker owns the ramp; never block ZMK's event thread here.
        prospector_wake_pending = true;
        k_sem_give(&prospector_wake_sem);
    }
    return 0;
}

ZMK_LISTENER(prospector_idle, prospector_activity_listener);
ZMK_SUBSCRIPTION(prospector_idle, zmk_keycode_state_changed);
ZMK_SUBSCRIPTION(prospector_idle, zmk_layer_state_changed);

#endif // CONFIG_PROSPECTOR_IDLE_TIMEOUT_S > 0