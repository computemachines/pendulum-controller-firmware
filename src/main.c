#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"

#include "common.h"

void on_pwm_wrap_isr();
void on_gpio_isr(uint gpio, uint32_t events);

#define FORWARD_PULSE_PIN 17 // should be PWM_CHANNEL_0B
#define REVERSE_PULSE_PIN 19 // should be PWM_CHANNEL_1B
#define QUADRATURE_ERROR_PIN 16
#define PULSES_PER_REV 2400

/* Set up pwm block as counter on gpio pin .
 * Returns pwm slice number.
 */
uint init_pwm_counter(uint gpio)
{
    assert(pwm_gpio_to_channel(gpio) == PWM_CHAN_B);

    uint slice_num = pwm_gpio_to_slice_num(gpio);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING); // sets B as input to counter
    // pwm_config_set_wrap(&cfg, (1 << 16));

    pwm_init(slice_num, &cfg, false);

    gpio_set_function(gpio, GPIO_FUNC_PWM);

    return slice_num;
}

static int16_t position_wrap_count = 0;
void on_pwm_wrap_isr()
{
    const uint forward_pwm_slice_num = pwm_gpio_to_slice_num(FORWARD_PULSE_PIN);
    const uint reverse_pwm_slice_num = pwm_gpio_to_slice_num(REVERSE_PULSE_PIN);

    if (pwm_get_irq_status_mask() & (1 << forward_pwm_slice_num))
    {
        position_wrap_count++;
        pwm_clear_irq(forward_pwm_slice_num);
    }
    if (pwm_get_irq_status_mask() & (1 << reverse_pwm_slice_num))
    {
        position_wrap_count--;
        pwm_clear_irq(reverse_pwm_slice_num);
    }
}

static uint16_t error_count = 0;
void on_gpio_isr(uint gpio, uint32_t events)
{
    if (gpio == QUADRATURE_ERROR_PIN && (events & GPIO_IRQ_EDGE_RISE))
    {
        error_count++;
    }
}

int32_t get_position()
{
    const uint forward_pwm_slice_num = pwm_gpio_to_slice_num(FORWARD_PULSE_PIN);
    const uint reverse_pwm_slice_num = pwm_gpio_to_slice_num(REVERSE_PULSE_PIN);

    int32_t forward_counts = (int32_t)pwm_get_counter(forward_pwm_slice_num);
    int32_t reverse_counts = (int32_t)pwm_get_counter(reverse_pwm_slice_num);

    return ((int32_t)position_wrap_count << 16) + (forward_counts - reverse_counts);
}

float get_raw_angle_deg()
{
    return get_position() * 360.0 / PULSES_PER_REV;
}
float get_angle_deg()
{
    return (get_position() % PULSES_PER_REV) * 360.0 / PULSES_PER_REV;
}

static float angular_velocity = 0;
int64_t alarm_callback(alarm_id_t id, void *user_data)
{
    printf("Timer %d fired\n", (int)id);
    return 0;
}

static float _last_position;
static absolute_time_t _last_time_measured;
static float _last_angular_velocity;
static struct repeating_timer _angular_velocity_timer;
bool repeat_timer_compute_angular_velocity_callback(struct repeating_timer *t);

void start_angular_velocity_thread()
{
    _last_position = get_raw_angle_deg();
    _last_time_measured = get_absolute_time();
    add_repeating_timer_ms(-1, repeat_timer_compute_angular_velocity_callback, NULL, &_angular_velocity_timer);
}
bool stop_angular_velocity_thread()
{
    return cancel_repeating_timer(&_angular_velocity_timer);
}

bool repeat_timer_compute_angular_velocity_callback(struct repeating_timer *t)
{
    float current_position = get_raw_angle_deg();

    float delta_position = current_position - _last_position;
    _last_angular_velocity = delta_position * 1000000 / t->delay_us;

    _last_position = current_position;
    return true;
}

float get_angular_velocity()
{
    return _last_angular_velocity;
}

int main()
{
    stdio_init_all();
    printf("\n\n---- START ----\n");

    // ---- init pwm frequency counter
    const uint forward_pwm_slice_num = init_pwm_counter(FORWARD_PULSE_PIN);
    const uint reverse_pwm_slice_num = init_pwm_counter(REVERSE_PULSE_PIN);

    pwm_set_irq_enabled(forward_pwm_slice_num, true);
    pwm_set_irq_enabled(reverse_pwm_slice_num, true);

    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap_isr);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_set_enabled(forward_pwm_slice_num, true);
    pwm_set_enabled(reverse_pwm_slice_num, true);

    // ---- init gpio error pin event
    gpio_init(QUADRATURE_ERROR_PIN);
    gpio_set_dir(QUADRATURE_ERROR_PIN, false);
    gpio_set_irq_enabled_with_callback(QUADRATURE_ERROR_PIN, GPIO_IRQ_EDGE_RISE, true, &on_gpio_isr);

    // ---- set up timers
    start_angular_velocity_thread();

    while (to_ms_since_boot(get_absolute_time()) <= 60 * 1000)
    {
        sleep_ms(50);
        absolute_time_t time = get_absolute_time();
        uint32_t elapsed = (uint32_t)to_us_since_boot(time);
        printf("Angle: %0.1f,\t Angular Velocity: %f\n", get_angle_deg(), get_angular_velocity());
    }
    sleep_ms(2000);
    stop_angular_velocity_thread();

    printf("---- DONE ----\n");
    return 0;
}