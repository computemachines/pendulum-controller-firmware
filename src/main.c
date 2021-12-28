#include <stdio.h>
#include <math.h>
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
#define FORWARD_MOTOR_PIN 20
#define REVERSE_MOTOR_PIN 21
#define MOTOR_ENABLE_PIN 14
#define PWM_WRAP_NUMBER 0xFFF

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

float control_loop(uint32_t time);
void set_motor_enabled(bool enabled);
void set_motor_duty_cycle(float ratio);
void set_motor_direction(bool forward);
void set_motor_brake(bool brake);

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

    // ---- init pwm ----
    gpio_set_function(MOTOR_ENABLE_PIN, GPIO_FUNC_PWM);
    gpio_init(FORWARD_MOTOR_PIN);
    gpio_init(REVERSE_MOTOR_PIN);
    gpio_set_dir(FORWARD_MOTOR_PIN, true);
    gpio_set_dir(REVERSE_MOTOR_PIN, true);

    uint slice = pwm_gpio_to_slice_num(MOTOR_ENABLE_PIN);
    pwm_set_wrap(slice, PWM_WRAP_NUMBER);
    set_motor_enabled(true);

    while (to_ms_since_boot(get_absolute_time()) <= 60 * 1000)
    {
        sleep_ms(50);
        absolute_time_t time = get_absolute_time();
        uint32_t elapsed = (uint32_t)to_us_since_boot(time);

        float velocity = control_loop(elapsed);

        // printf("Angle: %0.1f,\t Angular Velocity: %f\n", get_angle_deg(), velocity);
    }
    sleep_ms(2000);

    set_motor_enabled(false);

    printf("---- DONE ----\n");
    return 0;
}

void set_motor_duty_cycle(float ratio) {
    uint slice = pwm_gpio_to_slice_num(MOTOR_ENABLE_PIN);

    pwm_set_chan_level(slice, PWM_CHAN_A, clamp(ratio, 0, 1) * PWM_WRAP_NUMBER);
}

void set_motor_direction(bool forward) {
    if (forward) {
        gpio_put(FORWARD_MOTOR_PIN, true);
        gpio_put(REVERSE_MOTOR_PIN, false);
    } else {
        gpio_put(FORWARD_MOTOR_PIN, false);
        gpio_put(REVERSE_MOTOR_PIN, true);
    }
}

void set_motor_enabled(bool enabled){
    if (enabled) {
        uint slice = pwm_gpio_to_slice_num(MOTOR_ENABLE_PIN);
        pwm_set_enabled(slice, enabled);
    } else {
        set_motor_duty_cycle(0.f);
    }
}

void set_motor_brake(bool brake) {
    if(brake){
        set_motor_duty_cycle(1.f);
        // gpio_put(MOTOR_ENABLE_PIN, true);
        gpio_put(FORWARD_MOTOR_PIN, false);
        gpio_put(REVERSE_MOTOR_PIN, false);
    } else {
        set_motor_duty_cycle(0.f);
        // gpio_put(MOTOR_ENABLE_PIN, false);
    }
}

static float last_raw_angle = 0;
static uint32_t last_time_us;
float control_loop(uint32_t time_us)
{
    // get current configuration
    float current_raw_angle = get_raw_angle_deg();

    // compute current state
    uint32_t time_step_us = time_us - last_time_us;
    float angular_velocity = 1000000 * (current_raw_angle - last_raw_angle) / last_time_us;

    
    set_motor_direction(angular_velocity > -(get_raw_angle_deg() - 180)/sqrt(9.8));
    set_motor_duty_cycle(1.0);

    // finish up
    last_time_us = time_us;
    last_raw_angle = current_raw_angle;

    return angular_velocity;
}