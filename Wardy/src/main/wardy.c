#include <stdio.h>
#include <driver/ledc.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/projdefs.h>
// #include "driver/gpio.h"

#define GRIPPER_DUTY_OPEN 51
#define GRIPPER_DUTY_CLOSE 99
#define BASE_DUTY_MIN 25
#define BASE_DUTY_MAX 50
#define LINK_DUTY_MIN 51
#define LINK_DUTY_MAX 99
#define DUTY_STEP 5 

static uint32_t base_duty = (BASE_DUTY_MIN + BASE_DUTY_MAX) /2;
static uint32_t link_duty = (LINK_DUTY_MIN + LINK_DUTY_MAX) /2;

void setup(void) {
    // Timer Configuration
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT, // range for duty: 0 - 1023
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50
    };

    ledc_timer_config(&timer_config);

    // Gripper Arm
    ledc_channel_config_t channel_config = {
        .gpio_num = 16,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };

    ledc_channel_config(&channel_config);

    // Gripper Link
    ledc_channel_config_t link_channel_config = {
        .gpio_num = 14,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };

    ledc_channel_config(&link_channel_config);

    // Base Link
    ledc_channel_config_t base_link_channel_config = {
        .gpio_num = 27,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };

    ledc_channel_config(&base_link_channel_config);

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = 2048;
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 4, 5, 18, 19));

}

void gripper(int mode) {
    uint32_t duty = mode ? GRIPPER_DUTY_OPEN : GRIPPER_DUTY_CLOSE;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

void move_base(int direction) {
    if (direction > 0 && base_duty < BASE_DUTY_MAX) {
        base_duty += DUTY_STEP;
    } else if (direction < 0 && base_duty > BASE_DUTY_MIN) {
        base_duty -= DUTY_STEP;
    }
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, base_duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
}

void move_link(int direction) {
    if (direction > 0 && link_duty < LINK_DUTY_MAX) 
        link_duty += DUTY_STEP;
    else if (direction < 0 && link_duty > LINK_DUTY_MIN)
        link_duty -= DUTY_STEP;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, link_duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
}

void led_blink() {
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    for(int i = 0; i < 3; i++) {
        gpio_set_level(GPIO_NUM_2, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void test() {
    // Set default duty cycles
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, BASE_DUTY_MIN + (BASE_DUTY_MIN + BASE_DUTY_MAX) / 2);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, LINK_DUTY_MIN + (LINK_DUTY_MIN + LINK_DUTY_MAX) / 2);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // TESTING with LEDC_CHANNEL_0
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, GRIPPER_DUTY_OPEN);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(500 / portTICK_PERIOD_MS); 
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, GRIPPER_DUTY_CLOSE);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(2000 / portTICK_PERIOD_MS); 

    // TESTING with LEDC_CHANNEL_1
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, LINK_DUTY_MIN);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, LINK_DUTY_MAX);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // TESTING with LEDC_CHANNEL_2
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, BASE_DUTY_MIN);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, BASE_DUTY_MAX);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
}

void app_main(void)
{
    setup();

    test();

    // Read data from UART
    const uart_port_t uart_num = UART_NUM_2;
    uint8_t rx_buffer[128];


    
    while (1) {
        int length = uart_read_bytes(uart_num, rx_buffer, sizeof(rx_buffer) -1, 100 / portTICK_PERIOD_MS);
        if (length > 0) {
            rx_buffer[length] = '\0';
            char command = rx_buffer[0];
            switch (command) {
                case 'o': gripper(1); break;
                case 'c': gripper(0); break;
                case 'q': led_blink(); break;
                case 'w': move_link(1); break;
                case 's': move_link(-1); break;
                case 'a': move_base(-1); break;
                case 'd': move_base(1); break;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
