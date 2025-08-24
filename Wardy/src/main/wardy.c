#include <stdio.h>
#include <driver/ledc.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/projdefs.h>
#include <esp_log.h>

#define GRIPPER_DUTY_OPEN       51U
#define GRIPPER_DUTY_CLOSE      99U
#define BASE_DUTY_MIN           25U
#define BASE_DUTY_MAX           50U
#define LINK_DUTY_MIN           51U
#define LINK_DUTY_MAX           99U
#define DUTY_STEP               5U
#define UART_RX_BUFFER_SIZE     128U
#define UART_QUEUE_SIZE         10U
#define UART_READ_TIMEOUT_MS    1000U
#define TASK_DELAY_MS           5000U
#define TEST_DELAY_SHORT_MS     500U
#define TEST_DELAY_MED_MS       1000U
#define TEST_DELAY_LONG_MS      2000U
#define STATUS_SEND_INTERVAL_MS 5000U
#define STATUS_SEND_COUNTER     (STATUS_SEND_INTERVAL_MS / TASK_DELAY_MS)

static uint32_t base_duty = (BASE_DUTY_MIN + BASE_DUTY_MAX) / 2U;
static uint32_t link_duty = (LINK_DUTY_MIN + LINK_DUTY_MAX) / 2U;
static uint32_t gripper_duty = GRIPPER_DUTY_CLOSE;

static void configure_ledc_timer(void) {
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT, // range for duty: 0 - 1023
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50U
    };
    ledc_timer_config(&timer_config);
}

static void configure_ledc_channel(ledc_channel_t channel, gpio_num_t gpio) {
    ledc_channel_config_t channel_config = {
        .gpio_num = gpio,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0U,
        .hpoint = 0U,
    };
    ledc_channel_config(&channel_config);
}

static void configure_uart(void) {
    const uart_port_t uart_num = UART_NUM_2;
    const int uart_buffer_size = (UART_RX_BUFFER_SIZE * 2);
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, UART_QUEUE_SIZE, &uart_queue, 0));

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122U
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 4, 5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void setup(void) {
    configure_ledc_timer();
    configure_ledc_channel(LEDC_CHANNEL_0, GPIO_NUM_16);    // Gripper
    configure_ledc_channel(LEDC_CHANNEL_1, GPIO_NUM_14);    // Link
    configure_ledc_channel(LEDC_CHANNEL_2, GPIO_NUM_27);    // Base
    configure_uart();
}

void gripper(int mode) {
    gripper_duty = mode ? GRIPPER_DUTY_OPEN : GRIPPER_DUTY_CLOSE;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, gripper_duty);
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

// static void send_status(void) {
//     char buffer[128];
//     const char *gripper_str = (gripper_duty == GRIPPER_DUTY_OPEN) ? "Open" : "Closed";
//     int len = snprintf(buffer, sizeof(buffer), "Base: %lu, Link: %lu, Gripper: %s\n", base_duty, link_duty, gripper_str);
//     if (len > 0) {
//         uart_write_bytes(UART_NUM_2, buffer, (size_t)len);
//     }
// }

void test() {
    // Set default duty cycles
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, base_duty);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, link_duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
    vTaskDelay(TEST_DELAY_MED_MS / portTICK_PERIOD_MS);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    vTaskDelay(TEST_DELAY_MED_MS / portTICK_PERIOD_MS);

    // Test gripper (CHANNEL_0)
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, GRIPPER_DUTY_OPEN);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(TEST_DELAY_SHORT_MS / portTICK_PERIOD_MS);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, GRIPPER_DUTY_CLOSE);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(TEST_DELAY_LONG_MS / portTICK_PERIOD_MS);

    led_blink();
}

void app_main(void)
{
    setup();
    test();

    base_duty = BASE_DUTY_MAX;
    link_duty = LINK_DUTY_MAX;
    gripper_duty = GRIPPER_DUTY_CLOSE;

    // Read data from UART
    const uart_port_t uart_num = UART_NUM_2;
    uint8_t rx_buffer[128];
    uint32_t send_counter = 0U;

    
    while (1) {
        int length = uart_read_bytes(uart_num, rx_buffer, (UART_RX_BUFFER_SIZE - 1U), (UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS));
        if (length > 0) {
            rx_buffer[length] = '\0';
            char command = rx_buffer[0];
            ESP_LOGI("UART", "Received: %s", rx_buffer);
            switch (command) {
                case 'o': gripper(1); break;
                case 'c': gripper(0); break;
                case 'q': led_blink(); break;
                case 'w': move_link(1); break;
                case 's': move_link(-1); break;
                case 'a': move_base(-1); break;
                case 'd': move_base(1); break;
            }
        } else if (length < 0) {
            ESP_LOGE("UART", "Read failed with error: %d (esp_err: %s)", length, esp_err_to_name(length)); 
        }

        // send_counter++;
        // if (send_counter >= STATUS_SEND_COUNTER) {
        //     send_status();
        //     send_counter = 0U;
        // }

        vTaskDelay(TASK_DELAY_MS / portTICK_PERIOD_MS);
    }
}
