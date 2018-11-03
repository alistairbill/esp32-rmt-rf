/* RF remote RMT example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"

static const char* RMT_TAG = "RMT";

#define RMT_TX_GPIO_NUM 14 // Transmit GPIO pin
#define RMT_RX_GPIO_NUM 15 // Receive GPIO pin
#define RMT_CLK_DIV 100 // RMT counter clock divider
#define RMT_TICK_10_US (80000000/RMT_CLK_DIV/100000) // RMT counter value for 10 us (source clock is APB clock)

#define PULSE_LENGTH_US 350
#define HEADER_HIGH_US (1 * PULSE_LENGTH_US) // Sync bit: 1 pulse high
#define HEADER_LOW_US (31 * PULSE_LENGTH_US) // Sync bit: 31 pulses low
#define BIT_ONE_HIGH_US (3 * PULSE_LENGTH_US) // Bit 1: 3 pulses high
#define BIT_ONE_LOW_US (1 * PULSE_LENGTH_US) // Bit 1: 1 pulse low
#define BIT_ZERO_HIGH_US (1 * PULSE_LENGTH_US) // Bit 0: 1 pulse high
#define BIT_ZERO_LOW_US (3 * PULSE_LENGTH_US) // Bit 0: 3 pulses low
#define BIT_MARGIN 20 // Parse margin time

#define ITEM_DURATION(d)  ((d & 0x7fff)*10/RMT_TICK_10_US) // Parse duration time from memory register value
#define DATA_ITEM_NUM 25  // number of bits: header + 24bit data
#define rmt_item32_tIMEOUT_US 15000 // RMT receiver timeout value (us)

#define TX_REPEATS 1

/*
 * @brief Build register value of waveform for one data bit
 */
static inline void fill_item_level(rmt_item32_t* item, int high_us, int low_us)
{
    item->level0 = 1;
    item->duration0 = (high_us) / 10 * RMT_TICK_10_US;
    item->level1 = 0;
    item->duration1 = (low_us) / 10 * RMT_TICK_10_US;
}

/*
 * @brief Generate sync bit: 1 pulse high + 31 pulses low
 */
static void fill_item_header(rmt_item32_t* item)
{
    fill_item_level(item, HEADER_HIGH_US, HEADER_LOW_US);
}

/*
 * @brief Generate data bit 1: 3 pulses high + 1 pulse low
 */
static void fill_item_bit_one(rmt_item32_t* item)
{
    fill_item_level(item, BIT_ONE_HIGH_US, BIT_ONE_LOW_US);
}

/*
 * @brief Generate data bit 0: 1 pulse high + 3 pulses low
 */
static void fill_item_bit_zero(rmt_item32_t* item)
{
    fill_item_level(item, BIT_ZERO_HIGH_US, BIT_ZERO_LOW_US);
}

/*
 * @brief Check whether duration is around target_us
 */
inline bool check_in_range(int duration_ticks, int target_us, int margin_us)
{
    if(( ITEM_DURATION(duration_ticks) < (target_us + margin_us))
        && ( ITEM_DURATION(duration_ticks) > (target_us - margin_us))) {
        return true;
    } else {
        return false;
    }
}

/*
 * @brief Check whether this value represents a sync bit
 */
static bool header_if(rmt_item32_t* item)
{
    if((item->level0 == 1 && item->level1 == 0)
        && check_in_range(item->duration0, HEADER_HIGH_US, BIT_MARGIN)
        && check_in_range(item->duration1, HEADER_LOW_US, BIT_MARGIN)) {
        return true;
    }
    return false;
}

/*
 * @brief Check whether this value represents a data bit 1
 */
static bool bit_one_if(rmt_item32_t* item)
{
    if((item->level0 == 1 && item->level1 == 0)
        && check_in_range(item->duration0, BIT_ONE_HIGH_US, BIT_MARGIN)
        && (check_in_range(item->duration1, BIT_ONE_LOW_US, BIT_MARGIN)
            || item->duration1 == 0)) {
        return true;
    }
    return false;
}

/*
 * @brief Check whether this value represents a data bit 0
 */
static bool bit_zero_if(rmt_item32_t* item)
{
    if((item->level0 == 1 && item->level1 == 0)
        && check_in_range(item->duration0, BIT_ZERO_HIGH_US, BIT_MARGIN)
        && (check_in_range(item->duration1, BIT_ZERO_LOW_US, BIT_MARGIN)
            || item->duration1 == 0)) {
        return true;
    }
    return false;
}


/*
 * @brief Parse 25 bit waveform to data
 */
static int parse_items(rmt_item32_t* item, int item_num, uint32_t* data)
{
    int w_len = item_num;
    if(w_len < DATA_ITEM_NUM) {
        return -1;
    }
    int i = 0, j = 0;
    if(!header_if(item++)) {
        return -1;
    }
    uint32_t data_t = 0;
    for(j = 0; j < 24; j++) {
        if(bit_one_if(item)) {
            data_t |= (1 << j);
        } else if(bit_zero_if(item)) {
            data_t |= (0 << j);
        } else {
            return -1;
        }
        item++;
        i++;
    }
    *data = data_t;
    return i;
}

/*
 * @brief Build 25bit waveform.
 */
static int build_items(rmt_item32_t* item, int item_num, uint32_t data)
{
    int i = 0, j = 0;
    if(item_num < DATA_ITEM_NUM) {
        return -1;
    }
    fill_item_header(item++);
    i++;
    for(j = 0; j < 24; j++) {
        if(data & 0x1) {
            fill_item_bit_one(item);
        } else {
            fill_item_bit_zero(item);
        }
        item++;
        i++;
        data >>= 1;
    }
    return i;
}

static void print_rx_buffer(rmt_item32_t* item, int item_num)
{
    for (int i = 0; i < item_num; i++) {
        ESP_LOGI(RMT_TAG, "%d %d %d %d", item[i].level0, item[i].duration0, item[i].level1, item[i].duration1);
    }
}

/*
 * @brief RMT transmitter initialization
 */
static void tx_init()
{
    rmt_config_t rmt_tx;
    rmt_tx.channel = 1;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 0;
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = 0;
    rmt_tx.tx_config.idle_level = 0;
    rmt_tx.tx_config.idle_output_en = false;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

/*
 * @brief RMT receiver initialization
 */
static void rx_init()
{
    rmt_config_t rmt_rx;
    rmt_rx.channel = 0;
    rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
    rmt_rx.clk_div = RMT_CLK_DIV;
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0);
}

/**
 * @brief RMT receiver demo, this task will print each received data.
 *
 */
static void rmt_example_nec_rx_task()
{
    int channel = 0; // RMT channel for receiver
    rx_init();
    RingbufHandle_t rb = NULL;
    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(channel, &rb);
    rmt_rx_start(channel, 1);
    while(rb) {
        size_t rx_size = 0;
        //try to receive data from ringbuffer.
        //RMT driver will push all the data it receives to its ringbuffer.
        //We just need to parse the value and return the spaces of ringbuffer.
        rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
        if(item) {
            uint32_t rmt_data;
            int offset = 0;
            int num_items = rx_size / sizeof(rmt_item32_t);
            while (1) {
                int res = parse_items(item + offset, num_items - offset, &rmt_data);
                if (res > 0) {
                    offset += res + 1;
                    ESP_LOGI(RMT_TAG, "data: 0x%06x", rmt_data);
                } else {
                    break;
                }
            }
            //after parsing the data, return spaces to ringbuffer.
            vRingbufferReturnItem(rb, (void*) item);
        } else {
            break;
        }
    }
    vTaskDelete(NULL);
}

/**
 * @brief RMT transmitter demo, this task will periodically send data. (25 bits each time)
 *
 */
static void rmt_example_nec_tx_task()
{
    vTaskDelay(10);
    tx_init();
    esp_log_level_set(RMT_TAG, ESP_LOG_INFO);
    int channel = 1; // RMT TX channel
    uint32_t data = 0xabcdef;
    int tx_num = TX_REPEATS;
    while(1) {
        ESP_LOGI(RMT_TAG, "RMT TX DATA");
        size_t size = (sizeof(rmt_item32_t) * DATA_ITEM_NUM * tx_num);
        rmt_item32_t* item = (rmt_item32_t*) malloc(size);
        int item_num = DATA_ITEM_NUM * tx_num;
        memset((void*) item, 0, size);
        int i, offset = 0;
        while (1) {
            i = build_items(item + offset, item_num - offset, data);
            if (i < 0) {
                break;
            }
            offset += i;
        }
        //To send data according to the waveform items.
        rmt_write_items(channel, item, item_num, true);
        //Wait until sending is done.
        rmt_wait_tx_done(channel, portMAX_DELAY);
        //before we free the data, make sure sending is already done.
        free(item);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(rmt_example_nec_rx_task, "rmt_nec_rx_task", 2048, NULL, 10, NULL);
    xTaskCreate(rmt_example_nec_tx_task, "rmt_nec_tx_task", 2048, NULL, 10, NULL);
}
