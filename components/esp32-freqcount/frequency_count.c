/*
 * MIT License
 *
 * Copyright (c) 2018 David Antliff
 * Copyright (c) 2018 Chris Morgan <chmorgan@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/pcnt.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "rom/ets_sys.h"
#include "esp_sleep.h"

#ifdef INCLUDE_ESP_IDF_VERSION_H
# include "esp_idf_version.h"
#endif

#include "frequency_count.h"

#define TAG "frequency_counter"
static RTC_DATA_ATTR struct timeval sleep_enter_time;
// In ESP-IDF v4.1-beta1 (and presumably newer), the macro RMT_MEM_BLOCK_BYTE_NUM has been removed
#if (ESP_IDF_VERSION_MAJOR == 4)
# define RMT_MEM_BLOCK_BYTE_NUM (512 * 4) //((RMT_CHANNEL_MEM_WORDS) * 4)
#endif

static void init_rmt(uint8_t tx_gpio, rmt_channel_t channel, uint8_t clk_div)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    rmt_config_t rmt_tx = {
        .rmt_mode = RMT_MODE_TX,
        .channel = channel,
        .gpio_num = tx_gpio,
        .mem_block_num = 1,  // single block
        .clk_div = clk_div,
        .tx_config.loop_en = false,
        .tx_config.carrier_en = false,
        .tx_config.idle_level = RMT_IDLE_LEVEL_LOW,
        .tx_config.idle_output_en = true,
    };
    ESP_ERROR_CHECK(rmt_config(&rmt_tx));
    ESP_ERROR_CHECK(rmt_driver_install(rmt_tx.channel, 0, 0));
}

static int create_rmt_window(rmt_item32_t * items, double sampling_window_seconds, double rmt_period)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    int num_items = 0;

    // enable counter for exactly x seconds:
    int32_t total_duration = (uint32_t)(sampling_window_seconds / rmt_period);
    ESP_LOGD(TAG, "total_duration %f seconds = %d * %g seconds", sampling_window_seconds, total_duration, rmt_period);

    // max duration per item is 2^15-1 = 32767
    while (total_duration > 0)
    {
        uint32_t duration = total_duration > 32767 ? 32767 : total_duration;
        items[num_items].level0 = 1;
        items[num_items].duration0 = duration;
        total_duration -= duration;
        ESP_LOGD(TAG, "duration %d", duration);

        if (total_duration > 0)
        {
            uint32_t duration = total_duration > 32767 ? 32767 : total_duration;
            items[num_items].level1 = 1;
            items[num_items].duration1 = duration;
            total_duration -= duration;
        }
        else
        {
            items[num_items].level1 = 0;
            items[num_items].duration1 = 0;
        }
        ESP_LOGD(TAG, "[%d].level0 %d", num_items, items[num_items].level0);
        ESP_LOGD(TAG, "[%d].duration0 %d", num_items, items[num_items].duration0);
        ESP_LOGD(TAG, "[%d].level1 %d", num_items, items[num_items].level1);
        ESP_LOGD(TAG, "[%d].duration1 %d", num_items, items[num_items].duration1);

        ++num_items;
    }
    ESP_LOGD(TAG, "num_items %d", num_items);
    return num_items;
}

static void init_pcnt(uint8_t pulse_gpio, uint8_t ctrl_gpio, pcnt_unit_t unit, pcnt_channel_t channel, uint16_t filter_length)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    // set up counter
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = pulse_gpio,
        .ctrl_gpio_num = ctrl_gpio,
        .lctrl_mode = PCNT_MODE_DISABLE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,  // count both rising and falling edges
        .neg_mode = PCNT_COUNT_INC,
        .counter_h_lim = 0,
        .counter_l_lim = 0,
        .unit = unit,
        .channel = channel,
    };

    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));

    // set the GPIO back to high-impedance, as pcnt_unit_config sets it as pull-up
    ESP_ERROR_CHECK(gpio_set_pull_mode(pulse_gpio, GPIO_FLOATING));

    // enable counter filter - at 80MHz APB CLK, 1000 pulses is max 80,000 Hz, so ignore pulses less than 12.5 us.
    ESP_ERROR_CHECK(pcnt_set_filter_value(unit, filter_length));
    ESP_ERROR_CHECK(pcnt_filter_enable(unit));
}

void frequency_count_task_function(void * pvParameter)
{
    frequency_count_configuration_t configuration;

    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    configuration = *(frequency_count_configuration_t*)pvParameter;
    frequency_count_configuration_t *task_inputs = &configuration;

    ESP_LOGI(TAG, "led_gpio %d, pcnt_gpio1 %d, pcnt_gpio2 %d, pcnt_unit1 %d, pcnt_unit2 %d, pcnt_channel1 %d, pcnt_channel2 %d, rmt_gpio %d, rmt_clk_div %d, sampling_period_seconds %f, sampling_window_seconds %f, filter_length %d",
        task_inputs->led_gpio,
        task_inputs->pcnt_gpio1,
        task_inputs->pcnt_gpio2,
        task_inputs->pcnt_unit1,
        task_inputs->pcnt_unit2,
        task_inputs->pcnt_channel1,
        task_inputs->pcnt_channel2,
        task_inputs->rmt_gpio,
        task_inputs->rmt_clk_div,
        task_inputs->sampling_period_seconds,
        task_inputs->sampling_window_seconds,
        task_inputs->filter_length);

    init_rmt(task_inputs->rmt_gpio, task_inputs->rmt_channel, task_inputs->rmt_clk_div);
    init_pcnt(task_inputs->pcnt_gpio1, task_inputs->rmt_gpio, task_inputs->pcnt_unit1, task_inputs->pcnt_channel1, task_inputs->filter_length);
    init_pcnt(task_inputs->pcnt_gpio2, task_inputs->rmt_gpio, task_inputs->pcnt_unit2, task_inputs->pcnt_channel2, task_inputs->filter_length);

    // assuming 80MHz APB clock
    const double rmt_period = (double)(task_inputs->rmt_clk_div) / 80000000.0;

    const size_t items_size = RMT_MEM_BLOCK_BYTE_NUM * task_inputs->rmt_max_blocks;
    rmt_item32_t * rmt_items = malloc(items_size);
    assert(rmt_items);
    memset(rmt_items, 0, items_size);
    int num_rmt_items = create_rmt_window(rmt_items, task_inputs->sampling_window_seconds, rmt_period);
    assert(num_rmt_items <= task_inputs->rmt_max_blocks * RMT_MEM_ITEM_NUM);
    
    // make sure led off
    printf("INSIDE TASK\n");
    //gpio_set_level(task_inputs->led_gpio,1);
    //vTaskDelay(5000 / portTICK_PERIOD_MS);

    TickType_t last_wake_time = xTaskGetTickCount();
    double frequency1_hz;
    double frequency2_hz;
    double off_freq1_hz = 0;
    double off_freq2_hz = 0;
    double on_freq1_hz = 0;
    double on_freq2_hz = 0;     
    double final_freq1_hz = 0;
    double final_freq2_hz = 0;
    int i = 1;

    while (1)
    {
        // take measurements with LED off
        int j = 1;

        while (1)
        {
            last_wake_time = xTaskGetTickCount();

            // clear counters
            ESP_ERROR_CHECK(pcnt_counter_clear(task_inputs->pcnt_unit1));
            ESP_ERROR_CHECK(pcnt_counter_clear(task_inputs->pcnt_unit2));

            // start sampling window
            ESP_ERROR_CHECK(rmt_write_items(task_inputs->rmt_channel, rmt_items, num_rmt_items, false));

            //// call window-start callback if set
            //if (task_inputs->window_start_callback)
            //{
            //    (task_inputs->window_start_callback)();
            //}

            // wait for window to finish
            ESP_ERROR_CHECK(rmt_wait_tx_done(task_inputs->rmt_channel, portMAX_DELAY));

            // read counter
            int16_t count1 = 0;
            int16_t count2 = 0;
            ESP_ERROR_CHECK(pcnt_get_counter_value(task_inputs->pcnt_unit1, &count1));
            ESP_ERROR_CHECK(pcnt_get_counter_value(task_inputs->pcnt_unit2, &count2));

            // TODO: check for overflow?

            frequency1_hz = count1 / 2.0 / task_inputs->sampling_window_seconds;
            frequency2_hz = count2 / 2.0 / task_inputs->sampling_window_seconds;
            
            off_freq1_hz = off_freq1_hz + frequency1_hz;
            off_freq2_hz = off_freq2_hz + frequency2_hz;

            // Average the frequencies and come out of the loop
            if (j == 10)
            {
                off_freq1_hz = off_freq1_hz / j ;
                off_freq2_hz = off_freq2_hz / j ;
                break;            
            }
            ++j;

        }

        // turn LED on
        printf("Turn LED on\n");
        gpio_set_level(task_inputs->led_gpio,0);

        // delay tasks for 5s to allow LED to warm up
        //vTaskDelay(5000 / portTICK_PERIOD_MS);

        // take measurements with LED on
        j = 1;

        while (1)
        {
            last_wake_time = xTaskGetTickCount();

            // clear counters
            ESP_ERROR_CHECK(pcnt_counter_clear(task_inputs->pcnt_unit1));
            ESP_ERROR_CHECK(pcnt_counter_clear(task_inputs->pcnt_unit2));

            // start sampling window
            ESP_ERROR_CHECK(rmt_write_items(task_inputs->rmt_channel, rmt_items, num_rmt_items, false));

            //// call window-start callback if set
            //if (task_inputs->window_start_callback)
            //{
            //    (task_inputs->window_start_callback)();
            //}

            // wait for window to finish
            ESP_ERROR_CHECK(rmt_wait_tx_done(task_inputs->rmt_channel, portMAX_DELAY));

            // read counter
            int16_t count1 = 0;
            int16_t count2 = 0;
            ESP_ERROR_CHECK(pcnt_get_counter_value(task_inputs->pcnt_unit1, &count1));
            ESP_ERROR_CHECK(pcnt_get_counter_value(task_inputs->pcnt_unit2, &count2));

            // TODO: check for overflow?

            frequency1_hz = count1 / 2.0 / task_inputs->sampling_window_seconds;
            frequency2_hz = count2 / 2.0 / task_inputs->sampling_window_seconds;

            on_freq1_hz = on_freq1_hz + frequency1_hz;
            on_freq2_hz = on_freq2_hz + frequency2_hz;

            // Average the frequencies and come out of the loop
            if (j == 10)
            {
                on_freq1_hz = on_freq1_hz / j ;
                on_freq2_hz = on_freq2_hz / j ;
                break;
            }
            ++j;

        }

        // turn LED off
        printf("Turn LED off\n");
        gpio_set_level(task_inputs->led_gpio,1);

        // delay tasks for 5s to allow LED to reset
        //vTaskDelay(5000 / portTICK_PERIOD_MS);

        // take difference of frequencies
        final_freq1_hz = on_freq1_hz - off_freq1_hz ;
        final_freq2_hz = on_freq2_hz - off_freq2_hz ;

        // call the frequency update callback
        if (task_inputs->frequency_update_callback)
        {
            (task_inputs->frequency_update_callback)(off_freq1_hz, off_freq2_hz, on_freq1_hz, on_freq2_hz, final_freq1_hz, final_freq2_hz,i);
        }

        // Come out of loop after taking 10 measurements
        ++i;
        if (i > 5)
        {
            break;
        }

        //ESP_LOGD(TAG, "counter1 %d, frequency1 %f Hz", count1, final_freq1_hz);
        //ESP_LOGD(TAG, "counter2 %d, frequency2 %f Hz", count2, final_freq2_hz);

        int delay_time = task_inputs->sampling_period_seconds * 1000 / portTICK_PERIOD_MS;
        if (delay_time > 0)
        {
            vTaskDelayUntil(&last_wake_time, delay_time);
        }
    }

    //free(rmt_items);
    //free(task_inputs);  // TODO: avoid this if not dynamically allocated

const int wakeup_time_sec = 60;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

   printf("Entering deep sleep\n");
   gettimeofday(&sleep_enter_time, NULL);

   esp_deep_sleep_start();

    free(rmt_items);
    free(task_inputs);  // TODO: avoid this if not dynamically allocated
    vTaskDelete(NULL);
}
