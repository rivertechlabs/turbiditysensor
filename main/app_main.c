/*
 * Copyright (c) JESSICA DROUJKO (@rivertechjess)
 * This code is provided under the GNU GPLv3 license
 * Please see file LICENSE for for more details 
 *
 * For questions on how to use this code and the corresponding turbidity sensor,
 * please visit: www.rivertechlabs.org 
 * or email me at droujko@ifu.baug.ethz.ch :)
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp32/rom/ets_sys.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "esp32/ulp.h"
#include "driver/touch_pad.h"
#include "driver/adc.h"
#include "driver/pcnt.h"
#include "driver/rmt.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include "soc/sens_periph.h"
#include "soc/rtc.h"
#include "frequency_count.h"

#define TAG "app"

#ifdef INCLUDE_ESP_IDF_VERSION_H
# include "esp_idf_version.h"
#endif

static RTC_DATA_ATTR struct timeval sleep_enter_time;

#ifdef CONFIG_IDF_TARGET_ESP32
#include "driver/sdmmc_host.h"
#endif

#define GPIO_SIGNAL_INPUT_F (CONFIG_F_FREQ_SIGNAL_GPIO)
#define GPIO_SIGNAL_INPUT_N (CONFIG_N_FREQ_SIGNAL_GPIO)
#define GPIO_RMT_GATE     (CONFIG_SAMPLING_WINDOW_GPIO)
#define LEDC_OUTPUT_IO    (16) // Output GPIO of a sample 1Hz pulse generator
#define GPIO_OUTPUT_IO_LED  (19) // Pin that controls IR lED flashing
#define GPIO_OUTPUT_IO_SD  (27) // Pin used to pull up SD, detectors, etc.

// internal signals for GPIO constant levels
#define GPIO_CONSTANT_LOW   0x30
#define GPIO_CONSTANT_HIGH  0x38

#define PCNT_UNIT_F         (0)
#define PCNT_UNIT_N         (1)
#define PCNT_CHANNEL_F      (PCNT_CHANNEL_0)
#define PCNT_CHANNEL_N      (PCNT_CHANNEL_1)
#define RMT_CHANNEL       (RMT_CHANNEL_0)
#define RMT_MAX_BLOCKS    (3)   // allow up to 2 * 64 * (2 * 32767) RMT periods in window
#define RMT_CLK_DIV       10    // results in 0.25us steps (80MHz / 10 = 8 MHz

// How often to take a measurement?
#define SAMPLE_PERIOD 0.5  // seconds

// The counter is signed 16-bit, so maximum positive value is 32767
// The filter is unsigned 10-bit, maximum value is 1023. Use full period of maximum frequency.
// For higher expected frequencies, the sample period and filter must be reduced.

// suitable up to 1,638,350 Hz
#define WINDOW_DURATION 0.02  // seconds
#define FILTER_LENGTH 12  // APB @ 80MHz, limits to < 3,333,333 Hz

#if (ESP_IDF_VERSION_MAJOR == 4)
# define RMT_MEM_BLOCK_BYTE_NUM (512 * 4) //((RMT_CHANNEL_MEM_WORDS) * 4)
#endif

#define MOUNT_POINT "/sdcard"

// DMA channel to be used by the SPI peripheral
#ifndef SPI_DMA_CHAN
#define SPI_DMA_CHAN    1
#endif //SPI_DMA_CHAN

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 15
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   13

/*
 * Offset (in 32-bit words) in RTC Slow memory where the data is placed
 * by the ULP coprocessor. It can be chosen to be any value greater or equal
 * to ULP program size, and less than the CONFIG_ESP32_ULP_COPROC_RESERVE_MEM/4 - 6,
 * where 6 is the number of words used by the ULP coprocessor.
 */       
#define ULP_DATA_OFFSET     36

static void window_start_callback(void)
{
    ESP_LOGI(TAG, "Begin sampling");
    gpio_matrix_in(GPIO_SIGNAL_INPUT_F, SIG_IN_FUNC228_IDX, false);
}

static void frequency_callback(double off_hz1, double  off_hz2, double on_hz1, double on_hz2, double final_hz1, double final_hz2, int i)
{
    gpio_matrix_in(GPIO_CONSTANT_LOW, SIG_IN_FUNC228_IDX, false);
    ESP_LOGI(TAG, ",%f,%f,%f,%f,%f,%f, Sample number: %d", off_hz1, off_hz2, on_hz1, on_hz2, final_hz1, final_hz2, i);
}

/*static void config_led(void)
{
    gpio_pad_select_gpio(GPIO_LED);
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);

    // route incoming frequency signal to onboard LED when sampling
    gpio_matrix_out(GPIO_LED, SIG_IN_FUNC228_IDX, false, false);
}
*/

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

void app_main()
{

struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
    }


// First configure the pin to pull up the SD card, detectors, etc.
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO27
    io_conf.pin_bit_mask = (1ULL << GPIO_OUTPUT_IO_SD);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

// Next configure the pin to pull up the IR led
    //gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO19
    io_conf.pin_bit_mask = (1ULL << GPIO_OUTPUT_IO_LED);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

// Turn on SD card reader, detectors, etc.
    gpio_set_level(GPIO_OUTPUT_IO_SD,1);
    printf("Turn SD power on\n");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.

    esp_err_t ret;
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t* card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

// This initializes the slot without card detect (CD) and write protect (WP) signals.
// Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // ensure IR led is off
    gpio_set_level(GPIO_OUTPUT_IO_LED,0);

    printf("Running task code in main app\n");

    init_rmt(GPIO_RMT_GATE, RMT_CHANNEL, RMT_CLK_DIV);
    init_pcnt(GPIO_SIGNAL_INPUT_F, GPIO_RMT_GATE, PCNT_UNIT_F, PCNT_CHANNEL_F, FILTER_LENGTH);
    init_pcnt(GPIO_SIGNAL_INPUT_N, GPIO_RMT_GATE, PCNT_UNIT_N, PCNT_CHANNEL_N, FILTER_LENGTH);

    // assuming 80MHz APB clock
    const double rmt_period = (double)(RMT_CLK_DIV) / 80000000.0;

    const size_t items_size = RMT_MEM_BLOCK_BYTE_NUM * RMT_MAX_BLOCKS;
    rmt_item32_t * rmt_items = malloc(items_size);
    assert(rmt_items);
    memset(rmt_items, 0, items_size);
    int num_rmt_items = create_rmt_window(rmt_items, WINDOW_DURATION, rmt_period);
    assert(num_rmt_items <= RMT_MAX_BLOCKS * RMT_MEM_ITEM_NUM);


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
            ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT_F));
            ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT_N));

            // start sampling window
            ESP_ERROR_CHECK(rmt_write_items(RMT_CHANNEL, rmt_items, num_rmt_items, false));

            // wait for window to finish
            ESP_ERROR_CHECK(rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY));

            // read counter
            int16_t count1 = 0;
            int16_t count2 = 0;
            ESP_ERROR_CHECK(pcnt_get_counter_value(PCNT_UNIT_F, &count1));
            ESP_ERROR_CHECK(pcnt_get_counter_value(PCNT_UNIT_N, &count2));

           // TODO: check for overflow?

            frequency1_hz = count1 / 2.0 / WINDOW_DURATION;
            frequency2_hz = count2 / 2.0 / WINDOW_DURATION;

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
        gpio_set_level(GPIO_OUTPUT_IO_LED,1);

        if (i > 2){
        // delay tasks for 1s to allow LED to warm up
	printf("Pause to let led warm up\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS); // this hasn't been tested, not sure if it's necessary
        }

        // take measurements with LED on
        j = 1;

	while (1)
	{
            last_wake_time = xTaskGetTickCount();

            // clear counters
            ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT_F));
            ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT_N));

            // start sampling window
            ESP_ERROR_CHECK(rmt_write_items(RMT_CHANNEL, rmt_items, num_rmt_items, false));

            // wait for window to finish
            ESP_ERROR_CHECK(rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY));

            // read counter
            int16_t count1 = 0;
            int16_t count2 = 0;
            ESP_ERROR_CHECK(pcnt_get_counter_value(PCNT_UNIT_F, &count1));
            ESP_ERROR_CHECK(pcnt_get_counter_value(PCNT_UNIT_N, &count2));

            // TODO: check for overflow?

            frequency1_hz = count1 / 2.0 / WINDOW_DURATION;
            frequency2_hz = count2 / 2.0 / WINDOW_DURATION;

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
        gpio_set_level(GPIO_OUTPUT_IO_LED,0);

	// Print measurements to SD card if not first two measurements
        if (i > 2)
        {
	// Use POSIX and C standard library functions to work with files.
	// First create a file.
	time_t rawtime;
 	struct tm *info;
	char buffer [255];

	time (&rawtime);

	info = localtime(&rawtime);
	strftime(buffer,255,"%Y-%m-%d_%H:%M:%S",info);
	printf("Adding iteration, date, time, frequencies: %d, %s, %f, %f, %f, %f\n", i, buffer, off_freq1_hz, off_freq2_hz, on_freq1_hz, on_freq2_hz);

	FILE* f;
	char dataToAppend[1000];

        // Open and append file if it exists, create file if it doesn't exist
        f = fopen(MOUNT_POINT"/turb.txt", "a");
        if (f == NULL){
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
        }
        ESP_LOGI(TAG, "Opening file again");

        snprintf(dataToAppend, 1000, "%d, %s, %f, %f, %f, %f\n", i, buffer, off_freq1_hz, off_freq2_hz, on_freq1_hz, on_freq2_hz);
        // writes character string into buffer
        printf("Writing %s to SD card.\n", dataToAppend);
        fputs(dataToAppend, f);
        fclose(f);
        ESP_LOGI(TAG, "File written again");
        }

        // Come out of loop after taking 100 measurement
        ++i;
        if (i > 102)
        {
            break;
        }

        int delay_time = SAMPLE_PERIOD * 1000 / portTICK_PERIOD_MS;
        if (delay_time > 0)
        {
            vTaskDelayUntil(&last_wake_time, delay_time);
        }
    }

    // All done, unmount partition and disable SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");
    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);

    // close all files and allow at least one second for the SD card to power down before pulling the plug
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Turn off SD card reader, detectors, etc.
    gpio_set_level(GPIO_OUTPUT_IO_SD,0);
    printf("Turn power off to SD card, detectors, etc.\n");

    // Sleep for 1min (turn off sensor)
    const int wakeup_time_sec = 60; 
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    printf("Entering deep sleep\n");
    gettimeofday(&sleep_enter_time, NULL);

#if CONFIG_IDF_TARGET_ESP32
    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    rtc_gpio_isolate(GPIO_NUM_12);
#endif

    esp_deep_sleep_start();

    free(rmt_items);
    //free(task_inputs);  // TODO: avoid this if not dynamically allocated
    //vTaskDelete(NULL);

}
