
/*
* Copyright (c) 2024, CATIE
* SPDX-License-Identifier: Apache-2.0
*/
/*
* Class A LoRaWAN sample application
*
* Copyright (c) 2020 Manivannan Sadhasivam <mani@kernel.org>
*
* SPDX-License-Identifier: Apache-2.0
*/
#include <zephyr/device.h>
#include <zephyr/lorawan/lorawan.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/poweroff.h> 
#include <zephyr/drivers/gpio.h> 
#include <zephyr/drivers/i2c.h> 
#include <zephyr/init.h>  


/* Customize based on network configuration */
#define LORAWAN_DEV_EUI { 0xCA, 0x1C, 0xEA, 0x43, 0x66, 0x37, 0xA9, 0x5E } 
#define LORAWAN_JOIN_EUI { 0x32, 0x42, 0xBA, 0x2E, 0x8D, 0x44, 0x5D, 0x31 } 
#define LORAWAN_APP_KEY { 0x9E, 0x64, 0xFF, 0xBB, 0x13, 0x8D, 0xD8, 0x05, 0xD5, 0xEF, 0x33, 0xDB, 0x91, 0x96, 0xDB, 0x41 } 


#define DELAY K_MSEC(10000)


/*  block to set RTC time  */ 
//#define SET_RTC_TIME 
#define RTC_SET_YEAR  26 /* e.g., 2026 */
#define RTC_SET_MONTH  4 
#define RTC_SET_MDAY  24 
#define RTC_SET_WDAY  5 /* 0=Sun, 1=Mon, 2=Tue... */ 
#define RTC_SET_HOUR  11 
#define RTC_SET_MINUTE 0 
#define RTC_SET_SECOND 0 


uint8_t ALARM_TRIGGER_HOUR = 11; 
uint8_t ALARM_TRIGGER_MINUTE = 2; 


#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lorawan_class_a);


char data[] = "RTC ON"; 


/* I2C and RV-8803 Setup */ 
#define RV8803_ADDRESS    0x32 // Raw hardware address
#define RV8803_SEC_REG    0x00 
#define RV8803_MIN_ALARM_REG 0x08 
#define RV8803_EXT_REG    0x0D // Extension register for 32.768kHz clock output
#define RV8803_FLAG_REG   0x0E 
#define RV8803_CTRL_REG   0x0F 


/* Power Hold Setup */ 
static const struct gpio_dt_spec hold_pin = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios); 


/* Early latch runs before main() to survive boot sequence */ 
static int latch_power_early(void) 
{ 
  if (gpio_is_ready_dt(&hold_pin)) { 
    gpio_pin_configure_dt(&hold_pin, GPIO_OUTPUT_ACTIVE); 
    gpio_pin_set_dt(&hold_pin, 1); 
  } 
  return 0; 
} 
SYS_INIT(latch_power_early, PRE_KERNEL_2, 0); 


/* Helper to convert Decimal to BCD for RTC registers */ 
static uint8_t decimal_to_bcd(uint8_t val) { 
  return ((val / 10) << 4) | (val % 10); 
} 


#ifdef SET_RTC_TIME /
/* Raw I2C write to set the time registers */ 
static void set_raw_rtc_time(const struct device *i2c_bus, uint8_t yr, uint8_t mon, uint8_t mday, uint8_t wday, uint8_t hr, uint8_t min, uint8_t sec) 
{ 
  uint8_t time_data[8]; 
  time_data[0] = RV8803_SEC_REG; /* Start at 0x00 (Seconds register) */ 
  time_data[1] = decimal_to_bcd(sec) & 0x7F; 
  time_data[2] = decimal_to_bcd(min) & 0x7F; 
  time_data[3] = decimal_to_bcd(hr) & 0x3F; 
  time_data[4] = (1 << wday) & 0x7F; /* Weekday is a bitmask (Bit0=Day1, Bit1=Day2) */ 
  time_data[5] = decimal_to_bcd(mday) & 0x3F; 
  time_data[6] = decimal_to_bcd(mon) & 0x1F; 
  time_data[7] = decimal_to_bcd(yr); 
  
  /* Write 7 sequential time registers */ 
  i2c_write(i2c_bus, time_data, 8, RV8803_ADDRESS); 
} 
#endif 


/* Raw I2C write to arm the 24h alarm */ 
static void set_raw_rtc_alarm(const struct device *i2c_bus, uint8_t hour, uint8_t min) 
{ 
  uint8_t alarm_data[4]; 
  alarm_data[0] = RV8803_MIN_ALARM_REG; 
  alarm_data[1] = decimal_to_bcd(min) & 0x7F; // Enable min, set value 
  alarm_data[2] = decimal_to_bcd(hour) & 0x3F; // Enable hr, set value 
  alarm_data[3] = 0x80; // Disable weekday alarm (Bit 7 = 1) 
  
  /* Write 3 alarm registers starting at 0x08 */ 
  i2c_write(i2c_bus, alarm_data, 4, RV8803_ADDRESS); 


  /* Force AIE (Alarm Interrupt Enable) bit to 1 in Control Reg */
  uint8_t ctrl_data[2] = {RV8803_CTRL_REG, 0x08}; 
  i2c_write(i2c_bus, ctrl_data, 2, RV8803_ADDRESS); 
} 


/* Raw I2C write to clear the Alarm Flag and release the INT pin */ 
static void clear_rtc_flag(const struct device *i2c_bus) 
{ 
  uint8_t flag_data[2] = {RV8803_FLAG_REG, 0x00}; 
  i2c_write(i2c_bus, flag_data, 2, RV8803_ADDRESS); 
} 



static void dl_callback(uint8_t port, uint8_t flags, int16_t rssi, int8_t snr, uint8_t len,
      const uint8_t *hex_data)
{
  LOG_INF("Port %d, Pending %d, RSSI %ddB, SNR %ddBm, Time %d", port,
    flags & LORAWAN_DATA_PENDING, rssi, snr, !!(flags & LORAWAN_TIME_UPDATED));
  if (hex_data) {
    LOG_HEXDUMP_INF(hex_data, len, "Payload: ");
  }
}


static void lorwan_datarate_changed(enum lorawan_datarate dr)
{
  uint8_t unused, max_size;


  lorawan_get_payload_sizes(&unused, &max_size);
  LOG_INF("New Datarate: DR_%d, Max Payload %d", dr, max_size);
}


int main(void)
{
  const struct device *lora_dev;
  struct lorawan_join_config join_cfg;
  uint8_t dev_eui[] = LORAWAN_DEV_EUI;
  uint8_t join_eui[] = LORAWAN_JOIN_EUI;
  uint8_t app_key[] = LORAWAN_APP_KEY;
  int ret;


  /* Re-verify latch in main to be safe */ 
  if (gpio_is_ready_dt(&hold_pin)) { 
    gpio_pin_set_dt(&hold_pin, 1); 
  } 


  /* Fetch I2C Controller Directly By Node Label (Avoids alias errors) */ 
  const struct device *i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c2)); 
  if (!device_is_ready(i2c_bus)) { 
    LOG_ERR("I2C bus not ready!"); 
    return 0; 
  } 


  /* Restore the 32.768kHz CLKOUT Timing  */ 
  uint8_t clk_data[2] = {RV8803_EXT_REG, 0x00}; // 0x00 sets FD (Frequency Definition) to 32.768kHz 
  i2c_write(i2c_bus, clk_data, 2, RV8803_ADDRESS); 


#ifdef SET_RTC_TIME 
  /* Set the RTC clock time using raw I2C */ 
  set_raw_rtc_time(i2c_bus, RTC_SET_YEAR, RTC_SET_MONTH, RTC_SET_MDAY, RTC_SET_WDAY, 
          RTC_SET_HOUR, RTC_SET_MINUTE, RTC_SET_SECOND); 
  LOG_INF("RTC time successfully written via I2C"); 
#endif 


  /* Set next alarm using the pre-defined variables */ 
  set_raw_rtc_alarm(i2c_bus, ALARM_TRIGGER_HOUR, ALARM_TRIGGER_MINUTE); 


  struct lorawan_downlink_cb downlink_cb = {
    .port = LW_RECV_PORT_ANY,
    .cb = dl_callback
  };


  lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
  if (!device_is_ready(lora_dev)) {
    LOG_ERR("%s: device not ready.", lora_dev->name);
    return 0;
  }


#if defined(CONFIG_LORAWAN_REGION_EU868)
  ret = lorawan_set_region(LORAWAN_REGION_EU868);
  if (ret < 0) {
    LOG_ERR("lorawan_set_region failed: %d", ret);
    return 0;
  }
#endif


  ret = lorawan_start();
  if (ret < 0) {
    LOG_ERR("lorawan_start failed: %d", ret);
    return 0;
  }


  lorawan_register_downlink_callback(&downlink_cb);
  lorawan_register_dr_changed_callback(lorwan_datarate_changed);


  join_cfg.mode = LORAWAN_ACT_OTAA;
  join_cfg.dev_eui = dev_eui;
  join_cfg.otaa.join_eui = join_eui;
  join_cfg.otaa.app_key = app_key;
  join_cfg.otaa.nwk_key = app_key;
  join_cfg.otaa.dev_nonce = 0u;


  LOG_INF("Joining network over OTAA");
  ret = lorawan_join(&join_cfg);
  if (ret < 0) {
    LOG_ERR("lorawan_join_network failed: %d", ret);
    return 0;
  }


  lorawan_enable_adr(false); 
  lorawan_set_datarate(LORAWAN_DR_0); 


  LOG_INF("Sending data...");
  ret = lorawan_send(2, data, sizeof(data) - 1, LORAWAN_MSG_UNCONFIRMED); 


  if (ret < 0) { 
    LOG_ERR("lorawan_send failed: %d", ret); 
  } else { 
    LOG_INF("Data sent!"); 
  } 


  /* Clear the hardware flag via I2C to release the INT pin */ 
  clear_rtc_flag(i2c_bus); 
  k_msleep(50); 


  LOG_INF("Entering deep sleep / system power off..."); 
  
  /* Drop the secondary D4 power hold */ 
  if (gpio_is_ready_dt(&hold_pin)) { 
    gpio_pin_set_dt(&hold_pin, 0); 
  } 
  
  sys_poweroff(); 


  return 0;
}