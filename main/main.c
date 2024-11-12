/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"

#define GATTS_TAG "ESP_SOIL_METER"
#define DEVICE_NAME "ESP_SOIL_METER"

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#define GATTS_SERVICE_UUID_TEST_A 0x00FF
#define GATTS_CHAR_UUID_TEST_A 0xFF01
#define GATTS_DESCR_UUID_TEST_A 0x3333
#define GATTS_NUM_HANDLE_TEST_A 4

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

static esp_gatt_if_t gatt_if = 0x00;
static uint16_t conn_id = 0x00;
static uint16_t attr_handle = 0x00;

static uint8_t adv_event_done = 0;
static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    // second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

struct gatts_profile_inst
{
  esp_gatts_cb_t gatts_cb;
  uint16_t gatts_if;
  uint16_t app_id;
  uint16_t conn_id;
  uint16_t service_handle;
  esp_gatt_srvc_id_t service_id;
  uint16_t char_handle;
  esp_bt_uuid_t char_uuid;
  esp_gatt_perm_t perm;
  esp_gatt_char_prop_t property;
  uint16_t descr_handle;
  esp_bt_uuid_t descr_uuid;
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static uint8_t char1_str[] = {0x11, 0x22, 0x33};
static esp_gatt_char_prop_t a_property = 1;

static esp_attr_value_t gatts_demo_char1_val =
    {
        .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
        .attr_len = sizeof(char1_str),
        .attr_value = char1_str,
};

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    }};

static void soil_meter_callback(void *arg);

static esp_timer_create_args_t soil_meter_timer_args = {
    .callback = soil_meter_callback};

static esp_timer_handle_t soil_meter_timer = NULL;

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
  switch (event)
  {
  case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    adv_event_done &= (~adv_config_flag);
    if (adv_event_done == 0)
    {
      esp_ble_gap_start_advertising(&adv_params);
    }
    break;
  case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
    adv_event_done &= (~scan_rsp_config_flag);
    if (adv_event_done == 0)
    {
      esp_ble_gap_start_advertising(&adv_params);
    }
    break;
  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    // advertising start complete event to indicate advertising start successfully or failed
    if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
    {
      ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
      break;
    }
    ESP_LOGI(GATTS_TAG, "Advertising start successfully");
    break;
  case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
    if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
    {
      ESP_LOGE(GATTS_TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
      break;
    }
    ESP_LOGI(GATTS_TAG, "Advertising stop successfully");
    break;
  case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
    ESP_LOGI(GATTS_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
             param->update_conn_params.status,
             param->update_conn_params.conn_int,
             param->update_conn_params.latency,
             param->update_conn_params.timeout);
    break;
  case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
    ESP_LOGI(GATTS_TAG, "Packet length update, status %d, rx %d, tx %d",
             param->pkt_data_length_cmpl.status,
             param->pkt_data_length_cmpl.params.rx_len,
             param->pkt_data_length_cmpl.params.tx_len);
    break;
  default:
    break;
  }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
  ESP_LOGI(GATTS_TAG, "gatts_event_handle");
  if (event == ESP_GATTS_REG_EVT)
  {
    if (param->reg.status == ESP_GATT_OK)
    {
      gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
    }
    else
    {
      ESP_LOGI(GATTS_TAG, "Reg app failed");
      return;
    }
  }

  for(int idx = 0; idx < PROFILE_NUM; idx++){
    if(gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[idx].gatts_if) {
      gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
    }
  }
  
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
  switch (event)
  {
  case ESP_GATTS_REG_EVT:
    ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
    gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
    gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
    gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
    gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = 0x00FF;

    esp_ble_gap_set_device_name(DEVICE_NAME);
    // config adv data
    esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret)
    {
      ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
    }
    adv_event_done |= adv_config_flag;
    // config scan response data
    ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
    if (ret)
    {
      ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
    }
    adv_event_done |= scan_rsp_config_flag;

    esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);

    break;
  case ESP_GATTS_READ_EVT:
  {
    ESP_LOGI(GATTS_TAG, "Characteristic read, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
    esp_gatt_rsp_t rsp;
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    rsp.attr_value.handle = param->read.handle;
    rsp.attr_value.len = 4;
    rsp.attr_value.value[0] = 0xde;
    rsp.attr_value.value[1] = 0xed;
    rsp.attr_value.value[2] = 0xbe;
    rsp.attr_value.value[3] = 0xef;
    esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                ESP_GATT_OK, &rsp);
    break;
  }
  case ESP_GATTS_WRITE_EVT:
  {
    ESP_LOGI(GATTS_TAG, "Characteristic write, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
    if (!param->write.is_prep)
    {
      ESP_LOGI(GATTS_TAG, "value len %d, value ", param->write.len);
      ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
      ESP_LOGI(GATTS_TAG, "write handle");
      if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2)
      {
        ESP_LOGI(GATTS_TAG, "DO I EVER GET IN HERE");
        uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
        if (descr_value == 0x0001)
        {
          if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
          {

            ESP_LOGI(GATTS_TAG, "Notification enable");
            esp_err_t timer_res = esp_timer_start_periodic(soil_meter_timer, 5 * 1000 * 1000); // 20 seconds
            
            gatt_if = gatts_if;
            conn_id = param->write.conn_id;

            uint8_t notify_data[15];
            for (int i = 0; i < sizeof(notify_data); ++i)
            {
              notify_data[i] = i % 0xff;
            }
            // the size of notify_data[] need less than MTU size
            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                        sizeof(notify_data), notify_data, false);
          }
        }
        else if (descr_value == 0x0002)
        {
          if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
          {
            ESP_LOGI(GATTS_TAG, "Indication enable");
            uint8_t indicate_data[15];
            for (int i = 0; i < sizeof(indicate_data); ++i)
            {
              indicate_data[i] = i % 0xff;
            }
            // the size of indicate_data[] need less than MTU size
            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                        sizeof(indicate_data), indicate_data, true);
          }
        }
        else if (descr_value == 0x0000)
        {
          ESP_LOGI(GATTS_TAG, "Notification/Indication disable");
        }
        else
        {
          ESP_LOGE(GATTS_TAG, "Unknown descriptor value");
          ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
        }
      }
    }
    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
    break;
  }
  case ESP_GATTS_EXEC_WRITE_EVT:
    ESP_LOGI(GATTS_TAG, "Execute write");
    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
    // example_exec_write_event_env(&a_prepare_write_env, param);
    break;
  case ESP_GATTS_MTU_EVT:
    ESP_LOGI(GATTS_TAG, "MTU exchange, conn_id %d, MTU %d", param->mtu.conn_id, param->mtu.mtu);
    break;
  case ESP_GATTS_UNREG_EVT:
    break;
  case ESP_GATTS_CREATE_EVT:
    ESP_LOGI(GATTS_TAG, "Service create, status %d, service_handle %d", param->create.status, param->create.service_handle);
    gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
    gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
    gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

    esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
    a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_INDICATE;
    esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                    a_property,
                                                    &gatts_demo_char1_val, NULL);
    if (add_char_ret)
    {
      ESP_LOGE(GATTS_TAG, "add char failed, error code =%x", add_char_ret);
    }
    break;
  case ESP_GATTS_ADD_INCL_SRVC_EVT:
    break;
  case ESP_GATTS_ADD_CHAR_EVT:
  {
    uint16_t length = 0;
    const uint8_t *prf_char;

    ESP_LOGI(GATTS_TAG, "Characteristic add, status %d, attr_handle %d, service_handle %d",
             param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
    gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
    gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
    gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
    esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
    if (get_attr_ret == ESP_FAIL)
    {
      ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
    }

    ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x", length);
    for (int i = 0; i < length; i++)
    {
      ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x", i, prf_char[i]);
    }
    esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                           ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
    if (add_descr_ret)
    {
      ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
    }
    break;
  }
  case ESP_GATTS_ADD_CHAR_DESCR_EVT:
    gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
    ESP_LOGI(GATTS_TAG, "Descriptor add, status %d, attr_handle %d, service_handle %d",
             param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
    break;
  case ESP_GATTS_DELETE_EVT:
    ESP_LOGI(GATTS_TAG, "DELETE_EVT");
    break;
  case ESP_GATTS_START_EVT:
    ESP_LOGI(GATTS_TAG, "Service start, status %d, service_handle %d",
             param->start.status, param->start.service_handle);
    break;
  case ESP_GATTS_STOP_EVT:
    break;
  case ESP_GATTS_CONNECT_EVT:
  {
    esp_ble_conn_update_params_t conn_params = {0};
    memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
    conn_params.latency = 0;
    conn_params.max_int = 0x10; // max_int = 0x10*1.25ms = 20ms
    conn_params.min_int = 0x08; // min_int = 0x05*1.25ms = 10ms
    conn_params.timeout = 200;  // timeout = 400*10ms = 4000ms
    ESP_LOGI(GATTS_TAG, "Connected, conn_id %u, remote " ESP_BD_ADDR_STR "",
             param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
    gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
    // start sent the update connection parameters to the peer device.
    esp_ble_gap_update_conn_params(&conn_params);



    break;
  }
  case ESP_GATTS_DISCONNECT_EVT:
    ESP_LOGI(GATTS_TAG, "Disconnected, remote " ESP_BD_ADDR_STR ", reason 0x%02x",
             ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
    esp_ble_gap_start_advertising(&adv_params);
    esp_timer_stop(soil_meter_timer);
    break;
  case ESP_GATTS_CONF_EVT:
    ESP_LOGI(GATTS_TAG, "Confirm receive, status %d, attr_handle %d", param->conf.status, param->conf.handle);
    if (param->conf.status != ESP_GATT_OK)
    {
      ESP_LOG_BUFFER_HEX(GATTS_TAG, param->conf.value, param->conf.len);
    }
    break;
  case ESP_GATTS_OPEN_EVT:
  case ESP_GATTS_CANCEL_OPEN_EVT:
  case ESP_GATTS_CLOSE_EVT:
  case ESP_GATTS_LISTEN_EVT:
  case ESP_GATTS_CONGEST_EVT:
  default:
    break;
  }
}

static void soil_meter_callback(void *arg)
{
  ESP_LOGE(GATTS_TAG, "Timer goes off");
  uint8_t notify_data[15];
  for (int i = 0; i < sizeof(notify_data); ++i)
  {
    notify_data[i] = esp_random() % 0xff;
  }
  esp_ble_gatts_send_indicate(gatt_if, conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle, sizeof(notify_data), notify_data, true);
}

void app_main(void)
{
  printf("Hello world!\n");

  /* Print chip information */
  esp_chip_info_t chip_info;
  uint32_t flash_size;
  esp_chip_info(&chip_info);
  printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
         CONFIG_IDF_TARGET,
         chip_info.cores,
         (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
         (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
         (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

  unsigned major_rev = chip_info.revision / 100;
  unsigned minor_rev = chip_info.revision % 100;
  printf("silicon revision v%d.%d, ", major_rev, minor_rev);
  if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
  {
    printf("Get flash size failed");
    return;
  }

  printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

  printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

  fflush(stdout);

  esp_err_t ret;
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);

  if (ret)
  {
    ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
    return;
  }
  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret)
  {
    ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_init();
  if (ret)
  {
    ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_enable();
  if (ret)
  {
    ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_ble_gatts_register_callback(gatts_event_handler);
  if (ret)
  {
    ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
    return;
  }
  ret = esp_ble_gap_register_callback(gap_event_handler);
  if (ret)
  {
    ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
    return;
  }

  ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
  if (ret)
  {
    ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
    return;
  }

  esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
  if (local_mtu_ret)
  {
    ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    return;
  }

  ret = esp_timer_create(&soil_meter_timer_args, &soil_meter_timer);
  if (ret)
  {
    ESP_LOGE(GATTS_TAG, "unable to register a timer, error code = %x", ret);
    return;
  }

  return;
}
