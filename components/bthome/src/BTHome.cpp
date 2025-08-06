
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <host/ble_hs.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <string>

#include <esp_random.h>
#include <esp_mac.h>

using std::string;

#include "BTHome.h"

#define TAG "BTHome"

bool nimble_synced = false;

static void ble_on_sync(void)
{
  nimble_synced = true;
}

static void ble_on_reset(int reason)
{
  ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
  switch (event->type)
  {
  case BLE_GAP_EVENT_ADV_COMPLETE:
    ESP_LOGI(TAG, "advertisement complete");
    break;
  }

  return 0;
}

void BTHome::end(void)
{
  int rc = 0;
  if (nimble_initialized)
  {
    rc = nimble_port_stop();
    if (rc == 0)
    {
      nimble_port_deinit();

      nimble_initialized = false;
      // m_synced      = false;
    }
  }
}

static void nimble_host_task(void *param)
{
  /* Task entry log */
  ESP_LOGI(TAG, "nimble host task has been started!");

  /* This function won't return until nimble_port_stop() is executed */
  nimble_port_run();

  /* Clean up at exit */
  vTaskDelete(NULL);
}

void BTHome::begin(string dname, bool encryption, uint8_t const *const key, bool trigger_based_device)
{
  /* Initialize NVS — it is used to store PHY calibration data */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ret = nimble_port_init();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to init nimble %d \n", ret);
    return;
  }

  /* Initialize the NimBLE host configuration */
  ble_hs_cfg.sync_cb = ble_on_sync;
  ble_hs_cfg.reset_cb = ble_on_reset;

  setDeviceName(dname);

  if (encryption)
  {
    this->m_encryptEnable = true;
    this->m_encryptCount = esp_random() % 0x427;
    memcpy(bindKey, key, sizeof(uint8_t) * BIND_KEY_LEN);
    mbedtls_ccm_init(&this->m_encryptCTX);
    mbedtls_ccm_setkey(&this->m_encryptCTX, MBEDTLS_CIPHER_ID_AES, bindKey, BIND_KEY_LEN * 8);
  }
  else
  {
    this->m_encryptEnable = false;
  }

  this->m_triggerdevice = trigger_based_device;

  resetMeasurement();

  xTaskCreate(nimble_host_task, "NimBLE Host", 4 * 1024, NULL, 5, NULL);
}

void BTHome::begin(string dname, bool encryption, string key, bool trigger_based_device)
{
  uint8_t bind_key[BIND_KEY_LEN];
  for (uint8_t i = 0; i < BIND_KEY_LEN; i++)
  {
    // convert hex string to binary
    bind_key[i] = strtol(key.substr(i * 2, 2).c_str(), NULL, 16);
  }
  begin(dname, encryption, bind_key, trigger_based_device);
}

void BTHome::setDeviceName(string dname)
{
  if (!dname.empty())
    this->dev_name = dname;
}

void BTHome::resetMeasurement()
{
  this->m_sensorDataIdx = 0;
  this->last_object_id = 0;
  this->m_sortEnable = false;
}

void BTHome::addMeasurement_state(uint8_t sensor_id, uint8_t state, uint8_t steps)
{
  if ((this->m_sensorDataIdx + 2 + (steps > 0 ? 1 : 0)) <= (MEASUREMENT_MAX_LEN - (this->m_encryptEnable ? 8 : 0)))
  {
    this->m_sensorData[this->m_sensorDataIdx] = static_cast<uint8_t>(sensor_id & 0xff);
    this->m_sensorDataIdx++;
    this->m_sensorData[this->m_sensorDataIdx] = static_cast<uint8_t>(state & 0xff);
    this->m_sensorDataIdx++;
    if (steps > 0)
    {
      this->m_sensorData[this->m_sensorDataIdx] = static_cast<uint8_t>(steps & 0xff);
      this->m_sensorDataIdx++;
    }
    if (!this->m_sortEnable)
    {
      if (sensor_id < this->last_object_id)
        this->m_sortEnable = true;
    }
    last_object_id = sensor_id;
  }
  else
  {
    sendPacket();
    addMeasurement_state(sensor_id, state, steps);
  }
}

void BTHome::addMeasurement(uint8_t sensor_id, uint64_t value)
{
  uint8_t size = getByteNumber(sensor_id);
  uint16_t factor = getFactor(sensor_id);
  if ((this->m_sensorDataIdx + size + 1) <= (MEASUREMENT_MAX_LEN - (this->m_encryptEnable ? 8 : 0)))
  {
    this->m_sensorData[this->m_sensorDataIdx] = static_cast<uint8_t>(sensor_id & 0xff);
    this->m_sensorDataIdx++;
    for (uint8_t i = 0; i < size; i++)
    {
      this->m_sensorData[this->m_sensorDataIdx] = static_cast<uint8_t>(((value * factor) >> (8 * i)) & 0xff);
      this->m_sensorDataIdx++;
    }
    if (!this->m_sortEnable)
    {
      if (sensor_id < this->last_object_id)
        this->m_sortEnable = true;
    }
    last_object_id = sensor_id;
  }
  else
  {
    sendPacket();
    addMeasurement(sensor_id, value);
  }
}

void BTHome::addMeasurement(uint8_t sensor_id, float value)
{
  uint8_t size = getByteNumber(sensor_id);
  uint16_t factor = getFactor(sensor_id);
  if ((this->m_sensorDataIdx + size + 1) <= (MEASUREMENT_MAX_LEN - (this->m_encryptEnable ? 8 : 0)))
  {
    uint64_t value2 = static_cast<uint64_t>(value * factor);
    this->m_sensorData[this->m_sensorDataIdx] = static_cast<uint8_t>(sensor_id & 0xff);
    this->m_sensorDataIdx++;
    for (uint8_t i = 0; i < size; i++)
    {
      this->m_sensorData[this->m_sensorDataIdx] = static_cast<uint8_t>((value2 >> (8 * i)) & 0xff);
      this->m_sensorDataIdx++;
    }
    if (!this->m_sortEnable)
    {
      if (sensor_id < this->last_object_id)
        this->m_sortEnable = true;
    }
    last_object_id = sensor_id;
  }
  else
  {
    sendPacket();
    addMeasurement(sensor_id, value);
  }
}

void BTHome::sortSensorData()
{
  uint8_t i, j, k, data_block_num;

  struct DATA_BLOCK
  {
    uint8_t object_id;
    uint8_t data[4];
    uint8_t data_len;
  };
  struct DATA_BLOCK data_block[MEASUREMENT_MAX_LEN / 2 + 1];
  struct DATA_BLOCK temp_data_block;

  for (i = 0, j = 0, data_block_num = 0; j < this->m_sensorDataIdx; i++)
  {
    // copy the object id
    data_block[i].object_id = this->m_sensorData[j];
    data_block_num++;
    // copy the data length
    if (this->m_sensorData[j] == EVENT_DIMMER)
    {
      if (this->m_sensorData[j + 1] == EVENT_DIMMER_NONE)
        data_block[i].data_len = 1;
      else
        data_block[i].data_len = 2;
    }
    else
    {
      data_block[i].data_len = getByteNumber(this->m_sensorData[j]);
    }
    // copy the data
    for (k = 0; k < data_block[i].data_len; k++)
    {
      data_block[i].data[k] = this->m_sensorData[j + 1 + k];
    }
    // move to the next object id location
    j = j + data_block[i].data_len + 1;
  }

  if (data_block_num > 1)
  {
    // bubble sort
    for (i = 0; i < data_block_num - 1; i++)
    {
      for (j = 0; j < data_block_num - 1 - i; j++)
      {
        if (data_block[j].object_id > data_block[j + 1].object_id)
        {
          memcpy(&temp_data_block, &data_block[j], sizeof(struct DATA_BLOCK));
          memcpy(&data_block[j], &data_block[j + 1], sizeof(struct DATA_BLOCK));
          memcpy(&data_block[j + 1], &temp_data_block, sizeof(struct DATA_BLOCK));
        }
      }
    }
    // copy the new order to m_sensorData array
    for (i = 0, j = 0; i < data_block_num && j < this->m_sensorDataIdx; i++)
    {
      this->m_sensorData[j] = data_block[i].object_id;
      for (k = 0; k < data_block[i].data_len; k++)
      {
        this->m_sensorData[j + 1 + k] = data_block[i].data[k];
      }
      j = j + data_block[i].data_len + 1;
    }
  }
}

void BTHome::buildPaket()
{
  // the Object ids have to be applied in numerical order (from low to high)
  if (this->m_sortEnable)
    sortSensorData();

  adv_payload.clear();

  std::string serviceData = "";
  uint8_t i;

  // head
  adv_payload += FLAG1;
  adv_payload += FLAG2;
  adv_payload += FLAG3;

  serviceData += SERVICE_DATA; // DO NOT CHANGE -- Service Data - 16-bit UUID
  serviceData += UUID1;        // DO NOT CHANGE -- UUID
  serviceData += UUID2;        // DO NOT CHANGE -- UUID

  // The encryption
  if (this->m_encryptEnable)
  {
    if (this->m_triggerdevice)
      serviceData += ENCRYPT_TRIGGER_BASE;
    else
      serviceData += ENCRYPT;

    uint8_t ciphertext[BLE_ADVERT_MAX_LEN];
    uint8_t encryptionTag[MIC_LEN];

    // build Nonce
    uint8_t nonce[NONCE_LEN];
    uint8_t *countPtr = (uint8_t *)(&this->m_encryptCount);

    esp_read_mac(&nonce[0], ESP_MAC_BT);

    ESP_LOGI(TAG, "MAC: %02x:%02x:%02x:%02x:%02x:%02x", nonce[0], nonce[1], nonce[2], nonce[3], nonce[4], nonce[5]);

    nonce[6] = UUID1;
    nonce[7] = UUID2;
    if (this->m_triggerdevice)
      nonce[8] = ENCRYPT_TRIGGER_BASE;
    else
      nonce[8] = ENCRYPT;

    memcpy(&nonce[9], countPtr, 4);

    // encrypt sensorData
    mbedtls_ccm_encrypt_and_tag(&this->m_encryptCTX, this->m_sensorDataIdx, nonce, NONCE_LEN, 0, 0,
                                &this->m_sensorData[0], &ciphertext[0], encryptionTag,
                                MIC_LEN);

    for (i = 0; i < this->m_sensorDataIdx; i++)
    {
      serviceData += ciphertext[i];
    }
    // writeCounter
    serviceData += nonce[9];
    serviceData += nonce[10];
    serviceData += nonce[11];
    serviceData += nonce[12];
    this->m_encryptCount++;
    // writeMIC
    serviceData += encryptionTag[0];
    serviceData += encryptionTag[1];
    serviceData += encryptionTag[2];
    serviceData += encryptionTag[3];
  }
  else
  {
    if (this->m_triggerdevice)
      serviceData += NO_ENCRYPT_TRIGGER_BASE;
    else
      serviceData += NO_ENCRYPT;
    for (i = 0; i < this->m_sensorDataIdx; i++)
    {
      serviceData += this->m_sensorData[i]; // Add the sensor data to the Service Data
    }
  }

  uint8_t sd_length = serviceData.length(); // Generate the length of the Service Data
  adv_payload += sd_length;                 // Add the length of the Service Data
  adv_payload += serviceData;               // Finalize the packet

  adv_response_payload.clear();

  // fill the local name into oScanResponseData
  if (!this->dev_name.empty())
  {
    int dn_length = this->dev_name.length() + 1;
    if (dn_length > 28)
      dn_length = 28; // BLE_ADVERT_MAX_LEN - FLAG = 31 - 3

    char cdata[2];
    cdata[0] = dn_length;
    cdata[1] = BLE_HS_ADV_TYPE_COMP_NAME; // 0x09
    adv_response_payload.append(std::string(cdata, 2) + this->dev_name.substr(0, dn_length - 1).c_str());
  }
}

void BTHome::stop()
{
  // pAdvertising->stop();

  int rc = ble_gap_adv_stop();
  if (rc != 0 && rc != BLE_HS_EALREADY)
  {
    ESP_LOGE(TAG, "ble_gap_adv_stop rc=%d", rc);
  }
}

void BTHome::start(uint32_t duration)
{
  int rc;

  while (!nimble_synced)
  {
    vTaskDelay(pdMS_TO_TICKS(4));
  }

  if (ble_gap_adv_active())
  {
    ESP_LOGE(TAG, "Advertising already active");
    return;
  }

  rc = ble_gap_adv_set_data((uint8_t *)adv_payload.data(), adv_payload.length());

  if (rc != 0)
  {
    ESP_LOGE(TAG, "ble_gap_adv_set_data error: %d ", rc);
  }

  rc = ble_gap_adv_rsp_set_data(
      (uint8_t *)adv_response_payload.data(),
      adv_response_payload.length());

  if (rc != 0)
  {
    ESP_LOGE(TAG, "ble_gap_adv_rsp_set_data error: %d ", rc);
  }

  struct ble_gap_adv_params adv_params;

  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  // adv_params.itvl_min = 720;
  // adv_params.itvl_max = 720;

  rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, duration,
                         &adv_params, ble_gap_event, NULL);

  if (rc != 0)
  {
    ESP_LOGE(TAG, "ble_gap_adv_start error: %d ", rc);
  }
}

bool BTHome::isAdvertising()
{
  while (!nimble_synced)
  {
    vTaskDelay(pdMS_TO_TICKS(4));
  }

  return ble_gap_adv_active();
}

void BTHome::sendPacket(uint32_t delay_ms)
{
  if (this->m_sensorDataIdx > 0)
  {
    buildPaket();
    if (!isAdvertising())
      start(delay_ms);

    // TODO better wait for a signal instead of waiting
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    stop();
    resetMeasurement();
  }
}

uint8_t BTHome::getByteNumber(uint8_t sens)
{
  switch (sens)
  {
  case ID_PACKET:
  case ID_BATTERY:
  case ID_COUNT:
  case ID_HUMIDITY:
  case ID_MOISTURE:
  case ID_UV:
  case STATE_BATTERY_LOW:
  case STATE_BATTERY_CHARHING:
  case STATE_CO:
  case STATE_COLD:
  case STATE_CONNECTIVITY:
  case STATE_DOOR:
  case STATE_GARAGE_DOOR:
  case STATE_GAS_DETECTED:
  case STATE_GENERIC_BOOLEAN:
  case STATE_HEAT:
  case STATE_LIGHT:
  case STATE_LOCK:
  case STATE_MOISTURE:
  case STATE_MOTION:
  case STATE_MOVING:
  case STATE_OCCUPANCY:
  case STATE_OPENING:
  case STATE_PLUG:
  case STATE_POWER_ON:
  case STATE_PRESENCE:
  case STATE_PROBLEM:
  case STATE_RUNNING:
  case STATE_SAFETY:
  case STATE_SMOKE:
  case STATE_SOUND:
  case STATE_TAMPER:
  case STATE_VIBRATION:
  case STATE_WINDOW:
  case EVENT_BUTTON:
    return 1;
  case ID_DURATION:
  case ID_ENERGY:
  case ID_GAS:
  case ID_ILLUMINANCE:
  case ID_POWER:
  case ID_PRESSURE:
    return 3;
  case ID_COUNT4:
  case ID_ENERGY4:
  case ID_GAS4:
  case ID_VOLUME:
  case ID_WATER:
    return 4;
  default:
    return 2;
  }
}

uint16_t BTHome::getFactor(uint8_t sens)
{
  switch (sens)
  {
  case ID_DISTANCEM:
  case ID_ROTATION:
  case ID_TEMPERATURE:
  case ID_VOLTAGE1:
  case ID_VOLUME1:
  case ID_UV:
    return 10;
  case ID_DEWPOINT:
  case ID_HUMIDITY_PRECISE:
  case ID_ILLUMINANCE:
  case ID_MASS:
  case ID_MASSLB:
  case ID_MOISTURE_PRECISE:
  case ID_POWER:
  case ID_PRESSURE:
  case ID_SPD:
  case ID_TEMPERATURE_PRECISE:
    return 100;
  case ID_CURRENT:
  case ID_DURATION:
  case ID_ENERGY:
  case ID_ENERGY4:
  case ID_GAS:
  case ID_GAS4:
  case ID_VOLTAGE:
  case ID_VOLUME:
  case ID_VOLUMEFR:
  case ID_WATER:
    return 1000;
  default:
    return 1;
  }
}