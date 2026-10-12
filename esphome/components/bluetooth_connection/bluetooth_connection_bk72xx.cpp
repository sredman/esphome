#include "bluetooth_connection_bk72xx.h"

#if defined(USE_BK72XX_BLE) && defined(USE_BLE_GATT_CLIENT)

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

extern "C" {
#include "prf.h"
#include "ble_api.h"
#include "ble_api_5_x.h"
#include "app_ble.h"
#include "app_ble_init.h"
#include "app_sdp.h"
#include "sdp_comm.h"
#include "sdp_comm_pub.h"
#include "gap.h"
#include "gapm_task.h"
#include "kernel_task.h"

extern struct sdp_env_init_t sdp_env_init;
}

namespace esphome::bluetooth_connection {

ESPHOME_LOG_TAG(TAG, "bluetooth_connection_bk72xx");

using ble_device_base::ESPBTUUID;

static constexpr int GATT_ERR_BUSY = ble_device_base::GATT_ERR_UNLIKELY;
static constexpr int ATT_ERR_INVALID_ATTR_VALUE_LEN = 0x0D;

static constexpr uint8_t HCI_REASON_CONNECTION_TIMEOUT = 0x08;
static constexpr uint8_t HCI_REASON_REMOTE_USER_TERMINATED = 0x13;
static constexpr uint8_t HCI_REASON_LOCAL_HOST_TERMINATED = 0x16;

static constexpr uint32_t CONNECT_TIMEOUT_MS = 20000;
static constexpr uint32_t MTU_TIMEOUT_MS = 1000;
static constexpr uint32_t DISCONNECT_TIMEOUT_MS = 5000;
static constexpr uint32_t DISCOVERY_TIMEOUT_MS = 15000;

static BK72xxGattClient *s_instance = nullptr;

void BK72xxGattClient::setup() {
  s_instance = this;
  if (!this->control_pool_.warm() || !this->data_pool_.warm()) {
    ESP_LOGE(TAG, "GATT event pool warm-up failed");
    this->mark_failed();
    return;
  }
  this->parent_->set_notice_listener(this);
  register_app_sdp_charac_callback(reinterpret_cast<app_sdp_charac_callback>(&BK72xxGattClient::sdp_charac_callback));

#ifdef USE_OTA_STATE_LISTENER
  ota::get_global_ota_callback()->add_global_state_listener(this);
#endif
}

float BK72xxGattClient::get_setup_priority() const { return setup_priority::AFTER_WIFI - 1.0f; }

void BK72xxGattClient::dump_config() { ESP_LOGCONFIG(TAG, "BK72xx GATT Client:"); }

#ifdef USE_OTA_STATE_LISTENER
void BK72xxGattClient::on_ota_global_state(ota::OTAState state, float progress, uint8_t error,
                                           ota::OTAComponent *comp) {
  if (state == ota::OTA_STARTED && this->state_ != EngineState::IDLE) {
    this->gatt_disconnect();
  }
}
#endif

void BK72xxGattClient::sdp_charac_callback(uint8_t type, uint8_t conidx, uint16_t hdl, uint16_t len, uint8_t *data) {
  if (s_instance == nullptr)
    return;

  if (type == CHARAC_READ) {
    BK72xxGattDataEvent *event = s_instance->data_pool_.allocate();
    if (event != nullptr) {
      event->type = BK72xxGattDataEvent::READ_VALUE;
      event->handle = hdl;
      event->len = (len > BK72XX_GATT_MAX_ATTR_LEN) ? BK72XX_GATT_MAX_ATTR_LEN : len;
      if (data != nullptr && event->len > 0)
        memcpy(event->data, data, event->len);
      s_instance->data_queue_.push(event);
    }
  } else if (type == CHARAC_NOTIFY || type == CHARAC_INDICATE) {
    BK72xxGattDataEvent *event = s_instance->data_pool_.allocate();
    if (event != nullptr) {
      event->type = (type == CHARAC_NOTIFY) ? BK72xxGattDataEvent::NOTIFY_VALUE : BK72xxGattDataEvent::INDICATE_VALUE;
      event->handle = hdl;
      event->len = (len > BK72XX_GATT_MAX_ATTR_LEN) ? BK72XX_GATT_MAX_ATTR_LEN : len;
      if (data != nullptr && event->len > 0)
        memcpy(event->data, data, event->len);
      s_instance->data_queue_.push(event);
    }
  } else if (type == CHARAC_READ_DONE) {
    BK72xxGattControlEvent *event = s_instance->control_pool_.allocate();
    if (event != nullptr) {
      event->type = BK72xxGattControlEvent::READ_DONE;
      event->status = 0;
      event->value = hdl;
      s_instance->control_queue_.push(event);
    }
  } else if (type == CHARAC_WRITE_DONE) {
    BK72xxGattControlEvent *event = s_instance->control_pool_.allocate();
    if (event != nullptr) {
      event->type = BK72xxGattControlEvent::WRITE_DONE;
      event->status = 0;
      event->value = hdl;
      s_instance->control_queue_.push(event);
    }
  }
}

void BK72xxGattClient::on_ble_notice(uint8_t notice, void *param) {
  if (notice == BLE_5_INIT_CONNECT_EVENT) {
    BK72xxGattControlEvent *event = this->control_pool_.allocate();
    if (event != nullptr) {
      event->type = BK72xxGattControlEvent::CONNECTED;
      event->status = 0;
      event->value = 0;
      this->control_queue_.push(event);
    }
  } else if (notice == BLE_5_INIT_DISCONNECT_EVENT) {
    const discon_ind_t *info = reinterpret_cast<const discon_ind_t *>(param);
    uint8_t reason = (info != nullptr) ? info->reason : HCI_REASON_REMOTE_USER_TERMINATED;
    BK72xxGattControlEvent *event = this->control_pool_.allocate();
    if (event != nullptr) {
      event->type = BK72xxGattControlEvent::DISCONNECTED;
      event->status = reason;
      event->value = 0;
      this->control_queue_.push(event);
    }
  } else if (notice == BLE_5_INIT_CONNECT_FAILED_EVENT) {
    const discon_ind_t *info = reinterpret_cast<const discon_ind_t *>(param);
    uint8_t reason = (info != nullptr) ? info->reason : HCI_REASON_CONNECTION_TIMEOUT;
    BK72xxGattControlEvent *event = this->control_pool_.allocate();
    if (event != nullptr) {
      event->type = BK72xxGattControlEvent::DISCONNECTED;
      event->status = reason;
      event->value = 0;
      this->control_queue_.push(event);
    }
  } else if (notice == BLE_5_MTU_CHANGE) {
    const mtu_change_t *info = reinterpret_cast<const mtu_change_t *>(param);
    uint16_t mtu = (info != nullptr) ? info->mtu_size : 23;
    BK72xxGattControlEvent *event = this->control_pool_.allocate();
    if (event != nullptr) {
      event->type = BK72xxGattControlEvent::MTU_CHANGED;
      event->status = 0;
      event->value = mtu;
      this->control_queue_.push(event);
    }
  }
}

void BK72xxGattClient::loop() {
  BK72xxGattControlEvent *control;
  while ((control = this->control_queue_.pop()) != nullptr) {
    BK72xxGattControlEvent copy = *control;
    this->control_pool_.release(control);
    this->handle_control_event_(copy);
  }

  BK72xxGattDataEvent *data;
  while ((data = this->data_queue_.pop()) != nullptr) {
    BK72xxGattDataEvent copy = *data;
    this->data_pool_.release(data);
    this->handle_data_event_(copy);
  }

  const uint32_t now = millis();
  if (this->state_ == EngineState::CONNECT_CREATE_INIT) {
    if (BLE_APP_MASTER_GET_IDX_STATE(0) == APP_INIT_STATE_CREATED) {
      this->state_ = EngineState::CONNECTING;
      bk_ble_init_start_conn(0, nullptr);
    } else if (now - this->connect_started_ > CONNECT_TIMEOUT_MS) {
      this->fail_connection_(HCI_REASON_CONNECTION_TIMEOUT);
    }
  } else if (this->state_ == EngineState::CONNECTING) {
    if (now - this->connect_started_ > CONNECT_TIMEOUT_MS) {
      this->fail_connection_(HCI_REASON_CONNECTION_TIMEOUT);
    }
  } else if (this->state_ == EngineState::MTU_EXCHANGE) {
    if (now - this->mtu_exchange_started_ > MTU_TIMEOUT_MS) {
      this->state_ = EngineState::READY;
      this->listener_->on_connection_state(true, this->mtu_, 0);
    }
  } else if (this->state_ == EngineState::DISCONNECTING) {
    if (now - this->disconnecting_started_ > DISCONNECT_TIMEOUT_MS) {
      this->handle_disconnected_(HCI_REASON_LOCAL_HOST_TERMINATED);
    }
  }

  if (this->discovery_pending_) {
    if (kernel_state_get(KERNEL_BUILD_ID(TASK_BLE_APP, 0)) == APPC_SERVICE_CONNECTED) {
      this->discovery_pending_ = false;
      this->listener_->on_service_discovery_done(0);
    } else if (now - this->discovery_started_ > DISCOVERY_TIMEOUT_MS) {
      this->discovery_pending_ = false;
      this->listener_->on_service_discovery_done(-1);
    }
  }
}

void BK72xxGattClient::handle_control_event_(const BK72xxGattControlEvent &event) {
  switch (event.type) {
    case BK72xxGattControlEvent::CONNECTED:
      this->handle_connected_(event.status);
      break;
    case BK72xxGattControlEvent::DISCONNECTED:
      this->handle_disconnected_(event.status);
      break;
    case BK72xxGattControlEvent::MTU_CHANGED:
      this->mtu_ = event.value;
      if (this->state_ == EngineState::MTU_EXCHANGE) {
        this->state_ = EngineState::READY;
        this->listener_->on_connection_state(true, this->mtu_, 0);
      }
      break;
    case BK72xxGattControlEvent::READ_DONE:
      if (this->op_type_ == OpType::READ_CHAR || this->op_type_ == OpType::READ_DESC) {
        this->op_type_ = OpType::NONE;
        this->listener_->on_read_result(this->op_handle_, this->op_buffer_, this->op_len_, 0);
      }
      break;
    case BK72xxGattControlEvent::WRITE_DONE:
      if (this->op_type_ == OpType::WRITE_CHAR || this->op_type_ == OpType::WRITE_DESC) {
        uint16_t hdl = this->op_handle_;
        this->op_type_ = OpType::NONE;
        this->listener_->on_write_result(hdl, 0);
      }
      break;
  }
}

void BK72xxGattClient::handle_data_event_(const BK72xxGattDataEvent &event) {
  if (event.type == BK72xxGattDataEvent::READ_VALUE) {
    if (this->op_type_ == OpType::READ_CHAR || this->op_type_ == OpType::READ_DESC) {
      this->op_len_ = event.len;
      if (event.len > 0)
        memcpy(this->op_buffer_, event.data, event.len);
    }
  } else if (event.type == BK72xxGattDataEvent::NOTIFY_VALUE || event.type == BK72xxGattDataEvent::INDICATE_VALUE) {
    if (this->notify_subscribed_(event.handle)) {
      this->listener_->on_notify_data(event.handle, event.data, event.len);
    }
  }
}

void BK72xxGattClient::handle_connected_(uint8_t status) {
  if (this->state_ != EngineState::CONNECTING && this->state_ != EngineState::CONNECT_CREATE_INIT) {
    return;
  }
  this->release_scan_inhibit_();
  if (status != 0) {
    this->fail_connection_(status);
    return;
  }
  if (this->cancel_requested_) {
    this->state_ = EngineState::DISCONNECTING;
    this->disconnecting_started_ = millis();
    bk_ble_disconnect(0);
    return;
  }
  this->state_ = EngineState::MTU_EXCHANGE;
  this->mtu_exchange_started_ = millis();
  this->mtu_ = 23;
  bk_ble_gatt_mtu_change(0);
}

void BK72xxGattClient::handle_disconnected_(uint8_t reason) {
  if (this->state_ == EngineState::IDLE)
    return;
  this->release_scan_inhibit_();
  this->release_services();
  this->state_ = EngineState::IDLE;
  this->op_type_ = OpType::NONE;
  this->discovery_pending_ = false;
  this->cancel_requested_ = false;
  this->notify_subscription_count_ = 0;
  this->listener_->on_connection_state(false, 0, reason);
}

void BK72xxGattClient::fail_connection_(uint8_t reason) {
  this->release_scan_inhibit_();
  this->state_ = EngineState::IDLE;
  this->op_type_ = OpType::NONE;
  this->cancel_requested_ = false;
  this->listener_->on_connection_state(false, 0, reason);
}

void BK72xxGattClient::release_scan_inhibit_() {
  if (this->holds_scan_inhibit_) {
    this->holds_scan_inhibit_ = false;
    this->parent_->release_scan_inhibit();
  }
}

bool BK72xxGattClient::notify_subscribed_(uint16_t handle) const {
  for (uint8_t i = 0; i < this->notify_subscription_count_; i++) {
    if (this->notify_subscriptions_[i] == handle)
      return true;
  }
  return false;
}

int BK72xxGattClient::connect(uint64_t address, uint8_t addr_type) {
  if (this->state_ != EngineState::IDLE) {
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  }
  this->peer_address_ = address;
  this->peer_addr_type_ = addr_type;
  this->cancel_requested_ = false;

  this->parent_->inhibit_scan();
  this->holds_scan_inhibit_ = true;

  struct bd_addr bdaddr;
  for (int i = 0; i < 6; i++) {
    bdaddr.addr[i] = (address >> (i * 8)) & 0xFF;
  }

  bk_ble_init_set_connect_dev_addr(0, &bdaddr, addr_type);

  this->connect_started_ = millis();

  if (BLE_APP_MASTER_GET_IDX_STATE(0) == APP_INIT_STATE_CREATED) {
    this->state_ = EngineState::CONNECTING;
    bk_ble_init_start_conn(0, nullptr);
  } else {
    this->state_ = EngineState::CONNECT_CREATE_INIT;
    bk_ble_create_init(0, 0, 0, 0, nullptr);
  }
  return 0;
}

int BK72xxGattClient::gatt_disconnect() {
  if (this->state_ == EngineState::IDLE) {
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  }
  if (this->state_ == EngineState::CONNECT_CREATE_INIT || this->state_ == EngineState::CONNECTING) {
    this->cancel_requested_ = true;
    bk_ble_init_stop_conn(0, nullptr);
    this->release_scan_inhibit_();
    this->state_ = EngineState::DISCONNECTING;
    this->disconnecting_started_ = millis();
    return 0;
  }
  this->state_ = EngineState::DISCONNECTING;
  this->disconnecting_started_ = millis();
  this->release_scan_inhibit_();
  bk_ble_disconnect(0);
  return 0;
}

int BK72xxGattClient::discover_services() {
  if (this->state_ != EngineState::READY) {
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  }
  if (kernel_state_get(KERNEL_BUILD_ID(TASK_BLE_APP, 0)) == APPC_SERVICE_CONNECTED) {
    this->listener_->on_service_discovery_done(0);
    return 0;
  }
  this->discovery_pending_ = true;
  this->discovery_started_ = millis();
  sdp_discover_all_service(0);
  return 0;
}

int BK72xxGattClient::read_characteristic(uint16_t handle) {
  if (this->state_ != EngineState::READY) {
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  }
  if (this->op_type_ != OpType::NONE) {
    return GATT_ERR_BUSY;
  }
  this->op_type_ = OpType::READ_CHAR;
  this->op_handle_ = handle;
  this->op_len_ = 0;
  uint8_t ret = appm_read_service_data_by_handle_req(0, handle);
  if (ret != 0) {
    this->op_type_ = OpType::NONE;
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  }
  return 0;
}

int BK72xxGattClient::write_characteristic(uint16_t handle, const uint8_t *data, uint16_t len, bool response) {
  if (this->state_ != EngineState::READY) {
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  }
  if (len > BK72XX_GATT_MAX_ATTR_LEN) {
    return ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }
  if (this->op_type_ != OpType::NONE) {
    return GATT_ERR_BUSY;
  }
  this->op_type_ = OpType::WRITE_CHAR;
  this->op_handle_ = handle;
  uint8_t ret = appc_write_service_data_req(0, handle, len, const_cast<uint8_t *>(data));
  if (ret != 0) {
    this->op_type_ = OpType::NONE;
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  }
  if (!response) {
    this->op_type_ = OpType::NONE;
    this->listener_->on_write_result(handle, 0);
  }
  return 0;
}

int BK72xxGattClient::read_descriptor(uint16_t handle) {
  if (this->state_ != EngineState::READY) {
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  }
  if (this->op_type_ != OpType::NONE) {
    return GATT_ERR_BUSY;
  }
  this->op_type_ = OpType::READ_DESC;
  this->op_handle_ = handle;
  this->op_len_ = 0;
  uint8_t ret = appm_read_service_data_by_handle_req(0, handle);
  if (ret != 0) {
    this->op_type_ = OpType::NONE;
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  }
  return 0;
}

int BK72xxGattClient::write_descriptor(uint16_t handle, const uint8_t *data, uint16_t len) {
  if (this->state_ != EngineState::READY) {
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  }
  if (this->op_type_ != OpType::NONE) {
    return GATT_ERR_BUSY;
  }
  this->op_type_ = OpType::WRITE_DESC;
  this->op_handle_ = handle;

  uint8_t ret;
  if (len == 2) {
    uint16_t ntf_cfg = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
    ret = appc_write_service_ntf_cfg_req(0, handle, ntf_cfg, 0);
  } else {
    ret = appc_write_service_data_req(0, handle, len, const_cast<uint8_t *>(data));
  }
  if (ret != 0) {
    this->op_type_ = OpType::NONE;
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  }
  return 0;
}

int BK72xxGattClient::notify_characteristic(uint16_t handle, bool enable) {
  if (this->state_ != EngineState::READY) {
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  }
  if (enable) {
    if (!this->notify_subscribed_(handle)) {
      if (this->notify_subscription_count_ >= BK72XX_GATT_MAX_NOTIFY_SUBSCRIPTIONS) {
        return ble_device_base::GATT_ERR_NO_MEMORY;
      }
      this->notify_subscriptions_[this->notify_subscription_count_++] = handle;
    }
  } else {
    for (uint8_t i = 0; i < this->notify_subscription_count_; i++) {
      if (this->notify_subscriptions_[i] == handle) {
        this->notify_subscriptions_[i] = this->notify_subscriptions_[--this->notify_subscription_count_];
        break;
      }
    }
  }
  this->listener_->on_notify_state(handle, enable, 0);
  return 0;
}

int BK72xxGattClient::pair() {
  if (this->state_ != EngineState::READY) {
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  }
  this->listener_->on_pairing_result(0);
  return 0;
}

int BK72xxGattClient::update_connection_params(uint16_t min_interval, uint16_t max_interval, uint16_t latency,
                                               uint16_t timeout) {
  if (this->state_ != EngineState::READY) {
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  }
  ble_err_t err = bk_ble_update_param(0, min_interval, max_interval, latency, timeout);
  return err == ERR_SUCCESS ? 0 : -1;
}

ble_device_base::GattServiceTable BK72xxGattClient::get_service_table() {
  if (this->arena_ == nullptr) {
    this->arena_ = new (std::nothrow) ServiceArena();  // NOLINT
  }
  ble_device_base::GattServiceTable table;
  if (this->arena_ == nullptr)
    return table;

  uint16_t s_idx = 0;
  uint16_t c_idx = 0;
  uint16_t d_idx = 0;

  for (int i = 0; i < BLE_NB_PROFILES_ADD_MAX; i++) {
    if (sdp_env_init.used_status[i] != USED_STATUS)
      continue;
    if (sdp_env_init.sdp_env[i].conidx != 0)
      continue;
    struct prf_sdp_db_env *db_env = sdp_env_init.sdp_env[i].prf_db_env;
    if (db_env == nullptr || db_env->sdp_cont == nullptr)
      continue;
    struct sdp_content *sdp_cont = db_env->sdp_cont;
    if (s_idx >= BK72XX_GATT_MAX_SERVICES)
      break;

    auto &svc = this->arena_->services[s_idx++];
    if (sdp_cont->svc.uuid_len == 2) {
      uint16_t u16 = static_cast<uint16_t>(sdp_cont->svc.uuid[0]) | (static_cast<uint16_t>(sdp_cont->svc.uuid[1]) << 8);
      svc.uuid = ESPBTUUID::from_uint16(u16);
    } else {
      svc.uuid = ESPBTUUID::from_raw(sdp_cont->svc.uuid);
    }
    svc.start_handle = sdp_cont->svc.shdl;
    svc.end_handle = sdp_cont->svc.ehdl;
    svc.first_characteristic = c_idx;
    svc.characteristic_count = 0;

    for (int j = 0; j < sdp_cont->chars_nb && c_idx < BK72XX_GATT_MAX_CHARACTERISTICS; j++) {
      auto &c_info = sdp_cont->chars_descs_inf.chars_inf[j];
      auto &chr = this->arena_->characteristics[c_idx++];
      svc.characteristic_count++;

      if (c_info.uuid_len == 2) {
        uint16_t u16 = static_cast<uint16_t>(c_info.uuid[0]) | (static_cast<uint16_t>(c_info.uuid[1]) << 8);
        chr.uuid = ESPBTUUID::from_uint16(u16);
      } else {
        chr.uuid = ESPBTUUID::from_raw(c_info.uuid);
      }
      chr.value_handle = c_info.val_hdl;
      chr.end_handle = c_info.val_hdl + c_info.char_ehdl_off;
      chr.properties = c_info.prop;
      chr.first_descriptor = d_idx;
      chr.descriptor_count = 0;

      for (int k = 0; k < sdp_cont->descs_nb && d_idx < BK72XX_GATT_MAX_DESCRIPTORS; k++) {
        auto &d_info = sdp_cont->chars_descs_inf.descs_inf[k];
        if (d_info.char_code == j) {
          auto &dsc = this->arena_->descriptors[d_idx++];
          chr.descriptor_count++;
          if (d_info.uuid_len == 2) {
            uint16_t u16 = static_cast<uint16_t>(d_info.uuid[0]) | (static_cast<uint16_t>(d_info.uuid[1]) << 8);
            dsc.uuid = ESPBTUUID::from_uint16(u16);
          } else {
            dsc.uuid = ESPBTUUID::from_raw(d_info.uuid);
          }
          dsc.handle = d_info.desc_hdl;
        }
      }
    }
  }

  this->service_count_ = s_idx;
  this->char_count_ = c_idx;
  this->desc_count_ = d_idx;

  table.services = this->arena_->services;
  table.characteristics = this->arena_->characteristics;
  table.descriptors = this->arena_->descriptors;
  table.service_count = this->service_count_;
  table.characteristic_count = this->char_count_;
  table.descriptor_count = this->desc_count_;
  return table;
}

void BK72xxGattClient::release_services() {
  if (this->arena_ != nullptr) {
    delete this->arena_;
    this->arena_ = nullptr;
  }
  this->service_count_ = 0;
  this->char_count_ = 0;
  this->desc_count_ = 0;
}

}  // namespace esphome::bluetooth_connection

#endif  // USE_BK72XX_BLE && USE_BLE_GATT_CLIENT
