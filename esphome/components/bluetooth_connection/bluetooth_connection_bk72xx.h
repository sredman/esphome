#pragma once

#include "esphome/core/defines.h"

#if defined(USE_BK72XX_BLE) && defined(USE_BLE_GATT_CLIENT)

#include "esphome/components/ble_device_base/ble_gatt_client.h"
#include "esphome/components/bk72xx_ble/bk72xx_ble.h"
#include "esphome/core/component.h"
#include "esphome/core/event_pool.h"
#include "esphome/core/helpers.h"
#include "esphome/core/lock_free_queue.h"

#ifdef USE_OTA_STATE_LISTENER
#include "esphome/components/ota/ota_backend.h"
#endif

#include <cstdint>

namespace esphome::bluetooth_connection {

static constexpr uint16_t BK72XX_GATT_MAX_SERVICES = 20;
static constexpr uint16_t BK72XX_GATT_MAX_CHARACTERISTICS = 64;
static constexpr uint16_t BK72XX_GATT_MAX_DESCRIPTORS = 64;

static constexpr uint8_t BK72XX_GATT_MAX_NOTIFY_SUBSCRIPTIONS = 16;
static constexpr uint16_t BK72XX_GATT_MAX_ATTR_LEN = 512;

struct BK72xxGattControlEvent {
  enum Type : uint8_t {
    CONNECTED,
    DISCONNECTED,
    MTU_CHANGED,
    READ_DONE,
    WRITE_DONE,
  };
  Type type;
  uint8_t status;
  uint16_t value;
  void release() {}
};

struct BK72xxGattDataEvent {
  enum Type : uint8_t {
    READ_VALUE,
    NOTIFY_VALUE,
    INDICATE_VALUE,
  };
  Type type;
  uint16_t handle;
  uint16_t len;
  uint8_t data[BK72XX_GATT_MAX_ATTR_LEN];
  void release() {}
};

static constexpr uint8_t BK72XX_GATT_CONTROL_QUEUE_SIZE = 8;
static constexpr uint8_t BK72XX_GATT_DATA_QUEUE_SIZE = 8;

class BK72xxGattClient final : public Component,
                               public Parented<bk72xx_ble::BK72xxBLE>,
                               public bk72xx_ble::BK72xxBLENoticeListener
#ifdef USE_OTA_STATE_LISTENER
    ,
                               public ota::OTAGlobalStateListener
#endif
{
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_listener(ble_device_base::GattClientListener *listener) { this->listener_ = listener; }

  // ---- ble_device_base::BLEGattConnection contract ----
  int connect(uint64_t address, uint8_t addr_type);
  int gatt_disconnect();
  bool cancel_gatt_disconnect() { return false; }
  int discover_services();
  int read_characteristic(uint16_t handle);
  int write_characteristic(uint16_t handle, const uint8_t *data, uint16_t len, bool response);
  int read_descriptor(uint16_t handle);
  int write_descriptor(uint16_t handle, const uint8_t *data, uint16_t len);
  int notify_characteristic(uint16_t handle, bool enable);
  int pair();
  int update_connection_params(uint16_t min_interval, uint16_t max_interval, uint16_t latency, uint16_t timeout);
  ble_device_base::GattServiceTable get_service_table();
  void set_connection_type(ble_device_base::ConnectionType ct) {}
  void release_services();

  // BK72xxBLENoticeListener contract
  void on_ble_notice(uint8_t notice, void *param) override;

#ifdef USE_OTA_STATE_LISTENER
  void on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) override;
#endif

 protected:
  enum class EngineState : uint8_t {
    IDLE,
    CONNECT_CREATE_INIT,
    CONNECTING,
    MTU_EXCHANGE,
    READY,
    DISCONNECTING,
  };

  enum class OpType : uint8_t {
    NONE,
    READ_CHAR,
    WRITE_CHAR,
    READ_DESC,
    WRITE_DESC,
  };

  struct ServiceArena {
    ble_device_base::GattService services[BK72XX_GATT_MAX_SERVICES];
    ble_device_base::GattCharacteristic characteristics[BK72XX_GATT_MAX_CHARACTERISTICS];
    ble_device_base::GattDescriptor descriptors[BK72XX_GATT_MAX_DESCRIPTORS];
  };

  static void sdp_charac_callback(uint8_t type, uint8_t conidx, uint16_t hdl, uint16_t len, uint8_t *data);

  void handle_control_event_(const BK72xxGattControlEvent &event);
  void handle_data_event_(const BK72xxGattDataEvent &event);

  void handle_connected_(uint8_t status);
  void handle_disconnected_(uint8_t reason);
  void fail_connection_(uint8_t reason);
  void release_scan_inhibit_();
  bool notify_subscribed_(uint16_t handle) const;

  ble_device_base::GattClientListener *listener_{nullptr};
  ServiceArena *arena_{nullptr};

  esphome::LockFreeQueue<BK72xxGattControlEvent, BK72XX_GATT_CONTROL_QUEUE_SIZE> control_queue_;
  esphome::EventPool<BK72xxGattControlEvent, BK72XX_GATT_CONTROL_QUEUE_SIZE - 1> control_pool_;

  esphome::LockFreeQueue<BK72xxGattDataEvent, BK72XX_GATT_DATA_QUEUE_SIZE> data_queue_;
  esphome::EventPool<BK72xxGattDataEvent, BK72XX_GATT_DATA_QUEUE_SIZE - 1> data_pool_;

  uint8_t op_buffer_[BK72XX_GATT_MAX_ATTR_LEN]{0};
  uint16_t op_len_{0};
  uint16_t op_handle_{0};
  OpType op_type_{OpType::NONE};

  uint16_t notify_subscriptions_[BK72XX_GATT_MAX_NOTIFY_SUBSCRIPTIONS]{0};
  uint8_t notify_subscription_count_{0};

  uint64_t peer_address_{0};
  uint32_t connect_started_{0};
  uint32_t mtu_exchange_started_{0};
  uint32_t disconnecting_started_{0};
  uint32_t discovery_started_{0};

  uint16_t mtu_{23};
  uint16_t service_count_{0};
  uint16_t char_count_{0};
  uint16_t desc_count_{0};

  uint8_t peer_addr_type_{0};
  EngineState state_{EngineState::IDLE};
  bool holds_scan_inhibit_{false};
  bool discovery_pending_{false};
  bool cancel_requested_{false};
};

}  // namespace esphome::bluetooth_connection

#endif  // USE_BK72XX_BLE && USE_BLE_GATT_CLIENT
