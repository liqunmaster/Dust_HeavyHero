#pragma once

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "ring_buffer.hpp"

#ifdef __cplusplus
extern "C" {
#endif

// 发送和接收环形队
#define FDCAN_TX_QUEUE_DEPTH     32U
#define FDCAN_RX_QUEUE_DEPTH     32U
#define FDCAN_MAX_DATA_LENGTH    64U

// 超时时间
typedef struct {
    int32_t milliseconds;
} fdcan_timeout;

#define FDCAN_NO_WAIT           ((fdcan_timeout){0})
#define FDCAN_WAIT_FOREVER      ((fdcan_timeout){-1})
#define FDCAN_MSEC(ms)          ((fdcan_timeout){(int32_t)(ms)})

// CAN 控制器编号
typedef enum {
    FDCAN_DEVICE_CAN0  = 0,                      // CAN0
    FDCAN_DEVICE_CAN1  = 1,                      // CAN1
    FDCAN_DEVICE_CAN2  = 2,                      // CAN2
    FDCAN_DEVICE_CAN3  = 3,                      // CAN3
    FDCAN_DEVICE_COUNT = 4,                      // 数量
} fdcan_device;

// CAN 帧协议
typedef enum {
    FDCAN_PROTOCOL_CLASSIC = 0,
    FDCAN_PROTOCOL_FD      = 1,
} fdcan_protocol;

// CAN 标识符类型 
typedef enum {
    FDCAN_ID_STANDARD = 0,                        // 11 位标准 ID
    FDCAN_ID_EXTENDED = 1,                        // 29 位扩展 ID
} fdcan_id_type;

// CAN FD 数据段波特率切换 
typedef enum {
    FDCAN_BRS_DISABLED = 0,                       // 不切换数据段波特率
    FDCAN_BRS_ENABLED  = 1,                       // 启用数据段波特率切换
} fdcan_brs;

// CAN 控制器工作模式 
typedef enum {
    FDCAN_MODE_NORMAL       = 0,                   // 正常模式
    FDCAN_MODE_LOOPBACK     = 1,                   // 回环测试模式
    FDCAN_MODE_LISTEN_ONLY  = 2,                   // 只监听模式
} fdcan_mode;

// CAN 自动重传
typedef enum {
    FDCAN_RETRANSMISSION_ENABLED  = 0,            // 发送失败后自动重传
    FDCAN_RETRANSMISSION_DISABLED = 1,            // 单次发送，不自动重传
} fdcan_retransmission;

// CAN 总线状态
typedef enum {
    FDCAN_BUS_ERROR_ACTIVE  = 0,                  // 错误主动
    FDCAN_BUS_ERROR_WARNING = 1,                  // 错误计数达到警告阈值
    FDCAN_BUS_ERROR_PASSIVE = 2,                  // 错误被动
    FDCAN_BUS_OFF           = 3,                  // 总线关闭
    FDCAN_BUS_STOPPED       = 4,                  // 控制器已停止
} fdcan_bus_state;

// BUS-OFF 恢复状态机
typedef enum {
    FDCAN_RECOVERY_IDLE          = 0,             // 空闲
    FDCAN_RECOVERY_STOPPING      = 1,             // 正在停止控制器
    FDCAN_RECOVERY_CLEARING_TX   = 2,             // 正在清空发送队列
    FDCAN_RECOVERY_RECONFIGURING = 3,             // 正在重新配置控制器
    FDCAN_RECOVERY_RESTARTING    = 4,             // 正在重新启动控制器
    FDCAN_RECOVERY_SUCCEEDED     = 5,             // 恢复成功
    FDCAN_RECOVERY_FAILED        = 6,             // 恢复失败
} fdcan_recovery_state;

// CAN 帧
typedef struct {
    uint32_t id;                                  // CAN ID
    fdcan_id_type id_type;                        // 标准帧或扩展帧
    fdcan_protocol protocol;                      // CAN 协议
    fdcan_brs bitrate_switch;                     // 是否启用 BRS
    uint8_t length;                               // 有效数据长度，单位：字节
    uint8_t data[FDCAN_MAX_DATA_LENGTH];          // 数据区
} fdcan_frame;

// CAN 控制器运行配置
typedef struct {
    fdcan_mode mode;                              // 控制器工作模式
    fdcan_retransmission retransmission;          // 自动重传策略
} fdcan_config;

// CAN 控制器错误计数
typedef struct {
    uint8_t transmit;                             // 发送错误计数
    uint8_t receive;                              // 接收错误计数
} fdcan_error_count;

// CAN 收发与恢复统计信息
typedef struct {
    uint32_t tx_queued;                           // 累计进入发送队列的帧数
    uint32_t tx_completed;                        // 累计发送完成的帧数
    uint32_t tx_dropped;                          // 累计丢弃的发送帧数
    uint32_t tx_errors;                           // 累计发送错误帧数
    uint32_t rx_received;                         // 累计进入接收队列的帧数
    uint32_t rx_dropped;                          // 累计丢弃的接收帧数
    uint32_t recovery_succeeded;                  // BUS-OFF 恢复成功次数
    uint32_t recovery_failed;                     // BUS-OFF 恢复失败次数
} fdcan_statistics;

int bsp_fdcan_init(fdcan_device device, const fdcan_config *config);

int bsp_fdcan_deinit(fdcan_device device);

int bsp_fdcan_transmit(fdcan_device device, const fdcan_frame *frame, fdcan_timeout timeout);

typedef void (*fdcan_rx_callback_t)(fdcan_device device, const fdcan_frame *frame, void *user_data);

int bsp_fdcan_set_rx_callback(fdcan_device device, fdcan_rx_callback_t callback, void *user_data);

int bsp_fdcan_recover(fdcan_device device, fdcan_timeout timeout);

int bsp_fdcan_get_state(fdcan_device device, fdcan_bus_state *state, fdcan_error_count *error_count);

fdcan_recovery_state bsp_fdcan_get_recovery_state(fdcan_device device);

void bsp_fdcan_get_statistics(fdcan_device device, fdcan_statistics *statistics);

void bsp_fdcan_clear_statistics(fdcan_device device);

bool bsp_fdcan_is_ready(fdcan_device device);

size_t bsp_fdcan_tx_pending(fdcan_device device);

#ifdef __cplusplus
}
#endif
