#pragma once

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

// Zbus通道类
template<typename MsgType>
class ZbusChannel{
public:
    /**
     * @brief 初始化
     *
     */
    void Init()
    {
        lock_ = {};
        has_msg_ = false;
    }

    /**
     * @brief 发布消息
     *
     * @param msg 消息
     */
    int Publish(const MsgType &msg)
    {
        k_spinlock_key_t key = k_spin_lock(&lock_);
        last_msg_ = msg;
        has_msg_ = true;
        k_spin_unlock(&lock_, key);
        return 0;
    }

    /**
     * @brief 读取消息
     *
     * @param msg 消息
     */
    int Read(MsgType &msg)
    {
        k_spinlock_key_t key = k_spin_lock(&lock_);
        if (has_msg_){
        msg = last_msg_;
        k_spin_unlock(&lock_, key);
        return 0;
        }
    k_spin_unlock(&lock_, key);
    return -ENOMSG;
  }

private:

    struct k_spinlock lock_;

    MsgType last_msg_;

    bool has_msg_ = false;
};
