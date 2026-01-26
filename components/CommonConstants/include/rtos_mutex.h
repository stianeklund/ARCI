#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

/**
 * @brief RAII wrapper for FreeRTOS mutex (non-recursive)
 *
 * Provides std::lock_guard-like semantics for FreeRTOS mutexes.
 * Supports priority inheritance and is lighter than std::mutex on ESP-IDF.
 */
class RtosMutex {
public:
    RtosMutex() : handle_(xSemaphoreCreateMutex()) {
        configASSERT(handle_ != nullptr);
    }

    ~RtosMutex() {
        if (handle_) {
            vSemaphoreDelete(handle_);
        }
    }

    // Non-copyable, non-movable
    RtosMutex(const RtosMutex&) = delete;
    RtosMutex& operator=(const RtosMutex&) = delete;
    RtosMutex(RtosMutex&&) = delete;
    RtosMutex& operator=(RtosMutex&&) = delete;

    void lock() {
        xSemaphoreTake(handle_, portMAX_DELAY);
    }

    bool try_lock() {
        return xSemaphoreTake(handle_, 0) == pdTRUE;
    }

    bool try_lock_for(TickType_t ticks) {
        return xSemaphoreTake(handle_, ticks) == pdTRUE;
    }

    void unlock() {
        xSemaphoreGive(handle_);
    }

    SemaphoreHandle_t native_handle() const { return handle_; }

private:
    SemaphoreHandle_t handle_;
};

/**
 * @brief RAII wrapper for FreeRTOS recursive mutex
 *
 * Allows the same task to acquire the mutex multiple times.
 * Required for nested/recursive function calls that need locking.
 */
class RtosRecursiveMutex {
public:
    RtosRecursiveMutex() : handle_(xSemaphoreCreateRecursiveMutex()) {
        configASSERT(handle_ != nullptr);
    }

    ~RtosRecursiveMutex() {
        if (handle_) {
            vSemaphoreDelete(handle_);
        }
    }

    // Non-copyable, non-movable
    RtosRecursiveMutex(const RtosRecursiveMutex&) = delete;
    RtosRecursiveMutex& operator=(const RtosRecursiveMutex&) = delete;
    RtosRecursiveMutex(RtosRecursiveMutex&&) = delete;
    RtosRecursiveMutex& operator=(RtosRecursiveMutex&&) = delete;

    void lock() {
        xSemaphoreTakeRecursive(handle_, portMAX_DELAY);
    }

    bool try_lock() {
        return xSemaphoreTakeRecursive(handle_, 0) == pdTRUE;
    }

    bool try_lock_for(TickType_t ticks) {
        return xSemaphoreTakeRecursive(handle_, ticks) == pdTRUE;
    }

    void unlock() {
        xSemaphoreGiveRecursive(handle_);
    }

    SemaphoreHandle_t native_handle() const { return handle_; }

private:
    SemaphoreHandle_t handle_;
};

/**
 * @brief RAII lock guard for RtosMutex (like std::lock_guard)
 */
template<typename MutexType>
class RtosLockGuard {
public:
    explicit RtosLockGuard(MutexType& mutex) : mutex_(mutex) {
        mutex_.lock();
    }

    ~RtosLockGuard() {
        mutex_.unlock();
    }

    // Non-copyable, non-movable
    RtosLockGuard(const RtosLockGuard&) = delete;
    RtosLockGuard& operator=(const RtosLockGuard&) = delete;

private:
    MutexType& mutex_;
};

/**
 * @brief RAII unique lock for RtosMutex with deferred/try locking (like std::unique_lock)
 */
template<typename MutexType>
class RtosUniqueLock {
public:
    // Lock immediately
    explicit RtosUniqueLock(MutexType& mutex) : mutex_(&mutex), owns_lock_(false) {
        lock();
    }

    // Adopt already-held lock
    RtosUniqueLock(MutexType& mutex, std::adopt_lock_t) : mutex_(&mutex), owns_lock_(true) {
        // Mutex already locked by caller
    }

    // Deferred locking
    RtosUniqueLock(MutexType& mutex, std::defer_lock_t) : mutex_(&mutex), owns_lock_(false) {
        // Don't lock yet
    }

    ~RtosUniqueLock() {
        if (owns_lock_) {
            mutex_->unlock();
        }
    }

    // Non-copyable
    RtosUniqueLock(const RtosUniqueLock&) = delete;
    RtosUniqueLock& operator=(const RtosUniqueLock&) = delete;

    // Movable
    RtosUniqueLock(RtosUniqueLock&& other) noexcept
        : mutex_(other.mutex_), owns_lock_(other.owns_lock_) {
        other.mutex_ = nullptr;
        other.owns_lock_ = false;
    }

    RtosUniqueLock& operator=(RtosUniqueLock&& other) noexcept {
        if (this != &other) {
            if (owns_lock_) {
                mutex_->unlock();
            }
            mutex_ = other.mutex_;
            owns_lock_ = other.owns_lock_;
            other.mutex_ = nullptr;
            other.owns_lock_ = false;
        }
        return *this;
    }

    void lock() {
        mutex_->lock();
        owns_lock_ = true;
    }

    bool try_lock() {
        owns_lock_ = mutex_->try_lock();
        return owns_lock_;
    }

    bool try_lock_for(TickType_t ticks) {
        owns_lock_ = mutex_->try_lock_for(ticks);
        return owns_lock_;
    }

    void unlock() {
        mutex_->unlock();
        owns_lock_ = false;
    }

    bool owns_lock() const { return owns_lock_; }
    explicit operator bool() const { return owns_lock_; }

private:
    MutexType* mutex_;
    bool owns_lock_;
};
