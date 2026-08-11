// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 changchuanyong

#include "head_controller.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <motor_driver.hpp>

namespace {
constexpr float kDeg2Rad = static_cast<float>(M_PI) / 180.0f;
// LRO_Motor_Model enum (roboparty_motors internal): 0=5550 1=6562 2=8462 3=10062
constexpr int kLroModel5550 = 0;
constexpr auto kInitialFeedbackTimeout = std::chrono::milliseconds(100);
}  // namespace

// ---------------------------------------------------------------------------
// Implementation details (Pimpl): control thread, mutexes, motor instances, MIT gains
// ---------------------------------------------------------------------------
struct HeadControllerImpl {
    static constexpr int AXIS_YAW = HeadController::AXIS_YAW;      // aliases (public axis definitions)
    static constexpr int AXIS_PITCH = HeadController::AXIS_PITCH;

    // MIT continuous-control gains (deploy-scale reference: kp 20~150 / kd 1~5; light-load setting for head)
    // kp=40 measured steady-state error ~1.5°, raised to 100 gives ~0.5° or better
    static constexpr float kDefaultKp = 100.0f;  // position stiffness
    static constexpr float kDefaultKd = 2.0f;    // damping
    static constexpr float kControlDt = 0.01f;   // control period (s)

    std::string can_interface_;
    uint16_t yaw_id_;
    uint16_t pitch_id_;
    std::array<std::shared_ptr<MotorDriver>, 2> motors_;  // [0]=yaw [1]=pitch
    std::array<float, 2> limit_min_;                      // deg
    std::array<float, 2> limit_max_;                      // deg
    std::mutex command_mutex_;                            // mutual exclusion between public API / combined actions
    std::atomic<bool> opened_{false};

    // MIT continuous-control state (guarded by control_mutex_)
    std::mutex control_mutex_;
    std::thread control_thread_;
    std::atomic<bool> control_running_{false};
    std::atomic<bool> enabled_{false};
    std::array<float, 2> final_target_;  // rad, final target (set by move)
    std::array<float, 2> cur_target_;    // rad, current target (approached speed-limited by the control thread)
    std::array<float, 2> speed_deg_s_;   // deg/s
    std::array<float, 2> kp_{kDefaultKp, kDefaultKp};
    std::array<float, 2> kd_{kDefaultKd, kDefaultKd};

    HeadControllerImpl(const std::string& can_interface, uint16_t yaw_id, uint16_t pitch_id);
    ~HeadControllerImpl();

    int axis_index(int axis) const;  // 1/2 -> 0/1
    float clamp_deg(int idx, float deg) const;
    void check_open() const;
    void refresh_unsafe();  // assumes command_mutex_ is held
    void wait_feedback_unsafe();  // refreshes both motors and throws on timeout
    float move_unsafe(int axis, float deg, float speed);  // returns the clamped actual target (deg)
    void control();  // MIT continuous-control thread

    void init_motors();
    void deinit_motors();
    void move(int axis, float deg, float speed);
    void look_at(float yaw_deg, float pitch_deg, float yaw_speed, float pitch_speed);
    float get_joint_deg(int axis);
    float get_temp(int axis);
    uint8_t get_error(int axis);
    void set_limits(int axis, float min_deg, float max_deg);
    void enable();
    void disable();
    void clear_error(int axis);
    bool set_zero(int axis);
};

HeadControllerImpl::HeadControllerImpl(const std::string& can_interface, uint16_t yaw_id, uint16_t pitch_id)
    : can_interface_(can_interface), yaw_id_(yaw_id), pitch_id_(pitch_id) {
    for (int i = 0; i < 2; ++i) {
        limit_min_[i] = HeadController::kDefaultLimitMin[i];
        limit_max_[i] = HeadController::kDefaultLimitMax[i];
    }
}

HeadControllerImpl::~HeadControllerImpl() {
    if (opened_.load()) {
        try {
            deinit_motors();  // stop control thread + disable
        } catch (...) {
        }
    }
}

void HeadControllerImpl::init_motors() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (opened_.load()) {
        return;
    }
    try {
        motors_[0] = MotorDriver::create_motor(yaw_id_, "canfd", can_interface_, "LRO", kLroModel5550);
        motors_[1] = MotorDriver::create_motor(pitch_id_, "canfd", can_interface_, "LRO", kLroModel5550);
        for (int i = 0; i < 2; ++i) {
            const uint8_t err = motors_[i]->init_motor();
            if (err != 0) {  // LRO_NO_ERROR
                throw std::runtime_error("head motor init failed (id=" +
                                         std::to_string(motors_[i]->get_motor_id()) +
                                         ", error=" + std::to_string(err) + ")");
            }
            // MIT continuous-control mode (LRO's POS mode is interrupted by MIT refresh frames; deploy convention is MIT)
            motors_[i]->set_motor_control_mode(MotorDriver::MIT);
        }
        // Require fresh feedback before using motor positions as initial targets.
        wait_feedback_unsafe();

        // Initial target = confirmed current motor position (no step jump).
        {
            std::lock_guard<std::mutex> control_lock(control_mutex_);
            for (int i = 0; i < 2; ++i) {
                final_target_[i] = motors_[i]->get_motor_pos();
                cur_target_[i] = final_target_[i];
                speed_deg_s_[i] = 30.0f;
            }
        }
        opened_ = true;
        enabled_ = true;
        control_running_ = true;
        control_thread_ = std::thread(&HeadControllerImpl::control, this);
    } catch (...) {
        for (auto& m : motors_) {
            if (m) {
                m->unlock_motor();
                m.reset();
            }
        }
        throw;
    }
}

void HeadControllerImpl::deinit_motors() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (control_running_.load()) {
        control_running_ = false;
        if (control_thread_.joinable()) {
            control_thread_.join();
        }
    }
    for (auto& m : motors_) {
        if (m) {
            m->unlock_motor();
            m.reset();
        }
    }
    opened_ = false;
    enabled_ = false;
}

void HeadControllerImpl::move(int axis, float deg, float speed) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    check_open();
    move_unsafe(axis, deg, speed);
}

float HeadControllerImpl::move_unsafe(int axis, float deg, float speed) {
    const int idx = axis_index(axis);
    const float target = clamp_deg(idx, deg);
    {
        std::lock_guard<std::mutex> control_lock(control_mutex_);
        final_target_[idx] = target * kDeg2Rad;                  // final target (rad)
        speed_deg_s_[idx] = (speed > 0.0f) ? speed : 30.0f;      // speed limit (deg/s); 0/negative uses the default
    }
    return target;
}

void HeadControllerImpl::look_at(float yaw_deg, float pitch_deg, float yaw_speed, float pitch_speed) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    check_open();
    const float yaw_target = clamp_deg(0, yaw_deg);
    const float pitch_target = clamp_deg(1, pitch_deg);
    std::lock_guard<std::mutex> control_lock(control_mutex_);
    final_target_[0] = yaw_target * kDeg2Rad;
    final_target_[1] = pitch_target * kDeg2Rad;
    speed_deg_s_[0] = (yaw_speed > 0.0f) ? yaw_speed : 30.0f;
    speed_deg_s_[1] = (pitch_speed > 0.0f) ? pitch_speed : 30.0f;
}

void HeadControllerImpl::refresh_unsafe() {
    for (auto& m : motors_) {
        // zero MIT frame requests type-1 feedback; same mode as command frames under MIT, does not interrupt control
        m->refresh_motor_status();
    }
}

void HeadControllerImpl::wait_feedback_unsafe() {
    refresh_unsafe();
    const auto feedback_deadline = std::chrono::steady_clock::now() + kInitialFeedbackTimeout;
    while ((motors_[0]->get_response_count() != 0 || motors_[1]->get_response_count() != 0) &&
           std::chrono::steady_clock::now() < feedback_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    for (int i = 0; i < 2; ++i) {
        if (motors_[i]->get_response_count() != 0) {
            throw std::runtime_error("head motor feedback timeout (id=" +
                                     std::to_string(motors_[i]->get_motor_id()) + ")");
        }
    }
}

void HeadControllerImpl::control() {
    int refresh_tick = 0;
    const auto period = std::chrono::milliseconds(static_cast<int>(kControlDt * 1000.0f));
    auto next_release = std::chrono::steady_clock::now();
    while (control_running_.load()) {
        next_release += period;
        {
            std::lock_guard<std::mutex> control_lock(control_mutex_);
            if (enabled_.load()) {
                for (int i = 0; i < 2; ++i) {
                    // approach the final target speed-limited (avoid overshoot from abrupt target jumps)
                    const float step = speed_deg_s_[i] * kDeg2Rad * kControlDt;
                    const float diff = final_target_[i] - cur_target_[i];
                    if (std::fabs(diff) <= step) {
                        cur_target_[i] = final_target_[i];
                    } else {
                        cur_target_[i] += (diff > 0.0f ? step : -step);
                    }
                    // PD position servo: target position + kp/kd, zero torque feedforward
                    motors_[i]->motor_mit_cmd(cur_target_[i], 0.0f, kp_[i], kd_[i], 0.0f);
                }
                // Regular MIT command frames carry no feedback; interleave zero MIT frames every 500 ms.
                // Keep the rate moderate: zero frames have no stiffness momentarily; high rates have triggered DRV_ERROR.
                if (++refresh_tick % 50 == 0) {
                    refresh_unsafe();
                }
            }
        }
        const auto now = std::chrono::steady_clock::now();
        if (now > next_release) {
            next_release = now;
        }
        std::this_thread::sleep_until(next_release);
    }
}

float HeadControllerImpl::get_joint_deg(int axis) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    check_open();
    // get_motor_pos() returns rad (with zero-offset compensation), convert to deg
    return motors_[axis_index(axis)]->get_motor_pos() / kDeg2Rad;
}

float HeadControllerImpl::get_temp(int axis) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    check_open();
    return motors_[axis_index(axis)]->get_motor_temperature();
}

uint8_t HeadControllerImpl::get_error(int axis) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    check_open();
    return motors_[axis_index(axis)]->get_error_id();
}

void HeadControllerImpl::set_limits(int axis, float min_deg, float max_deg) {
    if (min_deg > max_deg) {
        throw std::invalid_argument("set_limits: min_deg must be <= max_deg");
    }
    const int idx = axis_index(axis);
    std::lock_guard<std::mutex> lock(command_mutex_);
    limit_min_[idx] = min_deg;
    limit_max_[idx] = max_deg;
}

void HeadControllerImpl::enable() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    check_open();
    motors_[0]->lock_motor();
    motors_[1]->lock_motor();
    try {
        wait_feedback_unsafe();
        {
            std::lock_guard<std::mutex> control_lock(control_mutex_);
            for (int i = 0; i < 2; ++i) {
                final_target_[i] = motors_[i]->get_motor_pos();
                cur_target_[i] = final_target_[i];
            }
        }
        enabled_ = true;  // resume only after holding the confirmed current position
    } catch (...) {
        motors_[0]->unlock_motor();
        motors_[1]->unlock_motor();
        throw;
    }
}

void HeadControllerImpl::disable() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    check_open();
    std::lock_guard<std::mutex> control_lock(control_mutex_);
    enabled_ = false;
    motors_[0]->unlock_motor();
    motors_[1]->unlock_motor();
}

void HeadControllerImpl::clear_error(int axis) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    check_open();
    motors_[axis_index(axis)]->clear_motor_error();
}

bool HeadControllerImpl::set_zero(int axis) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    check_open();
    if (enabled_.load()) {
        throw std::runtime_error("set_zero requires disabled motors: call disable() first");
    }
    return motors_[axis_index(axis)]->set_motor_zero();  // includes an internal 500 ms wait
}

int HeadControllerImpl::axis_index(int axis) const {
    if (axis == AXIS_YAW) {
        return 0;
    }
    if (axis == AXIS_PITCH) {
        return 1;
    }
    throw std::invalid_argument("axis must be 1 (yaw) or 2 (pitch)");
}

float HeadControllerImpl::clamp_deg(int idx, float deg) const {
    if (deg < limit_min_[idx]) {
        return limit_min_[idx];
    }
    if (deg > limit_max_[idx]) {
        return limit_max_[idx];
    }
    return deg;
}

void HeadControllerImpl::check_open() const {
    if (!opened_.load()) {
        throw std::runtime_error("head motors not initialized: call init_motors() first");
    }
}

// ---------------------------------------------------------------------------
// HeadController public interface forwarding (Pimpl)
// ---------------------------------------------------------------------------
HeadController::HeadController(const std::string& can_interface, uint16_t yaw_id, uint16_t pitch_id)
    : impl_(std::make_unique<HeadControllerImpl>(can_interface, yaw_id, pitch_id)) {}

HeadController::~HeadController() = default;

void HeadController::init_motors() { impl_->init_motors(); }
void HeadController::deinit_motors() { impl_->deinit_motors(); }
void HeadController::move(int axis, float deg, float speed) { impl_->move(axis, deg, speed); }
void HeadController::look_at(float yaw_deg, float pitch_deg, float yaw_speed, float pitch_speed) {
    impl_->look_at(yaw_deg, pitch_deg, yaw_speed, pitch_speed);
}
float HeadController::get_joint_deg(int axis) { return impl_->get_joint_deg(axis); }
float HeadController::get_temp(int axis) { return impl_->get_temp(axis); }
uint8_t HeadController::get_error(int axis) { return impl_->get_error(axis); }
void HeadController::set_limits(int axis, float min_deg, float max_deg) {
    impl_->set_limits(axis, min_deg, max_deg);
}
void HeadController::enable() { impl_->enable(); }
void HeadController::disable() { impl_->disable(); }
void HeadController::clear_error(int axis) { impl_->clear_error(axis); }
bool HeadController::set_zero(int axis) { return impl_->set_zero(axis); }
