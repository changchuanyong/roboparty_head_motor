// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 changchuanyong

#pragma once

#include <cstdint>
#include <memory>
#include <string>

/**
 * @brief Robot head controller: two-axis LRO motors (axis 1 = yaw left/right, axis 2 = pitch up/down)
 *
 * Calls roboparty_motors' MotorDriver directly, no intermediate layer.
 * Motion is executed asynchronously by an internal MIT continuous-control thread (10 ms PD servo).
 * Implementation details (control thread, mutexes, motor instances, etc.) live in
 * HeadControllerImpl in src/head_controller.cpp (Pimpl).
 */
struct HeadControllerImpl;  ///< Forward declaration (global scope)

class HeadController {
   public:
    static constexpr int AXIS_YAW = 1;    ///< Axis 1: left/right yaw
    static constexpr int AXIS_PITCH = 2;  ///< Axis 2: up/down pitch

    /**
     * @brief Default software limit lower bounds (deg), [yaw, pitch]; overridable via set_limits()
     *
     * Sign convention: positive yaw = turn left, positive pitch = look up, zero = facing forward.
     */
    static constexpr float kDefaultLimitMin[2] = {-120.0f, -20.0f};
    /// @brief Default software limit upper bounds (deg), [yaw, pitch]; overridable via set_limits()
    static constexpr float kDefaultLimitMax[2] = {120.0f, 39.0f};

    /**
     * @brief Construct the head controller (does not initialize hardware; call init_motors())
     * @param can_interface CANFD interface name, e.g. "can0"
     * @param yaw_id CAN ID of the yaw motor, default 1
     * @param pitch_id CAN ID of the pitch motor, default 2
     * @note Construction does not touch hardware and throws nothing
     */
    HeadController(const std::string& can_interface, uint16_t yaw_id = 1, uint16_t pitch_id = 2);

    /// @brief Destructor: stops the control thread and deinitializes motors
    ~HeadController();

    HeadController(const HeadController&) = delete;
    HeadController& operator=(const HeadController&) = delete;

    /**
     * @brief Create and enable two LRO_PJ2_55_5550 motors, start the MIT continuous-control thread
     * @throws std::runtime_error if motor init fails, initial feedback times out, or the CAN interface is unavailable
     * @note On failure, created motor instances are released and disabled; repeated calls return immediately
     */
    void init_motors();

    /**
     * @brief Stop the control thread, disable and release motor instances
     * @note Called automatically by the destructor; after disable the pitch axis droops under gravity
     */
    void deinit_motors();

    /**
     * @brief Enable motors and resume the control thread's commands
     * @note After resuming, the control thread holds the current position as its target
     */
    void enable();

    /**
     * @brief Disable motors (the control thread stops commanding; motors hold no torque)
     * @note After disabling, the head droops under gravity on the pitch axis
     */
    void disable();

    /**
     * @brief Single-axis motion: set a target angle, the control thread approaches it speed-limited
     * @param axis Axis number: 1 = yaw (left/right), 2 = pitch (up/down); other values throw std::invalid_argument
     * @param deg Target angle (deg), clamped to the software limits
     * @param speed Maximum speed (deg/s), range (0, +∞); 0 or negative falls back to the default 30
     * @throws std::runtime_error if called before init_motors()
     * @note Non-blocking: sets the target and returns immediately
     */
    void move(int axis, float deg, float speed = 30.0f);

    /**
     * @brief Combined two-axis pose: set yaw and pitch targets simultaneously
     * @param yaw_deg Yaw target angle (deg)
     * @param pitch_deg Pitch target angle (deg)
     * @param yaw_speed Maximum yaw speed (deg/s)
     * @param pitch_speed Maximum pitch speed (deg/s)
     * @note Non-blocking; each axis approaches speed-limited independently
     */
    void look_at(float yaw_deg, float pitch_deg, float yaw_speed = 30.0f,
                 float pitch_speed = 30.0f);

    /**
     * @brief Read the current angle of one axis (latest cached feedback)
     * @param axis Axis number 1/2
     * @return Angle (deg), bounded by motor travel (LRO_PJ2_55_5550 approx ±716°)
     * @note Feedback is refreshed automatically by the control thread
     */
    float get_joint_deg(int axis);

    /**
     * @brief Read the motor temperature of one axis (latest cached feedback)
     * @param axis Axis number 1/2
     * @return Temperature (°C), normal range approx 25~80
     */
    float get_temp(int axis);

    /**
     * @brief Read the motor error code of one axis (latest cached feedback)
     * @param axis Axis number 1/2
     * @return Error code: 0 = no error; 1 = overheat 2 = overcurrent 3 = undervoltage 4 = encoder error 6 = brake overvoltage 7 = DRV error (power cycle required)
     */
    uint8_t get_error(int axis);

    /**
     * @brief Set the software limits of one axis (clamped before commanding; motor-internal limits act as backstop)
     * @param axis Axis number 1/2
     * @param min_deg Lower bound (deg)
     * @param max_deg Upper bound (deg)
     * @throws std::invalid_argument if min_deg > max_deg or the axis number is invalid
     */
    void set_limits(int axis, float min_deg, float max_deg);

    /**
     * @brief Record the current angle as 0 (write the motor's internal zero position)
     * @param axis Axis number 1/2
     * @return Whether the command was sent successfully; includes an internal 500 ms wait
     * @throws std::runtime_error if the motors are enabled; call disable() first
     */
    bool set_zero(int axis);

    /**
     * @brief Clear recoverable errors of one axis (e.g. overheat/overcurrent)
     * @param axis Axis number 1/2
     * @note DRV_ERROR (0x07) is non-recoverable; this command is ineffective for it, a power cycle is required
     */
    void clear_error(int axis);

   private:
    std::unique_ptr<HeadControllerImpl> impl_;
};
