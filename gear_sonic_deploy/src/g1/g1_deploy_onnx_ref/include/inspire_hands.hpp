/**
 * @file inspire_hands.hpp
 * @brief Driver for the Inspire RH56 6-DOF dexterous hands (left + right).
 *
 * Drop-in replacement for Dex3Hands on the command side: same public surface as
 * used by g1_deploy_onnx_ref.cpp (initialize / writeOnce / setAllJointsCommand /
 * SetMaxCloseRatio / GetMaxCloseRatio / open / close), plus getJointAngles() for
 * the state-logging read site.
 *
 * Unlike the Unitree dex3 (position + kp/kd over rt/dex3/*), the Inspire hand is
 * commanded with normalized integer drive units [0, 1000] in "angle mode" over
 * its own DDS SDK:
 *
 *   Direction | Left hand                    | Right hand
 *   ----------|------------------------------|------------------------------
 *   Command   | rt/inspire_hand/ctrl/l       | rt/inspire_hand/ctrl/r
 *   State     | rt/inspire_hand/state/l      | rt/inspire_hand/state/r
 *
 * A separate Inspire driver process (Headless_driver_l/r.py) bridges these DDS
 * messages to Modbus and actually moves the hardware.
 *
 * Commands are stored as radians (Inspire JOINT_ORDER: pinky, ring, middle,
 * index, thumb_bend, thumb_rot) and converted to drive units at publish time via
 * inspire_hand_spec, so the runtime max-close-ratio (X/C keys) is applied live.
 * The hand performs its own speed limiting, so no per-tick delta clamp is needed.
 */

#ifndef INSPIRE_HANDS_HPP
#define INSPIRE_HANDS_HPP

#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "inspire_idl/inspire_hand_ctrl.hpp"
#include "inspire_idl/inspire_hand_state.hpp"
#include "inspire_hand_spec.hpp"
#include "utils.hpp"

/// Inspire control-mode bitfield (see inspire_hand_ws/readme.md). 0b0001 = angle.
static constexpr int8_t INSPIRE_MODE_ANGLE = 0b0001;

/**
 * @class InspireHands
 * @brief Manages two Inspire RH56 hands (left + right) over the Inspire DDS SDK.
 *
 * Does not run its own thread - the owning class calls writeOnce() at its
 * desired cadence from the command-writer thread.
 */
class InspireHands
{
public:
    static constexpr int DOF = inspire_hand_spec::INSPIRE_DOF;  // 6

    InspireHands() = default;

    // Initializes channels for both hands. If networkInterface is empty, skips
    // ChannelFactory init (mirrors Dex3Hands; the deploy inits the factory itself).
    void initialize(const std::string &networkInterface)
    {
        if (!networkInterface.empty())
        {
            unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface.c_str());
        }

        left_.publisher.reset(new unitree::robot::ChannelPublisher<inspire::inspire_hand_ctrl>(
            "rt/inspire_hand/ctrl/l"));
        left_.subscriber.reset(new unitree::robot::ChannelSubscriber<inspire::inspire_hand_state>(
            "rt/inspire_hand/state/l"));
        left_.publisher->InitChannel();
        left_.subscriber->InitChannel(
            [this](const void *message) { this->onState(true, message); }, 1);

        right_.publisher.reset(new unitree::robot::ChannelPublisher<inspire::inspire_hand_ctrl>(
            "rt/inspire_hand/ctrl/r"));
        right_.subscriber.reset(new unitree::robot::ChannelSubscriber<inspire::inspire_hand_state>(
            "rt/inspire_hand/state/r"));
        right_.publisher->InitChannel();
        right_.subscriber->InitChannel(
            [this](const void *message) { this->onState(false, message); }, 1);

        // Default to fully open until a command arrives.
        left_.cmd_buffer.SetData(std::array<double, DOF>{});
        right_.cmd_buffer.SetData(std::array<double, DOF>{});
    }

    // Set max close ratio at runtime (bounded to [0.2, 1.0]).
    void SetMaxCloseRatio(double ratio) {
        max_close_ratio_ = std::max(0.2, std::min(1.0, ratio));
    }
    double GetMaxCloseRatio() const { return max_close_ratio_; }

    // Perform one publish tick; call at your own cadence from the writer thread.
    void writeOnce()
    {
        publishHand(left_, true);
        publishHand(right_, false);
    }

    // Sets the command for one hand from radian joint angles (Inspire JOINT_ORDER).
    // Accepts the deploy's 7-wide buffer; only the first 6 entries are used.
    void setAllJointsCommand(bool is_left, const std::array<double, 7> &q)
    {
        std::array<double, DOF> rad{};
        for (int i = 0; i < DOF; ++i) { rad[i] = q[i]; }
        HandCtx &ctx = is_left ? left_ : right_;
        ctx.cmd_buffer.SetData(rad);
    }

    // Fills q_out[0..5] with measured joint angles (rad) from angle_act; q_out[6]=0.
    // Returns false (and leaves q_out untouched) if no state has been received.
    bool getJointAngles(bool is_left, std::array<double, 7> &q_out) const
    {
        const HandCtx &ctx = is_left ? left_ : right_;
        const auto statePtr = ctx.state_buffer.GetDataWithTime().data;
        if (!statePtr) { return false; }
        const auto rad = inspire_hand_spec::drive_to_rad(statePtr->angle_act());
        for (int i = 0; i < DOF; ++i) { q_out[i] = rad[i]; }
        for (int i = DOF; i < 7; ++i) { q_out[i] = 0.0; }
        return true;
    }

    bool hasState(bool is_left) const
    {
        const HandCtx &ctx = is_left ? left_ : right_;
        return ctx.state_buffer.GetDataWithTime().HasData();
    }

    // Quick helper: OPEN - fully open (q = 0 rad for all drives).
    void open(bool is_left)
    {
        HandCtx &ctx = is_left ? left_ : right_;
        ctx.cmd_buffer.SetData(std::array<double, DOF>{});
    }

    // Quick helper: CLOSE - close to the current max-close-ratio of full travel.
    void close(bool is_left)
    {
        std::array<double, DOF> rad{};
        for (int i = 0; i < DOF; ++i) {
            rad[i] = max_close_ratio_ * inspire_hand_spec::Q_CLOSED[i];
        }
        HandCtx &ctx = is_left ? left_ : right_;
        ctx.cmd_buffer.SetData(rad);
    }

private:
    struct HandCtx
    {
        unitree::robot::ChannelPublisherPtr<inspire::inspire_hand_ctrl> publisher;
        unitree::robot::ChannelSubscriberPtr<inspire::inspire_hand_state> subscriber;

        DataBuffer<inspire::inspire_hand_state> state_buffer;
        DataBuffer<std::array<double, DOF>> cmd_buffer;  // radians, Inspire JOINT_ORDER
    };

    void publishHand(HandCtx &ctx, bool /*is_left*/)
    {
        const auto cmdPtr = ctx.cmd_buffer.GetDataWithTime().data;
        if (!ctx.publisher || !cmdPtr) { return; }

        // Apply the runtime close-ratio clamp in radian space: 0 (open) ..
        // max_close_ratio_ * Q_CLOSED (limited closure).
        std::array<double, DOF> rad = *cmdPtr;
        for (int i = 0; i < DOF; ++i) {
            const double q_close_limit = max_close_ratio_ * inspire_hand_spec::Q_CLOSED[i];
            rad[i] = std::max(0.0, std::min(rad[i], q_close_limit));
        }

        inspire::inspire_hand_ctrl cmd;
        cmd.pos_set(std::vector<int16_t>(DOF, 0));
        cmd.angle_set(inspire_hand_spec::rad_to_drive(rad));
        cmd.force_set(std::vector<int16_t>(DOF, 0));
        cmd.speed_set(std::vector<int16_t>(DOF, 0));
        cmd.mode(INSPIRE_MODE_ANGLE);
        ctx.publisher->Write(cmd);
    }

    void onState(bool is_left, const void *message)
    {
        HandCtx &ctx = is_left ? left_ : right_;
        const auto *incoming = static_cast<const inspire::inspire_hand_state *>(message);
        ctx.state_buffer.SetData(*incoming);
    }

    HandCtx left_;
    HandCtx right_;

    // Runtime adjustable max close ratio (default 1.0 = full closure allowed),
    // bounded to [0.2, 1.0]. Set via --max-close-ratio, adjusted with X/C keys.
    double max_close_ratio_ = 1.0;
};

#endif // INSPIRE_HANDS_HPP
