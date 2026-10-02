// Real feedback, KF adapter, model, LQR and FSM bodies; fake hardware endpoints.
#include "pyro_wl_forward_kf.h"
#include "wl_config.h"
#include "rdob_coef.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

static void check(bool condition, const char *message)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

namespace pyro
{
struct motor_base_t
{
    float position = 0.0f, rotate = 0.0f, torque = 0.0f;
    bool online = true;
    void update_feedback() {}
    float get_current_position() const { return position; }
    float get_current_rotate() const { return rotate; }
    float get_current_torque() const { return torque; }
    bool is_online() const { return online; }
    void enable() {}
};
struct pd_ctrl_t {};
struct pid_t {};
struct cmd_base_t {};
#include "chassis_types.inc"

struct dwt_drv_t
{
    static float get_delta_t(uint32_t *) { return FORWARD_KF_TS; }
};
struct ins_drv_t
{
    static ins_drv_t *get_instance() { static ins_drv_t instance; return &instance; }
    void get_rads_n(float *yaw, float *pitch, float *roll) { *yaw = *pitch = *roll = 0.0f; }
    void get_gyro_b(float *z, float *y, float *x) { *z = *y = *x = 0.0f; }
    void get_accel_without_g_b(float *x, float *y, float *z) { *x = *y = *z = 0.0f; }
};
namespace bsp_can { enum { can3 }; }
struct board_drv_t
{
    enum class role_t { CHASSIS };
    struct data_t { int imu_yaw_radps_100 = 0; };
    static board_drv_t &get_instance(role_t, int) { static board_drv_t instance; return instance; }
    data_t get_g2c_rx_data() const { return {}; }
};

struct wl_chassis_t
{
    struct state_stub_t { template<class T> void request_switch(T *) {} };
    struct fsm_active_t
    {
        struct state_normal_t
        {
            struct state_balance_t : state_stub_t
            {
                void enter(wl_chassis_t *);
                void execute(wl_chassis_t *);
                void exit(wl_chassis_t *);
            } _state_balance;
            struct state_spin_t : state_stub_t
            {
                void enter(wl_chassis_t *);
                void execute(wl_chassis_t *);
                void exit(wl_chassis_t *);
            } _state_spin;
            state_stub_t _state_step, _state_air;
        } _state_normal;
    } _state_active;
    wl_chassis_ctx_t _ctx{};
    wl_chassis_cmd_t _current_cmd{};
    wl_forward_kf_t _forward_kf;
    motor_base_t motors[7];
    int phase = 0, kf_updates = 0, rdob_updates = 0, lqr_updates = 0;
    control_vec_t model_input{};

    wl_chassis_t()
    {
        _forward_kf.init();
        for (int leg = 0; leg < 2; ++leg)
        {
            _ctx.motor.wheel[leg] = &motors[leg];
            _ctx.motor.joint[leg][0] = &motors[2 + 2 * leg];
            _ctx.motor.joint[leg][1] = &motors[3 + 2 * leg];
            _ctx.data.wheel[leg].direction = leg == 0 ? LEFT_WHEEL_DIRECTION : RIGHT_WHEEL_DIRECTION;
            auto &data = _ctx.data.leg[leg];
            data.current_leg_length = 0.28f;
            data.direction = leg == 0 ? LEFT_LEG_DIRECTION : RIGHT_LEG_DIRECTION;
            data.current_leg_rad = PI / 2.0f + evaluate_polynomial_ascending(
                0.28f, BETA_TRIM_POLY_COEF, BETA_TRIM_POLY_DEGREE);
        }
        _ctx.motor.yaw = &motors[6];
        motors[6].position = YAW_OFFSET;
        _ctx.data._dt = FORWARD_KF_TS;
    }
    void _update_feedback();
    void _reset_forward_kf();
    void _actual_output_update();
    void _gain_calculate_impl();
    void _calc_ddot_x_model_impl();
    void _update_forward_kf_impl();
    void _balance_control_impl();
    void _vmc_trans_j2v() {}
    void _calc_ddot_x_imu() { _ctx.data.odom.ddot_x_imu = 0.0f; }
    void _update_accel_heading_frame() {}
    void _calc_support_force() {}
    bool _detect_takeoff() { return false; }
    void _execute_landing_recovery() {}
    void _gain_calculate()
    {
        check(phase == 0, "one scheduled gain update per frame");
        phase = 1;
        _gain_calculate_impl();
    }
    void _calc_ddot_x_model()
    {
        check(phase == 1, "model uses fresh trim before KF");
        model_input = _ctx.data.output;
        _calc_ddot_x_model_impl();
        phase = 2;
    }
    void _update_forward_kf()
    {
        check(phase == 2, "KF uses current model acceleration");
        const auto other_states = _ctx.data.measured_state;
        _update_forward_kf_impl();
        ++kf_updates;
        for (int state = 2; state < STATE_DIM; ++state)
            check(_ctx.data.measured_state.data[state] == other_states.data[state],
                  "KF preserves the other twelve feedback entries");
        phase = 3;
    }
    void check_current_kf_feedback()
    {
        const auto &kf = _forward_kf.get_filter();
        check(_ctx.data.measured_state.x == kf.FilteredValue[0] &&
              _ctx.data.measured_state.dot_x == kf.FilteredValue[1], "consumer reads current KF x/v");
        check(_ctx.data.measured_state.x == _ctx.data.odom.X[0] &&
              _ctx.data.measured_state.dot_x == _ctx.data.odom.X[1], "state and diagnostic KF output agree");
    }
    void _rdob_update()
    {
        check(phase == 3, "RDOB runs after current KF");
        check_current_kf_feedback();
        ++rdob_updates;
        phase = 4;
    }
    void _calc_state_deviation() { check(phase == 4, "deviation follows RDOB"); }
    void _balance_control()
    {
        check(phase == 3 || phase == 4, "LQR runs after current KF");
        check_current_kf_feedback();
        _balance_control_impl();
        bool differs_from_raw = false;
        for (int input = 0; input < INPUT_DIM; ++input)
        {
            float expected = _ctx.data.U0[input];
            for (int state = 0; state < STATE_DIM; ++state)
            {
                float error = _ctx.data.target_state.data[state] - _ctx.data.measured_state.data[state];
                if (state == lqr_state_def::PSI)
                    error = loop_fp32_constrain(error, -PI, PI);
                expected += _ctx.data.K[input][state] * error;
            }
            const float limit = input < 2 ? MAX_T_W : input < 4 ? MAX_T_P : MAX_F_L;
            check(std::abs(_ctx.data.control.data[input] - std::clamp(expected, -limit, limit)) < 1.0e-4f,
                  "actual LQR command uses fused state");
            const float raw_command = expected + _ctx.data.K[input][0] *
                (_ctx.data.measured_state.x - _ctx.data.odom.wheel_x);
            differs_from_raw |= std::abs(_ctx.data.control.data[input] -
                std::clamp(raw_command, -limit, limit)) > 1.0e-3f;
        }
        check(differs_from_raw, "wheel odometry counterfactual gives a different command");
        ++lqr_updates;
        phase = 5;
    }
    void _vmc_trans_v2j()
    {
        check(phase == 5, "actuator conversion follows LQR");
        for (auto &leg : _ctx.data.leg)
        {
            leg.actual_out_F_L = std::clamp(leg.out_F_L, -90.0f, 90.0f);
            leg.actual_out_T_p = std::clamp(leg.out_T_p, -20.0f, 20.0f);
        }
    }
    void _send_joint_torque()
    {
        check(_ctx.data.output.F_l1 == _ctx.data.leg[0].actual_out_F_L &&
              _ctx.data.output.F_l2 == _ctx.data.leg[1].actual_out_F_L &&
              _ctx.data.output.T_p1 == _ctx.data.leg[0].actual_out_T_p &&
              _ctx.data.output.T_p2 == _ctx.data.leg[1].actual_out_T_p,
              "current limited joint output is cached before transmission");
    }
    void _send_wheel_torque()
    {
        check(_ctx.data.output.T_w1 == _ctx.data.wheel[0].out_T_w &&
              _ctx.data.output.T_w2 == _ctx.data.wheel[1].out_T_w,
              "current wheel output is cached before transmission");
        phase = 6;
    }
    void feedback_frame(float wheel_v)
    {
        for (int leg = 0; leg < 2; ++leg)
            motors[leg].rotate = wheel_v /
                (WHEEL_RADIUS * _ctx.data.wheel[leg].direction * rec_reduction_ratio);
        const float previous_x = _ctx.data.odom.X[0], previous_v = _ctx.data.odom.X[1];
        _ctx.data.odom.wheel_x = 1000.0f;
        _update_feedback();
        check(_ctx.data.measured_state.x == previous_x && _ctx.data.measured_state.dot_x == previous_v,
              "feedback does not overwrite KF with wheel odometry");
        phase = 0;
    }
};
} // namespace pyro

#include "chassis_functions.inc"

int main()
{
    pyro::wl_chassis_t chassis;
    auto &normal = chassis._state_active._state_normal;
    chassis.feedback_frame(0.0f);
    normal._state_balance.enter(&chassis);
    check(chassis._ctx.data.measured_state.x == 0.0f && chassis._ctx.data.odom.X[0] == 0.0f,
          "balance entry resets the same KF/state origin");
    for (int i = 0; i < 3; ++i)
    {
        chassis.feedback_frame(i == 1 ? 3.0f : 0.0f);
        const auto previous_output = chassis._ctx.data.output;
        const int previous_updates = chassis.kf_updates;
        normal._state_balance.execute(&chassis);
        check(chassis.phase == 6 && chassis.kf_updates == previous_updates + 1,
              "balance has one KF update before control");
        for (int j = 0; j < INPUT_DIM; ++j)
            check(chassis.model_input.data[j] == previous_output.data[j], "model uses preceding applied input");
    }
    check(chassis.rdob_updates == 3 && chassis.lqr_updates == 3, "balance RDOB/LQR run on fused feedback");
    normal._state_balance.exit(&chassis);
    normal._state_spin.enter(&chassis);
    check(chassis._ctx.data.measured_state.x == 0.0f && chassis._ctx.data.odom.X[0] == 0.0f,
          "spin entry resets the same KF/state origin");
    for (int i = 0; i < 3; ++i)
    {
        chassis.feedback_frame(i == 1 ? 3.0f : 0.0f);
        const auto previous_output = chassis._ctx.data.output;
        const int previous_updates = chassis.kf_updates;
        normal._state_spin.execute(&chassis);
        check(chassis.phase == 6 && chassis.kf_updates == previous_updates + 1,
              "spin has one KF update before control");
        for (int j = 0; j < INPUT_DIM; ++j)
            check(chassis.model_input.data[j] == previous_output.data[j], "spin model uses preceding applied input");
    }
    check(chassis.rdob_updates == 3 && chassis.lqr_updates == 6, "spin retains its original RDOB policy");
    normal._state_spin.exit(&chassis);
    check(chassis._ctx.data.odom.ddot_x_model == 0.0f, "spin exit clears model observation");
    std::puts("PASS: feedback, reset, current KF, RDOB/LQR and next-interval output in balance/spin.");
    return 0;
}
