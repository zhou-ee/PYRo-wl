#ifndef __PYRO_WL_CHASSIS_H__
#define __PYRO_WL_CHASSIS_H__

#include "pyro_algo_pd.h"
#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "wl_config.h"
#include "dsppp/memory_pool.hpp"
#include <dsppp/matrix.hpp>

namespace pyro
{
struct wl_chassis_deps_t
{
    struct motor_deps_t
    {
        motor_base_t *joint[2][2];
        motor_base_t *wheel[2];
    };

    struct pid_deps_t
    {
        pd_ctrl_t *leg_length[2];

        pd_ctrl_t *leg_rad[2];
        pyro::pid_t *leg_control_rad[2];   // 角度环
        pyro::pid_t *leg_control_radps[2]; // 速度环
    };
    motor_deps_t motor;
    pid_deps_t pid;
};

struct wl_chassis_cmd_t final : public cmd_base_t
{
    float delta_leg_length[2];
    float delta_leg_rad[2];
    float v;
    float wz;
    bool balance_flag = false;
    float dot_L;

    int reset_chassis_times;
    int step_times;


    enum class wl_chassis_mode_t : uint8_t
    {
        MANUAL,
    };
};

struct leg_ctx_t
{
    float target_leg_length;
    float target_leg_speed;
    float target_leg_rad;
    float target_leg_radps;
    float current_leg_length; // current_leg_length = f((θ1 - θ2) / 2)
    float current_leg_speed;
    float previous_leg_speed;
    float current_leg_accel;
    float current_leg_rad; // current_leg_rad = (θ1 + θ2) / 2
    float current_leg_radps;
    float previous_leg_radps;
    float current_leg_rad_accel;
    float error_leg_rad;

    float J_L;
    float L_wp;
    float out_F_L; // F_L * J_L = tau_2 - tau_1
    float virtual_wall_force;
    float gas_spring_force; // 沿腿长增大方向的被动广义力
    float out_T_p;          // T_p = tau_2 + tau_1
    float current_F_L;
    float current_T_p;

    float direction; // Motor-positive sign in the defined leg coordinate system
    float current_joint_rad[2]; // θ1，θ2
    float current_joint_radps[2];
    float current_joint_torque[2];
    float out_joint_torque[2];
};

struct wheel_ctx_t
{
    float direction;
    float current_radps;
    float current_T_w;
    float out_T_w;
    float out_current; // Current: 电流 (A)
};

struct odom_t
{
    float real_x;
    float real_dot_x[2]; // 0 : v_x(t) , 1 :v_x(t-1)
};

struct ins_data_t
{
    float euler_rad[3];
    float gyro[3];
    float accel[3];
};

enum class chassis_state_t : uint8_t
{
    NORMAL,
    AIR,
};

struct airborne_data_t
{
    chassis_state_t state    = chassis_state_t::NORMAL;
    bool landing_recovery    = false;
    uint16_t takeoff_counter = 0;
    uint16_t landing_counter = 0;
    float L_ref              = NORMAL_LENGTH_TARGET;
    float L_air_ref[2]       = {NORMAL_LENGTH_TARGET, NORMAL_LENGTH_TARGET};
    float accel_z_y          = 0.0f;
    float accel_z_y_lpf      = 0.0f;
    float support_force[2]   = {0.0f, 0.0f};
    float support_force_sum  = 0.0f;
};


struct flag_data_t
{
    bool leg_is_should_restart; // 紧急下力的标志位
    bool leg_is_ready;          // 复位成功的标志位
    bool step;                  // 是否上台阶的标志位
};


struct wl_chassis_data_ctx_t
{
    leg_ctx_t leg[2];
    wheel_ctx_t wheel[2];
    arm_cmsis_dsp::Vector<float, STATE_DIM> target_state{};
    arm_cmsis_dsp::Vector<float, STATE_DIM> error{};
    arm_cmsis_dsp::Vector<float, STATE_DIM> measured_state{};
    arm_cmsis_dsp::Vector<float, STATE_DIM> current_state{};
    arm_cmsis_dsp::Vector<float, STATE_DIM> next_state{};
    arm_cmsis_dsp::Vector<float, INPUT_DIM> dist{};
    arm_cmsis_dsp::Vector<float, INPUT_DIM> control{};
    arm_cmsis_dsp::Vector<float, INPUT_DIM> U0{};
    arm_cmsis_dsp::Vector<float, INPUT_DIM> output{};
    arm_cmsis_dsp::Matrix<float, INPUT_DIM, STATE_DIM> K{};
    arm_cmsis_dsp::Matrix<float, STATE_DIM, STATE_DIM> G{};
    arm_cmsis_dsp::Matrix<float, STATE_DIM, INPUT_DIM> H{};
    arm_cmsis_dsp::Matrix<float, STATE_DIM, STATE_DIM> L_x{};
    arm_cmsis_dsp::Matrix<float, INPUT_DIM, STATE_DIM> L_d{};


    flag_data_t flag;
    odom_t odom;
    ins_data_t ins;
    airborne_data_t airborne;
    float _dt;
};

struct wl_chassis_ctx_t
{
    wl_chassis_deps_t::motor_deps_t motor;
    wl_chassis_deps_t::pid_deps_t pid;
    wl_chassis_data_ctx_t data;
};

struct wl_chassis_param_t
{

    using CmdType    = wl_chassis_cmd_t;
    using ModuleDeps = wl_chassis_deps_t;
    using ModuleCtx  = wl_chassis_ctx_t;
};

class wl_chassis_t final
    : public module_base_t<wl_chassis_t, wl_chassis_param_t>
{
    friend class module_base_t<wl_chassis_t, wl_chassis_param_t>;

  public:
    wl_chassis_t(const wl_chassis_t &)            = delete;
    wl_chassis_t &operator=(const wl_chassis_t &) = delete;
    using data_ctx_t                              = wl_chassis_data_ctx_t;
    using ctx_t                                   = wl_chassis_ctx_t;

  private:
    wl_chassis_t();
    ~wl_chassis_t() override = default;

    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // 私有成员变量

    // 辅助函数
    void _vmc_trans_j2v();
    void _manual_control();
    void _fit_params();
    void _balance_control();
    void _vmc_trans_v2j();
    void _send_joint_torque() const;
    void _send_wheel_torque() const;
    void _update_accel_heading_frame();
    void _calc_support_force();
    bool _detect_takeoff();
    bool _detect_landing();
    void _execute_air_control();
    void _execute_landing_recovery();
    void _leso_update();

    float _calc_leg_length_wall_force(const leg_ctx_t &leg) const;
    float _calc_gas_spring_force(float leg_length) const;

    using owner = wl_chassis_t;


    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };
    struct fsm_active_t final : public fsm_t<owner>
    {
        void request_air();
        void request_normal();

        struct state_manual_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_normal_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_air_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_align_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_step_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        void on_enter(wl_chassis_t *ctx) override;
        void on_execute(wl_chassis_t *ctx) override;
        void on_exit(wl_chassis_t *ctx) override;

      private:
        state_manual_t _state_manual;
        state_normal_t _state_normal;
        state_air_t _state_air;
        state_align_t _state_align;
        state_step_t _state_step;
    };

    fsm_t<owner> _main_fsm;
    state_passive_t _state_passive;
    fsm_active_t _state_active;
};


} // namespace pyro
#endif // __PYRO_WL_CHASSIS_H__
