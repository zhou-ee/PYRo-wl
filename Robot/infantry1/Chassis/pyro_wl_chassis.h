#ifndef __PYRO_WL_CHASSIS_H__
#define __PYRO_WL_CHASSIS_H__

#include "pyro_algo_pd.h"
#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "wl_config.h"
#include "dsp/window_functions.h"

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
    float delta_h;

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
    float current_leg_rad; // current_leg_rad = (θ1 + θ2) / 2
    float current_leg_radps;
    float error_leg_rad;

    float J_L;
    float L_wp;
    float out_F_L; // F_L * J_L = tau_2 - tau_1
    float out_T_p; // T_p = tau_2 + tau_1
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

struct state_vec_t
{
    union
    {
        struct
        {
            float x, dot_x;
            float psi, dot_psi;
            float theta, dot_theta;
            float phi, dot_phi;
            float h, dot_h;
            float beta1, dot_beta1;
            float beta2, dot_beta2;
        };
        float data[14];
    };
};
struct control_vec_t
{
    union
    {
        struct
        {
            float T_w1,T_w2;
            float T_p1,T_p2;
            float F_l1,F_l2;
        };
        float data[6];
    };
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

struct flag_data_t
{
    bool leg_is_should_restart;  //紧急下力的标志位
    bool leg_is_ready;           //复位成功的标志位
    bool step;                   //是否上台阶的标志位
};


struct wl_chassis_data_ctx_t
{
    leg_ctx_t leg[2];
    wheel_ctx_t wheel[2];
    state_vec_t target_state;
    state_vec_t current_state;
    control_vec_t control;
    flag_data_t flag;
    float K[INPUT_DIM][STATE_DIM];
    float U0[INPUT_DIM];
    odom_t odom;
    ins_data_t ins;
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
    void _gain_calculate();
    void _balance_control();
    void _vmc_trans_v2j();
    void _send_joint_torque() const;
    void _send_wheel_torque() const;

    using owner = wl_chassis_t;


    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };
    struct fsm_active_t final : public fsm_t<owner>
    {
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
        state_align_t  _state_align;
        state_step_t _state_step;
    };

    fsm_t<owner> _main_fsm;
    state_passive_t _state_passive;
    fsm_active_t _state_active;
};


} // namespace pyro
#endif // __PYRO_WL_CHASSIS_H__
