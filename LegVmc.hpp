#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 轮腿机器人五连杆腿的虚拟模型控制（VMC）解算模块 / Virtual model control (VMC) solver Module for the five-bar legs of a wheel-legged robot
standalone: false
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <numbers>
#include <tuple>

/**
 * @brief 轮腿机器人五连杆腿的虚拟模型控制（VMC）解算类。
 *        Virtual model control (VMC) solver class for the five-bar leg of a
 *        wheel-legged robot.
 */
class LegVmc
{
 public:
  /**
   * @brief 连杆尺寸参数，单位 m。
   *        Link dimension parameters in m.
   */
  typedef struct
  {
    float leg_4;       ///< 前大腿长度 Front thigh length
    float leg_1;       ///< 后大腿长度 Back thigh length
    float leg_3;       ///< 前小腿长度 Front shank length
    float leg_2;       ///< 后小腿长度 Back shank length
    float hip_length;  ///< 两个髋关节之间的距离 Distance between the two hip joints
  } Param;

  /**
   * @brief VMC 反馈数据。
   *        VMC feedback data.
   */
  struct VMCFeedback
  {
    float L0 = 0.0f;       ///< 虚拟腿长度 (m) Virtual leg length (m)
    float d_L0 = 0.0f;     ///< 虚拟腿长度变化率 (m/s) Virtual leg length rate (m/s)
    float theta = 0.0f;    ///< 虚拟腿摆角 (rad) Virtual leg swing angle (rad)
    float d_theta = 0.0f;  ///< 虚拟腿摆角变化率 (rad/s)
                           ///< Virtual leg swing angular velocity (rad/s)
    float F = 0.0f;        ///< 虚拟腿推力 (N) Virtual leg thrust (N)
    float Tp = 0.0f;       ///< 虚拟腿摆力矩 (N·m) Virtual leg swing torque (N·m)
    float Fn = 0.0f;       ///< 地面支持力 (N) Ground support force (N)
    float torque_set[2] = {0.0f, 0.0f};  ///< 两个关节的输出力矩 (N·m)
                                         ///< Output torques of the two joints (N·m)
    float spring_angle = 0.0f;           ///< 弹簧与推力夹角 Spring-to-thrust angle
    float spring_force = 0.0f;           ///< 弹簧沿 phi0 方向的推力 (N)
                                         ///< Spring thrust along phi0 (N)
  };

  /**
   * @brief 构造 LegVmc 并复位内部状态。
   *        Construct LegVmc and reset the internal state.
   *
   * @param param 连杆尺寸参数。
   *              Link dimension parameters.
   */
  LegVmc(const Param& param = {.leg_4 = 0.25,
                               .leg_1 = 0.25,
                               .leg_3 = 0.215,
                               .leg_2 = 0.215,
                               .hip_length = 1e-05})
      : param_(param)
  {
    this->Reset();
  }

  /**
   * @brief 获取 VMC 反馈数据。
   *        Get the VMC feedback data.
   *
   * @return 反馈数据的引用。
   *         Reference to the feedback data.
   */
  const VMCFeedback& GetFeedback() const { return feedback_; }

  /* 符号约定：机体 pitch 及其角速度的正负方向如下
            /
           /  正+
          /
  x  ---------> 0
          \	负-
           \
            \
   phi 角及其角速度的正负方向如下
            /
           /  正+
          /
         /  正+
        /
x  ---------> 0
        \	负-
         \
          \
       后 <---->前
     /phi1-----phi4\
        /        \
        \        /
         \  OO  /
          \O轮O/
            OO
*/

  /**
   * @brief 正解：由两个大腿角度、机体 pitch 与关节角速度求虚拟腿长、腿长变化率、
   *        摆角和摆角速度，并更新反馈。
   *        Forward solution: compute the virtual leg length, its rate, the swing
   *        angle and the swing angular velocity from the two thigh angles, the body
   *        pitch and the joint angular velocities, and update the feedback.
   *
   * @param phi1 后大腿角度 (rad)。
   *             Back thigh angle (rad).
   * @param phi4 前大腿角度 (rad)。
   *             Front thigh angle (rad).
   * @param eulrPit 机体 pitch (rad)。
   *                Body pitch (rad).
   * @param d_eulrPit 机体 pitch 角速度 (rad/s)。
   *                  Body pitch angular velocity (rad/s).
   * @param omega1 后关节角速度 (rad/s)。
   *               Back joint angular velocity (rad/s).
   * @param omega4 前关节角速度 (rad/s)。
   *               Front joint angular velocity (rad/s).
   * @param dt 控制周期 (s)，保留参数。
   *           Control period (s), a reserved parameter.
   * @return `{L0, d_L0, theta, d_theta}`：虚拟腿长、腿长变化率、摆角、摆角速度。
   *         `{L0, d_L0, theta, d_theta}`: virtual leg length, leg length rate, swing
   *         angle and swing angular velocity.
   */
  std::tuple<float, float, float, float> VMCsolve(float phi1, float phi4, float eulrPit,
                                                  float d_eulrPit, float omega1,
                                                  float omega4, float dt)
  {
    static float body_pitch = 0.0f;
    static float d_body_pitch = 0.0f;
    body_pitch = eulrPit;
    d_body_pitch = d_eulrPit;

    /*点D B x y坐标 */
    this->vmc_leg_.YD = this->param_.leg_4 * sinf(phi4);
    this->vmc_leg_.YB = this->param_.leg_1 * sinf(phi1);
    this->vmc_leg_.XD = this->param_.hip_length + this->param_.leg_4 * cosf(phi4);
    this->vmc_leg_.XB = this->param_.leg_1 * cosf(phi1);

    /*BD长度*/
    this->vmc_leg_.lBD = sqrtf((this->vmc_leg_.XD - this->vmc_leg_.XB) *
                                   (this->vmc_leg_.XD - this->vmc_leg_.XB) +
                               (this->vmc_leg_.YD - this->vmc_leg_.YB) *
                                   (this->vmc_leg_.YD - this->vmc_leg_.YB));
    this->vmc_leg_.A0 = 2 * this->param_.leg_2 * (this->vmc_leg_.XD - this->vmc_leg_.XB);
    this->vmc_leg_.B0 = 2 * this->param_.leg_2 * (this->vmc_leg_.YD - this->vmc_leg_.YB);
    this->vmc_leg_.C0 = this->param_.leg_2 * this->param_.leg_2 +
                        this->vmc_leg_.lBD * this->vmc_leg_.lBD -
                        this->param_.leg_3 * this->param_.leg_3;
    this->vmc_leg_.phi2 =
        2 * atan2f((this->vmc_leg_.B0 + sqrtf(this->vmc_leg_.A0 * this->vmc_leg_.A0 +
                                              this->vmc_leg_.B0 * this->vmc_leg_.B0 -
                                              this->vmc_leg_.C0 * this->vmc_leg_.C0)),
                   this->vmc_leg_.A0 + this->vmc_leg_.C0);
    this->vmc_leg_.phi3 = atan2f(this->vmc_leg_.YB - this->vmc_leg_.YD +
                                     this->param_.leg_2 * sinf(this->vmc_leg_.phi2),
                                 this->vmc_leg_.XB - this->vmc_leg_.XD +
                                     this->param_.leg_2 * cosf(this->vmc_leg_.phi2));

    /*点C x y坐标 */
    this->vmc_leg_.XC =
        this->param_.leg_1 * cosf(phi1) + this->param_.leg_2 * cosf(this->vmc_leg_.phi2);
    this->vmc_leg_.YC =
        this->param_.leg_1 * sinf(phi1) + this->param_.leg_2 * sinf(this->vmc_leg_.phi2);

    /*点C 极坐标 */
    this->vmc_leg_.L0 = sqrtf((this->vmc_leg_.XC - this->param_.hip_length / 2.0f) *
                                  (this->vmc_leg_.XC - this->param_.hip_length / 2.0f) +
                              this->vmc_leg_.YC * this->vmc_leg_.YC);
    this->vmc_leg_.phi0 =
        atan2f(this->vmc_leg_.YC, (this->vmc_leg_.XC - this->param_.hip_length / 2.0f));
    /* 以下常数由弹簧连杆的受力推导得到 */
    vmc_leg_.n_angle = (3.1415926f - phi1 + vmc_leg_.phi2) - 0.21467f;
    vmc_leg_.n_length = sqrtf(0.043304f - 0.0202f * cosf(vmc_leg_.n_angle));

    vmc_leg_.x_length =
        0.202f * sqrtf(1.0f - powf((powf(vmc_leg_.n_length, 2) + 0.040804f - 0.0025f) /
                                       (0.404f * vmc_leg_.n_length),
                                   2));
    float k_spring = -0.5f * vmc_leg_.L0 + 1.3f;
    k_spring = std::clamp(k_spring, 1.0f, 1.5f);
    vmc_leg_.spring_torque = 475.0f * vmc_leg_.x_length / 0.25f;
    vmc_leg_.force_angle = sinf(std::numbers::pi / 2 + vmc_leg_.phi2 - vmc_leg_.phi0);

    this->vmc_leg_.alpha = 1.571f - this->vmc_leg_.phi0;

    this->vmc_leg_.d_phi0 = (omega1 + omega4) / 2.0f;

    /*虚拟腿 摆角theta 摆角速度d_theta */
    this->vmc_leg_.theta = 1.571f + body_pitch - this->vmc_leg_.phi0;
    this->vmc_leg_.d_theta = (-d_body_pitch - this->vmc_leg_.d_phi0);

    float sin_p2_p3 = sinf(this->vmc_leg_.phi2 - this->vmc_leg_.phi3);
    float d_phi2 =
        fabsf(sin_p2_p3) > 1e-6f
            ? (-this->param_.leg_1 * sinf(phi1 - this->vmc_leg_.phi3) * omega1 +
               this->param_.leg_4 * sinf(phi4 - this->vmc_leg_.phi3) * omega4) /
                  (this->param_.leg_2 * sin_p2_p3)
            : 0.0f;
    float dXC = -this->param_.leg_1 * sinf(phi1) * omega1 -
                this->param_.leg_2 * sinf(this->vmc_leg_.phi2) * d_phi2;
    float dYC = this->param_.leg_1 * cosf(phi1) * omega1 +
                this->param_.leg_2 * cosf(this->vmc_leg_.phi2) * d_phi2;
    /* 腿长变化率：对 C 点坐标求导得到 */
    this->vmc_leg_.d_L0 = ((this->vmc_leg_.XC - this->param_.hip_length / 2.0f) * dXC +
                           this->vmc_leg_.YC * dYC) /
                          this->vmc_leg_.L0;

    feedback_.L0 = vmc_leg_.L0;
    feedback_.d_L0 = vmc_leg_.d_L0;
    feedback_.theta = vmc_leg_.theta;
    feedback_.d_theta = vmc_leg_.d_theta;

    return std::make_tuple(vmc_leg_.L0, vmc_leg_.d_L0, vmc_leg_.theta, vmc_leg_.d_theta);
  }
  /**
   * @brief 按内置的弹簧几何与刚度常数计算弹簧沿腿方向的推力。
   *        Compute the spring thrust along the leg from the built-in spring geometry
   *        and stiffness constants.
   *
   * @return 弹簧推力 (N)。
   *         Spring thrust (N).
   */
  float GetSpringForce()
  {
    /* 弹簧力计算 */
    feedback_.spring_force = 2.0f * vmc_leg_.spring_torque * vmc_leg_.force_angle;
    return feedback_.spring_force;
  }
  /**
   * @brief 逆解：由期望的腿推力和摆力矩求两个关节的输出力矩，使用最近一次
   *        `VMCsolve()` 的几何量。
   *        Inverse solution: compute the output torques of the two joints from the
   *        desired leg thrust and swing torque, using the geometry of the latest
   *        `VMCsolve()`.
   *
   * @param phi1 后大腿角度 (rad)。
   *             Back thigh angle (rad).
   * @param phi4 前大腿角度 (rad)。
   *             Front thigh angle (rad).
   * @param Tp 期望的虚拟腿摆力矩 (N·m)。
   *           Desired virtual leg swing torque (N·m).
   * @param F0 期望的沿腿推力 (N)。
   *           Desired thrust along the leg (N).
   * @return `{torque_set[0], torque_set[1]}`：`phi1`、`phi4` 对应关节的输出力矩
   *         (N·m)。
   *         `{torque_set[0], torque_set[1]}`: output torques of the joints of `phi1`
   *         and `phi4` (N·m).
   */
  std::tuple<float, float> VMCinserve(float phi1, float phi4, float Tp, float F0)
  {
    /*jacobian矩阵计算*/
    this->vmc_leg_.j11 =
        (this->param_.leg_1 * sinf(this->vmc_leg_.phi0 - this->vmc_leg_.phi3) *
         sinf(phi1 - this->vmc_leg_.phi2)) /
        sinf(this->vmc_leg_.phi3 - this->vmc_leg_.phi2);
    this->vmc_leg_.j12 =
        (this->param_.leg_1 * cosf(this->vmc_leg_.phi0 - this->vmc_leg_.phi3) *
         sinf(phi1 - this->vmc_leg_.phi2)) /
        (this->vmc_leg_.L0 * sinf(this->vmc_leg_.phi3 - this->vmc_leg_.phi2));
    this->vmc_leg_.j21 =
        (this->param_.leg_4 * sinf(this->vmc_leg_.phi0 - this->vmc_leg_.phi2) *
         sinf(this->vmc_leg_.phi3 - phi4)) /
        sinf(this->vmc_leg_.phi3 - this->vmc_leg_.phi2);
    this->vmc_leg_.j22 =
        (this->param_.leg_4 * cosf(this->vmc_leg_.phi0 - this->vmc_leg_.phi2) *
         sinf(this->vmc_leg_.phi3 - phi4)) /
        (this->vmc_leg_.L0 * sinf(this->vmc_leg_.phi3 - this->vmc_leg_.phi2));

    /*得到前髋关节的输出轴期望力矩，F0为五连杆机构末端沿腿的推力*/
    this->vmc_leg_.torque_set[0] = this->vmc_leg_.j11 * F0 + this->vmc_leg_.j12 * Tp;
    /*得到后髋关节的输出轴期望力矩，Tp为虚拟腿摆力矩的力矩*/
    this->vmc_leg_.torque_set[1] = this->vmc_leg_.j21 * F0 + this->vmc_leg_.j22 * Tp;

    feedback_.torque_set[0] = vmc_leg_.torque_set[0];
    feedback_.torque_set[1] = vmc_leg_.torque_set[1];

    return std::make_tuple(this->vmc_leg_.torque_set[0], this->vmc_leg_.torque_set[1]);
  }
  /**
   * @brief 按当前雅可比矩阵计算给定关节力矩对应的推力。
   *        Compute the thrust corresponding to a given joint torque from the
   *        current Jacobian.
   *
   * @param target_tor 关节力矩 (N·m)。
   *                   Joint torque (N·m).
   * @return 推力 (N)。
   *         Thrust (N).
   */
  float MaxFnSolve(float target_tor)
  {
    return (-vmc_leg_.j22 * target_tor - vmc_leg_.j12 * target_tor) /
           (vmc_leg_.j11 * vmc_leg_.j22 - vmc_leg_.j12 * vmc_leg_.j21);
  }

  /**
   * @brief 由关节反馈力矩估计虚拟腿推力、摆力矩和地面支持力，结果经 10 Hz 二阶低通
   *        滤波。使用 `VMCsolve()` 与 `VMCinserve()` 的中间量，在二者之后调用。
   *        Estimate the virtual leg thrust, the swing torque and the ground support
   *        force from the joint feedback torques, with the result passed through a
   *        10 Hz second-order low-pass filter. It uses intermediate values of
   *        `VMCsolve()` and `VMCinserve()` and is called after both.
   *
   * @param T1 `phi1` 对应关节的反馈力矩 (N·m)。
   *           Feedback torque of the joint of `phi1` (N·m).
   * @param T2 `phi4` 对应关节的反馈力矩 (N·m)。
   *           Feedback torque of the joint of `phi4` (N·m).
   * @param imu_accl_z 机体 z 向加速度，保留参数。
   *                   Body z acceleration, a reserved parameter.
   * @param theta 虚拟腿摆角 (rad)。
   *              Virtual leg swing angle (rad).
   * @param d_theta 虚拟腿摆角速度 (rad/s)，保留参数。
   *                Virtual leg swing angular velocity (rad/s), a reserved parameter.
   * @param dt 滤波周期 (s)。
   *           Filter period (s).
   * @return 滤波后的地面支持力 (N)。
   *         Filtered ground support force (N).
   */
  float GndDetector(float T1, float T2, float imu_accl_z, float theta, float d_theta,
                    float dt)
  {
    vmc_leg_.F = GetSpringForce() +
                 (vmc_leg_.j22 * T1 - vmc_leg_.j12 * T2) /
                     (vmc_leg_.j11 * vmc_leg_.j22 - vmc_leg_.j12 * vmc_leg_.j21);
    vmc_leg_.Tp = (-vmc_leg_.j21 * T1 + vmc_leg_.j11 * T2) /
                  (vmc_leg_.j11 * vmc_leg_.j22 - vmc_leg_.j12 * vmc_leg_.j21);

    vmc_leg_.Fn = vmc_leg_.F * cosf(theta) + vmc_leg_.Tp * sinf(theta) / vmc_leg_.L0;

    if (std::isnan(vmc_leg_.Fn))
    {
      vmc_leg_.Fn = vmc_leg_.last_Fn;
    }
    vmc_leg_.Fn = LowpassFilter(vmc_leg_.Fn, 10.0f, dt);
    vmc_leg_.last_Fn = vmc_leg_.Fn;

    feedback_.F = vmc_leg_.F;
    feedback_.Tp = vmc_leg_.Tp;
    feedback_.Fn = vmc_leg_.Fn;

    return vmc_leg_.Fn;
  }

  /**
   * @brief 计算单变量三次拟合多项式 `coe[0]·len³ + coe[1]·len² + coe[2]·len + coe[3]`。
   *        Evaluate the cubic fitting polynomial in one variable
   *        `coe[0]·len³ + coe[1]·len² + coe[2]·len + coe[3]`.
   *
   * @param coe 4 个多项式系数。
   *            Four polynomial coefficients.
   * @param len 腿长 (m)。
   *            Leg length (m).
   * @return 多项式的值。
   *         Value of the polynomial.
   */
  float LqrKCalc(float* coe, float len)
  {
    return coe[0] * len * len * len + coe[1] * len * len + coe[2] * len + coe[3];
  }

  /**
   * @brief 计算双变量二次拟合多项式
   *        `coe[0] + coe[1]·len1 + coe[2]·len2 + coe[3]·len1² + coe[4]·len1·len2 +
   *        coe[5]·len2²`。
   *        Evaluate the quadratic fitting polynomial in two variables
   *        `coe[0] + coe[1]·len1 + coe[2]·len2 + coe[3]·len1² + coe[4]·len1·len2 +
   *        coe[5]·len2²`.
   *
   * @param coe 6 个多项式系数。
   *            Six polynomial coefficients.
   * @param len1 第一条腿的腿长 (m)。
   *             Leg length of the first leg (m).
   * @param len2 第二条腿的腿长 (m)。
   *             Leg length of the second leg (m).
   * @return 多项式的值。
   *         Value of the polynomial.
   */
  float Lqr2KCalc(float* coe, float len1, float len2)
  {
    return (coe[0] + coe[1] * len1 + coe[2] * len2 + coe[3] * len1 * len1 +
            coe[4] * len1 * len2 + coe[5] * len2 * len2);
  }
  /**
   * @brief 二阶低通滤波，与 `GndDetector()` 共用同一组滤波状态。
   *        Second-order low-pass filter that shares its filter state with
   *        `GndDetector()`.
   *
   * @param sample 输入采样值。
   *               Input sample.
   * @param cut_freq 截止频率 (Hz)。
   *                 Cut-off frequency (Hz).
   * @param dt 采样周期 (s)。
   *           Sampling period (s).
   * @return 滤波输出。
   *         Filter output.
   */
  float LowpassFilter(float sample, float cut_freq, float dt)
  {
    float k = cut_freq * 3.14159265f * dt;
    float k2 = k * k;
    float a0 = 1.0f + 1.41421356f * k + k2;
    float b = k2 / a0;
    float a1 = 2.0f * (k2 - 1.0f) / a0;
    float a2 = (1.0f - 1.41421356f * k + k2) / a0;

    float out = b * sample + 2.0f * b * vmc_leg_.lpf_x1_ + b * vmc_leg_.lpf_x2_ -
                a1 * vmc_leg_.lpf_y1_ - a2 * vmc_leg_.lpf_y2_;

    vmc_leg_.lpf_x2_ = vmc_leg_.lpf_x1_;
    vmc_leg_.lpf_x1_ = sample;
    vmc_leg_.lpf_y2_ = vmc_leg_.lpf_y1_;
    vmc_leg_.lpf_y1_ = out;
    return out;
  }

  /**
   * @brief 清零内部几何量、滤波状态和反馈。
   *        Clear the internal geometry, the filter state and the feedback.
   */
  void Reset()
  {
    vmc_leg_.L0 = 0;
    vmc_leg_.phi0 = 0;
    vmc_leg_.alpha = 0;

    vmc_leg_.lBD = 0;

    vmc_leg_.d_phi0 = 0;

    vmc_leg_.A0 = 0;
    vmc_leg_.B0 = 0;
    vmc_leg_.C0 = 0;
    vmc_leg_.phi2 = 0;
    vmc_leg_.phi3 = 0;

    vmc_leg_.j11 = 0;
    vmc_leg_.j12 = 0;
    vmc_leg_.j21 = 0;
    vmc_leg_.j22 = 0;
    vmc_leg_.torque_set[0] = 0;
    vmc_leg_.torque_set[1] = 0;

    vmc_leg_.theta = 0;
    vmc_leg_.d_theta = 0;
    vmc_leg_.d_L0 = 0;

    vmc_leg_.lpf_x1_ = vmc_leg_.lpf_x2_ = 0.0f;
    vmc_leg_.lpf_y1_ = vmc_leg_.lpf_y2_ = 0.0f;

    feedback_ = VMCFeedback{};
  }

 private:
  Param param_;
  VMCFeedback feedback_;

  struct
  {
    float XB, YB;  // B点的坐标
    float XD, YD;  // D点的坐标

    float XC, YC;    // C点的直角坐标
    float L0, phi0;  // C点的极坐标
    float alpha;

    float lBD;  // BD两点的距离

    float d_phi0;      // 现在C点角度phi0的变换率
    float A0, B0, C0;  // 中间变量
    float phi2, phi3;

    float j11, j12, j21, j22;  // 笛卡尔空间力到关节空间的力的雅可比矩阵系数
    float torque_set[2];

    float theta;
    float d_theta;  // theta的一阶导数
    float d_L0;     // L0的一阶导数

    float F;                   // 虚拟腿支持力
    float Tp;                  // 虚拟腿转矩
    float last_Fn, Fn = 0.0f;  // 大地支持力

    float lpf_x1_ = 0.0f, lpf_x2_ = 0.0f;
    float lpf_y1_ = 0.0f, lpf_y2_ = 0.0f;

    float n_length, n_angle, x_length, force_angle, spring_torque;

  } vmc_leg_;
};
