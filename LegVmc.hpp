#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 轮腿机器人五连杆解算
constructor_args:
    param:
      leg_4: 0.25
      leg_1: 0.25
      leg_3: 0.215
      leg_2: 0.215
      hip_length: 0.00001
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <tuple>

class LegVmc : public LibXR::Application {
 public:
  typedef struct {
    float leg_4;      /*前大腿*/
    float leg_1;      /*后大腿*/
    float leg_3;      /*前小腿*/
    float leg_2;      /*后小腿*/
    float hip_length; /*髋长度*/
  } Param;

  struct VMCFeedback {
    float L0 = 0.0f;                     // 虚拟腿长度
    float d_L0 = 0.0f;                   // 虚拟腿长度变化率
    float theta = 0.0f;                  // 虚拟腿摆角
    float d_theta = 0.0f;                // 虚拟腿摆角变化率
    float F = 0.0f;                      // 虚拟腿支持力
    float Tp = 0.0f;                     // 虚拟腿转矩
    float Fn = 0.0f;                     // 大地支持力
    float torque_set[2] = {0.0f, 0.0f};  // 输出力矩
    float spring_angle = 0.0f;           // 弹簧与推力夹角
    float spring_force = 0.0f;           // 弹簧沿phi0方向推力
  };

  /**
   * @brief VMC 的构造函数
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param param VMC参数
   * @param sample_freq 采样频率
   */
  LegVmc(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
         const Param& param)
      : param_(param) {
    UNUSED(hw);
    UNUSED(app);

    this->Reset();
  }

  /**
   * @brief 获取VMC反馈数据
   * @return const VMCFeedback& 反馈数据
   */
  const VMCFeedback& GetFeedback() const { return feedback_; }

  /**
   * @brief 监控函数（继承自Application）
   */
  void OnMonitor() override {}

  /* 正负极参考韭菜的菜 知乎 平衡步兵控制系统设计
   VMC 机体pitch正负极 d_pitch同 交龙pit反着来
            /
           /  正+
          /
  x  ---------> 0
          \	负-
           \
            \
   phi角正负极  d_phi同
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

  /* 两个大腿角度 机体角度 角速度  求出虚拟腿摆角 摆角速度  虚拟腿长
   * 虚拟腿长变化速度 */
  std::tuple<float, float, float, float> VMCsolve(float phi1, float phi4,
                                                  float eulrPit,
                                                  float d_eulrPit, float omega1,
                                                  float omega4, float dt) {
    static float body_pitch = 0.0f;
    static float d_body_pitch = 0.0f;
    body_pitch = eulrPit;
    d_body_pitch = d_eulrPit;

    /*点D B x y坐标 */
    this->vmc_leg_.YD = this->param_.leg_4 * sinf(phi4);
    this->vmc_leg_.YB = this->param_.leg_1 * sinf(phi1);
    this->vmc_leg_.XD =
        this->param_.hip_length + this->param_.leg_4 * cosf(phi4);
    this->vmc_leg_.XB = this->param_.leg_1 * cosf(phi1);

    /*BD长度*/
    this->vmc_leg_.lBD = sqrtf((this->vmc_leg_.XD - this->vmc_leg_.XB) *
                                   (this->vmc_leg_.XD - this->vmc_leg_.XB) +
                               (this->vmc_leg_.YD - this->vmc_leg_.YB) *
                                   (this->vmc_leg_.YD - this->vmc_leg_.YB));
    this->vmc_leg_.A0 =
        2 * this->param_.leg_2 * (this->vmc_leg_.XD - this->vmc_leg_.XB);
    this->vmc_leg_.B0 =
        2 * this->param_.leg_2 * (this->vmc_leg_.YD - this->vmc_leg_.YB);
    this->vmc_leg_.C0 = this->param_.leg_2 * this->param_.leg_2 +
                        this->vmc_leg_.lBD * this->vmc_leg_.lBD -
                        this->param_.leg_3 * this->param_.leg_3;
    this->vmc_leg_.phi2 =
        2 * atan2f((this->vmc_leg_.B0 +
                    sqrtf(this->vmc_leg_.A0 * this->vmc_leg_.A0 +
                          this->vmc_leg_.B0 * this->vmc_leg_.B0 -
                          this->vmc_leg_.C0 * this->vmc_leg_.C0)),
                   this->vmc_leg_.A0 + this->vmc_leg_.C0);
    this->vmc_leg_.phi3 =
        atan2f(this->vmc_leg_.YB - this->vmc_leg_.YD +
                   this->param_.leg_2 * sinf(this->vmc_leg_.phi2),
               this->vmc_leg_.XB - this->vmc_leg_.XD +
                   this->param_.leg_2 * cosf(this->vmc_leg_.phi2));

    /*点C x y坐标 */
    this->vmc_leg_.XC = this->param_.leg_1 * cosf(phi1) +
                        this->param_.leg_2 * cosf(this->vmc_leg_.phi2);
    this->vmc_leg_.YC = this->param_.leg_1 * sinf(phi1) +
                        this->param_.leg_2 * sinf(this->vmc_leg_.phi2);

    /*点C 极坐标 */
    this->vmc_leg_.L0 =
        sqrtf((this->vmc_leg_.XC - this->param_.hip_length / 2.0f) *
                  (this->vmc_leg_.XC - this->param_.hip_length / 2.0f) +
              this->vmc_leg_.YC * this->vmc_leg_.YC);
    this->vmc_leg_.phi0 =
        atan2f(this->vmc_leg_.YC,
               (this->vmc_leg_.XC - this->param_.hip_length / 2.0f));
    /* 此处magicnum来自于手撕四连杆力学 */
    vmc_leg_.n_angle = (3.1415926f - phi1 + vmc_leg_.phi2) - 0.21467f;
    vmc_leg_.n_length = sqrtf(0.043304f - 0.0202f * cosf(vmc_leg_.n_angle));

    vmc_leg_.x_length =
        0.202f *
        sqrtf(1.0f - powf((powf(vmc_leg_.n_length, 2) + 0.040804f - 0.0025f) /
                              (0.404f * vmc_leg_.n_length),
                          2));
    float k_spring = -0.5f * vmc_leg_.L0 + 1.3f;
    k_spring = std::clamp(k_spring, 1.0f, 1.5f);
    vmc_leg_.spring_torque = 475.0f * vmc_leg_.x_length / 0.25f;
    vmc_leg_.force_angle = sinf(M_PI_2 + vmc_leg_.phi2 - vmc_leg_.phi0);

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
    /* 腿长变化速度得出于雅可比 效果没有角速度反馈几何计算好 */
    this->vmc_leg_.d_L0 =
        ((this->vmc_leg_.XC - this->param_.hip_length / 2.0f) * dXC +
         this->vmc_leg_.YC * dYC) /
        this->vmc_leg_.L0;

    feedback_.L0 = vmc_leg_.L0;
    feedback_.d_L0 = vmc_leg_.d_L0;
    feedback_.theta = vmc_leg_.theta;
    feedback_.d_theta = vmc_leg_.d_theta;

    return std::make_tuple(vmc_leg_.L0, vmc_leg_.d_L0, vmc_leg_.theta,
                           vmc_leg_.d_theta);
  }
  float GetSpringForce() {
    /* 弹簧力计算 */
    feedback_.spring_force =
        2.0f * vmc_leg_.spring_torque * vmc_leg_.force_angle;
    return feedback_.spring_force;
  }
  /* 两个大腿角度 期望腿支持力 期望腿摆力矩 求出两个关节输出力矩 */
  std::tuple<float, float> VMCinserve(float phi1, float phi4, float Tp,
                                      float F0) {
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
    this->vmc_leg_.torque_set[0] =
        this->vmc_leg_.j11 * F0 + this->vmc_leg_.j12 * Tp;
    /*得到后髋关节的输出轴期望力矩，Tp为虚拟腿摆力矩的力矩*/
    this->vmc_leg_.torque_set[1] =
        this->vmc_leg_.j21 * F0 + this->vmc_leg_.j22 * Tp;

    feedback_.torque_set[0] = vmc_leg_.torque_set[0];
    feedback_.torque_set[1] = vmc_leg_.torque_set[1];

    return std::make_tuple(this->vmc_leg_.torque_set[0],
                           this->vmc_leg_.torque_set[1]);
  }
  float MaxFnSolve(float target_tor) {
    return (-vmc_leg_.j22 * target_tor - vmc_leg_.j12 * target_tor) /
           (vmc_leg_.j11 * vmc_leg_.j22 - vmc_leg_.j12 * vmc_leg_.j21);
  }

  /* 用到了前两个函数解算算出来的变量 尽量放在前两个函数之后 */
  float GndDetector(float T1, float T2, float imu_accl_z, float theta,
                    float d_theta, float dt) {
    vmc_leg_.F = GetSpringForce() + (vmc_leg_.j22 * T1 - vmc_leg_.j12 * T2) /
                                        (vmc_leg_.j11 * vmc_leg_.j22 -
                                         vmc_leg_.j12 * vmc_leg_.j21);
    vmc_leg_.Tp = (-vmc_leg_.j21 * T1 + vmc_leg_.j11 * T2) /
                  (vmc_leg_.j11 * vmc_leg_.j22 - vmc_leg_.j12 * vmc_leg_.j21);

    /* 角速度变化和腿长变化乘积太大了 有点失真*/
    // vmc_leg_.Fn =LowpassFilter(std::clamp(vmc_leg_.F * cosf(theta) +
    // vmc_leg_.Tp * sinf(theta) / vmc_leg_.L0 +
    //              5.0f +  (imu_accl_z * 9.8f  + 2.0f * vmc_leg_.d_L0 * d_theta
    //              * sinf(theta) +
    //                     vmc_leg_.L0 * d_theta * d_theta *
    //                     cosf(theta)),-200.0f,200.0f),100.0f,dt);
    vmc_leg_.Fn =
        vmc_leg_.F * cosf(theta) + vmc_leg_.Tp * sinf(theta) / vmc_leg_.L0;

    if (__constexpr_isnan(vmc_leg_.Fn)) {
      vmc_leg_.Fn = vmc_leg_.last_Fn;
    }
    vmc_leg_.Fn = LowpassFilter(vmc_leg_.Fn, 10.0f, dt);
    vmc_leg_.last_Fn = vmc_leg_.Fn;

    feedback_.F = vmc_leg_.F;
    feedback_.Tp = vmc_leg_.Tp;
    feedback_.Fn = vmc_leg_.Fn;

    return vmc_leg_.Fn;
  }

  /* 计算拟合函数结果 单变量 */
  float LqrKCalc(float* coe, float len) {
    return coe[0] * len * len * len + coe[1] * len * len + coe[2] * len +
           coe[3];
  }

  /* 计算拟合函数结果 双变量 */
  float Lqr2KCalc(float* coe, float len1, float len2) {
    return (coe[0] + coe[1] * len1 + coe[2] * len2 + coe[3] * len1 * len1 +
            coe[4] * len1 * len2 + coe[5] * len2 * len2);
  }
  /*偷偷塞一个低通滤波应该没人会管我*/
  float LowpassFilter(float sample, float cut_freq, float dt) {
    float k = cut_freq * 3.14159265f * dt;
    float k2 = k * k;
    float a0 = 1.0f + 1.41421356f * k + k2;
    float b = k2 / a0;
    float a1 = 2.0f * (k2 - 1.0f) / a0;
    float a2 = (1.0f - 1.41421356f * k + k2) / a0;

    float out = b * sample + 2.0f * b * vmc_leg_.lpf_x1_ +
                b * vmc_leg_.lpf_x2_ - a1 * vmc_leg_.lpf_y1_ -
                a2 * vmc_leg_.lpf_y2_;

    vmc_leg_.lpf_x2_ = vmc_leg_.lpf_x1_;
    vmc_leg_.lpf_x1_ = sample;
    vmc_leg_.lpf_y2_ = vmc_leg_.lpf_y1_;
    vmc_leg_.lpf_y1_ = out;
    return out;
  }

  /* 变量刷新 */
  void Reset() {
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

  struct {
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
