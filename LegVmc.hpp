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

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <tuple>

#include "app_framework.hpp"
#include "can.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "thread.hpp"

#define PI_2 1.571f

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
                                                  float d_eulrPit, float dt) {
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

    this->vmc_leg_.alpha = PI_2 - this->vmc_leg_.phi0;

    this->vmc_leg_.d_phi0 =
        (this->vmc_leg_.phi0 - this->vmc_leg_.last_phi0) / dt;
    this->vmc_leg_.d_alpha = 0.0f - this->vmc_leg_.d_phi0;

    /*虚拟腿 摆角theta 摆角速度d_theta */
    this->vmc_leg_.theta = PI_2 + body_pitch - this->vmc_leg_.phi0;
    this->vmc_leg_.d_theta = (-d_body_pitch - this->vmc_leg_.d_phi0);

    this->vmc_leg_.last_phi0 = this->vmc_leg_.phi0;

    /*虚拟腿 腿长L0 腿长变化速度d_L0 */
    this->vmc_leg_.d_L0 = (this->vmc_leg_.L0 - this->vmc_leg_.last_L0) / dt;
    this->vmc_leg_.dd_L0 =
        (this->vmc_leg_.d_L0 - this->vmc_leg_.last_d_L0) / dt;

    this->vmc_leg_.last_d_L0 = this->vmc_leg_.d_L0;
    this->vmc_leg_.last_L0 = this->vmc_leg_.L0;

    this->vmc_leg_.dd_theta =
        (this->vmc_leg_.d_theta - this->vmc_leg_.last_d_theta) / dt;
    this->vmc_leg_.last_d_theta = this->vmc_leg_.d_theta;

    // 更新反馈数据
    feedback_.L0 = vmc_leg_.L0;
    feedback_.d_L0 = vmc_leg_.d_L0;
    feedback_.theta = vmc_leg_.theta;
    feedback_.d_theta = vmc_leg_.d_theta;

    return std::make_tuple(vmc_leg_.L0, vmc_leg_.d_L0, vmc_leg_.theta,
                           vmc_leg_.d_theta);
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

    // 更新反馈数据
    feedback_.torque_set[0] = vmc_leg_.torque_set[0];
    feedback_.torque_set[1] = vmc_leg_.torque_set[1];

    return std::make_tuple(this->vmc_leg_.torque_set[0],
                           this->vmc_leg_.torque_set[1]);
  }

  /* 用到了前两个函数解算算出来的变量 尽量放在前两个函数之后 */
  float GndDetector(float T1, float T2, float imu_accl_z, float theta,
                    float d_theta) {
    vmc_leg_.F = (vmc_leg_.j22 * T1 - vmc_leg_.j12 * T2) /
                 (vmc_leg_.j11 * vmc_leg_.j22 - vmc_leg_.j12 * vmc_leg_.j21);
    vmc_leg_.Tp = (-vmc_leg_.j21 * T1 + vmc_leg_.j11 * T2) /
                  (vmc_leg_.j11 * vmc_leg_.j22 - vmc_leg_.j12 * vmc_leg_.j21);
    vmc_leg_.Fn = vmc_leg_.F * cosf(theta) +
                  vmc_leg_.Tp * sinf(theta) / vmc_leg_.L0 + 5 +
                  0.5 * (imu_accl_z * 9.8 - vmc_leg_.dd_L0 * cosf(theta) +
                         2 * vmc_leg_.d_L0 * d_theta * sinf(theta) +
                         vmc_leg_.L0 * d_theta * d_theta * cosf(theta));
    // fnfilter_.Apply(vmc_leg_.Fn);

    // 更新反馈数据
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

  /* 变量刷新 */
  void Reset() {
    vmc_leg_.L0 = 0;
    vmc_leg_.phi0 = 0;
    vmc_leg_.alpha = 0;
    vmc_leg_.d_alpha = 0;

    vmc_leg_.lBD = 0;

    vmc_leg_.d_phi0 = 0;
    vmc_leg_.last_phi0 = 0;

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
    vmc_leg_.last_d_theta = 0;
    vmc_leg_.dd_theta = 0;

    vmc_leg_.d_L0 = 0;
    vmc_leg_.dd_L0 = 0;
    vmc_leg_.last_L0 = 0;
    vmc_leg_.last_d_L0 = 0;
    vmc_leg_.first_flag = 0;
    vmc_leg_.leg_flag = 0;

    feedback_ = VMCFeedback{};
  }

 private:
  Param param_;
  // float dt_min_;
  VMCFeedback feedback_;

  struct {
    float XB, YB;  // B点的坐标
    float XD, YD;  // D点的坐标

    float XC, YC;    // C点的直角坐标
    float L0, phi0;  // C点的极坐标
    float alpha;
    float d_alpha;

    float lBD;  // BD两点的距离

    float d_phi0;     // 现在C点角度phi0的变换率
    float last_phi0;  // 上一次C点角度，用于计算角度phi0的变换率d_phi0

    float A0, B0, C0;  // 中间变量
    float phi2, phi3;

    float j11, j12, j21, j22;  // 笛卡尔空间力到关节空间的力的雅可比矩阵系数
    float torque_set[2];

    float theta;
    float d_theta;  // theta的一阶导数
    float last_d_theta;
    float dd_theta;  // theta的二阶导数

    float d_L0;   // L0的一阶导数
    float dd_L0;  // L0的二阶导数
    float last_L0;
    float last_d_L0;

    float F;   // 虚拟腿支持力
    float Tp;  // 虚拟腿转矩
    float Fn;  // 大地支持力

    uint8_t first_flag;
    uint8_t leg_flag;  // 腿长完成标志

  } vmc_leg_;
};
