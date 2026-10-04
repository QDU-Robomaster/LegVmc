# LegVmc

轮腿机器人五连杆腿的虚拟模型控制（VMC）解算模块 / Virtual model control (VMC) solver Module for the five-bar legs of a wheel-legged robot

## 1. 模块作用 / Purpose

LegVmc 是库型模块（`standalone: false`），提供一个计算类，在调用方调用成员函数时计算。它把两个髋关节角度换算成虚拟腿的长度和摆角（正解），把期望的虚拟腿推力和摆力矩换算成两个关节力矩（逆解），并由关节力矩估计地面支持力。角度单位 rad，长度单位 m，力单位 N，力矩单位 N·m。

`QDU-Robomaster/Wheelleg` 通过 `depends` 引入本仓库，并在内部用自己的参数创建左右两条腿的 `LegVmc` 对象。

接口：

- `VMCsolve(phi1, phi4, eulrPit, d_eulrPit, omega1, omega4, dt)`：正解。输入后、前大腿角度 `phi1`、`phi4`，机体 pitch 及其角速度，两个关节角速度，返回 `std::tuple{L0, d_L0, theta, d_theta}`（虚拟腿长、腿长变化率、摆角、摆角速度），同时更新 `GetFeedback()` 中的对应字段。
- `VMCinserve(phi1, phi4, Tp, F0)`：逆解。用上一次 `VMCsolve()` 得到的几何量构造雅可比矩阵，把沿腿推力 `F0` 和摆力矩 `Tp` 换算成两个关节力矩，返回 `std::tuple{torque_set[0], torque_set[1]}`。
- `GndDetector(T1, T2, imu_accl_z, theta, d_theta, dt)`：由两个关节反馈力矩反解虚拟腿推力 `F`（含弹簧力）和摆力矩 `Tp`，计算地面支持力 `Fn = F·cos(theta) + Tp·sin(theta) / L0`，结果为 NaN 时沿用上一次的值，再经 10 Hz 二阶低通后返回。调用顺序在 `VMCsolve()` 与 `VMCinserve()` 之后。
- `GetSpringForce()`：按内置的弹簧几何与刚度常数计算弹簧沿腿方向的推力。
- `MaxFnSolve(target_tor)`：按当前雅可比矩阵计算给定关节力矩对应的推力。
- `LqrKCalc(coe, len)`：三次多项式 `coe[0]·len³ + coe[1]·len² + coe[2]·len + coe[3]`，用于按腿长插值 LQR 增益。
- `Lqr2KCalc(coe, len1, len2)`：二元二次多项式（6 个系数），用于按左右腿长插值增益。
- `LowpassFilter(sample, cut_freq, dt)`：二阶低通滤波，与 `GndDetector()` 共用同一组滤波状态。
- `GetFeedback()`：返回 `VMCFeedback`（`L0`、`d_L0`、`theta`、`d_theta`、`F`、`Tp`、`Fn`、`torque_set[2]`、`spring_force` 等）。
- `Reset()`：清零内部几何量、滤波状态和反馈。

LegVmc is a library Module (`standalone: false`) that provides a computation class, which computes when the caller invokes its member functions. It converts the two hip joint angles into the length and swing angle of the virtual leg (forward solution), converts the desired virtual leg thrust and swing torque into two joint torques (inverse solution), and estimates the ground support force from the joint torques. Angles are in rad, lengths in m, forces in N and torques in N·m.

`QDU-Robomaster/Wheelleg` pulls in this repository through `depends` and internally creates the `LegVmc` objects of the left and right legs with its own parameters.

Interface:

- `VMCsolve(phi1, phi4, eulrPit, d_eulrPit, omega1, omega4, dt)`: the forward solution. The inputs are the back and front thigh angles `phi1` and `phi4`, the body pitch and its angular velocity, and the two joint angular velocities. It returns `std::tuple{L0, d_L0, theta, d_theta}` (virtual leg length, leg length rate, swing angle, swing angular velocity) and updates the matching fields in `GetFeedback()`.
- `VMCinserve(phi1, phi4, Tp, F0)`: the inverse solution. It builds the Jacobian from the geometry of the latest `VMCsolve()` and converts the thrust `F0` along the leg and the swing torque `Tp` into two joint torques, returning `std::tuple{torque_set[0], torque_set[1]}`.
- `GndDetector(T1, T2, imu_accl_z, theta, d_theta, dt)`: it solves the virtual leg thrust `F` (including the spring force) and the swing torque `Tp` from the two joint feedback torques, computes the ground support force `Fn = F·cos(theta) + Tp·sin(theta) / L0`, keeps the previous value when the result is NaN, and returns it after a 10 Hz second-order low-pass filter. It is called after `VMCsolve()` and `VMCinserve()`.
- `GetSpringForce()`: computes the spring thrust along the leg from the built-in spring geometry and stiffness constants.
- `MaxFnSolve(target_tor)`: computes the thrust corresponding to a given joint torque from the current Jacobian.
- `LqrKCalc(coe, len)`: the cubic polynomial `coe[0]·len³ + coe[1]·len² + coe[2]·len + coe[3]`, used to interpolate the LQR gains by leg length.
- `Lqr2KCalc(coe, len1, len2)`: a quadratic polynomial in two variables (6 coefficients), used to interpolate gains by the left and right leg lengths.
- `LowpassFilter(sample, cut_freq, dt)`: a second-order low-pass filter that shares its filter state with `GndDetector()`.
- `GetFeedback()`: returns `VMCFeedback` (`L0`, `d_L0`, `theta`, `d_theta`, `F`, `Tp`, `Fn`, `torque_set[2]`, `spring_force` and others).
- `Reset()`: clears the internal geometry, the filter state and the feedback.

## 2. 构造接口 / Constructor

```cpp
LegVmc(const Param& param = {.leg_4 = 0.25, .leg_1 = 0.25, .leg_3 = 0.215,
                             .leg_2 = 0.215, .hip_length = 1e-05});
```

依赖：无。

配置参数（`Param`，单位 m）：

- `leg_4`：前大腿长度，默认 `0.25`。
- `leg_1`：后大腿长度，默认 `0.25`。
- `leg_3`：前小腿长度，默认 `0.215`。
- `leg_2`：后小腿长度，默认 `0.215`。
- `hip_length`：两个髋关节之间的距离，默认 `1e-05`（近似共轴）。

Dependencies: none.

Configuration parameters (`Param`, in m):

- `leg_4`: front thigh length, default `0.25`.
- `leg_1`: back thigh length, default `0.25`.
- `leg_3`: front shank length, default `0.215`.
- `leg_2`: back shank length, default `0.215`.
- `hip_length`: distance between the two hip joints, default `1e-05` (approximately coaxial).

## 3. Topic

无 / None

## 4. 配置示例 / Configuration Example

LegVmc 是库，`xrobot instance add` 输出：

LegVmc is a library, and `xrobot instance add` prints:

```text
$ xrobot instance add QDU-Robomaster/LegVmc
QDU-Robomaster/LegVmc is a library (standalone: false) and cannot be instantiated
```

在 `User/xrobot.yaml` 中配置的是 `QDU-Robomaster/Wheelleg` 的实例，连杆尺寸写在它的 `vmc_left_param` 与 `vmc_right_param` 中，Wheelleg 构造时用这两组参数创建左右腿的 `LegVmc` 对象。C++ 中的典型用法：

The instance configured in `User/xrobot.yaml` is that of `QDU-Robomaster/Wheelleg`. The link dimensions are set in its `vmc_left_param` and `vmc_right_param`, from which Wheelleg creates the `LegVmc` objects of the left and right legs on construction. Typical use in C++:

```cpp
LegVmc leg({.leg_4 = 0.21f, .leg_1 = 0.21f, .leg_3 = 0.25f, .leg_2 = 0.25f,
            .hip_length = 0.0f});

std::tuple<float, float> LegTorque(float phi1, float phi4, float pitch, float d_pitch,
                                   float omega1, float omega4, float dt, float tp,
                                   float f0)
{
  leg.VMCsolve(phi1, phi4, pitch, d_pitch, omega1, omega4, dt);
  return leg.VMCinserve(phi1, phi4, tp, f0);
}
```

## 5. 依赖与硬件 / Dependencies and Hardware

依赖：C++ 标准库。

硬件：无。

Dependencies: the C++ standard library.

Hardware: none.
