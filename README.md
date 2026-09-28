# LegVmc

轮腿机器人五连杆腿的虚拟模型控制（VMC）解算。

`LegVmc` 是纯计算类：没有线程、topic 或定时器，只在调用方调用成员函数时计算。
它把两个髋关节角度换算成虚拟腿的长度和摆角（正解），把期望的虚拟腿推力和摆力矩换算成
两个关节力矩（逆解），并由关节力矩估计地面支持力。角度单位 rad，长度单位 m，力单位 N，
力矩单位 N·m。

`QDU-Robomaster/Wheelleg` 通过 `depends` 引入本仓库，并在内部用自己的参数创建左右两条腿
的 `LegVmc` 对象；使用 Wheelleg 时不需要单独添加 LegVmc 实例。

## 接口

- `VMCsolve(phi1, phi4, eulrPit, d_eulrPit, omega1, omega4, dt)`：正解。输入后/前大腿角度
  `phi1`/`phi4`、机体 pitch 及其角速度、两个关节角速度，返回
  `std::tuple{L0, d_L0, theta, d_theta}`（虚拟腿长、腿长变化率、摆角、摆角速度），同时更新
  `GetFeedback()` 中的对应字段。`dt` 当前未使用。
- `VMCinserve(phi1, phi4, Tp, F0)`：逆解。用上一次 `VMCsolve()` 得到的几何量构造雅可比矩阵，
  把沿腿推力 `F0` 和摆力矩 `Tp` 换算成两个关节力矩，返回 `std::tuple{torque_set[0], torque_set[1]}`。
- `GndDetector(T1, T2, imu_accl_z, theta, d_theta, dt)`：由两个关节反馈力矩反解虚拟腿推力 `F`
  （含弹簧力）和摆力矩 `Tp`，计算地面支持力 `Fn = F·cos(theta) + Tp·sin(theta) / L0`，NaN 时沿用
  上一次结果，再经 10 Hz 二阶低通后返回。`imu_accl_z`、`d_theta` 当前未使用。需在
  `VMCsolve()`、`VMCinserve()` 之后调用。
- `GetSpringForce()`：按源码中固定的弹簧几何与刚度常数计算弹簧沿腿方向的推力。
- `MaxFnSolve(target_tor)`：按当前雅可比矩阵计算给定关节力矩对应的推力。
- `LqrKCalc(coe, len)`：三次多项式 `coe[0]·len³ + coe[1]·len² + coe[2]·len + coe[3]`，用于按腿长
  插值 LQR 增益。
- `Lqr2KCalc(coe, len1, len2)`：二元二次多项式（6 个系数），用于按左右腿长插值增益。
- `LowpassFilter(sample, cut_freq, dt)`：二阶低通滤波，与 `GndDetector()` 共用同一组滤波状态。
- `GetFeedback()`：返回 `VMCFeedback`（`L0`、`d_L0`、`theta`、`d_theta`、`F`、`Tp`、`Fn`、
  `torque_set[2]`、`spring_force` 等）。
- `Reset()`：清零内部几何量、滤波状态和反馈。

## 依赖

无其他模块依赖，仅使用 LibXR 与 C++ 标准库。

## 构造接口

```cpp
LegVmc(const Param& param = {.leg_4 = 0.25, .leg_1 = 0.25, .leg_3 = 0.215,
                             .leg_2 = 0.215, .hip_length = 1e-05});
```

无依赖项。

配置（`Param`，单位 m）：

- `leg_4`：前大腿长度，默认 `0.25`。
- `leg_1`：后大腿长度，默认 `0.25`。
- `leg_3`：前小腿长度，默认 `0.215`。
- `leg_2`：后小腿长度，默认 `0.215`。
- `hip_length`：两个髋关节之间的距离，默认 `1e-05`（近似共轴）。

## 使用

```sh
xrobot module add QDU-Robomaster/LegVmc
xrobot setup
xrobot instance add QDU-Robomaster/LegVmc
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出。
LegVmc 没有依赖项，按实际连杆尺寸修改 `param` 即可：

```yaml
modules:
  - module: QDU-Robomaster/LegVmc
    id: legvmc_0
    args:
      - param:
          leg_4: '0.25'
          leg_1: '0.25'
          leg_3: '0.215'
          leg_2: '0.215'
          hip_length: '1e-05'
```

本模块不使用 BSP 对象，不需要 `XR_REGISTER`。实例本身不会主动运行，需由其他代码调用上面的接口。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/LegVmc`
（在 BSP 中）打印当前的构造函数。
