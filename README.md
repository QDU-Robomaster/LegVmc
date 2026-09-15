# LegVmc

## Static assembly source line

This source line uses explicit C++ constructor dependencies and ordered instance
arguments. Inspect the current primary header with `xrobot_mod_parser --path .`;
its declarations, not old manifest/config examples, define the interface.
Historical HardwareContainer/ApplicationManager examples below apply only to the
older dynamic source tags. Device/protocol descriptions remain relevant.
See the XRobot [migration guide](https://github.com/xrobot-org/XRobot/blob/dev/MIGRATION.md).
Compilation is not hardware validation; retain version-specific board evidence.


## 1. 模块作用
轮腿五连杆解算模块。提供腿部状态与控制中间量计算。
Manifest 描述：轮腿机器人五连杆解算

## 2. 主要函数说明
1. GetFeedback: 输出当前 VMC 解算结果。
2. LqrKCalc / Lqr2KCalc: 增益计算工具函数。
3. Reset: 重置内部状态。
4. OnMonitor: 监控钩子（当前为空实现）。

## 3. 接入步骤
1. 添加模块并配置连杆参数。
2. 在 Wheelleg 中创建左右腿实例并读取反馈。
3. 联调时先检查静态几何解算是否合理。

标准命令流程：
    xrobot_add_mod LegVmc --instance-id legvmc
    xrobot_gen_main
    cube-cmake --build /home/leo/Documents/bsp-dev-c/build/debug --

## 4. 配置示例（YAML）
module: LegVmc
entry_header: Modules/LegVmc/LegVmc.hpp
constructor_args:
    param:
      leg_4: 0.25
      leg_1: 0.25
      leg_3: 0.215
      leg_2: 0.215
      hip_length: 0.00001
template_args:
[]

## 5. 依赖与硬件
Required Hardware:
[]

Depends:
[]

## 6. 代码入口
Modules/LegVmc/LegVmc.hpp
