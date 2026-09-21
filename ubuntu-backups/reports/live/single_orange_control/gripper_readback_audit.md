# 自定义夹爪反馈接口只读核查

本阶段仅阅读文件；没有机械动作、开闭爪、回零、通信配置或寄存器请求。

## 结论

生产 ToolRs485ModbusGripper 只写命令并等待 Bool 回执，没有实际开口、电流、堵转或夹持状态反馈。写命令 PASS 不能证明夹爪已闭合或夹住果实。

## 已有读取方式

- `/home/li/hand_eye_calibration/windows_gripper_code/control/arm/arm_control.py:336` 的 `readGripperState()`：Read_Multiple_Holding_Registers(port=1,address=0x002B,num=2,device=1)，按大端字节组32位，再除以2240。
- `/home/li/hand_eye_calibration/ros_modbus_gripper_max_open_test.py` 的 `read_position_mm()`：发布 `/rm_driver/read_modbus_rtu_holding_registers_cmd`，消息 `Modbusrtureadparams(address=43,device=1,type=1,num=2)`，订阅对应 `_result`，获取 `state` 与 `read_data`。
- 该测试文件头明确：读回仅确认控制器寄存器，不能独立测量真实两指间距。不要执行其 `--execute`，因为会机械回零并张开120mm。

## 寄存器依据

原始 Windows 代码定义：0x0024 归零速度，0x002A 正反转/回零命令，0x002B 坐标模式/目标位置。生产 YAML 使用相同地址，2240steps/mm，80mm=179200steps，2mm=4480steps。

没有发现自定义夹爪厂家完整寄存器表，不能把0x002B当编码器实际位置，更不能猜测其他地址是电流/堵转。

## ROS 驱动解码注意

`/home/li/ros2_ws/src/ros2_rm_robot/rm_driver/src/rm_driver.cpp:1802` 实现通用holding读取：controller_type=3且num>1时返回num*2个字节；controller_type=4返回num个整型寄存器。因此仅读需先保留原始返回长度/数值再判断，不应盲目使用旧4字节解码。

## 标准夹爪SDK不等同当前自定义夹爪

`rm_get_gripper_state` 的标准结构包含actpos/current_force等，但生产使用工具RS485自定义步进寄存器接口，没有依据表明标准夹爪协议适用。

下一步可仅读取已知0x002B命令寄存器验证通信与最近目标；实际闭合和开口标定仍需独立视觉/测量证据。
