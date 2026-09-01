# Franka机械臂控制
此笔记记录了如何控制Franka机械臂进行方块的抓取

1. 配置运行参数并准备Isaacsim启动
2. 导入所需要的库
3. 定义物理常量
4. 构建环境和物体，并为物体加上刚体和碰撞
5. 初始化机械臂控制器（__init__）
6. 计算每个阶段的爪夹应该去哪（_ee_target）
7. 识别方块位置并计算爪夹抓取姿态（_read_cube）
8. 检查当前动作是否完成 (_converged)
9. 下发具体控制指令并自动切换阶段（forward）
10. 主程序
<img width="480" height="380" alt="机械臂控制" src="https://github.com/user-attachments/assets/cf0b590f-1dad-491c-b105-4623d97957e9" />


# Franka机械臂控制+传送带
此笔记对应 conveyor_pick_IK.py：方块在皮带上移动，Franka 跟踪、抓取，放到皮带外，循环若干次。并采用IK（逆运动学）：根据目标位置调整机械臂关节角完成抓取

1. 先创建 SimulationApp，再 import Isaac 的库
2. 导入库：场景、物理材质、IK
3. 跟踪方块位置预测方块拦截点
4. 计算爪夹位置+姿态
5. 检查当前动作是否完成
6. IK：解关节，成功后进入下一步，切换阶段
7. 主循环
<img width="480" height="380" alt="机械臂控制+传送带" src="https://github.com/user-attachments/assets/3d1a8037-b5fe-4622-bf2f-2edacb06830b" />
