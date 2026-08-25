# Franka机械臂控制
此笔记记录了如何控制Franka机械臂进行方块的抓取
代码路径：[08-Franka机械臂控制](notes/08-Franka机械臂控制.py)

1.创建场景，Franka和cube
2.给cube补刚体、改成 dynamic、补碰撞、质量 0.05、加摩擦、弹性 0
3.用prim path锁定抓取点和放置点
4.控制台应出现类似：
<img width="319" height="69" alt="image" src="https://github.com/user-attachments/assets/28544660-0c85-49b1-b257-f3091a55810a" />

AABB size 必须大约是 0.05 m。若仍是 1 或 2，缩放失败，夹不住。

5.保持 Play，创建抓放控制器
观察 10 段动作：上方 → 下降 → 停顿 → 合爪 → 抬起 → 平移 → 放下 → 松开 → 抬起 → 收回。

<img width="291" height="142" alt="image" src="https://github.com/user-attachments/assets/a43cb420-7265-41e7-873d-0cfd1620c0b1" />

6.打印最终位置和移动距离

