# 创建一个简单的场景

此笔记记录如何在 Isaac Sim 中放置场景对象、添加物理效果、配置接触/摩擦材质，以及分配视觉材质。

在本示例中：

- **Cube** → 机器人 **Body（车身）**
- **Cylinders** → 左右 **Wheels（车轮）**

## 1. 添加物体

1. 为 Body 和 Wheels 创建三个 **Xform**。
2. 创建一个 **Cube** 作为 Body，两个 **Cylinder** 作为 Wheels。
3. 调整每个物体的大小、位置和旋转，并将其放到对应的 Xform 下。

**Xform 与 Mesh 的区别：**

- **Xform**：变换节点，相当于 Mesh 的“容器”，主要存储位置、旋转、缩放。
- **Mesh**：网格节点，真正存储顶点与面片，也就是最终可见的几何形状。

## 2. 添加物理属性

1. 选中 Body 和 Wheels，按住左键点击 **+ Add**。
2. 选择 **Physics > Rigid Body with Colliders Preset**，为物体添加刚体与碰撞。

### 该预设的作用

**Rigid Body with Colliders Preset** 会同时应用：

- **Rigid Body API**：物体具有质量，并会受重力与外力影响。
- <img width="800" height="519" alt="只加刚体" src="https://github.com/user-attachments/assets/0de8d4e2-e65d-40e6-bfda-30ba4d5c9377" />

- **Collision API**：物体参与碰撞检测，避免穿模。
- <img width="800" height="480" alt="刚体和碰撞" src="https://github.com/user-attachments/assets/5e5de1d0-e56a-4159-8ee5-ddac3108192a" />


## 3. 添加接触与摩擦参数

1. 在菜单栏选择 **Create > Physics > Physics Material**。
2. 选择 **Rigid Body Material**；Stage 中会出现 `PhysicsMaterial`。
3. 在 Property 面板中调整摩擦系数等参数。

## 4. 分配视觉材质

1. 点击 **Create > Materials > OmniPBR**。
2. 选中 Body 或 Wheels，在 **Materials on Selected Model** 中选择合适的材质。
3. 编辑材质属性（例如 **Shader / Albedo**、roughness、reflectivity）。


# 坐标系转换

此笔记记录如何在 Blender 中设置物体原点与变换，再导入 Isaac Sim。  
关键点在于：在 Blender 中先执行 **Apply（Ctrl + A > All Transforms）**，把位置、旋转、缩放写入网格本身。这样导出后，**Blender 里原先的 Transform 数值不会原样出现在 Isaac Sim 的 Transform 面板中**；Isaac Sim 中通常会看到干净的初始变换（尤其是 **Rotate = 0**、**Scale = 1**），同时物体外观保持不变。

本示例模拟书本开合：一个蓝色 Cube、一个红色 Cube，分别作为两侧“书页”。

## 1. 在 Blender 中创建并准备物体

1. 新建场景后，删除默认的 Collection、Cube、Light、Camera。
2. 创建一个 Cube，并按需调整大小等参数。
3. 调整完成后，务必执行 **Ctrl + A > All Transforms**，将位置、旋转、缩放全部 Apply。

> **为什么要 Apply：**  
> Apply 会把当前变换写入网格数据。未 Apply 时，Blender 的 Scale / Rotate 只是“挂”在物体上；Apply 后这些参数被固化到几何体里，导入 Isaac Sim 时就不会把那组非初始 Transform 原样带过去。

## 2. 设置 3D Cursor

1. 打开 **View > 3D Cursor**。
2. 将 Cursor 移动到目标原点位置（例如书页的铰链轴附近）。

## 3. 将物体原点设到 Cursor

1. 选中 Cube。
2. 右键选择 **Set Origin > Origin to 3D Cursor**。
3. 确认物体原点已移动到 Cursor 所在位置。

对蓝色与红色两个 Cube 分别重复上述步骤，使两侧“书页”的旋转原点一致、便于开合。

## 4. 导出并导入 Isaac Sim

1. 导出时取消勾选 **Light**（如有其他不需要的对象也可一并排除）。
2. 将文件导入 Isaac Sim。
3. 打开物体的 **Transform** 面板检查：  
   - **Rotate** 应为 `0, 0, 0`  
   - **Scale** 应为 `1, 1, 1`  
   - **Translate** 可能仍有位移（取决于原点与摆放位置），这是正常的，以便于添加关节 

这说明 Blender 中 Apply 前的那些 Scale / Rotate **没有**被挪到 Isaac Sim 的 Transform 里，而是已经烘焙进网格；物体外观保持正确，可直接使用。

# 添加关节点

本笔记说明如何在两个物体之间添加关节（Joint）。  
前提是：关节位置应与物体在 Blender 中设置好的 **Origin** 重合；只要原点已设对，在 Isaac Sim 中通常**无需再手动计算或微调 Joint 位置**。

例子仍沿用上一篇的书本开合场景：蓝色 Cube（cube1）与红色 Cube（cube2）。

## 1. 导入后处理层级（Flattened）

将 red 与 blue 导入 Isaac Sim 后，可能会看到：

- 第二个物体（cube2）外面包了一层优先层级，例如 `red`
- 它与 cube1 **不在同一层级**
- 真正设置好的 **Origin** 在内部的 `cube` 上，而不在外层的 `red` 上

若希望 Joint 与 Origin 对齐，理想情况是删掉这层多余的 `red`。但此时常会遇到：

- `red` 删不掉
- `red` 下面的 `cube` 也挪不动

**原因：**  
导入后出现的这层优先层级（如 `red`）往往相当于一个**引用（Reference）**，类似超链接，指向其他文件中的内容。因此该层级下的对象在当前 Stage 里通常**只能查看，不能自由编辑/移动/删除**。

如图：flattened前<img width="216" height="47" alt="截图 2026-08-14 15-55-40" src="https://github.com/user-attachments/assets/6c38ac10-70bf-4d64-b0f5-7cb94cf63f0b" />      
     flattened后<img width="216" height="47" alt="截图 2026-08-14 15-57-14" src="https://github.com/user-attachments/assets/094521ca-b944-42bd-8a18-f36453b71344" />

**解决方法：**

1. 点击 **File > Save Flattened As...**
2. 打开生成的 Flattened 文件

Flattened 会把引用内容“拍平”并写入当前文件。之后你会发现：

- 优先层级下的 `cube` 可以移动了
- 多余的优先层级（如 `red`）也可以删除了

整理层级后，再在两个真正的 Cube 之间添加 Joint，才能与 Origin 正确对齐。

## 2. 同时选中两个物体

要在两个物体之间添加关节，必须先同时选中它们：

1. 在 **Stage** 中先单击 cube1 的父级变换
2. 按住 **Ctrl**，再选择 cube2

确保选中的是后续要连接的两个刚体对象。

## 3. 创建 Revolute Joint

选中两个物体后：

1. 右键选择 **Create > Physics > Joints > Revolute Joint**
2. Stage 中会出现 `RevoluteJoint`（通常出现在 cube2 下方）

该旋转关节用于模拟书本开合这类绕轴转动。

## 4. 给固定侧添加 Fixed Joint

创建 Revolute Joint 后，如果直接操作 cube2（红色），可能会出现整体乱飞。

这是因为两侧都还没有稳定的固定约束。解决方法：

- 给 cube1（蓝色，作为固定侧/底座侧）添加 **Fixed Joint**

这样蓝色一侧被固定，红色一侧才能围绕关节正常转动，而不是整组飞走。

## 5. 添加 Articulation Root

为了让这对由 Joint 连接的刚体被物理引擎按**关节系统（Articulation）**正确求解，还需要添加 **Articulation Root**。

1. 选中关节系统的根物体（通常是固定侧 cube1 / blue，或其父级 Xform）
2. 右键**Add > PhysicsPhysics > Articulation Root**

**为什么要添加：**

- Revolute / Fixed 等 Joint 把多个刚体连成一套机构后，需要一个明确的根节点
- **Articulation Root** 告诉物理引擎：从这里开始，把下游连接的刚体当作一个整体关节系统来计算
- 不加时，运动可能不稳定，或出现不符合预期的抖动、乱飞等问题

## 6. 调整关节限位，避免穿模

拖动 cube2 时，可能会穿过 cube1。

这是因为在 Isaac Sim 中，**由关节连接的相邻刚体默认通常不计算彼此碰撞**。
如下：只有不相邻的物体发生碰撞
<img width="800" height="526" alt="穿模问题1" src="https://github.com/user-attachments/assets/ac362f4f-79bc-4f4a-9958-5ea246fc8107" />

若希望两侧在开合时有接触限制、不要互相穿透，需要调整 Revolute Joint 的角度限位：

- 打开 `RevoluteJoint` 属性
- 设置 **Lower Limit / Upper Limit**

把开合角度限制在合理范围内后，红色与蓝色 Cube 就不会再无限制地穿模。设置Lower Limit为-179，Upper Limit为179
<img width="640" height="457" alt="穿模问题" src="https://github.com/user-attachments/assets/10fa75e1-c7c5-4e1b-be03-907a48a68ac3" />

# 路径问题

本笔记说明如何在**不丢失素材**的前提下，将 USD 相关文件打包并发送给他人。

## 为什么需要打包

USD 文件常通过 **Payload / Reference** 等方式引用外部素材（如其他 USD、贴图、网格等）。  
若这些 **Asset Path** 指向本机绝对路径，或指向未一并发送的文件夹，别人打开时就会出现：

- 场景打不开，或
- 能打开但素材缺失（模型、贴图、引用层丢失）

因此，制作完成后需要把主文件和它所依赖的素材**一起打包**，而不是只发单个 USD。

## 正确打包方式

1. 点击 **File > Collect and Save As...**
2. 选择保存位置后，点击 **Start**

该功能会：

- 收集当前文件所依赖的素材
- 将它们整理到同一套输出文件夹中
- 把 **Payloads** 等引用的 **Asset Path** 改写为指向该文件夹内的相对路径

# 给物体添加摄像头和传感器

本笔记基于「创建一个简单的场景」中的小车示例（**Body** + **Wheels**），说明如何为物体添加摄像头，并将其固定到车身上。

## 1. 添加摄像头
1. 在菜单栏选择 **Create > Camera**。  
   Stage 树中会出现摄像机图标，视口中也会出现表示相机视野的灰色线框。
2. 调整摄像机的位置与朝向，使其对准需要观察的方向。
3. 点击 **Window > Viewport > Viewport 2**，打开第二个视口。
4. 将其中一个视口保持为 **Perspective** 透视视图；另一个切换为车载相机视图：  
   点击该视口顶部的 **Camera**，选择 **Camera > car_camera**（以你实际创建的相机名为准）。
5. 将 `car_camera` 拖到 **Body** 下方，使其成为 Body 的子物体。  
完成后，摄像头会随 Body 一起移动，相当于固定在车身上。
<img width="800" height="430" alt="camera" src="https://github.com/user-attachments/assets/c5060a2b-7ccf-4191-b804-9aeddba05658" />
