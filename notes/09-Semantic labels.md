# Semantic Labels

本笔记记录 Isaac Sim 中语义标注的作用，以及 **Semantics API / Semantic Label** 如何为合成数据提供类别信息。

## 为什么需要语义标注

真实照片做分割、检测标注成本高、耗时长。仿真里用三维模型渲染出合成图像（Synthetic Data），可以在渲染的同时自动得到标注。

做法是：给场景中的 3D 物体（prim）挂上语义标签，再由 Replicator 根据标签生成：

- 语义分割 / 实例分割
- **2D bounding box**（紧框 / 松框）
- **3D bounding box**

这样感知模型就能用仿真数据做识别、分类训练，并用同一套标签做验证。

## Semantics API 的作用

**Semantics API** 用于给场景里的每一个模型挂上属性说明，而不是改外形或物理。

常见字段可以理解为：

- **Type（类型）**：标签属于哪一类信息，最常用的是 `class`（类别）
- **Data / Label（取值）**：具体名字，例如 `cube`、`refrigerator`、`franka`

例如：给冰箱 prim 加上 `class = refrigerator` 后，分割图和检测框就会把它识别为 `refrigerator`，而不是无名几何体。

## Semantic Label 是什么

**Semantic Label** 

此功能提供了语义标签属性的要求。这些语义可用于通过验证感知系统对对象的识别和分类，为机器学习训练提供真实标签。

常见用途：

| 标签用途 | 对应数据 |
|---------|---------|
| 类别名（`class`） | 语义分割、检测类别 |
| 实例区分 | 实例分割（谁是哪一个） |
| 结合相机投影 | 2D bounding box |
| 结合物体位姿/包围盒 | 3D bounding box |
