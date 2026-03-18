# Welding Vision Prototype

面向船舶中组立场景的 3D 点云焊缝识别原型。

这个项目把“环境点云干扰”拆成一条分层处理链路：先从原始点云中稳定提取工件点云，再从工件点云中提取焊缝候选，最后用工艺和场景约束压低误检，适合作为免编程焊接视觉前端的原型基础。

当前版本是一个纯 Python 实现，不依赖 `numpy`、`open3d` 或 `pcl`，便于在依赖受限的环境里直接运行、验证和二次开发。

## 项目目标

- 从 3D 相机采集的原始点云中去除环境干扰
- 在无 CAD、无模板的前提下提取工件主体
- 从工件几何关系中识别焊缝候选中心线
- 用长度、连续性、位置和可达性规则筛掉误检
- 为后续接入真实点云库、机器人系统和深度学习分割提供稳定基线

## 适用场景

- 船舶中组立、协作臂搬运到固定工位
- 工位布局相对稳定，但工件姿态存在一定偏差
- 可以采集空工位背景点云
- 当前优先级是降低误检、提高稳定性，而不是追求全品类泛化

## 当前实现能力

### 1. 前处理与环境去除

- ROI 空间裁剪
- 体素下采样
- 半径离群点去除
- 背景建模扣除
- 基于 RANSAC 的环境大平面剔除
- 欧式聚类保留工件簇

### 2. 焊缝候选提取

- 在工件点云中提取多个平面基元
- 计算相邻平面的交线作为焊缝候选
- 基于交线邻域点的连续性构造线段
- 过滤过短、支撑点不足的候选线段

### 3. 规则评分与结果输出

- 几何角度评分
- 连续性评分
- 高度位置评分
- 近似可达性评分
- 置信度阈值筛选最终焊缝

## 算法流程

完整链路如下：

1. 输入原始点云 `P_raw`
2. 使用 ROI 裁剪，只保留工件理论工作空间
3. 下采样并去除离群噪点
4. 使用空工位背景点云做差分，去除静态环境
5. 提取并剔除贴近工位边界的大平面
6. 通过欧式聚类保留工件主体簇
7. 在工件点云中做平面分割
8. 通过平面交线生成焊缝候选
9. 对候选焊缝进行长度、连续性、位置、可达性评分
10. 输出高置信焊缝 `S_final`

## 仓库结构

```text
welding/
├── configs/
│   └── default_pipeline.json
├── src/
│   └── welding_vision/
│       ├── cli.py
│       ├── config.py
│       ├── geometry.py
│       ├── io.py
│       ├── models.py
│       ├── pipeline.py
│       └── synthetic.py
├── tests/
│   └── test_pipeline.py
└── README.md
```

## 关键模块说明

- `src/welding_vision/pipeline.py`
  核心处理管线，串联环境去除、工件分割、平面提取和焊缝评分。
- `src/welding_vision/geometry.py`
  纯 Python 几何计算，包括向量运算、平面拟合、平面交线和线段支撑分析。
- `src/welding_vision/config.py`
  读取和管理管线参数。
- `src/welding_vision/cli.py`
  命令行入口，便于直接接入点云文件运行。
- `src/welding_vision/synthetic.py`
  生成模拟的船舶中组立场景点云，用于演示和回归测试。
- `tests/test_pipeline.py`
  单元测试，覆盖背景扣除、焊缝检出和空场景无误检。

## 输入格式

支持 `JSON` 和 `CSV`。

### JSON 对象格式

```json
{
  "points": [
    {"x": 0.0, "y": 0.0, "z": 0.0},
    {"x": 0.1, "y": 0.0, "z": 0.0}
  ]
}
```

### JSON 数组格式

```json
[
  {"x": 0.0, "y": 0.0, "z": 0.0},
  {"x": 0.1, "y": 0.0, "z": 0.0}
]
```

### CSV 格式

```csv
x,y,z
0.0,0.0,0.0
0.1,0.0,0.0
```

## 快速开始

### 1. 生成演示数据

```bash
PYTHONPATH=src python3 -m welding_vision.synthetic --output-dir examples
```

会生成两个文件：

- `examples/scene.json`
- `examples/background.json`

其中：

- `scene.json` 包含工件、焊缝附近点和环境干扰
- `background.json` 是空工位背景点云

### 2. 运行点云处理与焊缝识别

```bash
PYTHONPATH=src python3 -m welding_vision.cli \
  --input examples/scene.json \
  --background examples/background.json \
  --config configs/default_pipeline.json \
  --output out/result.json
```

如果不传 `--output`，结果会直接输出到标准输出。

### 3. 查看输出结果

输出文件 `out/result.json` 包含：

- `metrics`
  各处理阶段剩余点数统计
- `workpiece_points`
  最终保留的工件点云
- `planes`
  工件点云中识别出的平面
- `seam_candidates`
  所有焊缝候选
- `final_seams`
  通过阈值筛选后的最终焊缝

## 命令行参数

```bash
PYTHONPATH=src python3 -m welding_vision.cli --help
```

支持参数：

- `--input`
  原始点云文件，必填
- `--background`
  空工位背景点云，可选但强烈建议提供
- `--config`
  管线参数配置文件，必填
- `--output`
  结果输出路径，可选
- `--seed`
  RANSAC 随机种子，可选

## 默认配置说明

默认配置文件：`configs/default_pipeline.json`

这套参数更偏向船舶中组立固定工位的低风险路线。关键字段如下：

- `roi`
  视觉工作空间边界。越准确，环境去除效果越好。
- `voxel_size`
  体素下采样分辨率。
- `outlier_radius`
  半径离群点过滤邻域。
- `outlier_min_neighbors`
  保留点至少需要的邻域点数。
- `background_tolerance`
  背景差分匹配容差。
- `plane_distance_threshold`
  点到平面的内点距离阈值。
- `environment_plane_min_span`
  被视为环境大平面的最小尺寸跨度。
- `environment_boundary_margin`
  与工位边界接近到什么程度时，平面会被判定为环境面。
- `cluster_keep_top_k`
  工件被夹具或遮挡分裂时允许保留的簇数量。
- `seam_line_distance_threshold`
  点被视为候选焊缝线支撑点的最大距离。
- `seam_segment_gap`
  焊缝支撑点序列中允许的最大间隔。
- `seam_min_length`
  最短有效焊缝长度。
- `expected_height_range`
  预期焊缝高度范围。
- `preferred_approach_vector`
  近似焊枪偏好接近方向。
- `final_score_threshold`
  最终焊缝输出阈值。

## 当前验证情况

已实现并验证以下内容：

- 背景差分后能保留工件主体
- 在合成 L 型工件场景中能稳定检出主焊缝
- 纯环境场景下不输出误检焊缝

执行测试：

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
```

当前测试覆盖：

- `test_background_subtraction_keeps_workpiece`
- `test_pipeline_detects_primary_seam`
- `test_empty_scene_has_no_seam`

## 设计取舍

这个版本没有直接上深度学习或复杂点云库，原因是：

- 先把前端环境去除链路打稳，能最快降低误检
- 在无 CAD、无模板的条件下，几何方法更容易解释和调试
- 纯 Python 版本更适合作为算法逻辑参考和最小验证基线

相应地，这个版本也有明确边界：

- 不适合大规模高密度工业点云实时处理
- 真实工业精度会受限于纯 Python 数值效率和简化几何模型
- 可达性评估还是近似规则，不是完整机器人运动学验证
- 当前没有处理多视角融合、曲面焊缝和学习型分割

## 推荐的下一步升级方向

1. 接入真实现场点云数据，按工位重新标定 `roi` 和阈值
2. 切换到 `Open3D` 或 `PCL`，提高速度、鲁棒性和可维护性
3. 加入法向、曲率和局部截面验证，增强焊缝几何约束
4. 接入机器人可达性和姿态约束，减少“能看到但不能焊”的误检
5. 在工件点云上增加轻量学习分割，只做“母材/焊缝/背景残留”三分类

## 开发与提交建议

如果你要把这个原型接到真实项目，建议按下面顺序推进：

1. 先采集空工位背景点云
2. 采集 5 到 10 组代表性工件点云
3. 先把 `P_workpiece` 提取稳定，再调焊缝识别
4. 先优化低误检，再追求高召回

## 许可证

当前仓库未单独声明许可证。如需开源或对外协作，建议后续补充 `LICENSE` 文件。
