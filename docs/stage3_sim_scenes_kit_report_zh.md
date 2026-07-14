# Stage3 Sim Scenes Kit 开发报告

## 摘要

Stage3 在 Stage2 双臂控制与录制闭环上增加配置驱动的任务场景能力：

```text
Stage3 scene YAML -> Isaac task objects/camera
  + Stage2 dual TargetFrame/RDK/Studio control
  -> gateway -> recorder -> Unitree JSON
  -> LeRobot-style dataset -> H264 MP4
```

原始 Stage2 平地双臂场景保持不变；墙挂双臂桌面任务通过新的 scene/pipeline config 显式启用。

## 主要成果

- 新增 `flexiv_sim_scenes/`：解析 Stage3 scene YAML，校验 Unitree/Isaac 资产路径，并在 Isaac runtime 中加载 USD、cuboid、cylinder 和 drawer articulation。
- 双臂 Isaac app 轻量接入 scene-object loader，并支持 scene config 覆盖每只 Rizon4 的 `initial_q`，用于墙挂、肘部外拐、末端下垂的桌面操作姿态。
- 新增四个任务场景：
  - `pick_place_redblock_flexiv_dual`
  - `pick_redblock_into_drawer_flexiv_dual`
  - `stack_rgyblock_flexiv_dual`
  - `move_cylinder_flexiv_dual`
- 扩展 fake sender：保留 legacy sine 模式，新增四个任务 waypoint profile，使用 identity/reference orientation 避免姿态突变。
- 新增 `scripts/run_stage3_sim_scene_validation.py`：复用 Stage2 双臂真实闭环编排，输出 Unitree JSON、LeRobot-style dataset、H264 MP4 和 Stage3 summary。

## 配置边界

- 原始平地双臂场景仍为 `configs/scenes/dual_rizon4_cam_front.yaml`。
- Stage3 墙挂桌面基准为 `configs/scenes/dual_rizon4_wall_table_base.yaml`，四个任务 scene 在此基础上复制并放置任务物体。
- Unitree 任务 Python 不作为 runtime 依赖；只通过 `${UNITREE_ASSET_ROOT}` 引用 USD 资产。资产根目录优先读取 `UNITREE_SIM_ISAACLAB_ASSETS`，否则自动发现相邻 Unitree workspace。
- 默认只验收 `cam_front/color_0` 一个主视角，30fps H264。
- 本阶段验收“任务场景 + fake 遥操闭环视频”，不要求真实物理抓取、堆叠或放入抽屉成功。

## 验收命令

```bash
python scripts/run_stage3_sim_scene_validation.py \
  --config configs/pipelines/stage3_pick_place_redblock_dual.yaml

python scripts/run_stage3_sim_scene_validation.py \
  --config configs/pipelines/stage3_pick_redblock_into_drawer_dual.yaml

python scripts/run_stage3_sim_scene_validation.py \
  --config configs/pipelines/stage3_stack_rgyblock_dual.yaml

python scripts/run_stage3_sim_scene_validation.py \
  --config configs/pipelines/stage3_move_cylinder_dual.yaml
```

每次成功后报告路径形如：

```text
datasets/stage1_records/quest_isaac_flexiv_stage3_<task>_real_<stamp>/
├── raw/episode_001/data.json
├── stage3_sim_scene_validation.json
└── stage3_sim_scene_summary.json

datasets/lerobot/qiming/quest_isaac_flexiv_stage3_<task>_<stamp>/
└── videos/observation.images.cam_front/chunk-000/file-000.mp4
```

## 回归要求

- `python -m unittest discover -s tests -p 'test_*.py'` 通过。
- Stage1 单臂严格右臂零占位不变。
- Stage2 平地双臂配置和真实验收脚本继续可用。
- 只有选用 Stage3 scene config 时才加载任务物体和墙挂桌面布局。
