# UR5 Pick-and-Place — Gazebo Simulation

簡化版 UR5 機械手臂 pick-and-place 系統，使用 Gazebo (gz-sim) + Python 狀態機，
作為後續**視覺導引抓取系統**的基礎架構。

---

## 📁 專案結構

```
ur5_pick_place/
├── worlds/
│   └── pick_place.sdf          # 主要 Gazebo 場景（UR5 + 桌子 + 物體）
├── models/
│   ├── ur5_robot/
│   │   ├── model.sdf           # UR5 機器人 SDF（6關節 + 夾爪）
│   │   └── model.config        # Gazebo 模型設定
│   └── objects/
│       └── pick_object.sdf     # 待抓取物體
├── scripts/
│   ├── pick_place_controller.py  # 主控制器（狀態機）
│   ├── joint_state_monitor.py    # 即時關節角度顯示
│   └── manual_joint_cmd.py       # 手動關節指令工具
├── config/
│   └── joint_configs.yaml      # 所有姿態定義
└── launch.sh                   # 一鍵啟動腳本
```

---

## 🚀 快速開始

### 1. 確認環境

```bash
conda activate gz_run
gz sim --version   # 應顯示 Gazebo 版本
```

### 2. 設定模型路徑

```bash
export GZ_SIM_RESOURCE_PATH=/path/to/ur5_pick_place/models
```

### 3. 啟動 Gazebo 模擬

```bash
cd ur5_pick_place
gz sim worlds/pick_place.sdf -r
```

`-r` 旗標讓模擬自動開始執行（不用手動按播放鍵）。

### 4. 啟動控制器（新的 terminal）

```bash
conda activate gz_run
cd ur5_pick_place/scripts
python pick_place_controller.py
```

---

## 🎮 執行模式

| 指令 | 說明 |
|------|------|
| `python pick_place_controller.py` | 執行一次完整流程 |
| `python pick_place_controller.py --cycle` | 持續重複執行 |
| `python pick_place_controller.py --interactive` | 逐步手動執行（每個狀態按 ENTER） |
| `python pick_place_controller.py --verbose` | 顯示每個關節角度值 |

### 一鍵啟動（含 Gazebo）

```bash
chmod +x launch.sh
./launch.sh                    # 自動執行
./launch.sh --interactive      # 逐步執行
./launch.sh --cycle            # 持續循環
./launch.sh --gz-only          # 只啟動 Gazebo
```

---

## 🤖 狀態機流程

```
IDLE
  │
  ▼
HOME ──────────────── 手臂直立、夾爪開啟
  │
  ▼
ABOVE_OBJECT ──────── 移動到物體正上方（hover ~15cm）
  │
  ▼
APPROACH ──────────── 向下接近物體（end-effector 降到物體高度）
  │
  ▼
GRASP ─────────────── 關閉夾爪（gripper 從 40mm → 5mm）
  │
  ▼
LIFT ──────────────── 抬起（回到 above_object 高度，夾著物體）
  │
  ▼
ABOVE_PLACE ───────── 旋轉 pan 90°，移動到放置區上方
  │
  ▼
LOWER_TO_PLACE ─────── 向下降至放置面
  │
  ▼
RELEASE ───────────── 打開夾爪（放下物體）
  │
  ▼
RETREAT ───────────── 縮回（保持 pan 方向，往上抬）
  │
  ▼
RETURN_HOME ───────── 回到 Home 姿態
  │
  ▼
DONE ✅
```

---

## 🔧 關節說明

| 關節名稱 | 類型 | 說明 |
|---------|------|------|
| `shoulder_pan_joint` | revolute | 底座旋轉（Z 軸）|
| `shoulder_lift_joint` | revolute | 肩部抬降（Y 軸）|
| `elbow_joint` | revolute | 肘部彎曲（Y 軸）|
| `wrist_1_joint` | revolute | 腕部 1（Y 軸）|
| `wrist_2_joint` | revolute | 腕部 2（Z 軸）|
| `wrist_3_joint` | revolute | 工具旋轉（Y 軸）|
| `left_finger_joint` | prismatic | 左夾爪（0=關閉, 0.04m=開啟）|
| `right_finger_joint` | prismatic | 右夾爪（0=關閉, 0.04m=開啟）|

### 控制 Topics

```
/model/ur5_robot/joint/shoulder_pan_joint/cmd_pos
/model/ur5_robot/joint/shoulder_lift_joint/cmd_pos
/model/ur5_robot/joint/elbow_joint/cmd_pos
/model/ur5_robot/joint/wrist_1_joint/cmd_pos
/model/ur5_robot/joint/wrist_2_joint/cmd_pos
/model/ur5_robot/joint/wrist_3_joint/cmd_pos
/model/ur5_robot/joint/left_finger_joint/cmd_pos
/model/ur5_robot/joint/right_finger_joint/cmd_pos
/model/ur5_robot/joint_states  ← 讀取所有關節狀態
```

---

## 🛠️ 偵錯工具

### 手動關節指令

```bash
cd scripts

# 傳送指定姿態
python manual_joint_cmd.py --pose home
python manual_joint_cmd.py --pose above_object

# 控制單一關節（弧度）
python manual_joint_cmd.py --joint shoulder_pan_joint --value 1.57

# 控制夾爪
python manual_joint_cmd.py --open-gripper
python manual_joint_cmd.py --close-gripper

# 列出所有可用姿態
python manual_joint_cmd.py --list-poses
```

### 即時監控關節狀態

```bash
cd scripts
python joint_state_monitor.py           # 持續監控
python joint_state_monitor.py --once    # 讀取一次
python joint_state_monitor.py --list-topics  # 列出所有 topics
```

### gz 命令列工具

```bash
# 列出所有 topics
gz topic -l

# 手動發布指令（測試用）
gz topic -t /model/ur5_robot/joint/shoulder_pan_joint/cmd_pos \
          -m gz.msgs.Double -p "data: 0.5" --once

# 監聽關節狀態
gz topic -e -t /model/ur5_robot/joint_states
```

---

## ✏️ 自訂姿態

編輯 `config/joint_configs.yaml` 或修改 `scripts/pick_place_controller.py`
中的 `JOINT_CONFIGS` 字典。

例如新增一個「掃描姿態」：

```python
JOINT_CONFIGS["scan_pose"] = JointConfig(
    name="scan_pose",
    description="Camera scan position",
    joints={
        "shoulder_pan_joint":   0.5,
        "shoulder_lift_joint": -1.2,
        "elbow_joint":          1.0,
        "wrist_1_joint":       -1.5,
        "wrist_2_joint":        0.0,
        "wrist_3_joint":        0.0,
        "left_finger_joint":    0.04,
        "right_finger_joint":   0.04,
    },
    hold_duration=3.0,
)
```

然後在 `STATE_SEQUENCE` 中插入：

```python
(State.SCAN, "scan_pose"),
```

---

## 🔭 後續擴充：視覺導引系統

此架構設計為視覺導引的**基礎層**，後續可以：

### 1. 加入相機

在 `model.sdf` 的 `ee_link` 加入相機 sensor：

```xml
<sensor name="wrist_camera" type="camera">
  <pose>0 0 -0.05 0 0 0</pose>
  <camera>
    <image><width>640</width><height>480</height></image>
  </camera>
  <topic>/wrist_camera/image</topic>
</sensor>
```

### 2. 視覺偵測替換固定姿態

```python
# 從相機影像推算物體位置
object_pos = vision_detect_object(camera_image)

# 使用 IK 求解關節角度（替換固定 config）
joint_angles = inverse_kinematics(object_pos)

# 發送給控制器
controller.go_to_joint_config(joint_angles)
```

### 3. 相關工具推薦

| 功能 | 工具 |
|------|------|
| 物體偵測 | OpenCV, YOLO, Detectron2 |
| 姿態估計 | FoundationPose, DexYCB |
| 逆運動學 | ikpy, roboticstoolbox-python |
| 深度感測 | OpenCV stereo, RealSense in Gazebo |

---

## ❓ 常見問題

**Q: Gazebo 啟動後機器人沒有移動？**
- 確認 Gazebo 模擬有在執行（按播放鍵，或使用 `-r` 旗標）
- 確認 controller 已啟動

**Q: `gz: command not found`？**
```bash
conda activate gz_run
which gz
```

**Q: 關節不動但沒有錯誤？**
- 用 `gz topic -l | grep ur5` 確認 topics 存在
- 用 `python manual_joint_cmd.py --pose home` 手動測試

**Q: 物體穿透桌子？**
- 調低 `max_step_size`（在 world SDF physics 區塊）
- 或增加 collision 的 `kp` 值

---

## 📋 系統需求

- Conda environment: `gz_run` with Python 3.10
- `gz-sim` installed via conda-forge
- No GPU, no sudo required
- Tested with: Gazebo Garden / Harmonic
