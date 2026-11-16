# 機器人軟體架構（通用）

最後更新：2026-11-16

實驗室每台機器人都照這份架構做。各機器人自己的細節（接線、ID、控制策略）寫在各自 repo 的文件裡。

## 1. 三層 image

```
ros_jazzy_base_image     OS、ROS 2 Jazzy、開發工具
└─ ros2_control_image    ros2_control 全套 + 共用元件（hw_transport、mit_controller、bringup）
   └─ <機器人> image      這台的硬體資料夾 + URDF + 硬體對應

功能（走路、夾取、導航、policy）：不建 image，執行時 bind mount 進機器人 image
```

| 層 | 放什麼 | 不放什麼 |
|---|---|---|
| base | 作業系統、ROS、常用工具 | ros2_control |
| ros2_control | 跟特定機器人、特定品牌無關的東西 | 任何品牌的驅動 |
| 機器人 | 讀寫這台所有感測器和馬達所需的一切 | 功能、演算法 |

機器人 image 對外只提供 ROS topic、action 和 service。功能程式只透過這些介面跟硬體溝通。

## 2. 機器人 repo 的結構

```
<機器人 repo>/
├── robot_image/                      ← 這個資料夾就是機器人 image
│   ├── Dockerfile                    FROM ros2_control_image
│   ├── hardware/                     從 hardware_library 複製來的資料夾
│   │   ├── boards/<開發板>/          transport plugin（實作 hw_transport::CanBus）
│   │   ├── motors/<協定>/            ros2_control SystemInterface plugin
│   │   ├── sensors/<感測器>/         SensorInterface plugin，或 ROS driver node 的設定和 launch
│   │   └── sim/<模擬器>/             模擬後端
│   ├── <機器人>_description/         URDF/xacro、meshes，不含 <ros2_control>
│   └── <機器人>_bringup/             <ros2_control> 硬體對應、controllers.yaml、launch
├── ros2_ws/src/                      功能，bind mount 到 /workspaces/src
└── …                                 模擬、訓練等不進 ROS image 的東西
```

URDF 只有一份，模擬、訓練、實機都讀同一份，所以機器人 image 從機器人自己的 repo build。

## 3. 規則

1. **機器人 image 只讀寫硬體。** 裡面沒有任何功能或演算法。
2. **控制迴圈內的用 ros2_control plugin，其他的用 ROS driver node。** 馬達、IMU、力感測器這類要跟關節同步讀取、頻率高的，做成 plugin，在同一個迴圈裡讀出來，時間戳一致。相機、LiDAR、GPS 這類資料量大、原廠已經有 ROS 2 driver 的，用 driver node，由 bringup 啟動。兩種都放在 `hardware/` 底下，一樣可以抽換。
3. **一律使用關節端的 SI 單位。** 減速比、方向（正負號）、零點偏移寫在 URDF 的 `<transmission>`，plugin 透過 transmission_interface 換算，不寫死在程式裡。
4. **interface 名稱照第 4 節的命名表。**
5. **plugin 要自己做健康檢查。** 啟動時確認每個裝置都有回應、ID 正確。執行時，偶爾漏一幀就把狀態記在 `error` 之類的 state interface，讓上層判斷；持續失聯才讓 `read()` 回傳 ERROR，controller_manager 會停掉這個 component 和用到它的 controller。
6. **後端用 `hardware` 參數切換。** `hardware:=real | mock | isaac` 只換 `<plugin>`，controller、topic 和功能程式都不用改。
7. **安全分兩層。** 保命的放在機器人層：通訊逾時就進阻尼、過溫、限位、驅動器本身的 timeout 設定。行為上的放在功能層：傾角、障礙物、落差。
8. **write() 不等回覆。** 每個裝置送一幀就返回；回覆由 transport 的 RX thread 收好，read() 只拿最新的一筆。控制頻率的上限應該是匯流排頻寬，不是程式。

## 4. interface 命名表

| 名稱 | 種類 | 意思和單位 |
|---|---|---|
| `position` `velocity` `effort` | command / state | 標準：rad（或 m）、rad/s、N·m |
| `kp` `kd` | command | MIT 增益：N·m/rad、N·m·s/rad |
| `temperature` | state | 馬達或線圈溫度，°C |
| `temperature_driver` | state | 驅動器或 MOS 溫度，°C |
| `voltage` `current` | state | V、A |
| `error` | state | 原廠錯誤碼，0 表示正常 |
| IMU | state | 照 `imu_sensor_broadcaster`：`orientation.x/y/z/w`、`angular_velocity.x/y/z`、`linear_acceleration.x/y/z` |
| 力感測器 | state | 照 `force_torque_sensor_broadcaster`：`force.x/y/z`、`torque.x/y/z` |

表裡沒有的值可以自己取名，但同一種意思不同品牌要用同一個名字，否則換馬達後讀它的功能程式就要改。所有 state interface 都會出現在 `/dynamic_joint_states`；`/joint_states` 只有 position、velocity、effort。

## 5. 硬體資料夾

每個資料夾是一個獨立的 ROS 2 package，母版放在 `hardware_library` repo，機器人 repo 複製過去使用。

| 內容 | 說明 |
|---|---|
| `package.xml` | 版本號（看得出這台用的是哪一版）、相依套件（`rosdep` 會自動安裝） |
| plugin（C++） | 或 driver node 的設定和 launch |
| `<名稱>.xacro` | 給 bringup 用的 macro，例如一個關節一行 |
| `README.md` | 參數、提供哪些 interface、ID 規則、接線注意事項 |
| `tools/` | debug 用的 CLI，可以是 Python（不在控制迴圈裡，不影響頻率） |

- 同一套協定只做一個資料夾，型號用參數選（例如 DM 的 J4310 和 J10010）。
- 開發板（USB-CAN、SocketCAN 介面…）跟馬達分開。馬達 plugin 只呼叫 `hw_transport::CanBus`，用哪塊開發板由 URDF 的 `transport`、`port` 參數決定。同一條匯流排上不同品牌的裝置，各自是一個 hardware component，共用同一個 port。
- 序列埠用 udev rule 依序號固定名稱（例如 `/dev/can_lf`），設定檔裡不寫 `/dev/ttyACM0`。

## 6. 換東西時要做什麼

改完都要重 build 機器人 image（push tag）。

| 要換 | 要做的事 |
|---|---|
| 馬達品牌 | `hardware/motors/` 換資料夾；改 bringup 裡那一段 `<ros2_control>`（plugin 名稱、ID） |
| 同一套協定的不同型號 | 只改 `model` 參數 |
| 感測器 | `hardware/sensors/` 換資料夾；改 bringup 裡的 `<sensor>` 或 launch |
| 開發板 | `hardware/boards/` 換資料夾；改 `transport`、`port` 參數。SocketCAN 開發板內建在第 2 層，不用放資料夾 |
| URDF | 換 `<機器人>_description`；關節名稱不變的話，其他都不用改 |
| 主機（x86 ↔ ARM） | 不用改，每一層都同時 build amd64 和 arm64 |

controller 和功能程式在上面這些情況都不用改，前提是新的硬體提供一樣的 interface（第 4 節）。

## 7. 功能層

- 功能程式放在機器人 repo 的 `ros2_ws/src/`，執行時 bind mount 到 container 的 `/workspaces/src`，用 `r` 編譯。機器人的 package 在 underlay，不會被重新編譯。
- 建議每個功能另開一個 container（用同一個機器人 image），掛掉時不會連帶停掉硬體。
- MIT 馬達的指令用 `mit_msgs/MitCommand` 送到 `mit_controller`；其他的用 ros2_controllers 的標準 controller。

## 8. 模擬

- 模擬後端也是 `hardware/sim/` 下的資料夾，用 `hardware:=<模擬器>` 切換。
- Isaac Sim 不放進機器人 image：用 NVIDIA 官方的 image 另開 container，模擬程式 bind mount 進去，跟機器人 container 透過 ROS 2 溝通。Isaac Lab 不經過 ROS，跟機器人 image 無關。

## 9. 尚未決定

- 功能層需要的函式庫（onnxruntime、PyTorch、nav2…）放在哪裡。目前的建議：機器人 repo 列一份清單，由機器人 image 安裝這些函式庫；功能的程式碼一律 bind mount。
- image 和 repo 放在 `ghcr.io/screamlab` 還是個人帳號。
