# ros2_control_image

以 ROS 2 Jazzy 與 ros2_control 為基礎的機器人 image 中間層。提供 ros2_control 全套、開發板抽象層 `hw_transport`、MIT 模式控制器 `mit_controller` 與通用 bringup，各機器人的 image 建立於其上。

```
ros_jazzy_base_image → ros2_control_image → <機器人> image
```

## 架構

### 三層 image

| 層 | 內容 | 不包含 |
|---|---|---|
| `ros_jazzy_base_image` | 作業系統、ROS 2 Jazzy、開發工具 | ros2_control |
| `ros2_control_image`（本 repo） | ros2_control、標準 controller、與機器人和品牌無關的共用元件 | 任何品牌的驅動 |
| 機器人 image | 讀寫該機器人所有感測器與致動器所需的一切 | 功能與演算法 |

功能（行走、夾取、導航、控制策略等）不建立 image，執行時以 bind mount 掛入機器人 image 的 `/workspaces`。機器人 image 對外只提供 ROS topic、service 與 action，功能程式只透過這些介面存取硬體。

### 機器人 repo 結構

```
<機器人 repo>/
├── robot_image/                    機器人 image 的內容
│   ├── Dockerfile                  FROM ros2_control_image
│   ├── hardware/
│   │   ├── boards/<開發板>/        transport plugin（實作 hw_transport::CanBus）
│   │   ├── motors/<協定>/          ros2_control SystemInterface plugin
│   │   ├── sensors/<感測器>/       SensorInterface plugin，或 ROS driver node 的設定與 launch
│   │   └── sim/<模擬器>/           模擬後端
│   ├── <機器人>_description/       URDF/xacro 與 meshes，不含 <ros2_control>
│   └── <機器人>_bringup/           <ros2_control> 硬體對應、controllers.yaml、launch
└── ros2_ws/src/                    功能，執行時 bind mount 至 /workspaces/src
```

機器人 image 由機器人自身的 repo 建置，URDF 因此只有一份，模擬、訓練與實機共用。

### 設計規則

1. **機器人 image 只負責讀寫硬體。** 不包含任何功能或演算法。
2. **控制迴圈內的裝置使用 ros2_control plugin，其餘使用 ROS driver node。** 馬達、IMU、力感測器等需要與關節同步讀取的高頻裝置實作為 plugin，於同一個控制迴圈內讀取，時間戳一致。相機、LiDAR、GPS 等資料量大且已有原廠 ROS 2 driver 的裝置使用 driver node，由 bringup 啟動。兩者皆置於 `hardware/` 之下，可同樣抽換。
3. **一律使用關節端 SI 單位。** 減速比、方向與零點偏移寫在 URDF 的 `<transmission>`，由 plugin 透過 transmission_interface 換算，不寫死在程式中。
4. **interface 名稱遵循[命名表](#interface-命名)。**
5. **plugin 自行檢查硬體狀態。** 啟動時確認每個裝置皆有回應且 ID 正確。執行中偶發的漏幀記錄於 `error` 等 state interface，交由上層判斷；持續失聯時 `read()` 回傳 ERROR，controller_manager 隨即停用該 component 以及使用它的 controller。
6. **以 `hardware` 參數切換後端。** `hardware:=real | mock | <模擬器>` 只更換 `<plugin>`，controller、topic 與功能程式不受影響。
7. **安全分兩層。** 機器人層負責保護硬體：通訊逾時進入阻尼、過溫、限位、驅動器自身的逾時設定。功能層負責行為上的安全：傾角、障礙物、落差。
8. **`write()` 不等待回覆。** 每個裝置送出一幀即返回；回覆由 transport 的接收執行緒收取，`read()` 取用最新的一筆。控制頻率的上限由匯流排頻寬決定，而非程式。

### interface 命名

| 名稱 | 種類 | 意義與單位 |
|---|---|---|
| `position` `velocity` `effort` | command / state | 標準：rad（或 m）、rad/s、N·m |
| `kp` `kd` | command | MIT 增益：N·m/rad、N·m·s/rad |
| `temperature` | state | 馬達或線圈溫度，°C |
| `temperature_driver` | state | 驅動器或 MOS 溫度，°C |
| `voltage` `current` | state | V、A |
| `error` | state | 原廠錯誤碼，0 表示正常 |
| IMU | state | 依 `imu_sensor_broadcaster`：`orientation.x/y/z/w`、`angular_velocity.x/y/z`、`linear_acceleration.x/y/z` |
| 力感測器 | state | 依 `force_torque_sensor_broadcaster`：`force.x/y/z`、`torque.x/y/z` |

表中未列出的值可自行命名，但相同意義在不同品牌之間必須使用相同名稱，否則更換硬體後讀取該值的功能程式也須修改。所有 state interface 皆發布於 `/dynamic_joint_states`；`/joint_states` 僅包含 position、velocity、effort。

### 硬體資料夾

每個資料夾為一個獨立的 ROS 2 package：

| 內容 | 說明 |
|---|---|
| `package.xml` | 版本號，以及相依套件（建置時由 `rosdep` 自動安裝） |
| plugin（C++） | 或 driver node 的設定與 launch |
| `<名稱>.xacro` | 供 bringup 使用的 macro，例如每個關節一行 |
| `README.md` | 參數、提供的 interface、ID 規則、接線注意事項 |
| `tools/` | 除錯用 CLI，可使用 Python（不在控制迴圈內，不影響頻率） |

- 同一套通訊協定只建立一個資料夾，型號以參數選擇。
- 開發板與馬達分開。馬達 plugin 只呼叫 `hw_transport::CanBus`，所用的開發板由 URDF 的 `transport`、`port` 參數決定。同一條匯流排上不同品牌的裝置各為一個 hardware component，共用同一個 port。
- 序列裝置以 udev rule 依序號固定名稱（例如 `/dev/can_lf`），設定檔中不使用 `/dev/ttyACM0` 這類會變動的名稱。
- 硬體資料夾集中維護一份母版，機器人 repo 複製使用，並以 `package.xml` 的版本號標示所用版本。

### 抽換

修改後重新建置機器人 image。

| 更換對象 | 步驟 |
|---|---|
| 馬達品牌 | 更換 `hardware/motors/` 下的資料夾；修改 bringup 中對應的 `<ros2_control>`（plugin 名稱、ID） |
| 同一協定的不同型號 | 只修改 `model` 參數 |
| 感測器 | 更換 `hardware/sensors/` 下的資料夾；修改 bringup 中的 `<sensor>` 或 launch |
| 開發板 | 更換 `hardware/boards/` 下的資料夾；修改 `transport`、`port` 參數。SocketCAN 開發板已內建於本 image，不需資料夾 |
| URDF | 更換 `<機器人>_description`；關節名稱不變時，其餘皆不需修改 |
| 主機架構（x86 ↔ ARM） | 不需修改，各層 image 皆同時建置 amd64 與 arm64 |

只要新硬體提供相同的 interface，controller 與功能程式皆不需修改。

### 模擬

模擬後端為 `hardware/sim/` 下的資料夾，以 `hardware:=<模擬器>` 切換。模擬器本身（例如 Isaac Sim）以其官方 image 在獨立的 container 執行，模擬程式以 bind mount 掛入，與機器人 container 透過 ROS 2 溝通。

## 本 image 的內容

| | 內容 |
|---|---|
| apt 套件 | ros2_control、ros2_controllers（joint_state_broadcaster、forward、JTC、diff_drive、imu / force_torque / range broadcaster、gpio_controllers 等）、robot_state_publisher、xacro、can-utils |
| `hw_transport` | 開發板抽象層：`CanBus` 介面、讓多個硬體共用同一個 port 的 registry、內建 SocketCAN、`SerialPort`、`can_tool` |
| `mit_msgs` | `MitCommand`：q*、q̇*、kp、kd、τ_ff |
| `mit_controller` | 將 MitCommand 寫入 MIT 馬達的 5 個 command interface，指令中斷時自動進入阻尼 |
| `robot_bringup` | 通用 bringup launch，以及兩關節的 mock 範例 |

```
ros2_ws/src/
├── hw_transport/
├── mit_msgs/
├── mit_controller/
└── robot_bringup/
    ├── launch/bringup.launch.py    各機器人的 bringup 皆 include 此檔
    ├── launch/mit_demo.launch.py   範例，亦為機器人 bringup launch 的範本
    └── examples/                   mit_demo 的 xacro 與 controllers.yaml
```

## 使用

### 建置與試跑

```bash
bash scripts/local_build.sh    # 建置 ros2_control_image:local
bash scripts/run.sh            # 進入 container（已設定 host network、/dev、即時優先權）
```

以本機建置的 base image 建置：`bash scripts/local_build.sh -b ros_jazzy_base_image:local`。

於 container 中啟動 mock 硬體的範例，不需連接任何硬體：

```bash
ros2 launch robot_bringup mit_demo.launch.py
```

另開終端機以 `docker exec -it <container> bash` 進入：

```bash
ros2 topic pub -r 50 /mit_controller/commands mit_msgs/msg/MitCommand \
  "{position: [0.5, -0.5], velocity: [0, 0], kp: [20, 20], kd: [1, 1], effort: [0, 0]}"
ros2 topic echo /joint_states
```

停止 `ros2 topic pub` 0.5 秒後，launch 的 log 會出現 `switching to damping`。

### 工作區

| 路徑 | 內容 | 來源 |
|---|---|---|
| `/opt/ros2_control_ws/install` | 本 image 的四個 package | 本 image |
| `/opt/robot_ws/install` | 硬體資料夾、description、bringup | 機器人 image |
| `/workspaces`（`$ROS2_WS`） | 功能程式 | 執行時 bind mount，以 `r` 建置 |

`ROS_UNDERLAY` 指向最上層的 underlay。base image（1.1.0 起）的 `.bashrc` 開頭依序 source `/opt/ros/jazzy`、`$ROS_UNDERLAY`、`/workspaces/install`（存在時）；entrypoint 與 `docker exec -it <container> bash` 皆經過此段，因此本 image 與機器人 image 都不需要自己的 entrypoint。機器人 image 只需將 `ROS_UNDERLAY` 改為自己的路徑，其 `setup.bash` 會連帶 source 本層。

`r` 只建置 `/workspaces` 中的 package，underlay 中的 package 不會被重新建置。

### 建立機器人 image

```dockerfile
ARG BASE_IMAGE=ghcr.io/jcjcjc0705/ros2_control_image:latest
FROM ${BASE_IMAGE}

ENV ROBOT_WS=/opt/robot_ws
COPY robot_image/hardware ${ROBOT_WS}/src/hardware
COPY robot_image/my_robot_description ${ROBOT_WS}/src/my_robot_description
COPY robot_image/my_robot_bringup ${ROBOT_WS}/src/my_robot_bringup
RUN apt-get update && \
    rosdep install --from-paths ${ROBOT_WS}/src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/* && \
    source ${ROS_UNDERLAY}/setup.bash && \
    cd ${ROBOT_WS} && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf build log
ENV ROS_UNDERLAY=${ROBOT_WS}/install
```

`rosdep install` 依各資料夾 `package.xml` 中的相依套件自動安裝，更換資料夾不需修改 Dockerfile。

### 執行選項

| 選項 | 用途 |
|---|---|
| `--network host` | DDS 通訊，並使 container 可存取 host 的 SocketCAN 介面 |
| `--ipc host` | Fast DDS 的 shared memory |
| `-v /dev:/dev`、`--privileged` | USB 轉接器 |
| `--cap-add SYS_NICE --ulimit rtprio=99` | 供 controller_manager 使用即時優先權。缺少時 log 會出現 `Could not enable FIFO RT scheduling policy`，仍可執行，但時序較不穩定 |

`scripts/run.sh` 已包含上述選項。

## 套件說明

### robot_bringup

`bringup.launch.py` 啟動 robot_state_publisher 與 ros2_control_node，並 spawn 指定的 controller。

| 參數 | 預設值 | 說明 |
|---|---|---|
| `description_file` | 必填 | xacro 或 URDF 的絕對路徑 |
| `description_args` | `''` | 其他 xacro 參數，例如 `"prefix:=left_ port:=can1"` |
| `hardware` | `real` | 以 `hardware:=<值>` 傳入 xacro，慣例為 `real`、`mock`、`isaac` |
| `controllers_file` | 必填 | controller_manager 參數：`update_rate`、各 controller 的類型與參數 |
| `controllers` | `joint_state_broadcaster` | 載入並啟動的 controller，以空白分隔 |
| `inactive_controllers` | `''` | 載入但不啟動的 controller |
| `namespace` | `''` | 所有 node 的 namespace |
| `use_sim_time` | `false` | 連接模擬器時設為 `true` |

機器人的 bringup launch 複製 `mit_demo.launch.py`，修改兩個檔案路徑與 controller 清單即可。controllers.yaml 的 key 寫作 `/**/controller_manager:`、`/**/<controller 名稱>:`，無論是否加上 namespace 皆可對應。

### mit_controller

訂閱 `~/commands`（`mit_msgs/MitCommand`），每個週期將指令寫入每個關節的 `position`、`velocity`、`kp`、`kd`、`effort`。

| 參數 | 預設值 | 說明 |
|---|---|---|
| `joints` | 必填 | 控制的關節 |
| `command_timeout` | `0.5` | 超過此秒數未收到新指令即進入阻尼；≤ 0 表示停用 |
| `damping_kd` | `1.0` | 阻尼時使用的 kd，N·m·s/rad |

- **阻尼**：kp = 0、kd = `damping_kd`、q̇* = 0、τ_ff = 0、q* = 量測到的位置。啟動後到第一筆指令之前、指令逾時、controller 停用時皆會進入。
- **拒收**：陣列長度不符、含 NaN 或 inf、kp 或 kd 為負、`joint_names` 含未知或重複的關節時，整筆指令拒收，原因記錄於 log（每秒最多一次）。
- **逾時**以 controller_manager 的時間計算，不使用訊息的 `header.stamp`。
- **硬體端**須為每個關節宣告上述 5 個 command interface，以及 `position` state interface。

### hw_transport

馬達 plugin 取得匯流排的方式：

```cpp
#include "hw_transport/registry.hpp"

// on_configure()：開發板類型與 port 取自 URDF 的 <hardware> param
bus_ = hw_transport::acquire_can_bus(
  info_.hardware_parameters.at("transport"),   // 例如 "hw_transport/SocketCan"
  info_.hardware_parameters.at("port"));       // 例如 "can0"、"/dev/can_lf"
bus_->subscribe(0x11, false, [this](const hw_transport::CanFrame & frame) {
  // 於接收執行緒中執行：僅保存資料，由 read() 取用最新一筆
});

// write()：每個裝置送出一幀，不等待回覆
bus_->send(frames);
```

- 同一個 port 於第一次取得時開啟，同一 process 內的其他硬體取得同一個實例；因此同一條匯流排上的關節馬達與輪子驅動可各為一個 hardware component。最後一個持有者釋放時關閉。
- 以不同的 transport 類型開啟已開啟的 port 時，拋出例外。

新增開發板（以序列埠形式的 USB-CAN 為例）：

1. 繼承 `hw_transport::CanBus`，實作 `open`、`close`、`is_open`、`send`；於 `open` 中啟動接收執行緒，每收到一幀呼叫 `dispatch()`。序列埠部分可使用 `hw_transport::SerialPort`。
2. 以 `PLUGINLIB_EXPORT_CLASS(my_board::Usb2Can, hw_transport::CanBus)` 匯出，並於 CMake 中加入 `pluginlib_export_plugin_description_file(hw_transport plugins.xml)`。
3. 以 `can_tool` 驗證，不需任何馬達程式：

```bash
ros2 run hw_transport can_tool dump hw_transport/SocketCan can0
ros2 run hw_transport can_tool send my_board/Usb2Can /dev/can_lf 001#FFFFFFFFFFFFFFFC
ros2 run hw_transport can_tool send hw_transport/SocketCan can0 011#00 -n 6000 -r 6000   # 負載測試，輸出被拒絕的幀數
```

SocketCAN 介面須先於 host 上啟用：`sudo ip link set can0 up type can bitrate 1000000`。

## 發佈

`.github/workflows/docker-publish.yml` 建置 amd64 與 arm64 並發布至 `ghcr.io/jcjcjc0705/ros2_control_image`：

- 推送版本 tag（`1.0.0`、`v1.0.0`、`1.0.0-rc1`）：發布 `<版本>` 與 `latest`；帶後綴的版本視為 pre-release，不更新 `latest`。
- 於 Actions 頁面手動執行 *Build and publish Docker image*：輸入版本，並選擇是否同時標記 `latest`。

Dockerfile 的 `BASE_IMAGE` 預設為 base image 的 `latest`，CI 以 `--pull` 建置，因此每次皆使用最新的 base image；需要固定版本時，將其改為指定的版本號。
