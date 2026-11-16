# ros2_control_image (Jazzy)

所有機器人 image 共用的第二層：ros2_control 全套，加上幾個跟特定機器人無關的 C++ 元件。

```
ros_jazzy_base_image → ros2_control_image（這裡）→ <機器人> image
```

整體架構，以及每台機器人都要遵守的規則，見 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

## 這裡面有什麼

| | 內容 |
|---|---|
| apt 套件 | ros2_control、ros2_controllers（joint_state_broadcaster、forward、JTC、diff_drive、imu / force_torque / range broadcaster、gpio_controllers…）、robot_state_publisher、xacro、can-utils |
| `hw_transport` | 開發板的抽象層：`CanBus` 介面、讓多個硬體共用同一個 port 的 registry、內建 SocketCAN、`SerialPort`、`can_tool` |
| `mit_msgs` | `MitCommand`：q*、q̇*、kp、kd、τ_ff |
| `mit_controller` | 把 MitCommand 寫進 MIT 馬達的 5 個 command interface，指令中斷時自動切到阻尼 |
| `robot_bringup` | 通用的 bringup launch，加上一個兩關節的 mock 範例 |

```
ros2_ws/src/
├── hw_transport/
├── mit_msgs/
├── mit_controller/
└── robot_bringup/
    ├── launch/bringup.launch.py    # 每台機器人都 include 這支
    ├── launch/mit_demo.launch.py   # 範例，也是機器人 bringup launch 的範本
    └── examples/                   # mit_demo 的 xacro 和 controllers.yaml
```

## Build 和試跑

```bash
bash scripts/local_build.sh    # 產生 ros2_control_image:local
bash scripts/run.sh            # 進 container（已帶 host network、/dev、即時優先權）
```

要用本地 build 的 base image（例如還沒發佈的版本）：`bash scripts/local_build.sh -b ros_jazzy_base_image:local`。

在 container 裡：

```bash
ros2 launch robot_bringup mit_demo.launch.py    # mock 硬體，不用接任何東西
```

另開一個終端機，用 `docker exec -it <container> bash` 進去：

```bash
ros2 topic pub -r 50 /mit_controller/commands mit_msgs/msg/MitCommand \
  "{position: [0.5, -0.5], velocity: [0, 0], kp: [20, 20], kd: [1, 1], effort: [0, 0]}"
ros2 topic echo /joint_states
```

停掉 `ros2 topic pub` 0.5 秒後，launch 的 log 會出現 `switching to damping`。

## 工作區：underlay 和 overlay

| 路徑 | 內容 | 由誰放進來 |
|---|---|---|
| `/opt/ros2_control_ws/install` | 這一層的四個 package | 這個 image |
| `/opt/robot_ws/install` | 硬體資料夾、description、bringup | 機器人 image |
| `/workspaces`（`$ROS2_WS`） | 功能程式（走路、夾取…） | 執行時 bind mount 進來，用 `r` 編譯 |

`ROS_UNDERLAY` 指向最上面那一層 underlay。source 的動作在 base image（1.1.0 起）的 `.bashrc` 開頭：依序 source `/opt/ros/jazzy`、`$ROS_UNDERLAY`、`/workspaces/install`（存在的話）。entrypoint 和 `docker exec -it ... bash` 都走這一段，所以這一層和機器人 image 都不需要自己的 entrypoint。機器人 image 只要把 `ROS_UNDERLAY` 改成自己的路徑，它的 `setup.bash` 會自動連帶 source 這一層。

## 在這一層上建機器人 image

```dockerfile
ARG BASE_IMAGE=ghcr.io/<owner>/ros2_control_image:latest
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

`rosdep install` 會照每個資料夾 `package.xml` 裡寫的相依套件自動安裝，所以換資料夾不用改 Dockerfile。

## robot_bringup：`bringup.launch.py`

| 參數 | 預設值 | 說明 |
|---|---|---|
| `description_file` | 必填 | xacro 或 URDF 的絕對路徑 |
| `description_args` | `''` | 其他 xacro 參數，例如 `"prefix:=left_ port:=can1"` |
| `hardware` | `real` | 以 `hardware:=<值>` 傳給 xacro。慣例：`real`、`mock`、`isaac` |
| `controllers_file` | 必填 | controller_manager 的參數：`update_rate`、各 controller 的類型和參數 |
| `controllers` | `joint_state_broadcaster` | 要載入並啟動的 controller，以空白分隔 |
| `inactive_controllers` | `''` | 載入但不啟動的 controller |
| `namespace` | `''` | 所有 node 的 namespace |
| `use_sim_time` | `false` | 接模擬器時設成 `true` |

機器人的 bringup launch 複製 `mit_demo.launch.py`，改兩個檔案路徑和 controller 清單即可。controllers.yaml 的 key 寫成 `/**/controller_manager:`、`/**/<controller 名稱>:`，這樣有沒有加 namespace 都對得到。

## mit_controller

訂閱 `~/commands`（`mit_msgs/MitCommand`），每個週期把指令寫進每個關節的 `position`、`velocity`、`kp`、`kd`、`effort`。

| 參數 | 預設值 | 說明 |
|---|---|---|
| `joints` | 必填 | 要控制的關節 |
| `command_timeout` | `0.5` | 超過幾秒沒有新指令就切到阻尼，≤ 0 表示關閉這個功能 |
| `damping_kd` | `1.0` | 阻尼時使用的 kd，單位 N·m·s/rad |

- **阻尼**：kp = 0、kd = `damping_kd`、q̇* = 0、τ_ff = 0、q* = 目前位置。剛啟動還沒收到指令、指令逾時、controller 被停用時都會進入阻尼。
- **整筆拒收**：陣列長度不對、有 NaN 或 inf、kp 或 kd 是負的、`joint_names` 裡有不認得或重複的關節。拒收原因會印在 log，每秒最多一次。
- **逾時**用 controller_manager 的時間計算，不看訊息的 `header.stamp`。
- **硬體端**要為每個關節宣告這 5 個 command interface，以及 `position` state interface。

## hw_transport：寫硬體資料夾時用

馬達 plugin 這樣取得匯流排：

```cpp
#include "hw_transport/registry.hpp"

// on_configure()：開發板類型和 port 從 URDF 的 <hardware> param 讀
bus_ = hw_transport::acquire_can_bus(
  info_.hardware_parameters.at("transport"),   // 例如 "hw_transport/SocketCan"
  info_.hardware_parameters.at("port"));       // 例如 "can0"、"/dev/can_lf"
bus_->subscribe(0x11, false, [this](const hw_transport::CanFrame & frame) {
  // 在 RX thread 裡執行：只把資料存起來，read() 再拿最新的一筆
});

// write()：每個裝置送一幀，不等回覆
bus_->send(frames);
```

- **共用同一個 port：** 某個 port 第一次被要求時才開啟，之後同一個 process 裡的其他硬體拿到的是同一個實例。所以同一條線上的 DM 馬達和輪子驅動可以各自是一個 hardware component。最後一個持有者釋放時，port 才會關閉。
- **類型要一致：** 同一個 port 被用不同的 transport 類型開啟時，會丟出例外。

### 加一種新的開發板

以序列埠形式的 USB-CAN 為例：

1. 繼承 `hw_transport::CanBus`，實作 `open`、`close`、`is_open`、`send`。在 `open` 裡開一條 RX thread，每收到一幀就呼叫 `dispatch()`。序列埠的部分可以直接用 `hw_transport::SerialPort`。
2. 加上 `PLUGINLIB_EXPORT_CLASS(my_board::Usb2Can, hw_transport::CanBus)`，CMake 裡寫 `pluginlib_export_plugin_description_file(hw_transport plugins.xml)`。
3. 先用 `can_tool` 驗證，還不用寫任何馬達程式：

```bash
ros2 run hw_transport can_tool dump hw_transport/SocketCan can0
ros2 run hw_transport can_tool send my_board/Usb2Can /dev/can_lf 001#FFFFFFFFFFFFFFFC
ros2 run hw_transport can_tool send hw_transport/SocketCan can0 011#00 -n 6000 -r 6000   # 負載測試，印出被拒絕的幀數
```

SocketCAN 介面要先在 host 上啟用：`sudo ip link set can0 up type can bitrate 1000000`。

## 執行時需要的 docker 選項

| 選項 | 用途 |
|---|---|
| `--network host` | DDS 通訊，也讓 container 看得到 host 的 SocketCAN 介面 |
| `--ipc host` | Fast DDS 的 shared memory |
| `-v /dev:/dev`、`--privileged` | USB 轉接器 |
| `--cap-add SYS_NICE --ulimit rtprio=99` | 讓 controller_manager 使用即時優先權。少了這兩個，log 會出現 `Could not enable FIFO RT scheduling policy`，仍然可以執行，但時序比較不穩 |

`scripts/run.sh` 已經帶好這些選項。

## CI

`.github/workflows/docker-publish.yml` 跟 base image 用同一份：push 版本 tag（例如 `0.1.0`），或在 Actions 頁手動執行，就會 build amd64 + arm64，推到 `ghcr.io/<owner>/ros2_control_image`。Dockerfile 的 `BASE_IMAGE` 預設是 base image 的 `latest`，CI 用 `--pull` build，所以每次都會用最新的 base；需要固定版本時，把這一行改成指定的版本號。
