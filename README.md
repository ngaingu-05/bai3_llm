# UR3 LLM Control Workspace

Workspace ROS 2 để điều khiển mô hình UR3 bằng LLM, Camera, MoveIt 2 và Gazebo. Hệ thống gồm một pipeline kế hoạch được xác thực trước khi thực thi: câu lệnh tự nhiên → 9Router → `camera_detector`/`scene_state` → `task_validator` → `robot_skills` → MoveIt 2.

## Cấu trúc workspace

```text
ur3_ws/
├── src/
│   ├── ur3_llm_control/
│   │   ├── config/
│   │   │   ├── 9router.env
│   │   │   └── student_config.yaml
│   │   ├── scripts/
│   │   │   ├── run_llm_robot.sh
│   │   │   ├── send_demo.sh
│   │   │   ├── send_demo_advanced.sh
│   │   │   ├── send_demo_conflict.sh
│   │   │   ├── send_local_plan.sh
│   │   │   └── source_llm_env.sh
│   │   ├── test/
│   │   │   └── test_task_validator.py
│   │   └── ur3_llm_control/
│   │       ├── camera_detector.py
│   │       ├── llm_planner.py
│   │       └── task_validator.py
│   └── ur3_llm_robot_skills/
│       ├── config/
│       │   ├── controllers.yaml
│       │   └── scene.yaml
│       ├── launch/
│       │   └── llm_robot.launch.py
│       ├── src/
│       │   └── robot_skills.cpp
│       ├── urdf/
│       │   ├── ur3_parallel_gripper.urdf.xacro
│       │   └── ur3_parallel_gripper.srdf.xacro
│       └── worlds/
│           └── ur3_pick_place.sdf
```

## Các package

- `ur3_llm_control`: nhận diện camera, gọi 9Router, kiểm tra kế hoạch và publish các lệnh.
- `ur3_llm_robot_skills`: thực thi skill bằng MoveIt 2, Gazebo, gripper và Planning Scene.

## Clone workspace và cài dependencies

Repo chứa mã dự án; hai repository ROS của Universal Robots được khai báo trong `ur3.repos` và tải riêng khi clone. Cần Ubuntu 24.04, ROS 2 Jazzy, cùng môi trường desktop để chạy Gazebo/RViz.

```bash
sudo apt update
sudo apt install -y python3-vcstool python3-rosdep
sudo rosdep init 2>/dev/null || true
rosdep update

git clone https://github.com/ngaingu-05/bai3_llm.git ~/ur3_ws
cd ~/ur3_ws
vcs import src < ur3.repos
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --packages-up-to ur3_llm_control ur3_llm_robot_skills --symlink-install
source install/setup.bash
```

Nếu `rosdep init` báo đã được khởi tạo trước đó, có thể bỏ qua thông báo này. Các package ROS phụ thuộc sẽ được cài tự động bằng `rosdep`.

## Cấu hình trước khi chạy

Tạo file local `src/ur3_llm_control/config/9router.env`:

```env
LLM_ENDPOINT=http://localhost:20128/v1/chat/completions
LLM_MODEL=gemini/gemini-3.5-flash-lite
LLM_API_KEY=YOUR_9ROUTER_API_KEY
```

File này bị bỏ qua Git. Không đưa API key vào README hoặc commit.

Cấu hình sinh viên nằm tại `src/ur3_llm_control/config/student_config.yaml`. Hiện tại được đặt cho:

- Học sinh: Nguyen Minh Nghia
- MSSV: 23020755
- Các vật thể: `red_cube`, `yellow_cube`, `blue_cube`, `green_cube`, `purple_cube`
- Các vùng: `zone_a`, `zone_b`, `zone_c`
- Vị trí tạm: `temp_1`, `temp_2`

## Yêu cầu

- Ubuntu 24.04 + ROS 2 Jazzy.
- 9Router server và API key hợp lệ. Repo không bao gồm dịch vụ 9Router hoặc thông tin xác thực.
- Gazebo và RViz có thể chạy giao diện.

## Build

```bash
cd ~/ur3_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to ur3_llm_control ur3_llm_robot_skills --symlink-install
source install/setup.bash
```

## Chạy hệ thống

```bash
cd ~/ur3_ws
src/ur3_llm_control/scripts/run_llm_robot.sh
```

Script này kiểm tra file `9router.env`, khởi động 9Router trên cổng `20128` nếu chưa chạy, rồi khởi động Gazebo, MoveIt 2, Camera Bridge, `camera_detector`, `ur3_llm_planner` và `ur3_skill_executor`.

## Gửi lệnh demo

### Xung đột vùng

```bash
src/ur3_llm_control/scripts/send_demo_conflict.sh
```

Lệnh mẫu: `Put the red cube in zone B`.

### Sắp xếp theo MSSV

```bash
src/ur3_llm_control/scripts/send_demo_advanced.sh
```

Lệnh mẫu: `Arrange all objects according to my student ID`.

### Chạy kế hoạch cục bộ

```bash
src/ur3_llm_control/scripts/send_local_plan.sh
```

Script này gửi trực tiếp kế hoạch lên topic `llm_task/validated_plan`, không gọi 9Router.

## ROS topics

- `/llm_task/command`: câu lệnh tự nhiên.
- `/scene_state`: trạng thái vật thể và zone do camera detector publish.
- `/llm_task/validated_plan`: kế hoạch đã được kiểm tra.
- `/llm_task/execution_status`: trạng thái cuối cùng của executor.
- `/overhead_cam`: ảnh camera từ Gazebo.

## Kiểm tra

```bash
python3 -m unittest src/ur3_llm_control/test/test_task_validator.py
```

Kết quả được chờ: `TASK SUCCESS` hoặc `TASK FAILED` từ executor.

## Cách hoạt động

1. Camera nhận diện vật thể và zone.
2. Planner đọc `/scene_state`, gửi trạng thái vào 9Router và nhận kế hoạch JSON.
3. Validator kiểm tra cấu trúc, vật thể, vị trí và thứ tự thao tác.
4. Executor thực thi pick/place/home qua MoveIt 2 và Gazebo gripper.
5. Sau mỗi `place`, executor cập nhật vị trí vật thể trong Planning Scene.

## Lưu ý

- Không chỉnh sửa trực tiếp joint value hoặc trajectory trong LLM planner.
- Không publish plan trực tiếp khi cần bảo đảm an toàn; hãy dùng `task_validator`.
- Không lưu API key trong Git. File `9router.env` phải được tạo local sau khi clone.
- Build artifacts `build/`, `install/` và `log/` đã bị bỏ qua Git.
