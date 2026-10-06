# BÁO CÁO HỆ THỐNG ĐIỀU KHIỂN ROBOT UR3 BẰNG LLM, CAMERA VÀ MOVEIT 2

## 1. Thông tin chung

- **Tên dự án:** Hệ thống điều khiển Robot UR3 bằng LLM, Camera và MoveIt 2.
- **Học phần:** Điều khiển Robot Thông minh / Robot Thao tác.
- **Học sinh:** Nguyen Minh Nghia.
- **MSSV:** 23020755.
- **Nền tảng:** ROS 2 Jazzy.
- **Mô phỏng:** Gazebo.
- **Điều khiển:** MoveIt 2 và gripper mô phỏng.

## 2. Mục tiêu của hệ thống

Xây dựng hệ thống điều khiển Robot UR3 bằng cách sử dụng Language Model để lập kế hoạch, Camera để nhận thức trạng thái môi trường và MoveIt 2 để thực thi các thao tác vật lý trong mô phỏng.

Hệ thống được thiết kế theo pipeline:

```text
Câu lệnh tự nhiên
        ↓
9Router / LLM Planner
        ↓
Plan Validator
        ↓
Robot Skills C++
        ↓
MoveIt 2 + Gazebo Gripper
```

LLM không trực tiếp sinh joint value hoặc trajectory. LLM chỉ chịu trách nhiệm lập kế hoạch cao cấp, còn mọi thao tác được kiểm tra và thực thi qua các module xác thực.

## 3. Kiến trúc hệ thống

### 3.1. Pipeline dữ liệu

```text
Camera Gazebo
      │
      ▼
/overhead_cam
      │
      ▼
camera_detector
      │
      ▼
/scene_state
      │
      ▼
ur3_llm_planner
      │
      ▼
9Router
      │
      ▼
/llm_task/validated_plan
      │
      ▼
ur3_skill_executor
      │
      ▼
MoveIt 2 + Gazebo UR3e
```

### 3.2. Các bước hoạt động

1. Camera nhận diện vật thể và các vùng thao tác.
2. `camera_detector` publish trạng thái vật thể và vùng vào topic `/scene_state`.
3. `ur3_llm_planner` đọc trạng thái môi trường và gửi nó cho 9Router.
4. 9Router trả về kế hoạch JSON từ câu lệnh tự nhiên.
5. `task_validator` kiểm tra cấu trúc, danh sách vật thể, vị trí và thứ tự thao tác.
6. `ur3_skill_executor` thực thi các skill `pick`, `place` và `home`.
7. MoveIt 2 tạo trajectory và Gazebo gripper thực thi thao tác.

## 4. Cấu trúc workspace

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
├── build/
├── install/
├── log/
└── README.md
```

## 5. Các package và chức năng

### 5.1. Package `ur3_llm_control`

Chứa các module chịu trách nhiệm:

- `camera_detector.py`: phát hiện vật thể và vùng bằng OpenCV HSV.
- `llm_planner.py`: giao tiếp với 9Router và publish kế hoạch.
- `task_validator.py`: kiểm tra kế hoạch trước khi thực thi.
- `test_task_validator.py`: bộ test xác thực cho validator.

### 5.2. Package `ur3_llm_robot_skills`

Chứa:

- `robot_skills.cpp`: thực thi `pick`, `place`, `home` và kết quả.
- `llm_robot.launch.py`: khởi động Gazebo, MoveIt, Camera, Planner và Executor.
- `scene.yaml`: cấu hình vật thể, vùng và vị trí tạm.
- `ur3_pick_place.sdf`: mô phỏng bàn, vật thể và camera.
- `controllers.yaml`: cấu hình controller của Robot UR3.

## 6. Vật thể, vùng và vị trí

### 6.1. Vật thể

- `red_cube`
- `yellow_cube`
- `blue_cube`
- `green_cube`
- `purple_cube`

### 6.2. Vùng thao tác

- `zone_a`
- `zone_b`
- `zone_c`

### 6.3. Vị trí tạm

- `temp_1`
- `temp_2`

Cấu hình sinh viên được lưu trong `student_config.yaml`. Hiện tại, hệ thống được đặt cho học sinh Nguyen Minh Nghia, MSSV 23020755.

## 7. Skill thực thi

### 7.1. Skill `pick`

- Mở gripper.
- Di chuyển robot lên trên vật thể.
- Hạ gripper để kẹp vật.
- Đổi vật thể vào Planning Scene qua `attachObject`.
- Nâng vật lên để tránh va chạm.

### 7.2. Skill `place`

- Di chuyển robot tới vị trí đích.
- Hạ vật xuống.
- Mở gripper để nhả vật.
- Tách vật khỏi Planning Scene qua `detachObject`.
- Cập nhật vị trí vật thể sau khi đặt thành công.

### 7.3. Skill `home`

- Xóa các target pose.
- Trả robot về vị trí khởi đầu an toàn.

## 8. Xử lý xung đột vùng

Khi zone đích đang có vật cản, hệ thống sẽ thực hiện các bước sau:

1. PICK vật cản.
2. PLACE vật cản vào vị trí tạm.
3. PICK vật đích.
4. PLACE vật đích vào zone đích.
5. HOME.

Ví dụ:

```text
Put the red cube in zone B
```

Nếu `blue_cube` đang chiếm `zone_b`, hệ thống sẽ di dời `blue_cube` sang `temp_1` trước khi đặt `red_cube` vào `zone_b`.

## 9. Cấu hình

File cấu hình 9Router:

```env
LLM_ENDPOINT=http://localhost:20128/v1/chat/completions
LLM_MODEL=gemini/gemini-3.5-flash-lite
LLM_API_KEY=YOUR_9ROUTER_API_KEY
```

File này nằm local và bị bỏ qua Git. Không được lưu API key trong README, notebook hoặc commit.

## 10. Cài đặt và build

### 10.1. Chạy build

```bash
cd ~/ur3_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to ur3_llm_control ur3_llm_robot_skills --symlink-install
source install/setup.bash
```

### 10.2. Khởi động hệ thống

```bash
cd ~/ur3_ws
src/ur3_llm_control/scripts/run_llm_robot.sh
```

Script này kiểm tra API key, khởi động 9Router nếu chưa chạy và khởi động Gazebo, MoveIt 2, Camera, Planner và Executor.

## 11. Demo sử dụng

### 11.1. Demo xung đột vùng

```bash
src/ur3_llm_control/scripts/send_demo_conflict.sh
```

Câu lệnh mẫu:

```text
Put the red cube in zone B
```

### 11.2. Demo sắp xếp theo MSSV

```bash
src/ur3_llm_control/scripts/send_demo_advanced.sh
```

Câu lệnh mẫu:

```text
Arrange all objects according to my student ID
```

### 11.3. Chạy kế hoạch cục bộ

```bash
src/ur3_llm_control/scripts/send_local_plan.sh
```

Kế hoạch này gửi trực tiếp lên topic `/llm_task/validated_plan` và không gọi 9Router.

## 12. ROS 2 topics

| Topic | Mục đích |
|---|---|
| `/overhead_cam` | Ảnh camera Gazebo |
| `/scene_state` | Trạng thái vật thể và vùng |
| `/llm_task/command` | Câu lệnh tự nhiên |
| `/llm_task/validated_plan` | Kế hoạch đã được validator kiểm tra |
| `/llm_task/execution_status` | Trạng thái cuối cùng của executor |

## 13. Kiểm tra

### 13.1. Unit test

```bash
python3 -m unittest src/ur3_llm_control/test/test_task_validator.py
```

### 13.2. Build

```bash
colcon build --base-paths src --packages-up-to ur3_llm_control ur3_llm_robot_skills --symlink-install
```

### 13.3. Kiểm tra runtime

```bash
ros2 node list
ros2 topic list
ros2 topic echo --no-arr /llm_task/execution_status
```

Kết quả thực thi được đánh giá theo trạng thái `TASK SUCCESS` hoặc `TASK FAILED`.

## 14. Kết quả kiểm tra

- Unit test: 9/9 thành công.
- Build: 6 package thành công.
- Cấu trúc package và launch đã được kiểm tra.
- Gazebo, MoveIt, Camera, Planner và Executor đã được khởi động trong môi trường hiện tại.
- API key và build artifacts không được lưu trong Git.
- Thực thi cuối cùng qua 9Router cần được xác nhận lại khi hệ thống đang chạy đầy đủ để thu được trạng thái `TASK SUCCESS`.

## 15. Lưu ý

- Không để LLM sinh joint value hoặc trajectory trực tiếp.
- Mọi kế hoạch phải được validator kiểm tra trước khi thực thi.
- Không lưu API key trong Git.
- Đổi hoặc thêm vật thể phải đồng bộ với `student_config.yaml`, `scene.yaml` và validator.
- Không dùng file build hoặc install như là nguồn mã chính khi phát triển.

## 16. Tài liệu đi kèm

- Notebook: `docs/ur3_llm_workspace.ipynb`.
- README: `README.md`.
- Cấu hình sinh viên: `src/ur3_llm_control/config/student_config.yaml`.
- Cấu hình 9Router: `src/ur3_llm_control/config/9router.env`.
- Launch: `src/ur3_llm_robot_skills/launch/llm_robot.launch.py`.
- Executor: `src/ur3_llm_robot_skills/src/robot_skills.cpp`.
