# MoveIt Task Constructor (MTC) & Warehouse_ROS_Mongo Installation Guide – ROS 2 Jazzy

This guide walks you through installing the MoveIt Task Constructor (MTC) and `warehouse_ros_mongo` packages and applying necessary fixes for a successful setup on ROS 2 Jazzy.

---

## 🔧 Prerequisites

Ensure you have:
- ROS 2 Jazzy installed and sourced
- A working colcon workspace, e.g. `~/ros2_ws`

---

## 🚀 Installation Steps

### 1. Install System Dependencies

```bash
sudo apt-get update
sudo apt-get install gnupg curl
```

---

### 2. Install MongoDB (v7.0)

```bash
curl -fsSL https://www.mongodb.org/static/pgp/server-7.0.asc | \
   sudo gpg -o /usr/share/keyrings/mongodb-server-7.0.gpg \
   --dearmor

echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-7.0.gpg ] https://repo.mongodb.org/apt/ubuntu jammy/mongodb-org/7.0 multiverse" | \
   sudo tee /etc/apt/sources.list.d/mongodb-org-7.0.list

sudo apt-get update
sudo apt-get install -y mongodb-org
```

Start and enable MongoDB:

```bash
sudo systemctl daemon-reload
# Kill any existing MongoDB process that might block startup
sudo pkill mongod
sudo systemctl start mongod
sudo systemctl enable mongod
sudo systemctl status mongod
```

---

## 🏛️ Install `warehouse_ros_mongo`

```bash
cd ~/ros2_ws/src
git clone https://github.com/moveit/warehouse_ros_mongo.git -b ros2
cd warehouse_ros_mongo/
git reset --hard 7f6a901eef21225141a2d68c82f3d2ec8373bcab

# Edit and remove unwanted dependency
sed -i '/<depend>mongodb<\/depend>/d' package.xml
```

Install dependencies:

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
colcon build
source ~/.bashrc
```

---

## 📚 Install MoveIt Task Constructor (MTC)

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/moveit/moveit_task_constructor.git -b jazzy
cd moveit_task_constructor
git reset --hard 9ced9fc10a15388224f0741e5a930a33f4ed6255
```

### 2. Install Dependencies

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace

```bash
colcon build
source ~/.bashrc
```

> ⚠️ **Note:** You can ignore warnings like:
>
> ```
> --- stderr: rviz_marker_tools
> rviz_marker_tools: You did not request a specific build type: Choosing 'Release' for maximum performance
> ```

---

## 🔧 Fix Known Issues

### ♻️ Fix 1: Planning Scene Diff

**File:** `core/src/storage.cpp`

Replace:
```cpp
if (this->end()->scene()->getParent() == this->start()->scene())
    this->end()->scene()->getPlanningSceneDiffMsg(t.scene_diff);
else
    this->end()->scene()->getPlanningSceneMsg(t.scene_diff);
```

With:
```cpp
this->end()->scene()->getPlanningSceneDiffMsg(t.scene_diff);
```

---

### 📏 Fix 2: Cartesian Path Jump Threshold

**File:** `core/src/solvers/cartesian_path.cpp`

Replace:
```cpp
moveit::core::JumpThreshold(props.get<double>("jump_threshold")), is_valid,
```

With:
```cpp
moveit::core::JumpThreshold::relative(props.get<double>("jump_threshold")), is_valid,
```

---

### 📆 Rebuild the Workspace

```bash
cd ~/ros2_ws
colcon build
source ~/.bashrc
# OR
source install/setup.bash
```

---

## 🎉 Success!

You have now successfully installed and patched:
- MongoDB and `warehouse_ros_mongo`
- MoveIt Task Constructor (MTC)

You're ready to use MTC for pick-and-place and complex motion planning tasks in ROS 2 Jazzy!

