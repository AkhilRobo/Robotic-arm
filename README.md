# Panda Arm

A modular and extensible project for controlling and simulating a Panda robotic arm with perception-driven pick-and-place automation.

---

## 🚀 Features

- ROS2-based mission controller  
- Perception-driven pick & place workflow  
- PCL-based 3D clustering and object classification  
- Multi-object mission execution (Laptop, Beer, etc.)  
- Robust filtering, retry loops, and queue-based execution  
- Fully decoupled logic between Perception ↔ Control  

---

## 🎯 How the Mission System Works

This project follows a **professional robotics pipeline** using three modules:

1. **Mission Node (Brain)** – Decides what to pick next  
2. **Perception Node (PCL Node)** – Scans and reports all objects  
3. **Execution Node (MoveIt)** – Picks and places the chosen object  

Each mission target (e.g., *Laptop*, *Beer*) is processed in phases.

---

# 🟦 **Phase 1: The "Laptop" Mission**

### 1. Mission Control  
Mission list:  
Current target → **Laptop**

### 2. Perception Scan  
Mission node sends a request:

> “Scan scene for 5 seconds.”

PCL node returns:

| Object | Label   | Position (x,y,z) |
|--------|----------|------------------|
| A      | Laptop  | (0.4, 0.1, 0.2)  |
| B      | Beer    | (0.3, -0.2, 0.2) |
| C      | Wall    | (1.5, 0.0, 0.0)  |

### 3. Filtering (Brain Logic)

- **Laptop** → matches target, reachable → **added to queue**
- **Beer** → wrong target → ignored
- **Wall** → wrong target + unreachable → ignored

### 4. Execution

- Robot **picks** the Laptop  
- Robot **places** the Laptop  
- Queue empties  
- Robot returns to **Home Pose**  

---

# 🟨 **Phase 2: The "Beer" Mission**

Current target → **Beer**

### 1. Perception Scan (Scene Changed)

Laptop is gone. PCL node returns:

| Object | Label | Position |
|--------|--------|----------|
| B      | Beer  | (0.3, -0.2, 0.2) |

### 2. Filtering

- Beer → matches target + reachable → **queued**

### 3. Execution

- Robot **picks** the Beer  
- Robot **places** the Beer  
- Robot returns to **Home Pose**

---

# 🟩 **Phase 3: Mission Complete**

Mission list is empty.  
System prints:

> **“All tasks completed.”**

---

# 🧠 Why This Architecture Is Professional

### ✔ Decoupled Logic  
You can change the mission list from `"Laptop"` to `"Coke"` without changing perception code.

### ✔ Robust Filtering  
Even if camera misdetects objects, the condition  
`if (label != current_target)`  
prevents wrong picks.

### ✔ Stable Motion  
Perception scans for 5 seconds → stops → sends a stable object list.  
Robot does **not** chase jittery real-time detections.

---

## 🛠️ Getting Started

Clone the repository:

```bash
git clone https://github.com/yourusername/panda_arm.git
cd panda_arm

ros2 launch panda_description panda_arm_and_sensor.launch.py 

ros2 run my_pcl_processor cloud_tf

ros2 run my_pcl_processor clasification_service_node.py

ros2 run my_pcl_processor feature_extractor_server

ros2 run my_pcl_processor pcl_node
