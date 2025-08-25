Nice move 🚀 — having a **README.md** makes your repo professional and self-explanatory.
Here’s a polished version tailored to your project:

---

# **Radiation-Safe UR5e Robotic Arm Simulation**

This repository contains the MATLAB-based simulation framework developed for the **MSc dissertation project**:
*“MATLAB-Based Simulation of a UR5e Robotic Arm for Radiation-Safe Adhesive Application in Radiopharmaceutical Packaging.”*

The project focuses on **radiation-aware path planning** for the UR5e collaborative manipulator, with the goal of automating repetitive packaging tasks while ensuring safety in environments where ionising radiation is present.

---

## **Features**

* UR5e robot modelled in MATLAB using Robotics System Toolbox.
* Inverse Kinematics (IK) solver with multiple seed configurations.
* Joint-space interpolation for smooth motion.
* Obstacle avoidance using axis-aligned bounding boxes (AABB).
* Radiation zone detection using spherical exclusion regions.
* Multi-view (2×2 subplot) visualisation of robot, obstacles, and radiation zones.
* Performance evaluation across multiple simulation trials.
 add the `code/` folder to your path:

 

## **Results Summary**

* Average placement accuracy: **≤ 5 cm**
* Task success rate: **100%** across 15 trials
* Radiation zone violations: **0**
* Multi-view visualisation confirmed safe, collision-free trajectories

---


## **Acknowledgement**

This work was carried out as part of the **MSc dissertation project** at \[Your University Name].
The author also acknowledges supportive use of **AI tools (OpenAI’s ChatGPT)** for formatting, clarity, and documentation purposes.

