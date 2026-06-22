# IMBKA-Vortex: Hardware-Accelerated UAV Path Planner 🚁⚡

**A full-stack hardware/software co-design implementing the Black-Winged Kite Algorithm (BKA) for real-time, obstacle-aware UAV path planning on the Zynq-7000 SoC.**

## 📌 Overview
This repository contains the high-level synthesis (HLS) C++ hardware engine and the interactive Python (PYNQ) overlay for a dynamic path-planning agent. It utilizes an Improved Metaheuristic Black-Winged Kite Algorithm (IMBKA) coupled with a custom vortex evasion protocol to navigate dense, complex obstacle maps. 

The project demonstrates end-to-end System-on-Chip (SoC) integration, bridging complex algorithmic design with strict physical hardware constraints. Swarm intelligence algorithms are traditionally computationally heavy, making them difficult to run in real-time on embedded microcontrollers. By offloading the swarm fitness evaluations and heavy trigonometry to programmable logic, this custom architecture operates **over 5x faster than a Raspberry Pi** running the software-equivalent model, freeing up the ARM cortex for higher-level autonomous flight control.

## 🧠 Mathematical Foundation & Vortex Logic
Standard metaheuristic algorithms often fail in dense environments due to the "local minima" problem, where the UAV gets trapped in a concave obstacle or chatters endlessly between two paths. 

* **The IMBKA Core:** Handles global goal-seeking behavior using mathematically modeled migration and attack phases to find the most efficient vector toward the target.
* **Map-Aware Vortex Evasion:** Replaces the standard local left/right obstacle vote with a dynamic corridor cost logic. On vortex entry, the planner samples tangential corridors against all nearby obstacles, map boundaries, and the distance-to-goal. It dynamically calculates the lowest-cost route, resolving single  direction focus bias and preventing trajectory hysteresis.

## 🚀 True Hardware Architecture & Optimizations
The hardware accelerator was synthesized using Xilinx Vitis HLS with a strict focus on minimizing the silicon footprint on the xc7z020 fabric.
* **Sequential Execution & Area Reduction:** To comfortably fit within the strict area budget, loop unrolling and pipelining were explicitly disabled (`PIPELINE off`, `unroll factor=1`). By forcing sequential execution and heavily recycling mathematical blocks, the multiplexer footprint was drastically reduced, resulting in a hyper-lightweight core utilizing only <u>**~40% of available LUTs**</u>.
* **Native Fixed-Point Math:** Floating-point extensions (`fpext`) were entirely eradicated from the critical path. The design utilizes native `ap_ufixed` types and `hls::sqrt` routines, isolating trigonometric functions to 24-bit precision. This prevents data wrap-around overflows while conserving valuable DSP slices.

## 🔗 Hardware/Software Interface (PS-PL Boundary)
The system utilizes a high-speed memory-mapped interface between the Zynq Processing System (PS) and the Programmable Logic (PL).
* **AXI-Lite Control:** The Python overlay communicates with the FPGA fabric via AXI-Lite registers (e.g., `REG_START_X`, `REG_GOAL_Y`) to send instantaneous state parameters and read back the `GOAL_REACHED` flags.
* **Direct Physical Memory:** For the heavy lifting, the system uses `pynq.allocate` to reserve contiguous physical memory buffers, allowing the hardware engine to directly write the massive trajectory array (`REG_PATH_X`, `REG_PATH_Y`) without interrupting the ARM processor.

## 💻 Interactive PYNQ UI
Features a Jupyter Notebook interface running natively on the board. Interact with the matplotlib map to build your environment:
* **Click and drag** to add custom obstacles.
* **Press `+` or `-`** to adjust the radius of the next obstacle.
* **Press `u`** to undo/remove the last added obstacle.
* **Press `s` or `g`** to enter Start Point or Goal Point placement mode.
* **Press `c`** to clear the entire map.
* **Press `r`** to trigger the hardware execution via AXI-Lite and generate the trajectory.

<p align="center">
  <br><br>
  <img width="800" height="700" alt="image" src="https://github.com/user-attachments/assets/3244a55a-c18d-4abb-be77-89dbbfcf381b" />
</p>

### 📊 Data Extraction
The hardware engine saves the exact mathematical trajectory calculated during the run. To access the raw data array for external plotting or verification, create a new cell in Jupyter and run:
```python
print(ui.path)
