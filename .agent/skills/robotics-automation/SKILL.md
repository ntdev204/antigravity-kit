---
name: robotics-automation
description: Robotics systems and industrial automation design. Robot types, locomotion, sensor integration, PLC/SCADA systems, automation patterns, and system design methodology. Use when designing robotic systems, industrial automation, or automated manufacturing solutions.
---

# Robotics & Automation

> Principles for designing intelligent robotic and automated systems.

---

## 1. Robot Types & Applications

### Mobile Robots

| Type                | Characteristics                  | Applications                        |
| ------------------- | -------------------------------- | ----------------------------------- |
| **Wheeled**         | Fast, efficient on flat surfaces | Warehouses, delivery, inspection    |
| **Tracked**         | High traction, rough terrain     | Military, agriculture, construction |
| **Legged**          | Navigate obstacles, stairs       | Research, search and rescue         |
| **Aerial (Drones)** | 3D mobility, aerial view         | Surveillance, delivery, inspection  |
| **Underwater**      | Aquatic navigation               | Marine research, inspection         |

### Manipulators

| Type              | DOF | Workspace     | Applications                |
| ----------------- | --- | ------------- | --------------------------- |
| **Cartesian**     | 3   | Rectangular   | Pick-and-place, 3D printing |
| **SCARA**         | 4   | Cylindrical   | Assembly, packaging         |
| **Articulated**   | 6+  | Complex       | Welding, painting, assembly |
| **Delta**         | 3-4 | Inverted cone | High-speed picking          |
| **Collaborative** | 6-7 | Human-safe    | Assembly, inspection        |

---

## 2. Robot Components

### Actuation Systems

| Type                 | Characteristics             | Use Cases                 |
| -------------------- | --------------------------- | ------------------------- |
| **DC Motors**        | Simple, continuous rotation | Wheels, conveyors         |
| **Servo Motors**     | Precise position control    | Arms, grippers            |
| **Stepper Motors**   | Discrete steps, no feedback | 3D printers, CNC          |
| **Brushless Motors** | High efficiency, power      | Drones, high-performance  |
| **Linear Actuators** | Linear motion               | Lifts, sliding mechanisms |
| **Pneumatic**        | Fast, simple                | Grippers, pistons         |
| **Hydraulic**        | High force                  | Heavy machinery           |

### Sensors

| Category           | Sensors                     | Purpose              |
| ------------------ | --------------------------- | -------------------- |
| **Proprioception** | Encoders, IMU, force/torque | Internal state       |
| **Exteroception**  | Camera, LIDAR, ultrasonic   | External environment |
| **Localization**   | GPS, wheel odometry, SLAM   | Position tracking    |
| **Safety**         | Proximity, force sensors    | Collision avoidance  |

---

## 3. Locomotion Systems

### Wheeled Locomotion

| Configuration          | Characteristics    | Applications        |
| ---------------------- | ------------------ | ------------------- |
| **Differential Drive** | Two wheels, simple | Simple robots, toys |
| **Ackermann Steering** | Car-like steering  | Autonomous vehicles |
| **Mecanum Wheels**     | Omnidirectional    | Warehouses, AGVs    |
| **Skid Steering**      | Tank-like          | Rough terrain       |

### Legged Locomotion

| Type          | Stability      | Complexity | Use Cases             |
| ------------- | -------------- | ---------- | --------------------- |
| **Bipedal**   | Dynamic        | Very high  | Humanoids, research   |
| **Quadruped** | Static/dynamic | High       | Boston Dynamics Spot  |
| **Hexapod**   | Static         | Medium     | Stable walking robots |

---

## 4. Industrial Automation Architecture

### ISA-95 Automation Pyramid

| Level              | Function           | Systems            | Timeframe       |
| ------------------ | ------------------ | ------------------ | --------------- |
| **5: Enterprise**  | Business planning  | ERP (SAP, Oracle)  | Months-years    |
| **4: Operations**  | Site operations    | MES                | Days-weeks      |
| **3: Supervisory** | Production control | SCADA, HMI         | Hours-days      |
| **2: Control**     | Real-time control  | PLC, DCS           | Seconds-minutes |
| **1: Field**       | Sensing, actuation | Sensors, actuators | Milliseconds    |

---

## 5. PLC Systems

### Major PLC Brands

| Brand             | Series                     | Strengths                |
| ----------------- | -------------------------- | ------------------------ |
| **Siemens**       | S7-1200, S7-1500           | Performance, integration |
| **Allen-Bradley** | CompactLogix, ControlLogix | North America standard   |
| **Mitsubishi**    | FX, Q series               | Compact, reliable        |
| **Schneider**     | Modicon M580               | Energy management        |
| **Omron**         | CJ, NJ series              | Motion control           |

### PLC Programming Languages (IEC 61131-3)

| Language                            | Type       | Use Cases                       |
| ----------------------------------- | ---------- | ------------------------------- |
| **Ladder Logic (LD)**               | Graphical  | Relay replacement, simple logic |
| **Function Block (FBD)**            | Graphical  | Process control, data flow      |
| **Structured Text (ST)**            | Text-based | Complex algorithms, math        |
| **Sequential Function Chart (SFC)** | Graphical  | Sequential processes            |
| **Instruction List (IL)**           | Text-based | Low-level optimization          |

---

## 6. SCADA Systems

### SCADA Architecture

| Component         | Function                                 |
| ----------------- | ---------------------------------------- |
| **HMI**           | Human-machine interface, visualization   |
| **RTU/PLC**       | Remote terminal units, field controllers |
| **Communication** | Network protocols (Modbus, OPC)          |
| **Historian**     | Data logging, trending                   |
| **Alarm System**  | Event notification                       |

### Popular SCADA Platforms

| Platform        | Vendor               | Strengths                      |
| --------------- | -------------------- | ------------------------------ |
| **WinCC**       | Siemens              | Integration with Siemens PLCs  |
| **Ignition**    | Inductive Automation | Web-based, unlimited licensing |
| **Wonderware**  | AVEVA                | Mature, widely used            |
| **FactoryTalk** | Rockwell             | Integration with Allen-Bradley |

---

## 7. Industrial Communication Protocols

### Fieldbuses

| Protocol        | Speed    | Distance | Applications           |
| --------------- | -------- | -------- | ---------------------- |
| **Profibus**    | 12 Mbps  | 1200m    | Siemens ecosystem      |
| **Profinet**    | 100 Mbps | 100m     | Ethernet-based Siemens |
| **EtherNet/IP** | 100 Mbps | 100m     | Rockwell Automation    |
| **Modbus RTU**  | 115 kbps | 1200m    | Legacy systems         |
| **Modbus TCP**  | 100 Mbps | Ethernet | Modern systems         |
| **CANopen**     | 1 Mbps   | 40m      | Motion control         |

### Industrial Ethernet

| Protocol        | Real-time      | Vendor   |
| --------------- | -------------- | -------- |
| **Profinet**    | Yes            | Siemens  |
| **EtherNet/IP** | Yes            | Rockwell |
| **EtherCAT**    | Hard real-time | Beckhoff |
| **Powerlink**   | Hard real-time | B&R      |

---

## 8. Automation Design Patterns

### Sequential Control

```
State Machine Pattern:
- IDLE → START → RUNNING → STOPPING → IDLE
- Each state has entry/exit actions
- Transitions based on conditions
```

### Batch Processing

```
ISA-88 Batch Control:
- Recipe management
- Equipment phases
- Process stages
- Material tracking
```

### Continuous Control

```
Feedback Control Loops:
- PID controllers
- Cascade control
- Feedforward control
```

---

## 9. Robot Programming Approaches

### Teaching Methods

| Method                  | Characteristics       | Use Cases                   |
| ----------------------- | --------------------- | --------------------------- |
| **Manual Teaching**     | Move robot physically | Simple paths, prototyping   |
| **Teach Pendant**       | Joystick programming  | Traditional industrial      |
| **Lead-Through**        | Force feedback        | Collaborative robots        |
| **Offline Programming** | Software simulation   | Complex paths, optimization |

### Programming Paradigms

| Paradigm            | Description         | Examples                |
| ------------------- | ------------------- | ----------------------- |
| **Point-to-Point**  | Defined waypoints   | Pick-and-place          |
| **Continuous Path** | Smooth trajectories | Welding, painting       |
| **Force Control**   | Force feedback      | Assembly, polishing     |
| **Vision-Guided**   | Camera-based        | Bin picking, inspection |

---

## 10. Safety Systems

### Safety Standards

| Standard      | Scope                   |
| ------------- | ----------------------- |
| **ISO 10218** | Industrial robots       |
| **ISO 13849** | Safety-related controls |
| **IEC 62061** | Functional safety       |
| **ISO 15066** | Collaborative robots    |

### Safety Devices

| Device             | Function             |
| ------------------ | -------------------- |
| **E-Stop**         | Emergency shutdown   |
| **Light Curtains** | Perimeter protection |
| **Safety PLCs**    | Redundant control    |
| **Safety Relays**  | Fail-safe switching  |
| **Laser Scanners** | Area monitoring      |

### Safety Integrity Levels (SIL)

| Level     | Risk Reduction | Applications    |
| --------- | -------------- | --------------- |
| **SIL 1** | 10-100x        | Low risk        |
| **SIL 2** | 100-1000x      | Medium risk     |
| **SIL 3** | 1000-10000x    | High risk       |
| **SIL 4** | >10000x        | Critical safety |

---

## 11. Motion Control

### Motion Profile Types

| Type               | Characteristics     | Use Cases           |
| ------------------ | ------------------- | ------------------- |
| **Trapezoidal**    | Simple, predictable | Point-to-point      |
| **S-Curve**        | Smooth acceleration | High precision      |
| **Synchronized**   | Coordinated axes    | Multi-axis systems  |
| **Electronic Cam** | Following master    | Packaging, printing |

### Motion Controllers

| Type               | Characteristics              |
| ------------------ | ---------------------------- |
| **Standalone**     | Independent motion control   |
| **PLC-integrated** | Combined logic and motion    |
| **CNC**            | Complex machining operations |
| **Servo Drives**   | Closed-loop position control |

---

## 12. Gripper & End Effector Design

### Gripper Types

| Type             | Mechanism        | Applications               |
| ---------------- | ---------------- | -------------------------- |
| **Parallel Jaw** | Two-finger grasp | Standard parts             |
| **Three-Jaw**    | Centered grip    | Cylindrical parts          |
| **Vacuum**       | Suction          | Flat, smooth surfaces      |
| **Magnetic**     | Magnetic force   | Ferrous materials          |
| **Soft Gripper** | Compliant        | Fragile, irregular objects |

### Selection Criteria

- Object shape and size
- Weight capacity
- Surface material
- Precision requirements
- Speed requirements

---

## 13. System Integration

### Integration Layers

1. **Field Layer** - Sensors, actuators, drives
2. **Control Layer** - PLCs, motion controllers
3. **Supervisory Layer** - SCADA, HMI
4. **Information Layer** - MES, databases
5. **Enterprise Layer** - ERP, business systems

### Data Flow

```
Sensor → PLC → SCADA → MES → ERP
   ↕         ↕       ↕      ↕      ↕
Actuator ← PLC ← HMI ← DB ← Analytics
```

---

## 14. Commissioning & Testing

### Testing Phases

| Phase          | Focus              | Methods                |
| -------------- | ------------------ | ---------------------- |
| **FAT**        | Factory Acceptance | Functional testing     |
| **SAT**        | Site Acceptance    | Integration testing    |
| **Dry Run**    | Without material   | Logic verification     |
| **Wet Run**    | With material      | Performance validation |
| **Production** | Full operation     | Continuous monitoring  |

---

## 15. Maintenance Strategies

### Maintenance Types

| Type             | Trigger         | Efficiency |
| ---------------- | --------------- | ---------- |
| **Reactive**     | After failure   | Low        |
| **Preventive**   | Schedule-based  | Medium     |
| **Predictive**   | Condition-based | High       |
| **Prescriptive** | AI-driven       | Very high  |

### Monitoring Parameters

- Vibration analysis
- Temperature trends
- Current consumption
- Cycle times
- Error rates

---

## 16. Common Pitfalls

| Problem                   | Solution                                   |
| ------------------------- | ------------------------------------------ |
| **Insufficient safety**   | Design safety from start, follow standards |
| **Poor cable management** | Plan routing, use cable chains             |
| **Vibration issues**      | Proper mounting, dampening                 |
| **Communication errors**  | Proper shielding, termination              |
| **Inadequate testing**    | Comprehensive FAT/SAT procedures           |
| **No documentation**      | Maintain as-built drawings                 |
| **Poor HMI design**       | User-centered design, consistency          |

---

> **Remember:** Safety first, test thoroughly, document everything. Industrial systems require reliability and traceability above all else.
