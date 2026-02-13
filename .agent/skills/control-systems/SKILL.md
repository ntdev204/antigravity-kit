---
name: control-systems
description: Control theory and advanced control methods. PID control (traditional and adaptive), Fuzzy logic control (Type-1, Type-2, higher-order), modern control methods (LQR, MPC, sliding mode), control system design and tuning. Use when designing control systems, implementing controllers, or optimizing control performance.
---

# Control Systems

> Principles for designing feedback control systems from classical to advanced methods.

---

## 1. Control System Fundamentals

### Control System Types

| Type              | Characteristics    | Applications             |
| ----------------- | ------------------ | ------------------------ |
| **Open Loop**     | No feedback        | Simple, low-cost systems |
| **Closed Loop**   | Feedback control   | Precise, stable systems  |
| **Feedforward**   | Predictive control | Disturbance rejection    |
| **Cascade**       | Multiple loops     | Complex processes        |
| **Multivariable** | MIMO systems       | Coupled processes        |

### Transfer Function Basics

| Domain               | Representation          | Use Cases                |
| -------------------- | ----------------------- | ------------------------ |
| **Time Domain**      | Differential equations  | Simulation, analysis     |
| **Laplace Domain**   | Transfer functions G(s) | Classical control design |
| **Frequency Domain** | Bode plots, Nyquist     | Stability analysis       |
| **State Space**      | dx/dt = Ax + Bu         | Modern control, MIMO     |

---

## 2. PID Control

### PID Components

| Component            | Effect                        | Mathematical Form |
| -------------------- | ----------------------------- | ----------------- |
| **Proportional (P)** | Immediate response to error   | Kp × e(t)         |
| **Integral (I)**     | Eliminates steady-state error | Ki × ∫e(t)dt      |
| **Derivative (D)**   | Anticipates future error      | Kd × de(t)/dt     |

### PID Tuning Methods

| Method              | Characteristics         | When to Use         |
| ------------------- | ----------------------- | ------------------- |
| **Ziegler-Nichols** | Quick, aggressive       | First approximation |
| **Cohen-Coon**      | Process reaction curve  | Simple processes    |
| **Lambda Tuning**   | Specified settling time | Known time constant |
| **IMC**             | Internal model control  | Model-based         |
| **Auto-tuning**     | Automated               | Adaptive systems    |
| **Manual**          | Trial and error         | Simple systems      |

### Tuning Guidelines

| Parameter | Too Low            | Optimal            | Too High            |
| --------- | ------------------ | ------------------ | ------------------- |
| **Kp**    | Slow response      | Stable, responsive | Oscillation         |
| **Ki**    | Steady-state error | No offset          | Overshoot, slow     |
| **Kd**    | Overshoot          | Damped             | Noise amplification |

### Tuning Process

1. Start with P-only control (Ki=0, Kd=0)
2. Increase Kp until stable oscillation
3. Add Ki to eliminate steady-state error
4. Add Kd to reduce overshoot
5. Fine-tune for optimal response

---

## 3. Advanced PID Variants

### Adaptive PID

| Type                | Adaptation              | Use Cases                |
| ------------------- | ----------------------- | ------------------------ |
| **Gain Scheduling** | Parameter tables        | Varying operating points |
| **Self-Tuning**     | Online parameter update | Changing dynamics        |
| **Model Reference** | Track reference model   | Complex systems          |

### Anti-Windup Techniques

| Method                      | Description                     |
| --------------------------- | ------------------------------- |
| **Clamping**                | Limit integrator output         |
| **Back Calculation**        | Feedback from saturation        |
| **Conditional Integration** | Stop integration when saturated |

### Derivative Filtering

```
Pure D: Kd × de/dt
Filtered D: Kd × s/(τs + 1) × e(s)
Typical τ: 0.1 × Td
```

---

## 4. Fuzzy Logic Control

### Fuzzy System Architecture

```
Input → Fuzzification → Inference Engine → Defuzzification → Output
         (Membership)    (Rule Base)        (Crisp Value)
```

### Membership Functions

| Type            | Characteristics    | Use Cases            |
| --------------- | ------------------ | -------------------- |
| **Triangular**  | Simple, linear     | General purpose      |
| **Trapezoidal** | Flat top           | Plateaus             |
| **Gaussian**    | Smooth, bell curve | Continuous variables |
| **Sigmoid**     | S-shaped           | Non-linear           |
| **Singleton**   | Single point       | Outputs              |

### Type-1 Fuzzy Logic

| Feature        | Description                |
| -------------- | -------------------------- |
| **Inputs**     | Crisp values               |
| **Membership** | Fixed membership functions |
| **Rules**      | IF-THEN fuzzy rules        |
| **Output**     | Defuzzified crisp value    |

**Example Rule:**

```
IF error is Positive_Large AND change_error is Negative_Small
THEN output is Medium_Increase
```

### Type-2 Fuzzy Logic

| Feature         | Type-1       | Type-2              |
| --------------- | ------------ | ------------------- |
| **Membership**  | Crisp        | Fuzzy (with FOU)    |
| **Uncertainty** | None         | Handles uncertainty |
| **Computation** | Fast         | More complex        |
| **Use Cases**   | Well-defined | Uncertain systems   |

**Footprint of Uncertainty (FOU):**

- Upper membership function
- Lower membership function
- Captures uncertainty

### Higher-Order Fuzzy

| Type               | Characteristics      | Applications     |
| ------------------ | -------------------- | ---------------- |
| **Type-2**         | Secondary membership | Noisy systems    |
| **Type-3**         | Tertiary membership  | Highly uncertain |
| **General Type-N** | N-level membership   | Theoretical      |

### Defuzzification Methods

| Method       | Description          | Characteristics |
| ------------ | -------------------- | --------------- |
| **Centroid** | Center of area       | Most common     |
| **Bisector** | Divides area in half | Balanced        |
| **MOM**      | Mean of maximum      | Fast            |
| **SOM**      | Smallest of maximum  | Conservative    |
| **LOM**      | Largest of maximum   | Aggressive      |

---

## 5. Modern Control Methods

### LQR (Linear Quadratic Regulator)

| Feature           | Description              |
| ----------------- | ------------------------ |
| **Cost Function** | J = ∫(x'Qx + u'Ru)dt     |
| **Q Matrix**      | State weighting          |
| **R Matrix**      | Control effort weighting |
| **Output**        | Optimal feedback gains K |

**Use Cases:**

- State feedback control
- Optimal performance
- Multi-input systems

### MPC (Model Predictive Control)

| Feature                | Description         |
| ---------------------- | ------------------- |
| **Prediction Horizon** | Future time steps   |
| **Control Horizon**    | Control moves       |
| **Constraints**        | Input/output limits |
| **Optimization**       | Online at each step |

**Advantages:**

- Handles constraints naturally
- Multivariable systems
- Predictive behavior

**Disadvantages:**

- Computational cost
- Requires accurate model

### Sliding Mode Control

| Feature             | Description              |
| ------------------- | ------------------------ |
| **Sliding Surface** | Desired dynamics         |
| **Reaching Law**    | Drive to surface         |
| **Chattering**      | High-frequency switching |

**Advantages:**

- Robust to uncertainties
- Finite-time convergence

**Mitigation of Chattering:**

- Boundary layer
- Higher-order sliding modes

---

## 6. State-Space Control

### State-Space Representation

```
ẋ = Ax + Bu  (State equation)
y = Cx + Du  (Output equation)

x: State vector
u: Input vector
y: Output vector
A, B, C, D: System matrices
```

### Controllability & Observability

| Property         | Condition               | Meaning              |
| ---------------- | ----------------------- | -------------------- |
| **Controllable** | rank([B AB A²B...]) = n | Can reach any state  |
| **Observable**   | rank([C' A'C' ...]) = n | Can infer all states |

### State Feedback Design

```
u = -Kx (State feedback)
K = Feedback gain matrix

Design methods:
- Pole placement
- LQR
```

### Observer Design

| Type                    | Characteristics          |
| ----------------------- | ------------------------ |
| **Luenberger Observer** | Full-state estimation    |
| **Kalman Filter**       | Optimal for noise        |
| **Reduced-Order**       | Partial state estimation |

---

## 7. Nonlinear Control

### Linearization

| Method              | Description             |
| ------------------- | ----------------------- |
| **Taylor Series**   | Local approximation     |
| **Jacobian**        | First-order derivatives |
| **Operating Point** | Equilibrium point       |

### Feedback Linearization

```
Transform nonlinear system to linear
via state transformation and feedback
```

### Lyapunov Stability

| Condition          | Stability                       |
| ------------------ | ------------------------------- |
| V(x) > 0, V(0) = 0 | Positive definite               |
| V̇(x) < 0           | Negative definite → Stable      |
| V̇(x) ≤ 0           | Negative semi-definite → Stable |

---

## 8. Robust Control

### H∞ Control

| Feature        | Description                       |
| -------------- | --------------------------------- |
| **Objective**  | Minimize worst-case gain          |
| **Norm**       | H∞ norm (peak frequency response) |
| **Robustness** | Guaranteed performance            |

### μ-Synthesis

| Feature         | Description               |
| --------------- | ------------------------- |
| **Uncertainty** | Structured uncertainty    |
| **μ**           | Structured singular value |
| **Iterations**  | D-K iterations            |

---

## 9. Adaptive Control

### Model Reference Adaptive Control (MRAC)

```
Reference Model: Desired response
Adaptive Law: Adjust controller parameters
Goal: Plant follows reference model
```

### Self-Tuning Regulators

| Component      | Function                  |
| -------------- | ------------------------- |
| **Identifier** | Estimate plant parameters |
| **Controller** | Update control law        |
| **Recursive**  | Online adaptation         |

---

## 10. Discrete-Time Control

### Sampling Considerations

| Factor                   | Guideline                          |
| ------------------------ | ---------------------------------- |
| **Sampling Period (Ts)** | 10-20x faster than system dynamics |
| **Shannon Theorem**      | fs > 2 × highest frequency         |
| **Aliasing**             | Anti-aliasing filter               |

### Discretization Methods

| Method                | Characteristics        |
| --------------------- | ---------------------- |
| **Forward Euler**     | s = (z-1)/Ts           |
| **Backward Euler**    | s = (z-1)/(z×Ts)       |
| **Tustin (Bilinear)** | s = (2/Ts)×(z-1)/(z+1) |
| **Zero-Order Hold**   | Exact for step inputs  |

---

## 11. Practical Implementation

### Anti-Aliasing

- Low-pass filter before ADC
- Cutoff < fs/2
- Analog or digital filter

### Derivative Kick

**Problem:** Sudden change in setpoint causes derivative spike

**Solution:**

```
D on measurement only:
u = Kp×e + Ki×∫e - Kd×dy/dt
```

### Bumpless Transfer

- Smooth mode switching
- Initialize integrator
- Track current output

---

## 12. Control System Metrics

### Time-Domain Performance

| Metric                 | Description         | Typical Target   |
| ---------------------- | ------------------- | ---------------- |
| **Rise Time**          | 10% to 90%          | Fast as possible |
| **Settling Time**      | Within ±2% of final | Minimize         |
| **Overshoot**          | Peak above final    | <10%             |
| **Steady-State Error** | Final error         | 0 for type-1+    |

### Frequency-Domain Performance

| Metric            | Description                         |
| ----------------- | ----------------------------------- |
| **Gain Margin**   | Safety before instability (dB)      |
| **Phase Margin**  | Safety before instability (degrees) |
| **Bandwidth**     | Frequency response range            |
| **Resonant Peak** | Maximum magnitude                   |

---

## 13. Stability Analysis

### Routh-Hurwitz Criterion

| Condition            | Stability  |
| -------------------- | ---------- |
| All coefficients > 0 | Necessary  |
| First column > 0     | Sufficient |

### Nyquist Criterion

```
Encirclements of -1:
N = P - Z
N: encirclements
P: RHP poles (open-loop)
Z: RHP poles (closed-loop)

Stable if Z = 0
```

### Bode Stability

| Metric           | Safe Value |
| ---------------- | ---------- |
| **Gain Margin**  | > 6 dB     |
| **Phase Margin** | > 45°      |

---

## 14. Common Control Architectures

### Cascade Control

```
Outer Loop (slow) → Inner Loop (fast)
Example: Temperature (outer) + Flow (inner)
```

### Ratio Control

```
Maintain ratio between two flows
Applications: Blending, mixing
```

### Override Control

```
Multiple controllers, select min/max
Example: Safety limits
```

---

## 15. Implementation Checklist

- [ ] Model the system (transfer function or state-space)
- [ ] Analyze controllability and observability
- [ ] Select control method based on requirements
- [ ] Design controller (tune parameters)
- [ ] Verify stability (margins, Nyquist)
- [ ] Simulate before deployment
- [ ] Implement anti-windup and saturation handling
- [ ] Add safety limits and fault detection
- [ ] Test with actual hardware
- [ ] Monitor and tune in operation

---

## 16. Common Pitfalls

| Problem              | Solution                             |
| -------------------- | ------------------------------------ |
| **Integral windup**  | Anti-windup schemes                  |
| **Derivative noise** | Filtering, derivative on measurement |
| **Saturation**       | Conditional integration, clamping    |
| **Poor sampling**    | Follow Nyquist, anti-aliasing        |
| **Model mismatch**   | Adaptive control, robust design      |
| **Unstable tuning**  | Conservative initial gains           |
| **Setpoint changes** | Setpoint weighting, filters          |

---

> **Remember:** Start with simple controllers (P or PI), add complexity only when needed. Stability first, then performance. Test thoroughly before deployment.
