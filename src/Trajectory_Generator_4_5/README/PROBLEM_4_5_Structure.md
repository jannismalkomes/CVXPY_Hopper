# 🚀 Parameterized Powered Descent Guidance Problem (DPP-Compliant)

This defines a **discretized convex optimization problem** for fuel-optimal powered descent, suitable for onboard use and C-code generation (DPP-compliant).  
It is based on the **G-FOLD algorithm (NASA/JPL)** formulation.

---

## 🧩 Overview

The problem computes an optimal **thrust and trajectory profile** for a vehicle to land from an initial state to a target state while respecting:
- dynamics,
- thrust and mass limits,
- velocity and acceleration constraints,
- thrust pointing cone limits, and
- surface avoidance.

It returns a **CVXPY parameterized problem** that can be solved repeatedly at runtime with different flight conditions.

---

## ⚙️ Parameters

| Name | Type | Description |
|------|------|--------------|
| `r_initial_param` | ℝ³ | Initial position (m) |
| `v_initial_param` | ℝ³ | Initial velocity (m/s) |
| `r_target_param` | ℝ³ | Target (landing) position (m) |
| `v_target_param` | ℝ³ | Target velocity (m/s) |
| `dt_param` | scalar | Time step between nodes (s); total time = `N * dt_param` |
| `v_max_param` | scalar | Max velocity magnitude (m/s) |
| `g_max_param` | scalar | Max total acceleration (in multiples of g₀) |
| `alpha_dt_param` | scalar | Fuel consumption rate scaled by `dt` (≈ α·Δt/2) |
| `cos_p_cs_param` | scalar | Cosine of max thrust cone angle (dimensionless) |
| `c1_param` | vector (N) | Precomputed constant: μ₁·(1 + z₁) (lower thrust linearization) |
| `c2_param` | vector (N) | Precomputed constant: μ₂·(1 + z₀) (upper thrust linearization) |

---

## 🧮 Constants

| Symbol | Description |
|---------|--------------|
| `g0 = 9.80665` | Standard gravity (m/s²) |
| `g = [0, 0, -g0]` | Gravity vector |
| `N` | Number of discretization points (fixed) |
| `m_dry` | Dry mass (kg) |
| `m_wet` | Wet (initial) mass (kg) |
| `thrust_min = r₁` | Minimum thrust (N) |
| `thrust_max = r₂` | Maximum thrust (N) |
| `α` | Mass depletion rate (kg/N·s) |

---

## 🧠 Optimization Variables

| Variable | Shape | Meaning |
|-----------|--------|----------|
| `x ∈ ℝ^(6×N)` | [position; velocity] over time |
| `u ∈ ℝ^(3×N)` | Specific thrust vector (thrust / mass) |
| `z ∈ ℝ^N` | Logarithm of vehicle mass (z = log m) |
| `s ∈ ℝ^N` | Thrust slack variable, upper bound on ‖u‖ |

---

## 🚀 Constraints and Dynamics

### 1️⃣ Initial and Final Conditions
- `x_pos(0) = r_initial`
- `x_vel(0) = v_initial`
- `x_pos(N−1) = r_target`
- `x_vel(N−1) = v_target`
- `z(0) = log(m_wet)`
- `u(:,0) = s₀ [0, 0, 1]`
- `u(:,N−1) = s_{N−1} [0, 0, 1]`

---

### 2️⃣ Dynamics (Leapfrog Integration)

For `n ∈ [0, N−2]`:
\[
\begin{aligned}
x_{vel}(n+1) &= x_{vel}(n) + \frac{Δt}{2}[(u(n)+g)+(u(n+1)+g)] \\
x_{pos}(n+1) &= x_{pos}(n) + \frac{Δt}{2}[x_{vel}(n+1)+x_{vel}(n)]
\end{aligned}
\]

This is a **time-centered integration** of  
\[
\dot{v} = u + g, \quad \dot{r} = v.
\]

---

### 3️⃣ Velocity Limit
\[
‖x_{vel}(n)‖ ≤ v_{max}
\]

---

### 4️⃣ Acceleration Limit
\[
‖u(n) + g‖ ≤ g_{max}·g₀
\]

Ensures the total acceleration never exceeds structural or control limits.

---

### 5️⃣ Mass Depletion
\[
z(n+1) = z(n) - α_{dt} · (s(n) + s(n+1))
\]

Models exponential mass loss due to fuel burn.

---

### 6️⃣ Thrust Magnitude and Pointing
\[
‖u(n)‖ ≤ s(n)
\]
\[
u_z(n) ≥ cos(p_{cs}) · s(n)
\]

Keeps thrust direction within an allowable cone about the +z axis.

---

### 7️⃣ Linearized Thrust–Mass Coupling (Convexified)
\[
\begin{aligned}
s(n) &≥ c1[n] - μ_1[n] · z(n) \\
s(n) &≤ c2[n] - μ_2[n] · z(n) \\
z_0[n] ≤ z(n) ≤ z_1[n]
\end{aligned}
\]

where

\[
\begin{aligned}
μ_1[n] &= \frac{T_{min}}{m_1[n]}, \quad μ_2[n] = \frac{T_{max}}{m_0[n]} \\
m_0[n] &= m_{wet} - α T_{max} n Δt, \quad m_1[n] = m_{wet} - α T_{min} n Δt \\
z_0[n] &= \log(m_0[n]), \quad z_1[n] = \log(m_1[n])
\end{aligned}
\]

---

### 8️⃣ Altitude Constraint
\[
x_z(n) ≥ 0 \quad ∀ n ∈ [0, N−2]
\]

Prevents subsurface flight before landing.

---

## 🎯 Objective Function

\[
\text{Maximize } z(N−1)
\]

Equivalent to **maximizing final mass** → **minimizing fuel usage**.

---

## 🧭 Summary

| Category | Element | Description |
|-----------|----------|-------------|
| **State** | `x`, `u`, `z`, `s` | Position, velocity, thrust, mass |
| **Dynamics** | Leapfrog integration | Accurate, time-centered discretization |
| **Constraints** | Position, thrust, velocity, mass, pointing, altitude | Physical feasibility |
| **Parameters** | `r_initial`, `r_target`, `v_initial`, `v_target`, `dt`, etc. | Adjustable at runtime |
| **Precomputed** | `c1_param`, `c2_param` | Linearized thrust–mass constants |
| **Objective** | Maximize final `z` | Minimize fuel consumption |
| **Convexity** | DCP-compliant | Ready for C code generation |

---

## 🧠 Physical Interpretation

This problem finds the **fuel-optimal thrust profile** for a lander to go from a given state to a target point while:
- Satisfying all physics and limits,
- Keeping thrust within a cone,
- Avoiding the ground until touchdown, and
- Using **as little fuel as possible**.

The **parameterization** allows you to adjust:
- Flight duration (`dt_param`),
- Start and end conditions,
- Velocity or acceleration limits,
without recompiling.

---

## 🧩 Notes

- The linearization terms `c1_param`, `c2_param` are computed **offline** from the nominal mass trajectory:
  \[
  c1 = μ_1(1 + z_1), \quad c2 = μ_2(1 + z_0)
  \]
- They can be updated at runtime if flight time (`dt_param`) or thrust limits change.

---

## ✅ Objective Summary

| Goal | Description |
|------|--------------|
| **Minimize fuel use** | Maximize final log-mass `z[N−1]` |
| **Convex problem** | All constraints and objective satisfy DCP rules |
| **Real-time capable** | Parameterized (DPP) for onboard computation |

---