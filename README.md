# Adaptive-LQR-Control-for-Lane-Keeping-Assist-(LKA)-Systems

![MATLAB](https://img.shields.io/badge/MATLAB-R202x-orange.svg)
![Simulink](https://img.shields.io/badge/Simulink-Control_System-blue.svg)
![Status](https://img.shields.io/badge/Status-Educational_Project-green.svg)

> **Development of a robust lateral control system (Lane Keeping Assist) for autonomous vehicles using Adaptative Linear Quadratic Regulator (LQR). Implemented on SCANeR Studio and MATLAB/Simulink.**



### Visual Demo of the LKS concept

| | |
| :---: | :---: |
| <img src="https://github.com/user-attachments/assets/908fb66c-4af1-4a84-b836-be040974ad3c" width="400" /><br><sub>*Figure 1: Autonomous Road Trajectory Tracking.*</sub> | <img src="https://github.com/user-attachments/assets/198d49ea-95ef-4caf-93b5-c8f8b1615212" width="400" /><br><sub>*Figure 2: Lane Departure Warning System.*</sub><br>|

<sub>*Source: Mitsubishi Motors Website*</sub>


      
---


### Disclaimer
**This project was developed and validated using the SCANeR Studio co-simulation middleware.**
Since the simulation environment is proprietary, the `.slx` files provided here (if still present) are for **algorithmic review and architectural demonstration**. They cannot be executed without the specific SCANeR Studio licenses and interfaces.

---

### Project Overview

This project implements a Lane Keeping Assistance (LKA) system for an autonomous vehicle. The goal is to minimize the lateral deviation ($y_L$) and heading error ($\psi_L$) relative to the road center. The control strategy relies on a Linear Quadratic Regulator (LQR) applied to a dynamic bicycle model. A key feature of this implementation is the **Gain Scheduling** mechanism, which adapts the control gains in real-time based on the vehicle's longitudinal speed ($V_x$) to ensure stability across different driving regimes.

The system was developed and validated using **MATLAB/Simulink**, interfaced with a driving simulator setup featuring a physical **steering wheel and pedals** to test the system's performance and driver interactions.

<img src="https://github.com/user-attachments/assets/3b7ab8bc-53dc-404e-ab14-d1bb0189257a" width="500" />

---

### Technical Approach
### 1. Mathematical Modeling

#### 1.1. Lateral Vehicle Dynamics (2-DOF Bicycle Model)

<img src="https://github.com/user-attachments/assets/e3ab5e3f-99f7-4182-9a5a-1ae2da905fa5" align="right" width="300" alt="Bicycle Model Schema" />

To simulate the vehicle's behavior efficiently, we utilize a **2-Degree-of-Freedom (2-DOF) Bicycle Model**. This model simplifies the 4-wheel dynamics by assuming that forces acting on wheels of the same axle are identical, effectively combining them into one equivalent wheel per axle.

<br clear="true" />

The Bicycle Model Dynamic can be written as :

$$
\begin{cases} 
\dot{v}_x = \frac{T_t - T_r}{I_{eff}} - \frac{c_x v_x |v_x|}{m} + v_y r, \\
\dot{v}_y = \frac{F_{yf} + F_{yr} + f_w - c_y v_y |v_y|}{m} - v_x r, \\
\dot{r} = \frac{l_f F_{yf} - l_r F_{yr} + l_w f_w}{I_z}.
\end{cases}
$$

Where:
* $\psi$: Yaw angle
* $r = \dot{\psi}$: Yaw rate
* $v_x, v_y$: Longitudinal and lateral velocities
* $T_t$: Traction/Braking torque
* $T_r$: Rolling resistance torque
* $F_{yf}, F_{yr}$: Front and rear lateral tire forces
* $f_w$: Lateral wind force
* $C_x, C_y$: Longitudinal and lateral aerodynamic drag coefficients
* $I_{eff}, I_z$: Effective inertia and Yaw inertia

Assuming small slip angles, the lateral tire forces are modeled as linear functions of the slip angles:

$$
\begin{cases} 
F_{yf} = C_f \alpha_f = C_f \left( \delta - \frac{v_y + l_f r}{v_x} \right), \\
F_{yr} = C_r \alpha_r = -C_r \left( \frac{v_y - l_r r}{v_x} \right).
\end{cases}
$$

With $\alpha_f$ and $\alpha_r$ are the front and rear side slip angles and where $\delta$ is front wheel steering angle.

---

#### 1.2. Linearized Model

To implement a linear control strategy (specifically LQR), the system dynamics must be expressed in a linear form. Therefore, the non-linear bicycle model was linearized around a constant operating point $v_x$ (assuming null longitudinal dynamics).

The associated linear dynamic State-Space Model (SSM) can be written as:

$$
\begin{bmatrix} 
\dot{v}_y \\
\dot{r} \\
\end{bmatrix} = 
  \begin{bmatrix} 
  a_{11} & a_{12} \\
  a_{21} & a_{22}
  \end{bmatrix} . 
  \begin{bmatrix}
  v_y \\
  v_r \\
  \end{bmatrix} +
  \begin{bmatrix}
  b_1 \\
  b_2 \\
  \end{bmatrix} \delta +
  \begin{bmatrix}
  e_1 \\
  e_2 \\
  \end{bmatrix} f_w
$$


Where : 

$$
\begin{aligned}
a_{11} &= -\frac{2\left(C_r + C_f\right)}{M v_x}, &
a_{12} &= -v_x + \frac{2\left(l_r C_r - l_f C_f\right)}{M v_x},\\
a_{21} &= \frac{2\left(l_r C_r - l_f C_f\right)}{I_z v_x}, &
a_{22} &= -\frac{2\left(l_r^{2} C_r + l_f^{2} C_f\right)}{I_z v_x},\\
b_{1} &= \frac{2C_f}{M}, &
b_{2} &= \frac{2 l_f C_f}{I_z},\\
e_{1} &= \frac{1}{M}, &
e_{2} &= \frac{l_w}{I_z}.
\end{aligned}
$$

---

#### 1.3. Lane Positioning Dynamics

To control the vehicle's position relative to the lane, we must model how the car moves with respect to the road geometry. This introduces the specific **Lane Keeping Dynamics**.

<img src="https://github.com/user-attachments/assets/feb7b0f5-c814-4108-8dc7-0da3b8d677ed" align="right" width="350" alt="Lane Positioning Model"/>

The system is defined by two key errors:

1.  **Heading Error ($\psi_L$):** The angle difference between where the car is pointing and the actual direction of the road lane.
2.  **Lateral Deviation ($y_L$):** The distance separating the vehicle from the center of the lane.

The error dynamics, assuming a small heading error ($\psi_L$), are modeled as:

$$
\begin{cases} 
\dot{y}_L = v_y + l_s r + v_x \psi_L \\
\dot{\psi}_L = r - v_x \kappa
\end{cases}
$$

Where $\kappa$ represents the road curvature.

<br clear="right"/>

---

#### 1.4. Global State-Space Representation

At this stage, we establish the global lateral dynamics by combining the linearized bicycle model (1.2) and the lane positioning dynamics (1.3).

To design a controller that minimizes lateral deviation and heading error, we define the **state vector $x$** containing both vehicle dynamics and lane errors as:

$$
x = \begin{bmatrix} \beta \\ 
r \\ 
\psi_L \\ 
y_L \end{bmatrix} = \begin{bmatrix} 
\text{Side slip angle} \\ 
\text{Yaw rate} \\ 
\text{Heading error} \\ 
\text{Lateral deviation} 
\end{bmatrix}
$$

By neglecting the wind effect ($f_w = 0$) and considering the side-slip angle relation $\beta = \frac{v_y}{v_x}$, the global state-space model becomes:

$$
\begin{bmatrix} 
\dot{\beta} \\ 
\dot{r} \\ 
\dot{\psi}_L \\ 
\dot{y}_L 
\end{bmatrix} = 
\begin{bmatrix} 
a_{11} & \frac{a_{12}}{v_x} & 0 & 0 \\ 
a_{21}v_x & a_{22} & 0 & 0 \\ 
0 & 1 & 0 & 0 \\ 
v_x & l_s & v_x & 0 
\end{bmatrix}
\begin{bmatrix} 
\beta \\ 
r \\ 
\psi_L \\ 
y_L 
\end{bmatrix} + 
\begin{bmatrix} 
\frac{b_1}{v_x} \\ 
b_2 \\ 
0 \\ 
0 
\end{bmatrix} \delta + 
\begin{bmatrix} 
0 \\ 
0 \\ 
-1 \\ 
0 
\end{bmatrix} \dot{\Psi}_{des}
$$

---

### 2. Control Strategy

#### 2.1. Method

At this stage, the system is designed to control the front wheel steering angle in order to maintain the vehicle at the center of the road.

> **Before proceeding, we verified that the system is controllable, confirming that a feasible solution exists to stabilize the vehicle dynamics.**

By defining the state variables as errors relative to the lane center, the control objective simplifies to a **regulation to zero**. Driving these errors to null naturally forces the vehicle to converge toward the desired trajectory. (actual_state + desired_state = 0 --> actual_state = desired_state).

To achieve this, I implemented a **state-feedback control** loop using a Linear Quadratic Regulator (LQR). 

<img width="600" alt="image" src="https://github.com/user-attachments/assets/42423a7b-3688-4100-a5b3-b960f6a30d57" />

The "optimality" of this controller comes from minimizing a cost function $J$ that balances tracking accuracy against control effort:

$$
J = \int_{0}^{\infty} (x^T Q x + u^T R u) dt
$$

* **$x^T Q x$**: Penalizes state errors (deviation from the center).
* **$u^T R u$**: Penalizes the control input (steering magnitude) to ensure smooth driving.

At this stage, the control law $u(t)$ is defined as:

$$
\delta = -K \cdot x(t)
$$

Where $\delta$ represents the steering angle command and $K$ is the optimal gain matrix computed by the LQR algorithm to minimize these errors.

---

#### 2.2. Gain Scheduling Implementation

Standard LQR controllers are optimal only around their specific linearization point.

**The Challenge:**
During initial testing with a fixed controller tuned for $v_x = 10 \text{ m/s}$, I observed that performance degraded significantly as the speed deviated from this value. This is expected behavior: a single gain matrix $K$ cannot accommodate the non-linear dynamics of the vehicle over a wide range, leading to understeer or instability at higher speeds.

**The Solution:**
To address this, I implemented a **Gain Scheduling** strategy to cover the operating range.

1.  **Offline Computation:** I linearized the model at multiple velocity setpoints and computed the corresponding optimal gain matrix $K$ for each point.
2.  **Online Adaptation:** During the simulation, the system reads the current vehicle speed ($v_{x,sim}$) and dynamically selects the appropriate gain matrix to ensure optimal stability at any speed. (Through a MATLAN FUNCTION)

>  Note : The gain scheduling is designed to match realistic driving conditions. Since cornering above 100 km/h is outside the intended operating range, the controller is scheduled up to 100 km/h only.

<table>
  <thead>
    <tr>
      <th>V<sub>x,from sim</sub> (km/h)</th>
      <th>V<sub>x</sub> (km/h)</th>
      <th><i>K</i></th>
    </tr>
    <tr>
      <th><em>Vehicle speed intervals<br>(from real-time Simulink)</em></th>
      <th><em>Linearization speed<br>(operating point)</em></th>
      <th><em>LQR controller gain</em></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>[0, 20]</td>
      <td align="center">10</td>
      <td>K<sub>1</sub> = [0.2801&nbsp;&nbsp;0.2067&nbsp;&nbsp;1.0665&nbsp;&nbsp;3.1623]</td>
    </tr>
    <tr>
      <td>]20, 40]</td>
      <td align="center">30</td>
      <td>K<sub>2</sub> = [1.1957&nbsp;&nbsp;0.4187&nbsp;&nbsp;1.7615&nbsp;&nbsp;3.1623]</td>
    </tr>
    <tr>
      <td>]40, 60]</td>
      <td align="center">50</td>
      <td>K<sub>3</sub> = [2.0605&nbsp;&nbsp;0.5083&nbsp;&nbsp;2.5596&nbsp;&nbsp;3.1623]</td>
    </tr>
    <tr>
      <td>]60, 80]</td>
      <td align="center">70</td>
      <td>K<sub>4</sub> = [2.9233&nbsp;&nbsp;0.5549&nbsp;&nbsp;3.4047&nbsp;&nbsp;3.1623]</td>
    </tr>
    <tr>
      <td>]80, 100]</td>
      <td align="center">90</td>
      <td>K<sub>5</sub> = [3.8056&nbsp;&nbsp;0.5825&nbsp;&nbsp;4.2869&nbsp;&nbsp;3.1623]</td>
    </tr>
    <tr>
      <td>]100, +&infin;[</td>
      <td align="center">110</td>
      <td>K<sub>6</sub> = [4.7138&nbsp;&nbsp;0.6003&nbsp;&nbsp;5.2024&nbsp;&nbsp;3.1623]</td>
    </tr>
  </tbody>
</table>

<p align="left"><em>Table 1: LQR gain scheduling based on vehicle longitudinal velocity</em></p>
<div style="height:6px;"></div>
<img width="500" alt="image" src="https://github.com/user-attachments/assets/3fead8f7-9c7b-4e05-94d2-cddddd51d896" />

<p align="left"><em>Figure: LQR gain scheduling illustration</em></p>

---

### 3. Simulation Architecture

The simulation loop is closed using **SCANeR Studio** for the environment and vehicle physics, and **MATLAB/Simulink** for the control logic.

The diagram below illustrates the control loop implementation. It receives real-time state data from the simulator ($v_x, \beta, r, \psi_L, y_L$), selects the appropriate gain matrix $K$ based on the velocity, and computes the steering command $\delta$.

<p align="center">
  <img width="800" src="https://github.com/user-attachments/assets/55b7d429-7e81-41cb-8941-6c2817f22d52" alt="Simulink LQR Model">
  <br>
  <em><strong>Figure 3:</strong> Simulink implementation of the architecture.</em>
</p>

### 4. System Integration & Results

To finalize the deployment, the optimal wheel angle $\delta$ computed by the LQR is converted into a **steering wheel angle** using the vehicle's steering ratio. In parallel, a longitudinal controller was implemented to regulate the vehicle's speed.

The complete system was fused into a Stateflow logic to handle three distinct driving modes:
* **Manual Mode:** The driver has full control via the hardware steering wheel and pedals.
* **Semi-Autonomous:** The driver controls the speed (pedals), while the LQR algorithm manages the trajectory (steering).
* **Full Autonomous:** The system manages both longitudinal speed and lateral positioning independently.

The project concludes with a complete **Co-Simulation setup** linking the MATLAB code, the SCANeR Studio physics engine, and a real cockpit (Logitech Steering Wheel & Pedals).

**Check out the final demonstration video:**


