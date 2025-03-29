# Optimal Control


## Preliminaries

Given the dynamics of a system:

$$
\dot{x}(t) = f(t, x(t), u(t)), \quad x(0) = x_0
$$

we want to compute the optimal control signal $u(t)$ to track a desired trajectory $x_d(t)$ over the time horizon $[0, T]$. The optimal control problem is:

$$
u^*(t) = \arg\min_{u(t)} J(u(t)) = \arg\min_{u(t)} \int_0^T \left[ (x(t) - x_d(t))^\top Q_x (x(t) - x_d(t)) + u(t)^\top R_u u(t) \right] dt + (x(T) - x_d(T))^\top P_1 (x(T) - x_d(T))
$$

subject to:

$$
x(t) = x_0 + \int_0^t f(x(\tau), u(\tau)) d\tau
$$

This can be solved via iterative LQR (iLQR). In iteration $k$, given $u^{[k]}(t)$ and corresponding $x^{[k]}(t)$, compute the optimal descent direction $v^{[k]}(t)$:

$$
v^{[k]}(t) = \arg\min_{v(t)} \int_0^T \left[ a_x(t)^\top z(t) + b_u(t)^\top v(t) \right] dt + p_1^\top z(T) + \int_0^T \left[ z(t)^\top Q_z z(t) + v(t)^\top R_v v(t) \right] dt
$$

with dynamics:

$$
z(t) = 0 + \int_0^t A(\tau) z(\tau) + B(\tau) v(\tau) d\tau
$$

Update control using Armijo line search.

---

The system can be expressed with:

$$
p(t)^\top B(t) + b_v(t)^\top = 0
$$

$$
\dot{p}(t) = -A(t)^\top p(t) - a_z(t)
$$

$$
\dot{z}(t) = A(t) z(t) + B(t) v(t)
$$

Boundary conditions:

$$
z(0) = 0, \quad p(T) = p_1
$$

This leads to a two-point boundary value problem (TPBVP): 

$$
\begin{bmatrix}
\dot{z}(t) \\
\dot{p}(t)
\end{bmatrix} = M
\begin{bmatrix}
z(t) \\ p(t)
\end{bmatrix} + 
\begin{bmatrix}
m_1 \\ m_2
\end{bmatrix}, \quad 
\begin{bmatrix}
z(0) \\ p(T)
\end{bmatrix} = 
\begin{bmatrix}
0 \\ p_1
\end{bmatrix}
$$

Expressions for $ a_z(t) $, $ b_v(t) $, matrix $ M $, vectors $ m_1 $, $ m_2 $, and how to compute $ v(t) $ from $ p(t) $, $ z(t) $.

---

Minimize:

$$
x^* = \arg\min_x f(x) = \arg\min_x \left[ 0.26(x_1^2 + x_2^2) - 0.46 x_1 x_2 \right]
$$

Use gradient descent with Armijo line search:
- Initial guess: $ x = [-4, -2] $
- Parameters: $ \alpha = 10^{-4}, \beta = 0.5, \eta_0 = 1 $
- Run for 100 iterations


### Algorithm: Armijo Line Search (Pseudocode)

```
Procedure Armijo(x[k], eta0, alpha, beta)
    eta = eta0
    z[k] = -grad J(x[k])
    while J(x[k] + eta * z[k]) > J(x[k]) + alpha * eta * grad J(x[k])^T z[k] do
        eta = beta * eta
    return x[k] + eta * z[k]
end procedure
```

---

## 3rd problem

Apply iLQR to a differential drive robot with time \( T = 2\pi \) to track:

$$
(x_d(t), y_d(t), \theta_d(t)) = \left( \frac{4}{2\pi}t, 0, \frac{\pi}{2} \right)
$$

with dynamics:

$$\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\theta}
\end{bmatrix} =
\begin{bmatrix}
\cos(\theta) u_1 \\
\sin(\theta) u_1 \\
u_2
\end{bmatrix}, \quad
(x(0), y(0), \theta(0)) = (0, 0, \frac{\pi}{2})
$$

**Turn in:**
- Three runs with different initial control trajectories and objective parameters
- One trajectory must use \( [1, -0.5] \) for the full horizon
- For each run:
  1. Plot the initial and final trajectory
  2. Plot optimal control signals
  3. Plot objective function vs. iteration
  4. List the parameters used

📎 iLQR template: [Google Drive Link](https://drive.google.com/file/d/1Br8DArJtnEZXjZok2aWh7PMVoTuRq1hc/view?usp=sharing)
