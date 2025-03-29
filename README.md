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
z(t) \\
p(t)
\end{bmatrix} + 
\begin{bmatrix}
m_1 \\ 
m_2
\end{bmatrix}, \quad 
\begin{bmatrix}
z(0) \\
p(T)
\end{bmatrix} = 
\begin{bmatrix}
0 \\ 
p_1
\end{bmatrix}
$$

Expressions for $a_z(t)$, $b_v(t)$, matrix $M$, vectors $m_1$, $m_2$, and how to compute $v(t)$ from $p(t)$, $z(t)$.


Rearranging (7) to get $ \int_0^T l_1(z(t),v(t)) dt + p_1^T z(T)$

$$\int_0^T \underbrace{D_1 l(x(t)^{[k]},u(t)^{[k]})\cdot z(t) + D_2 l(x(t)^{[k]}, u(t)^{[k]})\cdot v(t) + z(t)^TQ_z z(t) + v(t)^T R_v v(t)}_{\color{red}{l'(z(t),v(t))}} dt + \underbrace{Dm(x(T)^{[k]}\cdot z(T)}_{\color{red}{p_1^T\cdot z(T)}} $$

Factoring:
$$l'(z(t),v(t)) =(a_x(t) + z(t)^TQ_z)z(t) + (b_u(t)+v(t)^TR_v)v(t) $$

Taking a couple directional derivatives gives us

$$ Dl'(z(t),v(t)) = a_x(t) +2Q_zz(t) + b_u(t) +2R_vv(t)$$

Therefore

$$\boxed{a_z(t)  = D_1 l'(z(t),v(t)) = a_x(t) + 2Q_zz(t)  }$$

$$\boxed{b_v(t)  = D_2 l'(z(t),v(t)) = b_u(t) +2R_vv(t)  }$$

Using equation (1), we can factor to isolate for $b_v(t)$ and isolate for $v(t)$ later on.

$$p(t)^TB(t) + b_v(t)^T = 0$$
We find that $b_v(t)^T = -p(t)^TB(t)$, so $b_v(t) = -B(t)^Tp(t)$.
Plugging in what we know about $b_v(t)$, we get

$$b_u(t) + 2R_vv(t) = -B^Tp$$

$$2R_vv(t) = -B^Tp -b_u(t)$$

$$v(t) = -\frac{1}{2}R_v^{-1}B^Tp-\frac{1}{2}R_v^{-1}b_u(t)$$

$$ = -\frac{1}{2}R_v^{-1}(B^Tp+b_u(t))$$

Now we can substitute $v(t)$ and $a_z(t)$ in the $\dot{z}(t)$ and $\dot{p}(t)$ expressions.

$$\dot{z}(t) = A(t)z(t) + B(t)v(t) $$
$$\dot{p}(t) = -A(t)^Tp(t) -a_z(t) $$

$$\dot{z}(t) = Az = B(-\frac{1}{2}R_v^{-1}B^Tp-\frac{1}{2}R_v^{-1}b_u)$$
$$\dot{p}(t) = -A^Tp -2Q_zz-a_x $$

In matrix form, in the form $\dot{\mathbf{x}} = M\mathbf{x} + m$

$$
\boxed{
\begin{bmatrix}
\dot{z}(t)\\
\dot{p}(t)
\end{bmatrix} = \begin{bmatrix}
A & -\frac{1}{2} B R_v^{-1}B^T \\
-2Q_z & -A^T
\end{bmatrix}
\begin{bmatrix}
z(t) \\
p(t)
\end{bmatrix} +
 \begin{bmatrix}
-\frac{1}{2}BR_v^{-1}b_u \\
-a_x
\end{bmatrix}}
$$


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

![image](https://github.com/user-attachments/assets/2d259f4b-afdd-4e60-855c-33adab013e01)



---

## 3. Optimla Control Application

Apply iLQR to a differential drive robot with time $ T = 2\pi $ to track:

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

![image](https://github.com/user-attachments/assets/74b8809d-41d3-4bad-89d2-0534a0e32507)

