# Optimal Control and Ergodic Control

\documentclass{article}
\usepackage{amsmath, amssymb}
\usepackage{graphicx}


\maketitle

\section*{Preliminaries}
Given the dynamics $\dot{x}(t) = f(t, x(t), u(t))$ and initial state $x(0) = x_0$, we aim to calculate the optimal control signal $u(t)$ to track a desired trajectory $x_d(t)$ within the time horizon $[0, T]$. The optimal control problem is:

\[
u^*(t) = \arg\min_{u(t)} J(u(t)) = \arg\min_{u(t)} \int_0^T \left[ (x(t)-x_d(t))^\top Q_x(x(t)-x_d(t)) + u(t)^\top R_u u(t) \right] dt + (x(T)-x_d(T))^\top P_1(x(T)-x_d(T))
\]

subject to:

\[
x(t) = x_0 + \int_0^t f(x(\tau), u(\tau)) d\tau
\]

We solve this using iLQR. In the $k$-th iteration, given $u^{[k]}(t)$ and corresponding $x^{[k]}(t)$, we solve:

\[
v^{[k]}(t) = \arg\min_v \int_0^T a_x(t)^\top z(t) + b_u(t)^\top v(t) dt + p_1^\top z(T) + \int_0^T z(t)^\top Q_z z(t) + v(t)^\top R_v v(t) dt
\]

where $z(t), v(t)$ evolve as:

\[
z(t) = 0 + \int_0^t A(\tau)z(\tau) + B(\tau)v(\tau) d\tau
\]

Update $u^{[k+1]}(t)$ using Armijo line search.

\section*{Problem 1 (20 pts)}
Solve (7) using:

\[
p^\top(t) B(t) + b_v^\top(t) = 0
\]
\[
\dot{p}(t) = -A(t)^\top p(t) - a_z(t)
\]
\[
\dot{z}(t) = A(t)z(t) + B(t)v(t)
\]

with:
\[
z(0) = 0,\quad p(T) = p_1
\]

Reorganize as:

\[
\begin{bmatrix} \dot{z}(t) \\ \dot{p}(t) \end{bmatrix} = 
M \begin{bmatrix} z(t) \\ p(t) \end{bmatrix} + \begin{bmatrix} m_1 \\ m_2 \end{bmatrix}, \quad
\begin{bmatrix} z(0) \\ p(T) \end{bmatrix} = \begin{bmatrix} 0 \\ p_1 \end{bmatrix}
\]

Give expressions for $a_z(t), b_v(t), M, m_1, m_2$, and how to compute $v(t)$ from $p(t), z(t)$.

\section*{Problem 2 (20 pts)}
Minimize:

\[
x^* = \arg\min_x f(x) = \arg\min_x \left[ 0.26(x_1^2 + x_2^2) - 0.46 x_1 x_2 \right]
\]

Use gradient descent with Armijo line search:
- Initial $x = [-4, -2]$
- Parameters: $\alpha = 10^{-4}$, $\beta = 0.5$, $\eta_0 = 1$
- Run 100 iterations

\textbf{Turn in:} Plot the iteration trajectory over contour of $f(x)$.

\subsection*{Algorithm: Armijo Line Search}
\begin{verbatim}
procedure Armijo(x[k], eta0, alpha, beta)
    eta = eta0
    z[k] = -grad J(x[k])
    while J(x[k] + eta*z[k]) > J(x[k]) + alpha*eta*grad J(x[k])^T z[k] do
        eta = beta * eta
    return x[k] + eta*z[k]
end procedure
\end{verbatim}

\section*{Problem 3 (60 pts)}
Apply iLQR to a differential drive robot with time horizon $T = 2\pi$ to track:

\[
(x_d(t), y_d(t), \theta_d(t)) = \left( \frac{4}{2\pi} t, 0, \frac{\pi}{2} \right)
\]

Dynamics:

\[
\begin{bmatrix}
\dot{x} \\ \dot{y} \\ \dot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
\cos(\theta) u_1 \\
\sin(\theta) u_1 \\
u_2
\end{bmatrix}, \quad (x(0), y(0), \theta(0)) = (0, 0, \frac{\pi}{2})
\]

\textbf{Turn in:}
- Three different initial control trajectories and objective parameters (one should be $[1, -0.5]$).
- For each: convergence criterion, and plots showing:
  1. Initial and converged trajectory
  2. Optimal control signals
  3. Objective function iterations

Template link: \texttt{https://drive.google.com/file/d/1Br8DArJtnEZXjZok2aWh7PMVoTuRq1hc/view?usp=sharing}

\end{document}
