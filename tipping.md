# Tipping Stability, Zero Moment Point, and Wheel Normal Force Derivation

## 1. System Definition

**Robot drivebase:**
- $N_w$ wheels in contact with the ground, at positions $r_j^w$, $j = 1, \dots, N_w$
- All contacts coplanar, lying on $z = 0$ (the ground plane)
- Contacts form a convex support polygon $\mathcal{P}$ (no wheel is interior to the hull of the others)

**Robot mass distribution:**
- $N$ point masses $m_i$, $i = 1, \dots, N$
- Each at position $r_i(t)$, with known acceleration $a_i(t)$

**Gravity:**
$$
\vec g = -g\hat z
$$

**Wheel contact forces:**
- Normal force $F_j^N \hat z$ at each wheel (must satisfy $F_j^N \ge 0$ — ground can only push)
- In-plane friction force $f_j$ at each wheel (horizontal, does not affect tipping — shown below)

---

## 2. Effective Force per Mass

Define, for each point mass, the combined gravity + d'Alembert inertial force:

$$
\vec N_i = m_i(\vec g - a_i)
$$

Components:
$$
N_{ix} = -m_i a_{i,x}, \qquad N_{iy} = -m_i a_{i,y}, \qquad N_{iz} = m_i(-g - a_{i,z})
$$

---

## 3. Exact Moment Balance (Newton–Euler, whole robot)

Taking moments about any ground-plane point $P$ (internal structural forces cancel):

$$
\sum_i (r_i - P)\times m_i\vec g \;+\; \sum_j (r_j^w - P)\times F_j^N \hat z \;+\; \sum_j (r_j^w - P)\times f_j \;=\; \sum_i (r_i - P)\times m_i a_i
$$

Grouping gravity and inertia together as $\vec N_i$, and moving everything to one side:

$$
\sum_i (r_i-P)\times \vec N_i \;+\; \sum_j (r_j^w-P)\times F_j^N\hat z \;+\; \sum_j (r_j^w-P)\times f_j \;=\;0
$$

**Key fact:** If $P$ is chosen on the ground plane ($z=0$):
- The gravity+inertia term and the normal-force term are purely **horizontal** (their $z$-component vanishes automatically).
- The friction term is purely **vertical** (pure yaw) — it never contributes to tipping and can be dropped from the horizontal (x-y) analysis.

---

## 4. Tipping Criterion (edge-wise test)

For a candidate tipping edge $e$ of the polygon $\mathcal P$, connecting adjacent wheels $r_j^w, r_{j+1}^w$ (CCW-ordered):

$$
\hat e_e \propto r_{j+1}^w - r_j^w \quad\text{(edge direction)}, \qquad \hat n_e = \hat z \times \hat e_e \quad\text{(outward horizontal normal)}
$$

Pick $P$ on edge $e$. Wheels **on** the edge contribute nothing to the moment about $\hat e_e$ (their moment arm is parallel to $\hat e_e$). Wheels **off** the edge (interior side, since $\mathcal P$ convex) contribute only non-negative amounts (proportional to $F_j^N \ge 0$).

Define the **tipping moment**:

$$
\tau_P \equiv \Big[\sum_i (r_i-P)\times m_i(\vec g - a_i)\Big]_{xy}\cdot \hat e_e
$$

**Tipping condition:**

$$
\boxed{\text{Robot tips about edge } e \iff \tau_P > 0}
$$

(If $\tau_P > 0$, no non-negative distribution of off-edge $F_j^N$ can balance the equation — those wheels lift off.)

Check this over all $N_w$ edges; robot is stable iff none trip the inequality.

---

## 5. Zero Moment Point (ZMP) — simplification

Instead of testing each edge separately, define a single point $r_{zmp} = (x_{zmp}, y_{zmp}, 0)$ where the **horizontal** moment of all $\vec N_i$ vanishes.

**Derivation:** with $r_i - P = (x_i - x,\ y_i - y,\ z_i)$, the moment components are:

$$
\tau_x = \sum_i\big[(y_i-y)N_{iz} - z_i N_{iy}\big], \qquad
\tau_y = \sum_i\big[z_i N_{ix} - (x_i-x)N_{iz}\big]
$$

Setting $\tau_x = \tau_y = 0$ and solving for $(x,y) = (x_{zmp}, y_{zmp})$:

$$
\boxed{
x_{zmp} = \frac{\sum_i\big(x_i N_{iz} - z_i N_{ix}\big)}{\sum_i N_{iz}}
\;,\qquad
y_{zmp} = \frac{\sum_i\big(y_i N_{iz} - z_i N_{iy}\big)}{\sum_i N_{iz}}
}
$$

**Tipping condition (ZMP form):**

$$
\boxed{\text{Robot tips} \iff r_{zmp} \notin \mathcal P}
$$

i.e. a single point-in-polygon test.

**Equivalence to edge-wise test:** for point $P_e$ on edge $e$,

$$
\tau_{P_e}\cdot\hat n_e = \Big(\sum_i N_{iz}\Big)\big[(P_e - r_{zmp})\cdot \hat n_e\big]
$$

so (assuming $\sum_i N_{iz} > 0$) $\tau_{P_e}\cdot\hat n_e > 0 \iff r_{zmp}$ is outside edge $e$ — confirming the two formulations agree.

**Degenerate case:** if $\sum_i N_{iz} \to 0$ (e.g. free-fall), $r_{zmp}$ is ill-defined; handle as a separate case (all wheels unloaded).

---

## 6. Computing Individual Wheel Normal Forces $F_j^N$

The normal forces must satisfy vertical force balance + the two horizontal moment equations (same equations that define $r_{zmp}$, solved in reverse):

$$
\sum_{j=1}^{N_w} F_j^N = W_{\text{eff}} \equiv \sum_i N_{iz}
$$
$$
\sum_{j=1}^{N_w} x_j^w F_j^N = W_{\text{eff}}\, x_{zmp}, \qquad
\sum_{j=1}^{N_w} y_j^w F_j^N = W_{\text{eff}}\, y_{zmp}
$$

This is **3 equations** for $N_w$ unknowns.

### Case $N_w = 3$ (tripod): statically determinate

Solve directly:

$$
\begin{bmatrix}1&1&1\\x_1^w&x_2^w&x_3^w\\y_1^w&y_2^w&y_3^w\end{bmatrix}
\begin{bmatrix}F_1^N\\F_2^N\\F_3^N\end{bmatrix}
= W_{\text{eff}}\begin{bmatrix}1\\x_{zmp}\\y_{zmp}\end{bmatrix}
$$

Equivalently, express $r_{zmp}$ in **barycentric coordinates** of the triangle: $r_{zmp} = \lambda_1 r_1^w + \lambda_2 r_2^w + \lambda_3 r_3^w$, $\sum \lambda_j = 1$. Then:

$$
\boxed{F_j^N = \lambda_j\, W_{\text{eff}}}
$$

Each $\lambda_j \ge 0$ automatically iff $r_{zmp}$ is inside the triangle — consistent with the tipping criterion.

### Case $N_w > 3$: statically indeterminate

More unknowns than equations (rigid body on 4+ rigid point contacts, like a 4-legged table). Requires an added physical assumption.

**(a) Compliance / stiffness model** (most physically grounded — this is the one that remains valid even when $r_{zmp} \notin \mathcal P$):

Model each wheel/suspension with stiffness $k_j$: $F_j^N = k_j\, \delta_j$, where $\delta_j$ is compression, assumed to follow a rigid-plane fit:
$$
\delta_j = \delta_0 + \alpha x_j^w + \beta y_j^w
$$

For **equal stiffness** $k_j = k$ at every wheel (the common rigid 4-wheel platform case), shift to the wheel centroid $(\bar x, \bar y)$:

$$
x_j' = x_j^w - \bar x, \qquad y_j' = y_j^w - \bar y
$$

Second moments about the centroid:
$$
I_{xx} = \sum_j y_j'^2, \qquad I_{yy} = \sum_j x_j'^2, \qquad I_{xy} = \sum_j x_j' y_j'
$$

Horizontal moments taken about the centroid: $M_x', M_y'$ (same construction as $\tau_x, \tau_y$ above, but about $(\bar x, \bar y)$ instead of the origin).

Solve:
$$
\begin{bmatrix}I_{yy}&I_{xy}\\I_{xy}&I_{xx}\end{bmatrix}\begin{bmatrix}\alpha\\\beta\end{bmatrix}=\begin{bmatrix}M_y'\\-M_x'\end{bmatrix}
$$

Then:
$$
\boxed{F_j^N = \frac{W_{\text{eff}}}{N_w} + \alpha\, x_j' + \beta\, y_j'}
$$

This is the classic **bolt-pattern / four-point support formula**, and is equivalent to the minimum-norm (pseudoinverse) least-squares solution. It remains a valid (if physically approximate — some $F_j^N$ come out negative, meaning that wheel would need to pull rather than push) computation even when $r_{zmp} \notin \mathcal P$, since it doesn't presuppose the equilibrium constraint is feasible with $F_j^N \ge 0$ — it just returns whatever linear solution satisfies the fit, negative values included. This is the mechanism by which the compliance model still "works" (in the sense of returning a value) outside $\mathcal P$: it's an unconstrained solve, and the return of a negative $F_j^N$ is itself the signal of tipping/liftoff at that wheel.

**(b) Constrained optimization** (alternative, enforces $F_j^N \ge 0$ explicitly):

$$
\min_{F_j^N \ge 0} \sum_j (F_j^N)^2 \quad \text{s.t. the 3 equilibrium equations above}
$$

A small QP. Feasible only when $r_{zmp} \in \mathcal P$ — solving this and checking feasibility is itself an alternative implementation of the tipping test, and directly returns per-wheel loads when stable.

---

## 7. Summary of Variable Definitions

| Symbol | Meaning |
|---|---|
| $N_w$ | number of wheels |
| $r_j^w$ | position of wheel $j$ contact point (ground plane) |
| $\mathcal P$ | convex support polygon formed by wheel contacts |
| $N$ | number of point masses |
| $m_i$ | mass of point $i$ |
| $r_i$ | position of mass $i$ |
| $a_i$ | acceleration of mass $i$ |
| $\vec g$ | gravity vector, $-g\hat z$ |
| $F_j^N$ | normal (vertical) force at wheel $j$, $\ge 0$ |
| $f_j$ | friction (horizontal) force at wheel $j$ |
| $\vec N_i$ | effective force per mass, $m_i(\vec g - a_i)$ |
| $P$ | reference point for moment balance |
| $\tau_P$ | tipping moment (gravity + inertia only) about $P$, projected to x-y |
| $\hat e_e, \hat n_e$ | edge direction / outward normal for edge $e$ |
| $r_{zmp} = (x_{zmp}, y_{zmp})$ | zero moment point |
| $W_{\text{eff}}$ | total effective vertical force, $\sum_i N_{iz}$ |
| $\lambda_j$ | barycentric weight of wheel $j$ (tripod case) |
| $k_j$ | wheel/suspension stiffness |
| $\delta_j$ | wheel compression |
| $\bar x, \bar y$ | wheel centroid coordinates |
| $I_{xx}, I_{yy}, I_{xy}$ | second moments of wheel positions about centroid |
| $\alpha, \beta$ | rigid-plane tilt fit parameters |
