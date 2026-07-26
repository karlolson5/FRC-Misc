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

### (a′) Iterative zero-force correction (active-set method)

If the equal-stiffness compliance solution **(a)** returns a negative $F_j^N$ for some wheel, that wheel is not truly load-bearing (in a rigid-plane model, a negative value means the plane fit wants to *pull* on that wheel, which is unphysical). The standard fix is an **active-set iteration**:

1. Solve the equal-stiffness plane fit (Section 6a) over the current active wheel set (start with all $N_w$ wheels).
2. If any $F_j^N < 0$, remove that wheel from the active set (it has lifted off).
3. **Recompute the centroid $(\bar x,\bar y)$ and second moments $I_{xx}, I_{yy}, I_{xy}$ using only the remaining active wheels**, and re-solve. (Removing a wheel changes the geometry, so the fit must be redone from scratch — don't just zero the one entry and keep the rest.)
4. Repeat until all remaining $F_j^N \ge 0$.

This is guaranteed to terminate in at most $N_w - 3$ drops.

**Terminal case ($N_w \to 3$ active wheels):** switch to the exact tripod/barycentric solution (Section 6, $N_w=3$ case), which is self-checking — $\lambda_j \ge 0$ automatically iff $r_{zmp}$ lies inside the reduced 3-wheel triangle. If it doesn't, the robot is genuinely tipping about an edge of the *original* full polygon $\mathcal P$; the iteration should stop there rather than drop below 3 active contacts, since fewer than 3 non-collinear points can't support a general moment.

**Caveat:** for $N_w>3$, a negative $F_j^N$ in step 1 does not by itself prove the robot is tipping — it can be an artifact of the equal-stiffness/rigid-plane assumption even when $r_{zmp}\in\mathcal P$ on the full polygon. The iterative correction is the right fix regardless of cause, but true instability should still be confirmed by checking $r_{zmp}$ against $\mathcal P$ separately.

**(b) Constrained optimization** (alternative, enforces $F_j^N \ge 0$ explicitly):

$$
\min_{F_j^N \ge 0} \sum_j (F_j^N)^2 \quad \text{s.t. the 3 equilibrium equations above}
$$

A small QP. Feasible only when $r_{zmp} \in \mathcal P$ — solving this and checking feasibility is itself an alternative implementation of the tipping test, and directly returns per-wheel loads when stable.

---

## 7. Summary of Variable Definitions


# Wheel Friction / Traction Force Derivation

## 1. Setup

The same Newton–Euler moment balance (Section 3) also contains the horizontal (in-plane) equilibrium, which was previously discarded because it doesn't affect *tipping*. Recovering it gives the equations governing the friction/traction forces $f_j$ at each wheel.

Recall: gravity and $F_j^N\hat z$ are vertical-only in origin/direction, so they contribute **nothing** to horizontal force balance or to yaw (vertical-axis) moment balance. Only $f_j$ (horizontal, at the wheels) and the mass accelerations $a_i$ can. This is the mirror image of Section 4, where friction dropped out of the *tipping* equations — here, gravity and $F_j^N$ drop out instead.

---

## 2. Horizontal Force Balance

Newton's second law, horizontal components only:

$$
\boxed{\sum_{j=1}^{N_w} f_j = \sum_{i=1}^N m_i\, a_{i,xy}}
$$

where $a_{i,xy} = (a_{i,x}, a_{i,y})$. This is **2 scalar equations** (x and y).

*(Note: unlike the effective force $\vec N_i = m_i(\vec g - a_i)$ used for tipping/ZMP, here it's the actual $m_i a_i$ — gravity has no horizontal component to add.)*

---

## 3. Yaw Moment Balance

Take the $z$-component of the full moment balance about any point $P$ in the ground plane. As shown in Section 3, the gravity and normal-force terms vanish identically in $z$, leaving:

$$
\Big[\sum_i (r_i - P)\times m_i a_i\Big]_z = \sum_j \Big[(r_j^w - P)\times f_j\Big]_z
$$

Expanding the right-hand cross product (both vectors horizontal):

$$
\boxed{\sum_i \big[(x_i - x_P)m_i a_{i,y} - (y_i-y_P) m_i a_{i,x}\big] = \sum_j \big[(x_j^w - x_P)f_{j,y} - (y_j^w-y_P) f_{j,x}\big]}
$$

This is **1 scalar equation** (yaw, about the $z$-axis through $P$).

---

## 4. Indeterminacy — worse than the normal-force case

Total: **3 equations** (2 force + 1 yaw moment), but **$2N_w$ unknowns** ($f_{j,x}, f_{j,y}$ per wheel).

Compare to Section 6: normal forces had 1 unknown/wheel vs. 3 equations (determinate at $N_w=3$). Friction has **2 unknowns/wheel**, so it is indeterminate even at $N_w=3$ ($6$ unknowns vs. $3$ equations) — a physical consequence of each wheel contact being a 2D tangent plane rather than a 1D normal direction. An additional model is required at *any* $N_w \ge 2$ (well-actuated skid-steer robots are the classic real-world example of this indeterminacy).

---

## 5. Closing the System

### (a) Compliance model (tangential stiffness)

Analogous to Section 6(a). Model each wheel/tire contact patch with tangential stiffness $k_{t,j}$ and a rigid-body horizontal + yaw deflection field:

$$
f_j = k_{t,j}\, \delta_j^{\,t}, \qquad \delta_j^{\,t} = \delta_0^{\,t} + \vec\omega_\delta \times (r_j^w - \bar r^w)
$$

where $\delta_0^{\,t}$ is a common horizontal shear deflection and $\vec\omega_\delta \hat z$ a small yaw twist. Substituting into the 3 equilibrium equations (Sections 2–3) gives 3 equations for the 3 unknowns $(\delta_{0,x}^{\,t}, \delta_{0,y}^{\,t}, \omega_\delta)$, which then determine all $f_j$ uniquely — same structure as the equal-stiffness normal-force fit, just with an added rotational (yaw) degree of freedom since the tangent plane allows in-plane torsion.

### (b) Minimum-norm / optimization solve

$$
\min_{f_j} \sum_j \|f_j\|^2 \quad \text{s.t. Sections 2–3 equalities}
$$

Solved via the Moore–Penrose pseudoinverse of the $3\times 2N_w$ constraint matrix — the direct friction analogue of Section 6(b)'s QP.

### Friction cone constraint (unique to this problem)

Unlike $F_j^N$, which is only bounded below ($\ge 0$), $f_j$ is bounded by the **friction cone**:

$$
\|f_j\| \le \mu_j\, F_j^N
$$

using the $F_j^N$ already computed in Section 6. This makes (b) properly a **second-order cone program (SOCP)**, not a simple QP:

$$
\min_{f_j} \sum_j \|f_j\|^2 \quad \text{s.t. Sections 2–3 equalities}, \quad \|f_j\| \le \mu_j F_j^N \ \forall j
$$

If no feasible $f_j$ exists (constraint set empty), the robot's required traction exceeds available friction — **wheel slip**, not tipping. This is the friction-plane analogue of the tipping test: infeasibility here means slip; infeasibility in Section 6 means tip.

### (a′) Iterative correction (active-set analogue)

Same idea as Section (a′) for normal forces: if a compliance solution to (a) yields $\|f_j\| > \mu_j F_j^N$ at some wheel, that wheel is saturated (slipping), not sticking. Clamp it to the cone boundary $f_j = \mu_j F_j^N \, \hat f_j$ (direction from the unconstrained solve), remove it from the "sticking" active set, and re-solve the compliance fit over the remaining wheels — iterate to convergence, same termination logic as before.

---

## 6. Summary of New Variables


# Advanced Swerve Drive Controller Architecture

## 1. Restating the Givens & One Necessary Clarification

**Given (fixed/known):** $r_i, m_i$ (mass elements), $r_j^w, R_j, \mu_j$ (wheel position/radius/friction), $k_j$ (compliance spring constants for normal-force distribution, Section 6a), $\tau_j^{max}$ (drive torque limits).

**Given (state, measured each cycle):** $a_i$ — but note: $a_i$ can't be a free input alongside a *desired* $(a_x, a_y, \alpha_{yaw})$ command, since for a rigid chassis every mass element's acceleration is **determined by** the chassis's rigid-body motion. So the controller's first job is to treat $a_i$ as a function of the commanded chassis acceleration, using the current yaw rate $\omega$ (also measured state) for centripetal coupling:

$$
a_i(\vec u) = \underbrace{(a_x,\,a_y)}_{\text{translation}} + \underbrace{\alpha_{yaw}\,\hat z \times r_i'}_{\text{tangential}} + \underbrace{\omega\,\hat z\times(\omega\,\hat z\times r_i')}_{\text{centripetal, known}}, \qquad r_i' = r_i - r_{cm}
$$

where $\vec u = (a_x, a_y, \alpha_{yaw})$ is the commanded input. This makes every downstream quantity ($r_{zmp}$, $F_j^N$, required wheel wrench) an explicit function of $\vec u$, which is what lets the controller scale $\vec u$ down later.

---

## 2. Pipeline Overview

u_cmd = (ax, ay, α)
        │
        ▼
 [1] Rigid-body kinematics  →  a_i(u)
        │
        ▼
 [2] ZMP / tipping check    →  s_tip  (max safe scale, tipping alone)
        │
        ▼
 [3] Normal force solve     →  F_j^N(u)   (Section 6, compliance + active-set)
        │
        ▼
 [4] Per-wheel force cap    →  F_j^max(u) = min(μ_j F_j^N, τ_j^max / R_j)
        │
        ▼
 [5] Required wrench        →  F_req(u), τ_req(u)   (rigid-body Newton–Euler)
        │
        ▼
 [6] Wrench allocation SOCP →  f_j , s_force  (max achievable fraction)
        │
        ▼
 [7] Combine: s = min(s_tip, s_force, 1); recompute at s·u_cmd
        │
        ▼
 [8] Output: θ_j = atan2(f_j), τ_j = |f_j|·R_j

 The key structural idea: **stages 2–6 are all functions of a single scalar $s$ applied to $\vec u_{cmd}$**, because $a_i$ is affine in $\vec u$, and everything downstream (tipping margin, weight transfer, required wrench, friction budget) inherits that dependence. This is what makes "reduce the magnitude of desired acceleration proportionally" well-posed as a single 1-D search.

---

## 3. Stage Details

### 3.1 Tipping limit → $s_{tip}$

From Section 5, $r_{zmp}(\vec u)$ depends on $N_{iz} = m_i(-g - a_{i,z})$ and $N_{ix,iy} = -m_i a_{i,x,y}$. For a rigid planar chassis, $a_{i,z}=0$ always (no pitch/roll modeled), so $W_{eff} = \sum_i N_{iz}$ is **independent of $\vec u$** — only the numerator (horizontal moment) depends on $\vec u$, and it does so **affinely**. Therefore:

$$
r_{zmp}(s) = r_{zmp}(0) + s\big[r_{zmp}(1) - r_{zmp}(0)\big]
$$

is a straight line in $s\in[0,1]$ from the static ZMP to the fully-commanded ZMP. Finding $s_{tip}$ = the largest $s$ keeping $r_{zmp}(s)\in\mathcal P$ is a single **ray–polygon intersection** — closed form, no iteration needed. (Include a safety margin by shrinking $\mathcal P$ inward before the test.)

### 3.2 Normal force distribution → $F_j^N(s)$

Using $a_i(s\cdot\vec u_{cmd})$, run Section 6's compliance model with the given $k_j$, plus the active-set correction (Section (a′)) for $N_w > 3$. Because horizontal acceleration commands still shift $r_{zmp}$ within $\mathcal P$ (weight transfer), $F_j^N$ is *not* constant in $s$ even for $s \le s_{tip}$ — heavier braking/turning still loads some wheels more than others well before tipping occurs. This matters for the friction budget next.

### 3.3 Per-wheel force capacity → $F_j^{max}(s)$

Because swerve modules can steer to any azimuth, each wheel can (quasi-statically) apply its net contact force **in any direction** — the classic advantage over fixed-heading drives. The magnitude is capped by whichever binds first, friction or motor torque:

$$
\boxed{F_j^{max}(s) = \min\!\big(\mu_j\, F_j^N(s),\ \tau_j^{max}/R_j\big)}
$$

This reduces the friction-force indeterminacy problem from Section (friction derivation) — there we needed a *compliance* model because the forces were passive/reactive. Here the wheels are **actuated**, so instead of a physical compliance law we get to choose $f_j$ via optimization, subject only to this magnitude cap (an isotropic disk constraint per wheel).

### 3.4 Required wrench → $F_{req}(s), \tau_{req}(s)$

Since $a_i(s\vec u)$ is exactly rigid-body kinematics by construction, the sum in Section 3 collapses to the standard rigid-body form (about the center of mass):

$$
F_{req}(s) = s\,M_{tot}\,(a_x,a_y), \qquad \tau_{req}(s) = s\, I_{zz}\,\alpha_{yaw}
$$

with $M_{tot}=\sum_i m_i$, $I_{zz}=\sum_i m_i \|r_i'\|^2$ (about $r_{cm}$).

### 3.5 Wrench allocation (SOCP)

Solve for the $N_w$ contact forces $f_j$ (each a free 2D vector) that reproduce the required wrench while respecting each wheel's disk limit:

$$
\begin{aligned}
\min_{f_j}\quad & \sum_j \left(\frac{\|f_j\|}{F_j^{max}}\right)^2 \\
\text{s.t.}\quad & \sum_j f_j = F_{req} \\
& \sum_j (r_j^w - r_{cm})\times f_j = \tau_{req} \\
& \|f_j\| \le F_j^{max}\ \ \forall j
\end{aligned}
$$

Normalizing the cost by $F_j^{max}$ (rather than raw $\|f_j\|^2$) balances *utilization fraction* across wheels rather than raw force — a wheel carrying less normal load naturally gets asked for less, matching real friction-limited behavior. This is a second-order cone program (SOCP): quadratic-in-a-disk objective, linear equality constraints, disk (cone) inequality constraints.

### 3.6 Feasibility & proportional scale-back → $s_{force}$

Fold the scale factor directly into the allocation problem rather than solving allocation and *then* checking feasibility — this avoids a separate bisection when possible:

$$
\begin{aligned}
\max_{f_j,\ s}\quad & s \\
\text{s.t.}\quad & \sum_j f_j = s\,F_{req}(1), \qquad \sum_j (r_j^w-r_{cm})\times f_j = s\,\tau_{req}(1) \\
& \|f_j\| \le F_j^{max}(s) \ \ \forall j, \qquad 0\le s\le 1
\end{aligned}
$$

**Caveat (coupling with weight transfer):** $F_j^{max}(s)$ itself depends on $s$ through $F_j^N(s)$ (Section 3.2), so this is not a fixed convex set — it's a *family* of convex sets shrinking/growing with $s$. In practice:
- If $F_j^N(s)$ is treated as fixed at its $s=1$ value, the problem above is a single clean SOCP.
- For exactness, wrap it in an outer bisection on $s$: at each trial $s$, recompute $a_i(s)\to F_j^N(s)\to F_j^{max}(s)$, then test feasibility of the *equality* constraints at exactly that $s$ (not maximize) via a feasibility SOCP. Feasibility is monotonic in $s$ (smaller commanded accelerations only reduce required wrench, improve tipping margin, and reduce weight-transfer skew — all favorable), so bisection converges monotonically to $s_{force}$.

### 3.7 Combine and finalize

$$
s = \min(s_{tip},\ s_{force},\ 1)
$$

Recompute the full chain one final time at $\vec u_{final} = s\,\vec u_{cmd}$ to get self-consistent $a_i, F_j^N, f_j$. Output per wheel:

$$
\theta_j = \operatorname{atan2}(f_{j,y}, f_{j,x}), \qquad \tau_j = \|f_j\|\, R_j
$$

**Dimensional note on "proportional reduction":** $(a_x,a_y)$ and $\alpha_{yaw}$ have different units, so scaling "linear + yaw magnitude" together requires a consistent norm — e.g. define an effective combined command $\tilde u = (a_x, a_y, \rho\,\alpha_{yaw})$ using an effective radius $\rho$ (e.g. wheelbase circumradius) to convert yaw acceleration to an equivalent tangential linear acceleration at the wheels, and apply $s$ to $\tilde u$ as a single vector norm. This is a design choice, not derivable from physics alone.

---

## 4. Summary Table

---

## 5. Why This Differs From the Passive-Friction Case

The earlier friction derivation (Section on traction forces) needed a **compliance model** because those forces were reactive/passive and the system was underdetermined with no natural objective. Here, because swerve wheels are independently steered and torqued, the same indeterminacy (more force DOF than equilibrium equations) is resolved instead by an **allocation optimization** — the controller *chooses* $f_j$ to satisfy the required wrench while respecting hard physical caps, rather than solving for what compliance would passively produce. The tipping (Section 5) and normal-force (Section 6) machinery carries over unchanged, since those are governed by the same rigid-body/ZMP physics regardless of how the wheels are actuated.






# Advanced Swerve Drive Controller Architecture (Revised: Internal Mass Motion)

## 0. Key Addition: Internal Mass Accelerations

Each mass element's acceleration now has **two independent contributions**:

$$
a_i(\vec u, t) = a_i^{rigid}(\vec u) + a_i^{int}(t)
$$

- $a_i^{rigid}(\vec u)$ — from the chassis's rigid-body motion (translation + yaw + centripetal), as before, **affine in the commanded** $\vec u=(a_x,a_y,\alpha_{yaw})$.
- $a_i^{int}(t)$ — from internal mechanisms (arm extend/retract, elevator, slide), **known/measured, independent of $\vec u$**. This can include components not tangent to a rigid rotation (e.g. radial extension) and can itself vary each control cycle, but does **not** depend on the chassis command being solved for.

$$
a_i^{rigid}(\vec u) = (a_x,a_y) + \alpha_{yaw}\,\hat z\times r_i' + \omega\,\hat z\times(\omega\,\hat z\times r_i'), \qquad r_i' = r_i - r_{cm}
$$

**Consequence:** every downstream quantity is now **affine but not linear** in $\vec u$ (it has a nonzero offset at $\vec u=0$, coming from internal-mass reaction effects). All of Section 2's proportional-scaling logic still works, but $s=0$ no longer means "no net force/tipping effect" — it means "purely the reaction loads from internal mechanisms." This is physically correct: a fully extended, rapidly retracting arm can shift $r_{zmp}$ or demand wheel torque even while the chassis itself is commanded to zero acceleration.

---

## 1. Restating the Givens

**Fixed/known:** $r_i, m_i$ (mass elements — note $r_i$ may itself be time-varying, e.g. the extending arm's mass position), wheel data $r_j^w, R_j, \mu_j, k_j$, torque limits $\tau_j^{max}$.

**Given each cycle (measured/known state):** current yaw rate $\omega$, and $a_i^{int}(t)$ for every mass element with independent internal motion (from arm/elevator/slide kinematics — these are computed by whatever subsystem controls that mechanism and passed in as known quantities, not solved for here).

**Commanded input:** $\vec u_{cmd} = (a_x, a_y, \alpha_{yaw})$ — the *chassis's* desired rigid-body acceleration.

---

## 2. Pipeline Overview

```
u_cmd = (ax, ay, α)
        │
        ▼
 [1] Rigid-body kinematics  →  a_i(u)
        │
        ▼
 [2] ZMP / tipping check    →  s_tip  (max safe scale, tipping alone)
        │
        ▼
 [3] Normal force solve     →  F_j^N(u)   (Section 6, compliance + active-set)
        │
        ▼
 [4] Per-wheel force cap    →  F_j^max(u) = min(μ_j F_j^N, τ_j^max / R_j)
        │
        ▼
 [5] Required wrench        →  F_req(u), τ_req(u)   (rigid-body Newton–Euler)
        │
        ▼
 [6] Wrench allocation SOCP →  f_j , s_force  (max achievable fraction)
        │
        ▼
 [7] Combine: s = min(s_tip, s_force, 1); recompute at s·u_cmd
        │
        ▼
 [8] Output: θ_j = atan2(f_j), τ_j = |f_j|·R_j
 ```

 ---

## 3. Stage Details

### 3.1 Tipping limit → $s_{tip}$

As before, $N_{iz} = m_i(-g - a_{i,z})$; since $a_i^{int}$ can now have a $z$-component (elevator!), $W_{eff}=\sum_i N_{iz}$ **may itself depend on internal motion**, but critically it is still **independent of $s$** (the chassis command scale), since $a_i^{rigid}$ has no $z$-component for a planar rigid chassis. So the affine-in-$s$ structure survives:

$$
r_{zmp}(s) = r_{zmp}(0) + s\big[r_{zmp}(1)-r_{zmp}(0)\big]
$$

but now $r_{zmp}(0)$ (the $s=0$ intercept) reflects the **static-chassis, moving-mechanism** ZMP — e.g. a raised elevator or extended arm decelerating can by itself push $r_{zmp}(0)$ outside $\mathcal P$, meaning **the robot can tip with zero commanded drive acceleration**, purely from arm/elevator motion. The ray–polygon intersection for $s_{tip}\in[0,1]$ proceeds exactly as before, just starting from this shifted intercept. If $r_{zmp}(0)\notin\mathcal P$ already, $s_{tip}=0$ (or negative — flag as an unrecoverable-by-drive tipping event, since no amount of chassis acceleration scaling fixes a problem that exists at zero chassis command).

### 3.2 Normal force distribution → $F_j^N(s)$

Unchanged in method (Section 6, compliance + active-set), but evaluated using the full $a_i(s) = s\,a_i^{rigid}(1) + a_i^{int}(t)$. Internal mechanism motion now contributes to weight transfer even independent of drive commands (e.g. arm swinging redistributes wheel loads).

### 3.3 Per-wheel force capacity → $F_j^{max}(s)$

Unchanged formula, $F_j^{max}(s) = \min(\mu_j F_j^N(s),\ \tau_j^{max}/R_j)$ — inherits the internal-motion dependence through $F_j^N(s)$.

### 3.4 Required wrench → $F_{req}(s), \tau_{req}(s)$ **(revised)**

The drivebase must now react against **both** the commanded rigid-body motion **and** the momentum change of internally-moving masses (e.g. an extending arm's reaction force is transmitted to the chassis, and from the chassis to the ground through the wheels). Using Newton's second law and $\tau=dL/dt$ for the *whole* robot about $r_{cm}$:

$$
F_{req}(s) = \sum_i m_i\,a_i(s) = \underbrace{s\,M_{tot}(a_x,a_y)}_{\text{rigid-body term}} + \underbrace{\sum_i m_i\, a_i^{int}(t)}_{\text{internal reaction force, indep. of } s}
$$

$$
\tau_{req}(s) = \sum_i (r_i - r_{cm})\times m_i a_i(s) = \underbrace{s\,I_{zz}\,\alpha_{yaw}}_{\text{rigid-body term}} + \underbrace{\sum_i (r_i-r_{cm})\times m_i\,a_i^{int}(t)}_{\text{internal reaction moment, indep. of }s}
$$

Both remain **affine in $s$** — same structure as before, just with a nonzero, generally-nonzero intercept at $s=0$. Note also that if any $r_i$ is itself moving (e.g. the arm's mass position changing), $I_{zz}=\sum_i m_i\|r_i-r_{cm}\|^2$ and even $r_{cm}$ itself are time-varying — recompute both each control cycle from current $r_i(t)$, not cached from a prior cycle.

### 3.5 Wrench allocation (SOCP) — unchanged

Same SOCP as before (Section 3.5 of prior version), using the revised $F_{req}(s), \tau_{req}(s)$:

$$
\begin{aligned}
\min_{f_j}\quad & \sum_j \left(\frac{\|f_j\|}{F_j^{max}}\right)^2 \\
\text{s.t.}\quad & \sum_j f_j = F_{req}(s), \qquad \sum_j (r_j^w-r_{cm})\times f_j = \tau_{req}(s) \\
& \|f_j\| \le F_j^{max}(s)\ \ \forall j
\end{aligned}
$$

### 3.6 Feasibility & proportional scale-back → $s_{force}$ **(revised)**

Maximize $s$ such that the equality constraints (now with the internal-reaction offset) remain satisfiable within the disk caps:

$$
\begin{aligned}
\max_{f_j,\ s}\quad & s \\
\text{s.t.}\quad & \sum_j f_j = s\,F_{req}^{rigid}(1) + F_{req}^{int}, \qquad \sum_j (r_j^w-r_{cm})\times f_j = s\,\tau_{req}^{rigid}(1) + \tau_{req}^{int} \\
& \|f_j\| \le F_j^{max}(s) \ \ \forall j, \qquad 0\le s\le 1
\end{aligned}
$$

where $F_{req}^{rigid}, \tau_{req}^{rigid}$ are the $s$-scaling (chassis-command) parts and $F_{req}^{int},\tau_{req}^{int}$ are the fixed internal-reaction offsets from Section 3.4.

**Important new failure mode:** if $F_{req}^{int}, \tau_{req}^{int}$ alone (at $s=0$) already exceed what the wheels can supply — i.e. the SOCP is infeasible even at $s=0$ — **the controller cannot achieve the wrench by scaling the chassis command at all**, since $s$ only scales the *drive-related* part of the wrench, not the internal-mechanism reaction part. This must be detected and reported distinctly (e.g. "internal mechanism motion exceeds drivebase capability" — arm/elevator must be slowed independently, outside this controller's authority) rather than folded into "$s_{force}=0$", since $s_{force}=0$ implies zero chassis command fixes it, which is false here.

### 3.7 Combine and finalize

$$
s = \min(s_{tip},\ s_{force},\ 1)
$$

with the caveat above: if either sub-problem is infeasible at $s=0$, flag as an internal-motion-induced fault rather than reporting $s=0$ as a solution, since $s=0$ does not actually resolve the constraint violation. Otherwise, recompute the full chain at $\vec u_{final}=s\,\vec u_{cmd}$ (with $a_i^{int}(t)$ unchanged — it's not scaled, since it's not part of the command being reduced) to get self-consistent $a_i, F_j^N, f_j$, then output:

$$
\theta_j = \operatorname{atan2}(f_{j,y}, f_{j,x}), \qquad \tau_j = \|f_j\|\, R_j
$$

---

## 4. Summary Table (new/changed entries marked)

---

## 5. Physical Summary of the Change

Internal mechanism motion acts like an **exogenous disturbance wrench** on the drivebase — mathematically just an offset term added to the required force/moment balance, entirely analogous to how gravity acts as a constant offset in the tipping/ZMP equations. The chassis-command scale-back $s$ can only trade off the *drive-commanded* portion of the required wrench; it has no authority over the internal-mechanism portion. This means a sufficiently aggressive arm swing or elevator stop/start can force $s_{force}=0$ or even render the wrench infeasible at *any* $s$ (Section 3.6) — a case that must be surfaced explicitly, since it represents a limit the drivebase controller cannot resolve on its own.




| Symbol | Meaning |
|---|---|
| $f_j$ | horizontal (friction/traction) force at wheel $j$ |
| $\mu_j$ | friction coefficient at wheel $j$ |
| $k_{t,j}$ | tangential (shear) stiffness at wheel $j$ |
| $\delta_j^{\,t}$ | tangential deflection at wheel $j$ |
| $\delta_0^{\,t}$ | common horizontal shear deflection (rigid-plane fit) |
| $\vec\omega_\delta$ | yaw-twist rate of the rigid tangential deflection field |


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

| Symbol | Meaning |
|---|---|
| $\vec u_{cmd} = (a_x,a_y,\alpha_{yaw})$ | commanded chassis acceleration |
| $\omega$ | current chassis yaw rate (measured state) |
| $a_i(\vec u)$ | resulting rigid-body acceleration field at each mass |
| $s\in[0,1]$ | overall command scale factor |
| $s_{tip}$ | max scale before $r_{zmp}$ exits $\mathcal P$ |
| $F_j^N(s)$ | normal force at wheel $j$ (Section 6 compliance/active-set) |
| $F_j^{max}(s)$ | per-wheel force cap, $\min(\mu_j F_j^N, \tau_j^{max}/R_j)$ |
| $F_{req}, \tau_{req}$ | required net force / yaw moment for rigid-body motion |
| $f_j$ | commanded contact force vector at wheel $j$ (magnitude + direction free, swerve) |
| $s_{force}$ | max scale achievable given wheel force caps |
| $\theta_j, \tau_j$ | output: module steer angle, drive torque |

| Symbol | Meaning |
|---|---|
| $\vec u_{cmd}=(a_x,a_y,\alpha_{yaw})$ | commanded **chassis** acceleration |
| $a_i^{rigid}(\vec u)$ | acceleration of mass $i$ from chassis rigid-body motion |
| **$a_i^{int}(t)$** | **acceleration of mass $i$ from internal mechanism motion (arm/elevator/slide), known, independent of $\vec u$** |
| $a_i(\vec u,t) = a_i^{rigid}+a_i^{int}$ | total acceleration of mass $i$ |
| $s\in[0,1]$ | chassis-command scale factor (does **not** scale $a_i^{int}$) |
| $r_{zmp}(0)$ | **ZMP at zero chassis command — may already be outside $\mathcal P$ due to internal motion alone** |
| $s_{tip}$ | max scale before $r_{zmp}(s)$ exits $\mathcal P$ |
| $F_{req}^{rigid},\tau_{req}^{rigid}$ | wrench components from commanded chassis motion (scale with $s$) |
| **$F_{req}^{int},\tau_{req}^{int}$** | **wrench components from internal-mass reaction forces (fixed, do not scale with $s$)** |
| $F_j^N(s), F_j^{max}(s)$ | per-wheel normal force / force cap (Sections 6, 3.3) |
| $s_{force}$ | max scale achievable given wheel force caps |
| $\theta_j,\tau_j$ | output steer angle / drive torque |
