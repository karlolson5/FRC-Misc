# Rigid-Body Reduction of CAD Assemblies for Mass-Model Composition

## Abstract

A swerve-drive stability/wrench-allocation controller (tipping/ZMP check, normal-force distribution, wrench allocation) requires, at each control cycle, a small set of aggregate mass properties of the robot: total mass, center of mass, yaw inertia, and the net force/torque contributed by internally moving components. This note derives how those aggregate quantities can be obtained from a CAD assembly with assigned per-part materials, for both fixed onboard mechanisms (arms, elevators) and objects the robot picks up and puts down ("game pieces"). The central result is that any rigid subassembly moving on a single known joint reduces exactly to ten scalars — mass, center-of-mass position, and an inertia tensor about that center of mass — with no dependence on how finely its geometry is otherwise represented.


## 1. Required Aggregate Quantities

The controller's stability and wrench-allocation stages depend on sums of the form $\sum_i m_i r_i$, $\sum_i m_i a_i$, and $\sum_i (r_i - r_{cm})\times m_i a_i$ taken over the robot's mass distribution, together with the vertical-force sums entering the ZMP calculation. These sums fall into two classes:

- **Force-type sums** ($M_{tot}$, $r_{cm}$, $F_{req}$, $W_{eff}$) depend on a rigid subassembly only through its total mass and center-of-mass acceleration.
- **Torque/moment-type sums** ($\tau_{req}$, the ZMP numerators) additionally depend on the internal mass distribution whenever the subassembly rotates relative to the frame the moment is taken about.

Section 2 establishes the exact reduction underlying both classes.


## 2. Exact Reduction of a Rigid Subassembly

Let a rigid subassembly (chassis, one elevator stage, one arm link, a held game piece, etc.) consist of point masses $m_i$ at positions $r_i$, moving rigidly: rotating with angular velocity $\vec\omega$ and angular acceleration $\vec\alpha$ about a pivot $Q$, itself accelerating at $a_Q$. Each point's acceleration is

$$
a_i = a_Q + \vec\alpha\times r_i'' + \vec\omega\times(\vec\omega\times r_i''), \qquad r_i'' = r_i - Q.
$$

**Force sum.** Using $\sum_i m_i r_i'' = m\,r_{cm}''$,

$$
\sum_i m_i a_i = m\,a_Q + \vec\alpha\times(m\,r_{cm}'') + \vec\omega\times(\vec\omega\times m\,r_{cm}'') = m\,a_{cm}.
$$

This holds for any rigid motion — translation, rotation, or both — using only total mass and center-of-mass position.

**Torque sum about a reference point $P$.** Writing $r_i - P = r_i'' + (Q-P)$,

$$
\sum_i (r_i-P)\times m_i a_i = (r_{cm}-P)\times m\,a_{cm} \;+\; \sum_i r_i''\times m_i\big[\vec\alpha\times r_i'' + \vec\omega\times(\vec\omega\times r_i'')\big].
$$

The second term reduces to $I_Q\vec\alpha$ (plus a gyroscopic $\vec\omega\times(I_Q\vec\omega)$ term, which vanishes for the moment component along a fixed rotation axis — the standard case for a single-axis revolute joint), where $I_Q$ is the subassembly's inertia tensor about the pivot, equivalently $I_Q = I_{cm} + m[\|r_{cm}''\|^2\mathbb 1 - r_{cm}''r_{cm}''^{\,T}]$.

**Consequence.** A subassembly undergoing pure translation (no rotational DOF of its own — e.g. an elevator carriage) contributes to every controller sum exactly as a point mass at its COM. A subassembly with its own rotational joint (e.g. an arm) additionally requires its inertia tensor about its own COM or pivot. No finer description of its mass distribution is required in either case.


## 3. CAD-Derived Object: A Kinematic Tree of Rigid Links

The CAD assembly is converted into a tree with one node per rigid subassembly, matching the joints already instrumented for sensing:

- **Root**: the chassis, comprising everything rigidly fixed to the drivebase.
- **One child link per actuated mechanism**: one per elevator stage (prismatic joint), one per arm segment (revolute joint), etc.

For each link $k$, the following are extracted once, offline, from the CAD tool's mass-properties calculation over the assigned materials:

$$
m_k, \qquad c_k\ (\text{COM in the link's body-fixed frame}), \qquad I_k\ (\text{inertia tensor about } c_k,\ \text{body frame}).
$$

Each link's body frame is defined with its origin on the joint axis, so that forward-kinematic composition (Section 4) requires only a rotation/translation with no additional offset bookkeeping. These constants are re-extracted only when the corresponding CAD geometry or material assignment changes; they do not depend on the robot's current configuration.


## 4. Runtime Composition

Given each link's fixed $(m_k, c_k, I_k)$ and its current joint state, forward kinematics yields the link's current COM position $r_k(t)$ in the chassis frame and its current world-oriented inertia tensor $I_k^{world}(t) = R_k(t)\,I_k\,R_k(t)^T$. The whole-robot quantities compose as:

$$
M_{tot} = \sum_k m_k, \qquad r_{cm}(t) = \frac{1}{M_{tot}}\sum_k m_k\,r_k(t),
$$

$$
I_{zz}(t) = \sum_k\Big[I_{k,zz}^{world}(t) + m_k\big(\Delta x_k(t)^2 + \Delta y_k(t)^2\big)\Big], \qquad \Delta r_k(t) = r_k(t) - r_{cm}(t),
$$

$$
F_{req}^{int}(t) = \sum_k m_k\, a_k^{int}(t),
$$

$$
\tau_{req}^{int}(t) = \sum_k\Big[\Delta r_k(t)\times m_k\, a_k^{int}(t) + I_{k,\text{axis}}(t)\,\alpha_k^{int}(t)\Big],
$$

with the second term in $\tau_{req}^{int}$ vanishing identically for links that only translate. The ZMP horizontal-moment sums compose analogously, drawing on the off-axis components of $I_k^{world}(t)$ for links whose rotation axis is not vertical. All of this is $O(K)$ work per control cycle, $K$ being the number of rigid links.


## 5. Game Pieces as Dynamically Attached Links

Objects the robot picks up and places — game pieces — are handled by the same construction. Each game-piece type has its own CAD model with assigned material, and therefore its own fixed $(m_p, c_p, I_p)$ obtained exactly as in Section 3. While held, a game piece behaves as an additional rigid link attached at a known point on whichever link constitutes the end effector, connected by a fixed (non-actuated) joint if the grip is rigid, or by a small additional measured DOF if the piece has known play or compliant motion within the end effector. Its position, velocity, and acceleration then follow directly from the end effector's own kinematics plus this fixed offset, exactly as for any other link in Section 4.

The distinguishing feature of a game piece relative to an onboard mechanism link is that its presence in the tree is discrete rather than continuous: it is inserted as a leaf node when picked up and removed when released, rather than persisting through all configurations. This state (held / not held, and which piece type if the robot handles more than one) is tracked the same way joint state is tracked for any other mechanism. $M_{tot}$, $r_{cm}(t)$, and $I_{zz}(t)$ change discretely at pickup/release events, but this requires no special-case handling beyond recomputing the Section 4 sums over whichever links are currently instantiated — a game piece contributes to $F_{req}^{int}$ and $\tau_{req}^{int}$ while attached and contributes nothing once released.

Where a game piece's mass is significant relative to the robot's own moving components, its own inertia tensor $I_p$ matters for $\tau_{req}^{int}$ to the same extent as any other rotating link — e.g. an arm carrying a heavy piece at extension has a materially different effective inertia about the pivot than the same arm empty, captured automatically by the parallel-axis composition in Section 4 once the piece's node is present in the tree.


## 6. Parts Without a Full CAD/Material Definition

For components lacking a density-accurate CAD model — off-the-shelf hardware, cabling, fasteners modeled as stand-ins — the same $(m,c,I)$ triple is obtained by approximating the part with one or a few standard primitives (box, cylinder, or, for irregular shapes, a small number of tetrahedra), using closed-form inertia formulas and the parallel-axis theorem to combine them. This produces a link of the same form used throughout Sections 3–5, for parts where CAD does not already supply it directly.

Components with genuinely unsensed internal motion (cable chains, belts) are not exactly captured by the rigid-link formulation, since their configuration is not tied to a tracked joint; these are lumped into the nearest rigid link or represented as a fixed point mass at an approximate mean position.


## 7. Summary

| Object | Data required | Basis |
|---|---|---|
| Chassis | $(m,c,I)$ | CAD mass properties |
| Onboard moving link (arm, elevator stage) | $(m,c,I)$ per link + joint kinematics | CAD mass properties + existing joint sensing |
| Game piece (held) | $(m_p,c_p,I_p)$ per piece type + attachment offset | CAD mass properties + held/released state |
| Part without CAD/material data | $(m,c,I)$ approximated via primitives | Closed-form primitive formulas + parallel-axis theorem |

In all cases the runtime representation is the same ten-number rigid-body summary per link, composed each control cycle via forward kinematics and the parallel-axis theorem.