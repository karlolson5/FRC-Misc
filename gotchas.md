# Gotchas, Gaps, and Things to Consider

A review pass across all five documents — `tipping.md`, `CAD-to-mass-model.md`,
`Rigid-Body-Mass-Java.md`, `Rigid-Body-Mass-Java-Example.md`,
`Swerve-Controller-Implementation.md` — looking for actual bugs, unresolved
gaps, and physics caveats that are easy to lose track of once the derivation
turns into code. Organized worst-first: concrete bugs, then real gaps, then
things worth a deliberate decision, then scope limits inherited from the math
itself.

---


## 3. `normalForceCompliance(...)` is a stub, and its moment inputs aren't the same as the ZMP's

§3.3 calls `normalForceCompliance(wheels, rcm, wEff, /* moment terms from zmp
calc */)` without ever defining the function. Worth flagging explicitly: the
horizontal moments that formula needs (`M_x'`, `M_y'` in `tipping.md` §6(a))
are taken **about the wheel centroid**, not about the origin the way the ZMP
numerator sums in §3.2 are — they're related but not the same numbers, and
reusing the ZMP calc's intermediate values directly would be a mistake. This
was left as an explicit exercise in the sketch; if implementing it, re-derive
the centroid-relative moment sum separately rather than repurposing
`zeroMomentPoint`'s internals.

## 4. No guard against the degenerate ZMP case

`tipping.md` §5 calls out `sum_i N_iz <= 0` (free-fall, or a robot being
launched) as an ill-defined case requiring separate handling. The
`zeroMomentPoint` sketch divides by `sumNiz` unconditionally — worth an
explicit check (`sumNiz <= epsilon` → treat as "all wheels unloaded," skip
the polygon test, and fail safe) before this goes anywhere near a real robot.

---

## 5. Physics caveats from the source docs that are easy to lose in translation

- **A negative compliance-model `F_j^N` isn't proof of tipping**
  (`tipping.md` §6(a)/(a′), explicitly caveated there). It can be an artifact
  of the equal-stiffness rigid-plane assumption even when the ZMP is safely
  inside the polygon. The active-set correction (§3.3's second bullet) fixes
  the *force distribution*, but true instability should still be confirmed
  against the ZMP-in-polygon test independently — don't gate any safety
  behavior off the sign of a raw compliance-model output alone.
- **The gyroscopic term is dropped assuming a single fixed rotation axis**
  (`CAD-to-mass-model.md` §2: "vanishes for the moment component along a
  fixed rotation axis — the standard case for a single-axis revolute
  joint"). That assumption is fine for Examples 1–5 in
  `Rigid-Body-Mass-Java-Example.md`, but the 2-axis wrist and 6-DOF arm in
  §6 have joints whose rotation axes aren't fixed/vertical — `MassModel`'s
  `yawInertia`/`internalReactionYawTorque` only ever pull the `zz` component
  of each body's world inertia tensor and never reconstruct the dropped
  `ω×(Iω)` cross term. For a fast-moving 6-DOF arm this is a real
  approximation, not just a note in the margins — worth sanity-checking
  against a full rigid-body simulation if that arm moves quickly.
- **Everything assumes a flat, level, single-plane floor** (`tipping.md`
  §1's `z=0` ground plane). None of the five documents generalize to a
  ramp or uneven section of a field — if that's part of the game, the ZMP
  formalism as derived here doesn't directly apply and would need
  re-deriving against a tilted support plane.
- **The per-wheel force disk is a quasi-static assumption**
  (`tipping.md` §15.3: "quasi-statically... apply its net contact force in
  any direction"). If the commanded force direction from the SOCP swings
  faster than a module's steer motor can physically slew, the wheel can't
  actually deliver force in the direction the allocation assumed during that
  transient — the feasibility guarantee is only as good as the steer
  response time relative to how fast `f_j`'s direction is changing.

---

## 6. Design decisions the sketches punted on

- **Scaling velocity vs. scaling acceleration by `s`.** `tipping.md`'s whole
  derivation scales the *commanded acceleration* `u_cmd`. §3.7 of the swerve
  doc also scales `DesiredSpeeds` (the velocity setpoint) by the same `s`:
  `DesiredSpeeds.times(s)`. That's a much blunter intervention than the
  physics calls for — throttling velocity changes the steady-state target,
  not just how fast the robot gets there. Worth deciding deliberately
  whether `s` should touch the velocity setpoint at all, or only the
  acceleration feedforward (`DesiredAccel`) and the allocated
  `WheelForceFeedforwardX/Y`.
- **No steer-angle optimization/hysteresis.** The force-direction-derived
  steer angle (`new Rotation2d(fx, fy)`) is sent as-is, without calling
  `SwerveModuleState.optimize()` or anything equivalent to avoid a near-180°
  wheel flip when the allocation's preferred direction crosses over the
  module's current heading, and without any blending near the `‖f‖ ≈ 0`
  threshold where the code falls back to the kinematic angle. Both are
  likely to produce visible steer chatter without some smoothing.
- **`mu`/`stiffnessK` are static per-wheel constants** in `WheelGeometry`,
  but real friction coefficients drift with tread wear, temperature, and
  which competition carpet the robot's on. A force budget computed from
  stale constants can be silently over- or under-confident; consider whether
  slip detection or an on-the-fly `mu` estimate belongs in this pipeline
  eventually, even if the static constants are a fine starting point.
- **Units-library overload ambiguity.** `ModuleRequest`'s Javadoc notes
  `WheelForceFeedforwardX` has both a raw-`double` and a units-typed (`Force`)
  accessor/mutator pair. The sketch assumes the raw-`double`-newtons overload;
  confirm that matches how the rest of the vendored WPILib project uses (or
  doesn't use) the 2025+ Java units library before wiring this in, or it's a
  straightforward compile-time mismatch.

---

## 7. Performance/architecture concerns

- **Where this runs matters.** Phoenix 6 calls a `SwerveRequest`'s `apply()`
  from the drivetrain's odometry thread, which typically runs well above the
  main robot loop's 50 Hz. Solving a small QCQP through ojAlgo's
  `ExpressionsBasedModel`, 8–20 times per bisection, every single odometry
  cycle, is a nontrivial real-time budget ask on a RoboRIO — this may need
  to move to a slower periodic task (main loop, or its own timed thread)
  that computes `s` and the allocated forces and hands cached results to the
  odometry thread, rather than solving inline in `apply()`.
- **Redundant per-body recomputation.** `MassModel`'s `effectiveWeight`,
  `internalReactionForce`, `internalReactionYawTorque`, and the swerve doc's
  `zeroMomentPoint` each independently call `comAcceleration()` (and
  `yawInertia`/`internalReactionYawTorque` each independently call
  `inertiaWorld()`) on every `RigidBodyState`, every cycle — each of those
  involves several cross products / a 3×3 matrix multiply. Worth computing
  each body's acceleration and world inertia tensor once per cycle and
  passing the cached values into whichever aggregate sums need them, rather
  than recomputing per aggregate.
- **Telescoping-segment `RigidBody` rebuilds** (`Rigid-Body-Mass-Java-Example.md`
  §6) compound the above — a 9-DOF arm rebuilds three `RigidBody`s from
  scratch every cycle on top of the redundant aggregation work.
- **ojAlgo cold start.** First-call JIT/allocation overhead for a new solver
  library is worth warming up explicitly in `robotInit()` rather than
  discovering the hitch during the first competition match.

---

## 8. General caveats

- **Nothing here has been compiled or run.** API surfaces (`Rotation3d.toMatrix()`,
  `Matrix.eye()`, `ModuleRequest.withWheelForceFeedforwardX/Y`,
  `SwerveControlParameters.moduleLocations`, ojAlgo's `ExpressionsBasedModel`
  API) were checked against specific documentation snapshots during this
  conversation, but WPILib and Phoenix 6 both change method signatures
  between seasonal releases — verify against whatever vendordep version is
  actually pinned in the project before treating any of this as final.
- **CAD inertia-tensor sign convention.** `RigidBody.fromCad(...)` assumes
  its `ixy`/`ixz`/`iyz` arguments are products of inertia and negates them
  internally to build the tensor. Not every CAD tool reports products of
  inertia the same way (some report the tensor's off-diagonal entries
  directly, already negated) — a sign mismatch here is silent and would only
  show up as a subtly wrong tipping/force prediction, not a crash. Worth
  double-checking against your specific CAD export before trusting the
  numbers.
- **A held game piece changes `M_tot`/`r_cm`/`I_zz` discretely** at
  pickup/release (`CAD-to-mass-model.md` §5, by design). None of the docs
  discuss smoothing that transition — a one-cycle jump in the mass model
  could show up as a one-cycle jump in the allocated wheel forces. Probably
  fine in practice given control loop rates, but worth a quick check rather
  than an assumption.
