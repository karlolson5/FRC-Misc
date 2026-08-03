# Gotchas, Gaps, and Things to Consider

A review pass across all five documents — `tipping.md`, `CAD-to-mass-model.md`,
`Rigid-Body-Mass-Java.md`, `Rigid-Body-Mass-Java-Example.md`,
`Swerve-Controller-Implementation.md` — looking for actual bugs, unresolved
gaps, and physics caveats that are easy to lose track of once the derivation
turns into code. Organized worst-first: concrete bugs, then real gaps, then
things worth a deliberate decision, then scope limits inherited from the math
itself.

---

## 5. Physics caveats from the source docs that are easy to lose in translation

- **A negative compliance-model `F_j^N` isn't proof of tipping**
  (`tipping.md` §6(a)/(a′), explicitly caveated there). It can be an artifact
  of the equal-stiffness rigid-plane assumption even when the ZMP is safely
  inside the polygon. The active-set correction (§3.3's second bullet) fixes
  the *force distribution*, but true instability should still be confirmed
  against the ZMP-in-polygon test independently — don't gate any safety
  behavior off the sign of a raw compliance-model output alone.
  - Fix: any safety-critical decision (e-stop, forced brake, driver alert)
    should require **both** signals — `r_zmp ∉ P` from `zeroMomentPoint()`
    *and* a negative post-active-set `F_j^N` — rather than either alone.
    Since both are already computed in the pipeline (§3.2, §3.3), this is a
    matter of `AND`-ing the two booleans at the call site, not new math.
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
  - Fix: for those specific links, carry the *full* 3D `ω`/`α` (not just a
    yaw scalar) through `RigidBodyState`, compute the full moment
    `I_Q α + ω×(I_Q ω)` in `inertiaWorld()`'s caller, and add the extra
    off-axis term into `τ_req^int` for those links only — the rest of the
    tree (single-axis joints) keeps the current simplified path since the
    term is exactly zero there, so this only needs to be conditional on
    which links actually have non-fixed axes.
- **Everything assumes a flat, level, single-plane floor** (`tipping.md`
  §1's `z=0` ground plane). None of the five documents generalize to a
  ramp or uneven section of a field — if that's part of the game, the ZMP
  formalism as derived here doesn't directly apply and would need
  re-deriving against a tilted support plane.
  - Fix: re-derive §3–§6 with `P` restricted to a general plane
    `n̂·r = d` instead of `z=0` — the moment-balance/ZMP algebra carries
    over with `n̂` replacing `ẑ` throughout, but the "purely horizontal"
    simplifications in §3 no longer fall out for free and would need
    redoing term-by-term. Not a small patch; treat a ramp/uneven-field
    requirement as a separate derivation pass, not a runtime flag.
- **The per-wheel force disk is a quasi-static assumption**
  (`tipping.md` §15.3: "quasi-statically... apply its net contact force in
  any direction"). If the commanded force direction from the SOCP swings
  faster than a module's steer motor can physically slew, the wheel can't
  actually deliver force in the direction the allocation assumed during that
  transient — the feasibility guarantee is only as good as the steer
  response time relative to how fast `f_j`'s direction is changing.
  - Fix: no exact fix without a dynamic steer model in the loop, but a
    cheap mitigation is bounding how fast `solveAllocation`'s output
    direction is allowed to move cycle-to-cycle (a slew-rate clamp on
    `atan2(fy,fx)` per wheel before it's sent to `ModuleRequest`), sized to
    the module's known max steer slew rate — trades a small amount of
    allocation optimality for a guarantee the commanded direction stays
    physically achievable.

---

## 6. Design decisions the sketches punted on

- **No steer-angle optimization/hysteresis.** The force-direction-derived
  steer angle (`new Rotation2d(fx, fy)`) is sent as-is, without calling
  `SwerveModuleState.optimize()` or anything equivalent to avoid a near-180°
  wheel flip when the allocation's preferred direction crosses over the
  module's current heading, and without any blending near the `‖f‖ ≈ 0`
  threshold where the code falls back to the kinematic angle. Both are
  likely to produce visible steer chatter without some smoothing.
  - Fix: run the computed `moduleState` through the standard
    `optimize()`-equivalent (flip 180° + reverse drive speed when that's
    the shorter turn) before building `ModuleRequest`, and near `‖f‖ ≈ 0`
    blend the steer target between the last commanded angle and the
    kinematic fallback (e.g. a simple lerp keyed on `‖f‖` over a small
    threshold band) instead of a hard cutover.
- **`mu`/`stiffnessK` are static per-wheel constants** in `WheelGeometry`,
  but real friction coefficients drift with tread wear, temperature, and
  which competition carpet the robot's on. A force budget computed from
  stale constants can be silently over- or under-confident; consider whether
  slip detection or an on-the-fly `mu` estimate belongs in this pipeline
  eventually, even if the static constants are a fine starting point.
  - Fix (later, not launch-blocking): track commanded vs. achieved wheel
    velocity per module; when the discrepancy exceeds a threshold under
    non-negligible commanded force, treat that as a slip event and shrink
    that wheel's `mu` estimate for some decay window rather than trusting
    the static constant. Keep the static value as the default/floor.
- **Units-library overload ambiguity.** `ModuleRequest`'s Javadoc notes
  `WheelForceFeedforwardX` has both a raw-`double` and a units-typed (`Force`)
  accessor/mutator pair. The sketch assumes the raw-`double`-newtons overload;
  confirm that matches how the rest of the vendored WPILib project uses (or
  doesn't use) the 2025+ Java units library before wiring this in, or it's a
  straightforward compile-time mismatch.
  - Fix: no code change needed until confirmed either way — just check the
    project's vendordep version and existing `ModuleRequest` call sites for
    which overload is already in use, and match it consistently at every
    `.withWheelForceFeedforwardX/Y` call this pipeline makes.

---

## 7. Performance/architecture concerns

- **Redundant per-body recomputation.** `MassModel`'s `effectiveWeight`,
  `internalReactionForce`, `internalReactionYawTorque`, and the swerve doc's
  `zeroMomentPoint` each independently call `comAcceleration()` (and
  `yawInertia`/`internalReactionYawTorque` each independently call
  `inertiaWorld()`) on every `RigidBodyState`, every cycle — each of those
  involves several cross products / a 3×3 matrix multiply. Worth computing
  each body's acceleration and world inertia tensor once per cycle and
  passing the cached values into whichever aggregate sums need them, rather
  than recomputing per aggregate.
  - Fix: introduce a small per-cycle `EvaluatedBody` wrapper (`RigidBodyState`
    + precomputed `comWorld`, `totalAcceleration(s=1)`, `inertiaWorld`)
    built once per body at the top of `apply()`, and have `MassModel`'s
    methods and `zeroMomentPoint`/`normalForceCompliance` all take
    `List<EvaluatedBody>` instead of re-deriving these from `RigidBodyState`
    each time they're called.
- **Telescoping-segment `RigidBody` rebuilds** (`Rigid-Body-Mass-Java-Example.md`
  §6) compound the above — a 9-DOF arm rebuilds three `RigidBody`s from
  scratch every cycle on top of the redundant aggregation work.
  - Fix: since `(m, c, I)` for a telescoping segment is an analytic
    function of extension length alone (not of the full joint state),
    precompute that function's coefficients once offline and evaluate it
    cheaply per cycle, or cache the last `RigidBody` and only rebuild when
    extension has changed by more than a small tolerance since the last
    cycle, rather than unconditionally rebuilding every call.
- **ojAlgo cold start.** First-call JIT/allocation overhead for a new solver
  library is worth warming up explicitly in `robotInit()` rather than
  discovering the hitch during the first competition match.
  - Fix: in `robotInit()`, run one or two throwaway `solveAllocation(...)`
    calls against dummy/zeroed inputs before the match starts, discarding
    the result — forces JIT warm-up and allocation of ojAlgo's internal
    structures on a schedule that doesn't matter, instead of on the first
    real odometry cycle.

---

## 8. General caveats

- **Nothing here has been compiled or run.** API surfaces (`Rotation3d.toMatrix()`,
  `Matrix.eye()`, `ModuleRequest.withWheelForceFeedforwardX/Y`,
  `SwerveControlParameters.moduleLocations`, ojAlgo's `ExpressionsBasedModel`
  API) were checked against specific documentation snapshots during this
  conversation, but WPILib and Phoenix 6 both change method signatures
  between seasonal releases — verify against whatever vendordep version is
  actually pinned in the project before treating any of this as final.
  - Fix: before relying on any of this, do an actual compile against the
    project's pinned WPILib/Phoenix 6/ojAlgo versions, and add a couple of
    basic unit tests (e.g. a known symmetric 4-wheel robot at rest should
    give equal `F_j^N` on all wheels, a known box's inertia tensor should
    match the closed-form formula) as a cheap regression net rather than
    trusting the derivation by inspection alone.
- **CAD inertia-tensor sign convention.** `RigidBody.fromCad(...)` assumes
  its `ixy`/`ixz`/`iyz` arguments are products of inertia and negates them
  internally to build the tensor. Not every CAD tool reports products of
  inertia the same way (some report the tensor's off-diagonal entries
  directly, already negated) — a sign mismatch here is silent and would only
  show up as a subtly wrong tipping/force prediction, not a crash. Worth
  double-checking against your specific CAD export before trusting the
  numbers.
  - Fix: build one `RigidBody.fromCad(...)` test case from a CAD part with
    a known, hand-computable inertia tensor (e.g. an off-axis rectangular
    block, computable by hand or by a trusted closed-form formula), compare
    against what the CAD tool actually exports, and confirm the sign
    convention matches before trusting `fromCad(...)` on real mechanism
    parts — a one-time check per CAD tool version, not a runtime concern.
- **A held game piece changes `M_tot`/`r_cm`/`I_zz` discretely** at
  pickup/release (`CAD-to-mass-model.md` §5, by design). None of the docs
  discuss smoothing that transition — a one-cycle jump in the mass model
  could show up as a one-cycle jump in the allocated wheel forces. Probably
  fine in practice given control loop rates, but worth a quick check rather
  than an assumption.
  - Fix (only if it proves visible in practice): ramp the held piece's
    effective mass fraction from 0→1 (or 1→0) linearly over a handful of
    cycles around the pickup/release event, rather than inserting/removing
    the full-mass node instantaneously in `activeBodies()` — cheap to add
    if needed, but don't add it preemptively without first confirming the
    one-cycle jump is actually observable at the robot's control rate.