# Stability-Aware Swerve Controller — Phoenix 6 `ModuleRequest` Implementation

This sketches a custom `SwerveRequest` that runs the tipping/ZMP → normal-force
→ wrench-allocation pipeline from `tipping.md` (Part III, §14–15) each cycle,
using the `MassModel` built from `CAD-to-mass-model.md`, and drives each
module by **force** through
[`SwerveModule.ModuleRequest`](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/swerve/SwerveModule.ModuleRequest.html)'s
built-in `WheelForceFeedforwardX`/`WheelForceFeedforwardY` fields (robot-centric,
newtons) — no need to bypass `ModuleRequest` after all: it already exposes a
per-axis force feedforward alongside `State`, which is exactly what the
allocation solves for. `State` still carries the kinematic speed/angle
setpoint (closed-loop tracking); the feedforward force just gets summed in on
top of it by the drive motor's control loop, the same way it's meant to be
used for trajectory-following dynamics feedforward. It plugs into Phoenix 6's
`SwerveDrivetrain` the normal way: build it, pass it to
`drivetrain.setControl(...)`.

The wrench allocation (tipping.md §15.5–15.6) is solved with a real convex
solver rather than a least-norm/bisection approximation — this sketch uses
[ojAlgo](https://www.ojalgo.org/) (pure Java, on Maven Central, runs fine on
the RoboRIO), whose `ExpressionsBasedModel` handles the linear equality
constraints and the per-wheel quadratic disk (SOCP) constraints directly, and
minimizes the same utilization-fraction objective as tipping.md §15.5. If
your team already vendors an OSQP/ECOS JNI binding, swap that in instead —
nothing else in this file depends on which backend solves the QCQP, only that
it satisfies the same constraints.

---

## 1. Class shape

```java
public class StabilityAwareRequest implements SwerveRequest {
    // driver/autonomous input
    public ChassisSpeeds DesiredSpeeds = new ChassisSpeeds();       // vx, vy, omega (robot-centric)
    public ChassisSpeeds DesiredAccel  = new ChassisSpeeds();       // ax, ay, alphaYaw — feedforward accel
                                                                      // (from a trajectory, or a simple
                                                                      // (v_now - v_prev)/dt if none available)

    // static robot constants (tipping.md §1, §13; CAD-to-mass-model.md §3)
    private final RigidBody chassisBody;
    private final WheelGeometry wheels;          // §2 below: positions, mu, k, tauMax, wheelRadius
    private final Supplier<List<RigidBodyState>> mechanismBodies; // arm/elevator/etc. states, this cycle
    private final MassModel massModel = new MassModel();

    public StatusCode apply(SwerveDrivetrain.SwerveControlParameters parameters,
                             SwerveModule<?, ?, ?>... modulesToApply) {
        // ... §3 below ...
    }
}
```

`mechanismBodies` is exactly the `List<RigidBodyState>` supplier described in
`CAD-to-mass-model.md` §4/§5 — whatever combination of arm/elevator/held-piece
`RigidBodyState`s from `Rigid-Body-Mass-Java-Example.md` applies to this
robot, recomputed fresh each call since joint angles and extensions change
every cycle.

---

## 2. Static geometry constants

```java
public record WheelGeometry(
    Translation2d[] positions,  // r_j^w, chassis frame — same order as modulesToApply
    double[] mu,                // per-wheel friction coefficient
    double[] stiffnessK,        // per-wheel suspension stiffness, tipping.md §6(a) —
                                 // NOT currently used: normalForceComplianceOnce(...) implements
                                 // only the equal-stiffness fit (tipping.md §6(a) assumes k_j = k
                                 // for all j and cancels k out of the plane-fit solve entirely).
                                 // A general per-wheel-stiffness compliance model is a different,
                                 // undreived formula — tipping.md doesn't give it. If your wheels
                                 // have meaningfully different suspension stiffness, that
                                 // asymmetry is silently ignored right now, not approximated.
    double[] tauMaxNm,          // per-wheel max drive torque
    double wheelRadiusM
) {}
```

`positions` should match `parameters.moduleLocations` order (front-left,
front-right, back-left, back-right, per CTRE's convention) — assert this once
at construction rather than re-deriving it every cycle.

---

## 3. `apply()` — the pipeline

### 3.1 Aggregate mass properties (`CAD-to-mass-model.md` §4)

```java
List<RigidBodyState> bodies = new ArrayList<>();
bodies.add(new RigidBodyState(
    chassisBody,
    new Pose3d(parameters.currentPose),
    Translation3d.kZero,
    VecBuilder.fill(0, 0, 0),
    VecBuilder.fill(0, 0, 0)
));
bodies.addAll(mechanismBodies.get());

double mTot   = massModel.totalMass(bodies);
Translation3d rcm = massModel.centerOfMass(bodies, mTot);
double izz    = massModel.yawInertia(bodies, rcm);
Translation3d fReqInt = massModel.internalReactionForce(bodies);   // F_req^int, tipping.md §15.4
double tauReqInt = massModel.internalReactionYawTorque(bodies, rcm); // tau_req^int
double wEff   = massModel.effectiveWeight(bodies, 9.81);           // W_eff, tipping.md §5
```

### 3.2 Zero moment point and `s_tip` (tipping.md §5, §15.1)

`MassModel` gives `W_eff`; the ZMP itself needs the extra numerator sums from
tipping.md §5. Small addition alongside `MassModel`:

```java
/**
 * tipping.md §5. Returns empty when sum_i N_iz <= epsilon (all wheels
 * effectively unloaded — free-fall, being launched, or a commanded
 * acceleration that would drive net vertical reaction to zero/negative).
 * tipping.md §5 explicitly calls this case ill-defined for r_zmp, not an
 * edge case to silently push through the division.
 */
Optional<Translation2d> zeroMomentPoint(List<RigidBodyState> bodies, double g,
        double s, double ax, double ay, double alphaYawChassis,
        double omegaYawChassis, Translation3d pivot) {
    double sumXNiz = 0, sumYNiz = 0, sumNiz = 0;
    for (RigidBodyState b : bodies) {
        Translation3d com = b.comWorld();
        Translation3d a = b.totalAcceleration(s, ax, ay, alphaYawChassis, omegaYawChassis, pivot);
        double niz = b.body.massKg * (-g - a.getZ());
        sumXNiz += com.getX()*niz - com.getZ()*b.body.massKg*a.getX();
        sumYNiz += com.getY()*niz - com.getZ()*b.body.massKg*a.getY();
        sumNiz  += niz;
    }

    final double EPS = 1e-3; // ~newtons; small compared to any real robot's weight
    if (sumNiz <= EPS) {
        return Optional.empty();
    }
    return Optional.of(new Translation2d(sumXNiz / sumNiz, sumYNiz / sumNiz));
}
}
```

Evaluate this once at the *current* command (`s = 0`, i.e. with only
`fReqInt`/mechanism accelerations, no chassis command) and once at `s = 1`
(full `DesiredAccel` applied), then ray–polygon intersect against the wheel
positions to get `s_tip`, exactly as tipping.md §15.1 describes — a closed-form
segment/edge test against `wheels.positions()`, no iteration needed. Shrink
the polygon inward by a safety margin before testing.

### 3.3 Normal-force distribution (tipping.md §6(a)/(a′))

```java
double[] fNormal = normalForceCompliance(bodies, wheels, 9.81, 1.0, ax, ay,
    alphaYawChassis, omegaYawChassis, rcm, zmpAtFull);

/**
 * tipping.md §6(a): per-wheel normal force via the equal-stiffness rigid-plane
 * compliance fit, evaluated at total acceleration a_i(s) (internal + chassis-rigid,
 * gotcha #2's totalAcceleration(...)), NOT reusing any intermediate from zeroMomentPoint —
 * the moments here are about the wheel centroid, not the origin/rcm.
 *
 * Returns raw (possibly negative) F_j^N; §3.3's active-set correction wraps this.
 */
double[] normalForceComplianceOnce(List<RigidBodyState> bodies, WheelGeometry wheels,
        Translation2d[] activeWheels, double g, double s, double ax, double ay,
        double alphaYawChassis, double omegaYawChassis, Translation3d pivot) {

    int n = activeWheels.length;

    // Wheel centroid (x̄, ȳ) — over the *active* wheel set only (matters once
    // the active-set loop below starts dropping wheels).
    double xbar = 0, ybar = 0;
    for (Translation2d w : activeWheels) { xbar += w.getX(); ybar += w.getY(); }
    xbar /= n; ybar /= n;

    // Second moments about the centroid, tipping.md §6(a).
    double Ixx = 0, Iyy = 0, Ixy = 0;
    for (Translation2d w : activeWheels) {
        double xp = w.getX() - xbar, yp = w.getY() - ybar;
        Ixx += yp * yp;
        Iyy += xp * xp;
        Ixy += xp * yp;
    }

    // M_x', M_y': same construction as zeroMomentPoint's tau_x/tau_y, but about
    // the wheel centroid instead of the origin — recomputed independently here,
    // not pulled from zeroMomentPoint(...).
    double Mx = 0, My = 0, wEff = 0;
    for (RigidBodyState b : bodies) {
        Translation3d com = b.comWorld();
        Translation3d a = b.totalAcceleration(s, ax, ay, alphaYawChassis, omegaYawChassis, pivot);
        double Nix = -b.body.massKg * a.getX();
        double Niy = -b.body.massKg * a.getY();
        double Niz =  b.body.massKg * (-g - a.getZ());
        Mx += (com.getY() - ybar) * Niz - com.getZ() * Niy;
        My += com.getZ() * Nix - (com.getX() - xbar) * Niz;
        wEff += Niz;
    }

    if (wEff <= 1e-6) {
        // tipping.md §5's degenerate case (free-fall / unloaded) — same guard
        // gotcha #4 calls for in zeroMomentPoint, applies identically here.
        double[] zero = new double[n];
        Arrays.fill(zero, 0.0);
        return zero;
    }

    // Solve [Iyy Ixy; Ixy Ixx] [alpha; beta] = [My; -Mx]  (tipping.md §6(a)).
    double det = Iyy * Ixx - Ixy * Ixy;
    double alpha = (My * Ixx - (-Mx) * Ixy) / det;
    double beta  = ((-Mx) * Iyy - My * Ixy) / det;

    double[] fN = new double[n];
    for (int j = 0; j < n; j++) {
        double xp = activeWheels[j].getX() - xbar, yp = activeWheels[j].getY() - ybar;
        fN[j] = wEff / n + alpha * xp + beta * yp;
    }
    return fN;
}

/**
 * tipping.md §6(a′): active-set iteration — drop any wheel with F_j^N < 0,
 * recompute centroid + second moments over the remaining set, repeat.
 * Terminates at the 3-wheel tripod/barycentric case if it gets that far.
 */
double[] normalForceCompliance(List<RigidBodyState> bodies, WheelGeometry wheels,
        double g, double s, double ax, double ay, double alphaYawChassis,
        double omegaYawChassis, Translation3d pivot, Translation2d rzmp) {

    List<Integer> active = new ArrayList<>();
    for (int j = 0; j < wheels.positions().length; j++) active.add(j);

    while (true) {
        if (active.size() == 3) {
            return tripodBarycentric(wheels, active, g, s, ax, ay, alphaYawChassis,
                                      omegaYawChassis, pivot, rzmp, bodies);
        }

        Translation2d[] activeWheels = active.stream()
            .map(j -> wheels.positions()[j]).toArray(Translation2d[]::new);
        double[] fNActive = normalForceComplianceOnce(bodies, wheels, activeWheels,
            g, s, ax, ay, alphaYawChassis, omegaYawChassis, pivot);

        int dropIdx = -1;
        for (int k = 0; k < fNActive.length; k++) {
            if (fNActive[k] < 0) { dropIdx = k; break; }
        }
        if (dropIdx == -1) {
            // All non-negative: scatter back into full-length result, 0 for dropped wheels.
            double[] fN = new double[wheels.positions().length];
            for (int k = 0; k < active.size(); k++) fN[active.get(k)] = fNActive[k];
            return fN;
        }
        active.remove(dropIdx); // wheel lifted off; redo the fit without it
    }
}

/** tipping.md §6, N_w=3 case: F_j^N = lambda_j * W_eff, barycentric weights of r_zmp in the triangle. */
double[] tripodBarycentric(WheelGeometry wheels, List<Integer> active, double g, double s,
        double ax, double ay, double alphaYawChassis, double omegaYawChassis,
        Translation3d pivot, Translation2d rzmp, List<RigidBodyState> bodies) {
    Translation2d p1 = wheels.positions()[active.get(0)];
    Translation2d p2 = wheels.positions()[active.get(1)];
    Translation2d p3 = wheels.positions()[active.get(2)];

    double detT = (p2.getY()-p3.getY())*(p1.getX()-p3.getX()) + (p3.getX()-p2.getX())*(p1.getY()-p3.getY());
    double l1 = ((p2.getY()-p3.getY())*(rzmp.getX()-p3.getX()) + (p3.getX()-p2.getX())*(rzmp.getY()-p3.getY())) / detT;
    double l2 = ((p3.getY()-p1.getY())*(rzmp.getX()-p3.getX()) + (p1.getX()-p3.getX())*(rzmp.getY()-p3.getY())) / detT;
    double l3 = 1 - l1 - l2;

    double wEff = 0;
    for (RigidBodyState b : bodies) {
        Translation3d a = b.totalAcceleration(s, ax, ay, alphaYawChassis, omegaYawChassis, pivot);
        wEff += b.body.massKg * (-g - a.getZ());
    }

    double[] fN = new double[wheels.positions().length];
    fN[active.get(0)] = l1 * wEff;
    fN[active.get(1)] = l2 * wEff;
    fN[active.get(2)] = l3 * wEff;
    return fN; // if any lambda_j < 0 here, r_zmp is outside the reduced triangle —
               // genuine tipping about an edge of the *original* full polygon (§6a′'s terminal caveat)
}

```

Evaluated once at `s = 1` per tipping.md §15.6's "treat `F_j^N` as fixed at
its `s=1` value" simplification — avoids nesting a nonlinear feasibility
solve inside the tipping/force bisection below.

### 3.4 Per-wheel force cap (tipping.md §15.3)

```java
double[] fMax = new double[4];
for (int j = 0; j < 4; j++) {
    fMax[j] = Math.min(wheels.mu(j) * fNormal[j], wheels.tauMaxNm(j) / wheels.wheelRadiusM());
}
```

### 3.5 Required wrench (tipping.md §15.4)

```java
Translation2d requiredForce(double s) {
    double ax = DesiredAccel.vxMetersPerSecond, ay = DesiredAccel.vyMetersPerSecond; // accel fields, named per ChassisSpeeds reuse
    return new Translation2d(s * mTot * ax + fReqInt.getX(), s * mTot * ay + fReqInt.getY());
}
double requiredYawTorque(double s) {
    return s * izz * DesiredAccel.omegaRadiansPerSecond + tauReqInt;
}
```

(Using `ChassisSpeeds` fields to carry acceleration units is a convenience,
not a WPILib convention — a small dedicated `record ChassisAccel(double ax,
double ay, double alphaYaw)` is cleaner if this is going in real code.)

### 3.6 Wrench allocation via ojAlgo (tipping.md §15.5–15.6)

Set up the exact problem tipping.md §15.5 describes — minimize summed
utilization `sum (||f_j|| / F_j^max)^2` subject to the two force-balance
equalities, the yaw-moment equality, and each wheel's disk cap — as an
`ExpressionsBasedModel`, and solve it directly instead of approximating.
`F_req(s)`/`tau_req(s)` are affine in `s` (§3.5), so wrap the solve in the
same outer bisection tipping.md §15.6 recommends to find `s_force`: the
largest `s` for which the QCQP stays feasible.

```java
Optimisation.Result solveAllocation(double s, Translation2d rcm) {
    ExpressionsBasedModel model = new ExpressionsBasedModel();
    Variable[] fx = new Variable[4], fy = new Variable[4];
    for (int j = 0; j < 4; j++) {
        fx[j] = model.addVariable("fx" + j);
        fy[j] = model.addVariable("fy" + j);

        // disk constraint: fx_j^2 + fy_j^2 <= fMax_j^2 (the SOCP part)
        model.addExpression("disk" + j)
             .set(fx[j], fx[j], 1).set(fy[j], fy[j], 1)
             .upper(fMax[j] * fMax[j]);

        // objective term: (fx_j^2 + fy_j^2) / fMax_j^2
        model.addExpression("cost" + j)
             .set(fx[j], fx[j], 1.0 / (fMax[j]*fMax[j]))
             .set(fy[j], fy[j], 1.0 / (fMax[j]*fMax[j]))
             .weight(1);
    }

    Expression sumFx = model.addExpression("sumFx"), sumFy = model.addExpression("sumFy"), yaw = model.addExpression("yaw");
    for (int j = 0; j < 4; j++) {
        double dx = wheels.positions(j).getX() - rcm.getX(), dy = wheels.positions(j).getY() - rcm.getY();
        sumFx.set(fx[j], 1);
        sumFy.set(fy[j], 1);
        yaw.set(fy[j], dx).set(fx[j], -dy);
    }
    sumFx.level(requiredForce(s).getX());
    sumFy.level(requiredForce(s).getY());
    yaw.level(requiredYawTorque(s));

    return model.minimise();
}
```

```java
double bisectSForce(Translation2d rcm) {
    if (solveAllocation(1.0, rcm).getState().isFeasible()) return 1.0;
    double lo = 0, hi = 1;
    for (int i = 0; i < 20; i++) {                 // ~1e-6 resolution
        double mid = (lo + hi) / 2;
        if (solveAllocation(mid, rcm).getState().isFeasible()) lo = mid; else hi = mid;
    }
    return lo;
}
```

If `solveAllocation(0.0, rcm)` is itself infeasible, that's tipping.md
§15.6's flagged failure mode: internal-mechanism reaction alone exceeds the
wheels' traction budget, and no chassis command scale-back fixes it. Surface
this as a fault (e.g. cap `DesiredAccel` to zero and set a "drivebase
overloaded" alert) rather than silently returning `s = 0`.

`ExpressionsBasedModel` construction (~20 variables/constraints) each of the
20 bisection iterations is more solver work than the earlier least-norm
sketch — see §4 for the cost note on making this cheaper if it matters.

### 3.7 Combine, finalize, build module commands

```java
double ax = DesiredAccel.vxMetersPerSecond, ay = DesiredAccel.vyMetersPerSecond;
double alphaYawChassis = DesiredAccel.omegaRadiansPerSecond; // feedforward angular accel
double omegaYawChassis = parameters.currentChassisSpeeds.omegaRadiansPerSecond; // actual, not commanded

Optional<Translation2d> zmpAtRest = zeroMomentPoint(bodies, 9.81, 0.0, ax, ay, alphaYawChassis, omegaYawChassis, rcm);
Optional<Translation2d> zmpAtFull = zeroMomentPoint(bodies, 9.81, 1.0, ax, ay, alphaYawChassis, omegaYawChassis, rcm);

double sTip;
if (zmpAtRest.isEmpty() || zmpAtFull.isEmpty()) {
    // Fail safe: either the robot's current/internal-motion state alone is
    // already in the degenerate regime (zmpAtRest empty — this is serious,
    // independent of any chassis command), or the *commanded* acceleration
    // would drive it there (zmpAtFull empty). Either way, don't trust a
    // ray-polygon test built from an ill-defined endpoint.
    sTip = 0.0;
} else {
    sTip = rayPolygonIntersect(zmpAtRest.get(), zmpAtFull.get(), shrunkSupportPolygon(wheels.positions()));
}

double s = MathUtil.clamp(Math.min(sTip, bisectSForce(rcm)), 0.0, 1.0);
Optimisation.Result result = solveAllocation(s, rcm);   // final allocation at the chosen scale
double[] f = new double[8];
for (int j = 0; j < 4; j++) {
    f[2*j]   = result.get(2*j).doubleValue();      // fx_j
    f[2*j+1] = result.get(2*j+1).doubleValue();     // fy_j
}

ChassisSpeeds scaledSpeeds = DesiredSpeeds.times(s);   // ChassisSpeeds.times(double) scales all 3 components
SwerveModuleState[] kinematicStates = parameters.kinematics.toSwerveModuleStates(scaledSpeeds);

for (int j = 0; j < modulesToApply.length; j++) {
    double fx = f[2*j], fy = f[2*j+1];
    Rotation2d steerAngle = (Math.hypot(fx, fy) > 1e-6)
        ? new Rotation2d(fx, fy)                          // traction-optimal heading from the allocation
        : kinematicStates[j].angle;                        // no meaningful force direction: hold kinematic angle

    var moduleState = new SwerveModuleState(kinematicStates[j].speedMetersPerSecond, steerAngle);

    var moduleRequest = new SwerveModule.ModuleRequest()
        .withState(moduleState)
        .withWheelForceFeedforwardX(fx)                    // robot-centric, newtons — the allocation's own output
        .withWheelForceFeedforwardY(fy)
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
        .withUpdatePeriod(parameters.updatePeriod)
        .withEnableFOC(true);

    modulesToApply[j].apply(moduleRequest);
}
return StatusCode.OK;
```

**Force is closed-loop-plus-feedforward here, not open-loop.** `State` still
carries the kinematic speed/angle target so the module keeps tracking the
commanded chassis motion via its normal velocity/position loops;
`WheelForceFeedforwardX/Y` layers the allocation's force directly on top,
exactly the intended use (CTRE's own docs describe it for trajectory-dynamics
feedforward). Two things worth being explicit about:

- If `fx`/`fy` for a wheel is near zero (no meaningful direction to solve
  for), the feedforward fields default to `0` and the module falls back to
  pure velocity tracking for that wheel — no special-casing needed there
  beyond the steer-angle fallback already in the loop above.
- `wheels.tauMaxNm(j)` and `wheels.mu(j)` already bounded `f_j` inside the
  QCQP (§3.4, §3.6), so the feedforward force handed to the module is never
  larger than what that wheel can actually deliver — the module's own
  current limits are then a second, independent backstop, not the only thing
  keeping torque in bounds.

---

## 4. Per-cycle cost

`CAD-to-mass-model.md` §4's aggregation is `O(K)` in the number of mechanism
links `K` and cheap. The real cost here is §3.6: building and solving a small
QCQP with `ExpressionsBasedModel` 20 times per bisection. If profiling shows
that's too slow for the odometry-thread period, in order of impact: cut the
bisection to ~8–10 steps (millimeter-scale resolution on `s` is already far
finer than the mechanism needs), build the model's disk/objective terms once
and only re-`.level()` the three equality expressions per bisection step
rather than re-adding variables from scratch, and skip the active-set
correction in §3.3 when all `F_j^N` come back non-negative on the first pass
(the common case).
