# Rigid-Body Mass Model — Worked Examples

Five mechanisms, all rigidly mounted to the same drivebase chassis, built from
the `RigidBody` / `RigidBodyState` / `MassModel` classes in
`Rigid-Body-Mass-Java.md`. Each example just shows (a) the static
`RigidBody` definitions and (b) the per-cycle `RigidBodyState` construction —
i.e. how known joint angles/heights turn into `worldPose`, `baseAccel`,
`omega`, `alpha`. All positions are in the chassis frame; feed the chassis's
own `RigidBodyState` (its `RigidBody` at `worldPose = chassisPose`) in alongside
these for the full `List<RigidBodyState>` passed to `MassModel`.

Boilerplate shared by every example (mount point in the chassis frame, known
from CAD):

```java
Pose3d chassisPose;              // from odometry, updated each cycle
Translation3d chassisAccel;      // a_x, a_y, 0 — commanded/measured chassis accel
Pose3d mountPose = chassisPose.plus(mountOffset); // mountOffset: fixed Transform3d from CAD
```

---

## 1. Single-pivot arm

One revolute joint at a fixed point on the chassis, rotating about the Y axis
(pitch), angle `theta` measured from horizontal.

```java
RigidBody arm = RigidBody.fromCad(armMassKg, armComBody, ixx, iyy, izz, 0, 0, 0);

RigidBodyState armState(double theta, double thetaDot, double thetaDdot) {
    Pose3d pose = mountPose.plus(new Transform3d(Translation3d.kZero, new Rotation3d(0, -theta, 0)));
    Vector<N3> omega = VecBuilder.fill(0, thetaDot, 0);
    Vector<N3> alpha = VecBuilder.fill(0, thetaDdot, 0);
    Translation3d baseAccel = chassisAccel; // base is rigidly fixed to chassis
    return new RigidBodyState(arm, pose, baseAccel, omega, alpha);
}
```

`armComBody` is the arm's COM in its own body frame, with the frame origin
on the base axis — typically `new Translation3d(armLength / 2, 0, 0)` for a
uniform arm, straight from CAD otherwise.

---

## 2. Single-stage elevator

One prismatic joint, moving in Z only. No rotation, so `omega`/`alpha` are
zero and the whole body contributes no torque term beyond force x lever arm.

```java
RigidBody carriage = RigidBody.fromCad(carriageMassKg, carriageComBody, ixx, iyy, izz, 0, 0, 0);

RigidBodyState elevatorState(double heightM, double heightDot, double heightDdot) {
    Pose3d pose = mountPose.plus(new Transform3d(0, 0, heightM, Rotation3d.kZero));
    Translation3d baseAccel = chassisAccel.plus(new Translation3d(0, 0, heightDdot));
    return new RigidBodyState(carriage, pose, baseAccel, VecBuilder.fill(0,0,0), VecBuilder.fill(0,0,0));
}
```

---

## 3. Three-stage (cascade) elevator

Three rigid stages, each a separate `RigidBody`/`RigidBodyState`.

```java
RigidBody stage1 = RigidBody.fromCad(m1, com1, ixx1, iyy1, izz1, 0, 0, 0);
RigidBody stage2 = RigidBody.fromCad(m2, com2, ixx2, iyy2, izz2, 0, 0, 0);
RigidBody carriage3 = RigidBody.fromCad(m3, com3, ixx3, iyy3, izz3, 0, 0, 0);

List<RigidBodyState> elevatorStates(double h1, double h2, double h3,
                                    double v1, double v2, double v3,
                                    double a1, double a2, double a3) {

    RigidBodyState s1 = new RigidBodyState(stage1,
        mountPose.plus(new Transform3d(0, 0, h1, Rotation3d.kZero)),
        chassisAccel.plus(new Translation3d(0, 0, a1)), VecBuilder.fill(0,0,0), VecBuilder.fill(0,0,0));
    RigidBodyState s2 = new RigidBodyState(stage2,
        mountPose.plus(new Transform3d(0, 0, h2, Rotation3d.kZero)),
        chassisAccel.plus(new Translation3d(0, 0, a2)), VecBuilder.fill(0,0,0), VecBuilder.fill(0,0,0));
    RigidBodyState s3 = new RigidBodyState(carriage3,
        mountPose.plus(new Transform3d(0, 0, h3, Rotation3d.kZero)),
        chassisAccel.plus(new Translation3d(0, 0, a3)), VecBuilder.fill(0,0,0), VecBuilder.fill(0,0,0));

    return List.of(s1, s2, s3);
}
```

Each stage's `baseAccel` uses its *own* `a_k`, not the top carriage's —
`MassModel` sums every body independently, so the intermediate stages'
mass/inertia count exactly once each rather than being folded into the top
stage.

---

## 4. Two-stage elevator with an arm on top

Elevator as in Example 3 (here simplified to two stages), plus an arm whose
base rides on the top carriage. The arm's `mountPose` is therefore the
*carriage's* pose, not the chassis's — composition chains through the
elevator's current state.

```java
RigidBody stage1 = RigidBody.fromCad(m1, com1, ixx1, iyy1, izz1, 0, 0, 0);
RigidBody carriage2 = RigidBody.fromCad(m2, com2, ixx2, iyy2, izz2, 0, 0, 0);
RigidBody arm = RigidBody.fromCad(armMassKg, armComBody, ixxA, iyyA, izzA, 0, 0, 0);

List<RigidBodyState> elevatorArmStates(double h1, double h2,
                                       double v1, double v2,
                                       double a1, double a2,
                                       double theta, double thetaDot, double thetaDdot) {

    Pose3d carriagePose = mountPose.plus(new Transform3d(0, 0, h2, Rotation3d.kZero));
    Translation3d carriageAccel = chassisAccel.plus(new Translation3d(0, 0, a2));

    RigidBodyState s1 = new RigidBodyState(stage1,
        mountPose.plus(new Transform3d(0, 0, h1, Rotation3d.kZero)),
        chassisAccel.plus(new Translation3d(0, 0, a1)), VecBuilder.fill(0,0,0), VecBuilder.fill(0,0,0));
    RigidBodyState s2 = new RigidBodyState(carriage2, carriagePose, carriageAccel,
        VecBuilder.fill(0,0,0), VecBuilder.fill(0,0,0));

    // arm base is fixed to the carriage — its "chassis" is the carriage's current pose/accel
    Pose3d armPivotPose = carriagePose.plus(armMountOffset); // armMountOffset: fixed Transform3d from CAD
    Pose3d armPose = armPivotPose.plus(new Transform3d(Translation3d.kZero, new Rotation3d(0, -theta, 0)));
    RigidBodyState armState = new RigidBodyState(arm, armPose, carriageAccel,
        VecBuilder.fill(0, thetaDot, 0), VecBuilder.fill(0, thetaDdot, 0));

    return List.of(s1, s2, armState);
}
```

This is the general pattern for any "mechanism riding on another mechanism":
compute the parent's current pose/base-accel first, then treat it as the
child's `mountPose`/`chassisAccel` when building the child's `RigidBodyState`. Note
`comAcceleration()` (`tipping.md`'s rigid-body kinematics) only needs the *base's*
acceleration and the joint's own `omega`/`alpha` — it doesn't need the
parent's `omega`/`alpha` explicitly, since those are already baked into
`carriageAccel` for a purely-translating parent. (A rotating parent, as in
Example 5, does need to pass its own `omega` through — see below.)

---

## 5. Double-jointed arm

Two revolute joints (shoulder `theta1`, elbow `theta2`, both pitch axes),
second body's `RigidBodyState` built from the first body's pose *and* the
compounding angular velocity/acceleration (`omega`/`alpha` add across a
serial chain, per axis, since both joints share the same rotation axis here).

```java
RigidBody body1 = RigidBody.fromCad(m1, com1, ixx1, iyy1, izz1, 0, 0, 0);
RigidBody body2 = RigidBody.fromCad(m2, com2, ixx2, iyy2, izz2, 0, 0, 0);

List<RigidBodyState> doubleArmStates(double theta1, double theta1Dot, double theta1Ddot,
                                 double theta2, double theta2Dot, double theta2Ddot) {
    // Body 1: shoulder, fixed to chassis
    Pose3d pose1 = mountPose.plus(new Transform3d(Translation3d.kZero, new Rotation3d(0, -theta1, 0)));
    Vector<N3> omega1 = VecBuilder.fill(0, theta1Dot, 0);
    Vector<N3> alpha1 = VecBuilder.fill(0, theta1Ddot, 0);
    RigidBodyState s1 = new RigidBodyState(body1, pose1, chassisAccel, omega1, alpha1);

    // Body 2: elbow, mounted at the end of body 1 (elbowOffset: fixed Transform3d, body length from CAD)
    Pose3d elbowPivotPose = pose1.plus(elbowOffset);
    Pose3d pose2 = elbowPivotPose.plus(new Transform3d(Translation3d.kZero, new Rotation3d(0, -theta2, 0)));
    Vector<N3> omega2 = omega1.plus(VecBuilder.fill(0, theta2Dot, 0));   // rates add, shared axis
    Vector<N3> alpha2 = alpha1.plus(VecBuilder.fill(0, theta2Ddot, 0));

    // elbow base's linear accel: body-1's contribution to a point at elbowOffset from the shoulder
    Translation3d r1 = elbowOffset.getTranslation().rotateBy(pose1.getRotation());
    Translation3d elbowPivotAccel = chassisAccel
        .plus(cross3(alpha1, r1))
        .plus(cross3(omega1, cross3(omega1, r1)));

    RigidBodyState s2 = new RigidBodyState(body2, pose2, elbowPivotAccel, omega2, alpha2);
    return List.of(s1, s2);
}

private static Translation3d cross3(Vector<N3> w, Translation3d r) {
    double wx = w.get(0), wy = w.get(1), wz = w.get(2);
    return new Translation3d(wy*r.getZ() - wz*r.getY(), wz*r.getX() - wx*r.getZ(), wx*r.getY() - wy*r.getX());
}
```

`cross3` is the same helper as `RigidBodyState`'s private one — worth pulling out
to a shared `GeometryUtils` class once more than one mechanism needs it,
which a double-jointed arm is the first example to.

---

## 6. Nine-DOF robot arm (three segments, 3 DOF each: extend + 2-axis rotate)

Three segments in series, each with its own 2-axis "wrist" (rotate about a
known axis A, then about a known axis B fixed *relative to the first
rotation* — a universal-joint layout) plus its own telescoping extension —
3 DOF per segment, 9 total. This builds on Example 5 (serial chain) and the
previous single-axis telescoping case, with two added wrinkles:

- **Two rotation axes per joint, composed in sequence** — angular velocity
  from two non-parallel axes simply adds, but angular *acceleration* picks
  up a coupling term (`omega1 × omega2`) because axis B is itself being
  swept by axis A's rotation.
- **Telescoping**, exactly as before: `RigidBody` (COM/inertia) is a
  function of the current extension `s` and must be rebuilt each cycle, and
  `baseAccel` for the next segment needs the Coriolis (`2*omega x vRel`) and
  `aRel` terms on top of the usual rigid rotation terms.

```java
// mass/inertia of a segment as a function of current extension s (mid-cycle recompute,
// e.g. thin-rod approx: com at s/2 along local X, izz/iyy grow with s^2 — swap in your own CAD-fit curve)
RigidBody segmentBody(double massKg, double s, double ixx0, double iyy0, double izz0) {
    Translation3d com = new Translation3d(s / 2, 0, 0);
    return RigidBody.fromCad(massKg, com, ixx0, iyy0 + massKg*s*s/12, izz0 + massKg*s*s/12, 0, 0, 0);
}

Vector<N3> toVec(Translation3d t) { return VecBuilder.fill(t.getX(), t.getY(), t.getZ()); }
Translation3d toTrans(Vector<N3> v) { return new Translation3d(v.get(0), v.get(1), v.get(2)); }

RigidBodyState armSegmentState(RigidBody body, Pose3d parentPose, Translation3d parentBaseAccel,
        Vector<N3> parentOmega, Vector<N3> parentAlpha,
        Translation3d axisALocal, double theta1, double theta1Dot, double theta1Ddot,
        Translation3d axisBLocal, double theta2, double theta2Dot, double theta2Ddot,
        double s, double sDot, double sDdot) {

    // axis A is fixed in the parent frame; axis B is fixed in the frame *after* rotating by A
    Translation3d axisAWorld = axisALocal.rotateBy(parentPose.getRotation());
    Pose3d afterA = parentPose.plus(new Transform3d(Translation3d.kZero, new Rotation3d(axisALocal, theta1)));
    Translation3d axisBWorld = axisBLocal.rotateBy(afterA.getRotation());
    Pose3d pose = afterA.plus(new Transform3d(Translation3d.kZero, new Rotation3d(axisBLocal, theta2)));

    // this joint's local omega/alpha: velocities add; acceleration picks up an omega1 x omega2 coupling
    // term because axis B is being swept by axis A's rotation (standard two-axis gimbal result).
    Vector<N3> omega1 = toVec(axisAWorld.times(theta1Dot));
    Vector<N3> omega2 = toVec(axisBWorld.times(theta2Dot));
    Vector<N3> alpha1 = toVec(axisAWorld.times(theta1Ddot));
    Vector<N3> alpha2 = toVec(axisBWorld.times(theta2Ddot));
    Vector<N3> omegaLocal = omega1.plus(omega2);
    Vector<N3> alphaLocal = alpha1.plus(alpha2).plus(toVec(cross3(omega1, toTrans(omega2))));

    // and the joint's motion relative to the parent similarly couples with the parent's own spin
    Vector<N3> omega = parentOmega.plus(omegaLocal);
    Vector<N3> alpha = parentAlpha.plus(alphaLocal).plus(toVec(cross3(parentOmega, toTrans(omegaLocal))));

    // segment's own extension direction in world frame, and the tip's offset/velocity/accel relative to this joint
    Translation3d extentWorld = new Translation3d(1, 0, 0).rotateBy(pose.getRotation());
    Translation3d r = extentWorld.times(s);
    Translation3d vRel = extentWorld.times(sDot);
    Translation3d aRel = extentWorld.times(sDdot);

    // rigid rotation terms (Examples 1–5) plus Coriolis (2*omega x vRel) and aRel — Doc 1 §12's
    // "radial extension" term — evaluated with this joint's *own* omega/alpha, not the parent's.
    Translation3d baseAccel = parentBaseAccel
        .plus(cross3(alpha, r))
        .plus(cross3(omega, cross3(omega, r)))
        .plus(cross3(omega, vRel).times(2))
        .plus(aRel);

    return new RigidBodyState(body, pose, baseAccel, omega, alpha);
}

List<RigidBodyState> nineDofArmStates(
        Translation3d[] axisA, double[] theta1, double[] theta1Dot, double[] theta1Ddot,
        Translation3d[] axisB, double[] theta2, double[] theta2Dot, double[] theta2Ddot,
        double[] s, double[] sDot, double[] sDdot,
        double[] massKg, double[] ixx0, double[] iyy0, double[] izz0) {

    List<RigidBodyState> states = new ArrayList<>();
    Pose3d pose = mountPose;
    Translation3d baseAccel = chassisAccel;
    Vector<N3> omega = VecBuilder.fill(0, 0, 0);
    Vector<N3> alpha = VecBuilder.fill(0, 0, 0);

    for (int i = 0; i < 3; i++) {
        RigidBody body = segmentBody(massKg[i], s[i], ixx0[i], iyy0[i], izz0[i]);
        RigidBodyState state = armSegmentState(body, pose, baseAccel, omega, alpha,
                axisA[i], theta1[i], theta1Dot[i], theta1Ddot[i],
                axisB[i], theta2[i], theta2Dot[i], theta2Ddot[i],
                s[i], sDot[i], sDdot[i]);
        states.add(state);
        // next segment's parent frame is this one's tip, after its own 2-axis rotation + extension
        pose = state.worldPose.plus(new Transform3d(new Translation3d(s[i], 0, 0), Rotation3d.kZero));
        baseAccel = state.baseAccel; // simplified hand-off, see note below
        omega = state.omega;
        alpha = state.alpha;
    }
    return states;
}
```

Two things worth flagging rather than glossing over:

- The `baseAccel = state.baseAccel` hand-off at the end of each loop
  iteration is a simplification — the exact acceleration at segment `i`'s
  *tip* (as opposed to its base) is `armSegmentState`'s own `baseAccel`
  formula evaluated one more time, using segment `i`'s `r`/`vRel`/`aRel`.
  For a faithful implementation, compute that explicitly rather than
  reusing the base value — kept simplified here for readability.
- `segmentBody(...)` is rebuilt every cycle because `s` changes; unlike
  Examples 1–5's `RigidBody`s (constructed once, reused every cycle), a
  telescoping segment's mass properties are cheap to recompute but are
  *not* cache-able constants.

---

## Assembling for `MassModel`

Whichever mechanism(s) are actually on the robot, the control loop just
concatenates their `RigidBodyState`s with the chassis's own and (optionally) a
held game piece, then calls `MassModel`:

```java
List<RigidBodyState> bodies = new ArrayList<>();
bodies.add(chassisRigidBodyState);      // static RigidBody at chassisPose, zero relative accel
bodies.addAll(elevatorArmStates(...));
heldPiece.ifPresent(bodies::add);

double mTot = massModel.totalMass(bodies);
Translation3d rcm = massModel.centerOfMass(bodies, mTot);
double izz = massModel.yawInertia(bodies, rcm);
Translation3d fReqInt = massModel.internalReactionForce(bodies);
double tauReqInt = massModel.internalReactionYawTorque(bodies, rcm);
double wEff = massModel.effectiveWeight(bodies, 9.81);
```
