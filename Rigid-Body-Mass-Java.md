# Rigid-Body Mass Model in WPILib Java

This sketches how to implement the `(m, c, I)` rigid-body model from `Cad-to-mass-model.md` as a small class hierarchy in a WPILib Java robot
project, using `edu.wpi.first.math.geometry` (`Translation3d`, `Rotation3d`,
`Transform3d`) and `edu.wpi.first.math` linear algebra (`Matrix<N3,N3>`, `Nat`,
`MatBuilder`, `VecBuilder`) instead of hand-rolled vector/matrix code. Angles
and heights (joint axis locations, mounting offsets) are assumed known/CAD-derived
and are wired in as constants or `Transform3d`s per mechanism.

Goal: two classes.

- **`RigidBody`** — the *static*, CAD-derived description of one body: mass,
  COM in the body's own body frame, inertia tensor about that COM. Doesn't
  know about the current joint angle.
- **`RigidBodyState`** (or similar) — the *current* kinematic state of a `RigidBody`
  at this control cycle (world pose, base acceleration, angular
  velocity/acceleration), which is what the swerve controller actually needs.

Everything downstream just sums `RigidBodyState`s to get `M_tot`, `r_cm`, `I_zz`,
`F_req^int`, `τ_req^int`, `W_eff`, etc. (`Cad-to-mass-model.md`, Section 4).

---

## 1. `RigidBody` — static per-body CAD data

```java
/** Immutable, CAD-derived mass properties of one rigid body, in the body's own frame. */
public class RigidBody {
    public final double massKg;
    public final Translation3d comBody;      // COM in body frame (origin on joint axis)
    public final Matrix<N3, N3> inertiaBody; // about comBody, body-frame axes

    public RigidBody(double massKg, Translation3d comBody, Matrix<N3, N3> inertiaBody) {
        this.massKg = massKg;
        this.comBody = comBody;
        this.inertiaBody = inertiaBody;
    }

    /** Build from CAD principal/product-of-inertia values (kg*m^2), diagonal + off-diagonal. */
    public static RigidBody fromCad(double massKg, Translation3d comBody,
            double ixx, double iyy, double izz,
            double ixy, double ixz, double iyz) {
        Matrix<N3, N3> I = new MatBuilder<>(Nat.N3(), Nat.N3()).fill(
                 ixx, -ixy, -ixz,
                -ixy,  iyy, -iyz,
                -ixz, -iyz,  izz);
        return new RigidBody(massKg, comBody, I);
    }

    /** Combine two bodys rigidly fixed together (e.g. hardware without its own CAD model, `Cad-to-mass-model.md` §6). */
    public RigidBody combine(RigidBody other) {
        double m = this.massKg + other.massKg;
        Translation3d com = this.comBody.times(this.massKg)
                .plus(other.comBody.times(other.massKg)).div(m);
        Matrix<N3, N3> I = parallelAxisTo(com).plus(other.parallelAxisTo(com));
        return new RigidBody(m, com, I);
    }

    /** This body's inertia tensor shifted (parallel-axis theorem) to a new reference point, same body-frame axes. */
    public Matrix<N3, N3> parallelAxisTo(Translation3d point) {
        Translation3d d = comBody.minus(point);
        double dd = d.getX()*d.getX() + d.getY()*d.getY() + d.getZ()*d.getZ();
        Matrix<N3, N3> ddOuter = outer(d, d);
        Matrix<N3, N3> shift = Matrix.eye(Nat.N3()).times(dd).minus(ddOuter).times(massKg);
        return inertiaBody.plus(shift);
    }

    private static Matrix<N3, N3> outer(Translation3d a, Translation3d b) {
        return new MatBuilder<>(Nat.N3(), Nat.N3()).fill(
                a.getX()*b.getX(), a.getX()*b.getY(), a.getX()*b.getZ(),
                a.getY()*b.getX(), a.getY()*b.getY(), a.getY()*b.getZ(),
                a.getZ()*b.getX(), a.getZ()*b.getY(), a.getZ()*b.getZ());
    }
}
```

Standard primitives (`Cad-to-mass-model.md` §6, box/cylinder/tetrahedron stand-ins for parts
without CAD material) are just more `RigidBody.fromCad(...)` factory methods
using closed-form box/cylinder inertia formulas — plain `Math` calls, no
WPILib needed there.

---

## 2. `RigidBodyState` — current-cycle kinematics of a body

This is the runtime object: a `RigidBody` plus where it currently is and how
it's currently moving, following `Cad-to-mass-model.md` §2/§4. Joint state (angle, extension,
their derivatives) comes from your existing mechanism subsystems (encoders);
this class just turns that into world-frame quantities.

```java
public class RigidBodyState {
    public final RigidBody body;
    public final Pose3d worldPose;        // body frame -> world/chassis frame
    public final Translation3d baseAccel; // a_Q: acceleration of the joint base
    public final Vector<N3> omega;         // joint angular velocity (world frame), rad/s
    public final Vector<N3> alpha;         // joint angular accel (world frame), rad/s^2

    public RigidBodyState(RigidBody body, Pose3d worldPose,
                      Translation3d baseAccel, Vector<N3> omega, Vector<N3> alpha) {
        this.body = body;
        this.worldPose = worldPose;
        this.baseAccel = baseAccel;
        this.omega = omega;
        this.alpha = alpha;
    }

    public Translation3d comWorld() {
        return worldPose.getTranslation().plus(body.comBody.rotateBy(worldPose.getRotation()));
    }

    /** World-frame inertia tensor about this body's own COM: R * I_body * R^T. */
    public Matrix<N3, N3> inertiaWorld() {
        Matrix<N3, N3> r = worldPose.getRotation().toMatrix(); // 3x3 rotation matrix
        return r.times(body.inertiaBody).times(r.transpose());
    }

    /** a_i = a_Q + alpha x r'' + omega x (omega x r''), `Cad-to-mass-model.md` eq. in §2. */
    public Translation3d comAcceleration() {
        Translation3d rpp = body.comBody; // r_i'' in body frame == comBody, joint axis is body-frame origin
        Translation3d rppWorld = rpp.rotateBy(worldPose.getRotation());
        Translation3d alphaCrossR = cross(alpha, rppWorld);
        Translation3d omegaCrossR = cross(omega, rppWorld);
        Translation3d centripetal = cross(omega, omegaCrossR);
        return baseAccel.plus(alphaCrossR).plus(centripetal);
    }
    
    /**
     * a_i^rigid(s): the acceleration this body's COM inherits purely from the
     * chassis's own commanded/actual rigid motion (tipping.md §12), independent
     * of this body's own joint state.
     *
     *   a_i^rigid = s*(ax, ay, 0) + s*alphaYawChassis × r_i' + omegaYawChassis × (omegaYawChassis × r_i')
     *
     * r_i' is this body's COM position relative to `pivot` (the chassis
     * reference point tipping.md §12 measures r_i' from — confirm this matches
     * that section; rcm is used elsewhere in this doc for the analogous
     * parallel-axis sums, but tipping.md may define r_i' about a fixed chassis
     * frame origin instead of the instantaneous rcm).
     *
     * Only the acceleration terms (ax, ay, alphaYawChassis) scale with s — the
     * centripetal term omega x (omega x r') depends on the chassis's *actual*
     * current yaw rate, which exists independent of how much of the *new*
     * commanded acceleration we're asking for, so it is not scaled.
     */
    public Translation3d rigidAcceleration(double s, double ax, double ay,
            double alphaYawChassis, double omegaYawChassis, Translation3d pivot) {
        Translation3d rPrime = comWorld().minus(pivot);

        Vector<N3> alphaVec = VecBuilder.fill(0, 0, s * alphaYawChassis);
        Vector<N3> omegaVec = VecBuilder.fill(0, 0, omegaYawChassis);

        Translation3d translational = new Translation3d(s * ax, s * ay, 0);
        Translation3d alphaCrossR = cross(alphaVec, rPrime);
        Translation3d omegaCrossR = cross(omegaVec, rPrime);
        Translation3d centripetal = cross(omegaVec, omegaCrossR);

        return translational.plus(alphaCrossR).plus(centripetal);
    }

    /**
     * a_i(s) = a_i^int + a_i^rigid(s) — the *total* acceleration this body's
     * COM experiences, combining its own joint motion with the chassis's
     * commanded rigid motion. This is what zeroMomentPoint(...) should sum over,
     * not comAcceleration() alone.
     */
    public Translation3d totalAcceleration(double s, double ax, double ay,
            double alphaYawChassis, double omegaYawChassis, Translation3d pivot) {
        return comAcceleration().plus(
            rigidAcceleration(s, ax, ay, alphaYawChassis, omegaYawChassis, pivot));
    }

    // WPILib's Translation3d has no built-in cross product; small helper using plain Math.
    private static Translation3d cross(Vector<N3> w, Translation3d r) {
        double wx = w.get(0), wy = w.get(1), wz = w.get(2);
        return new Translation3d(
            wy*r.getZ() - wz*r.getY(),
            wz*r.getX() - wx*r.getZ(),
            wx*r.getY() - wy*r.getX());
    }
}
```

`worldPose` for a given mechanism is exactly what forward kinematics already
gives you — e.g. for an elevator stage: `chassisToBase.plus(new Transform3d(0,
0, heightMeters, Rotation3d.kZero))`; for an arm segment: `basePose.plus(new
Transform3d(new Translation3d(armLength, 0, 0), new Rotation3d(0, -armAngle,
0)))`. Since joint axes and mounting geometry are already known/measured,
this is just `Pose3d`/`Transform3d` composition, no new math.

---

## 3. Aggregating into the controller's whole-robot quantities

A thin `MassModel` sums `List<RigidBodyState>` into exactly the scalars Part III
of the controller doc needs (`Cad-to-mass-model.md` §4), each control cycle:

```java
public class MassModel {
    public double totalMass(List<RigidBodyState> bodies) {
        return bodies.stream().mapToDouble(b -> b.body.massKg).sum();
    }

    public Translation3d centerOfMass(List<RigidBodyState> bodies, double mTot) {
        Translation3d weighted = Translation3d.kZero;
        for (RigidBodyState b : bodies) weighted = weighted.plus(b.comWorld().times(b.body.massKg));
        return weighted.div(mTot);
    }

    /** I_zz about r_cm, via world inertia tensors + parallel axis (`Cad-to-mass-model.md` §4). */
    public double yawInertia(List<RigidBodyState> bodies, Translation3d rcm) {
        double izz = 0;
        for (RigidBodyState b : bodies) {
            Matrix<N3, N3> Iw = b.inertiaWorld();
            Translation3d d = b.comWorld().minus(rcm);
            izz += Iw.get(2, 2) + b.body.massKg * (d.getX()*d.getX() + d.getY()*d.getY());
        }
        return izz;
    }

    /** F_req^int = sum m_k a_k^int (`Cad-to-mass-model.md` §4). */
    public Translation3d internalReactionForce(List<RigidBodyState> bodies) {
        Translation3d f = Translation3d.kZero;
        for (RigidBodyState b : bodies) f = f.plus(b.comAcceleration().times(b.body.massKg));
        return f;
    }

    /** tau_req^int yaw component = sum (Δr x m*a_int)_z + I_axis*alpha_int_z (`Cad-to-mass-model.md` §4). */
    public double internalReactionYawTorque(List<RigidBodyState> bodies, Translation3d rcm) {
        double tau = 0;
        for (RigidBodyState b : bodies) {
            Translation3d d = b.comWorld().minus(rcm);
            Translation3d ma = b.comAcceleration().times(b.body.massKg);
            tau += d.getX()*ma.getY() - d.getY()*ma.getX();          // (d x ma)_z
            tau += b.inertiaWorld().get(2, 2) * b.alpha.get(2);       // I_axis * alpha_int, z-component
        }
        return tau;
    }

    /** W_eff = sum m_i(-g - a_iz), feeding the ZMP calc (Part I doc, §5). */
    public double effectiveWeight(List<RigidBodyState> bodies, double g) {
        return bodies.stream()
            .mapToDouble(b -> b.body.massKg * (-g - b.comAcceleration().getZ()))
            .sum();
    }
}
```

These four/five numbers (`M_tot`, `r_cm`, `I_zz`, `F_req^int`, `τ_req^int`,
`W_eff`) are exactly the inputs the swerve controller pipeline (`tipping.md`, §13–15)
consumes each cycle — the ZMP/tipping check, normal-force solve, and wrench
allocation stages don't need to know anything about bodies, joints, or CAD at
all past this point.

---

## 4. Game pieces (`Cad-to-mass-model.md` §5)

Since a held piece is just another `RigidBody` attached at a known offset from
the end-effector's current pose, model it as a body that's conditionally
present:

```java
Optional<RigidBodyState> heldPiece; // empty when not holding anything

List<RigidBodyState> activeBodies() {
    List<RigidBodyState> bodies = new ArrayList<>(mechanismBodies); // chassis + arm + elevator, always present
    heldPiece.ifPresent(bodies::add);
    return bodies;
}
```

`heldPiece`'s `worldPose` is just `endEffectorState.worldPose.plus(gripOffsetTransform)`,
and its acceleration terms fall out of `RigidBodyState.comAcceleration()` the same
way as any other body — no special-casing needed in `MassModel`, matching
`Cad-to-mass-model.md` §5's point that pickup/release is a discrete insert/remove into the
body list, not a different code path.

---

## 5. Library notes

- `Translation3d`, `Rotation3d`, `Pose3d`, `Transform3d` — `edu.wpi.first.math.geometry`, cover all the forward-kinematics composition.
- `Matrix<N3,N3>`, `MatBuilder`, `Nat.N3()`, `VecBuilder`/`Vector<N3>` — `edu.wpi.first.math` (and `edu.wpi.first.math.numbers.N3`), used for the inertia tensor and its parallel-axis shifts/rotations.
- `Rotation3d.toMatrix()` gives the rotation matrix `R` needed for `I_world = R * I_body * Rᵀ`.
- No built-in `Translation3d.cross()` — a 4-line helper using `Math`-level component arithmetic (as above) is simplest rather than fighting the geometry types.
- Everything else (box/cylinder inertia formulas, parallel-axis theorem, weighted sums) is plain `java.lang.Math` arithmetic — no need for an external linear-algebra library beyond what WPILib already ships.
