# Dynamic velocity limiting

BrainSTEM Pathing does **not** use a time-optimal trajectory generator.
It uses a simpler rule:

> Given the robot model and the path geometry right now, what is the fastest
> safe speed?

That answer comes from `VelocityConstraint` and is applied each tick by the
`PathFollower` adapter (today: `PedroFollowerAdapter`).

```
PathSpec (segment.maxVelocity)
        ↓
PathFollower.update()
        ↓
VelocityConstraint(curvature, RobotModel, segmentCap)
        ↓
RobotModel.setPathVelocityCeiling(v)
        ↓
Pedro cruise FF uses motorLimitedVelocity() ≤ ceiling
```

Autos never see Pedro. They only build `PathSpec` and call `PathFollower`.

---

## Why constant velocity is inefficient

If you always cruise at one fixed speed:

- **Tight curves** — too fast → slip, cut corners, tip risk  
- **Long straights** — too slow → wasted time  

Constant speed optimizes neither safety nor cycle time. Dynamic limiting slows
only where geometry or friction demands it.

---

## Why shortest path ≠ fastest path

Path length is not cycle time.

| Geometry | Distance | Typical speed | Cycle time |
|----------|----------|---------------|------------|
| Sharp corner / polyline | Short | Must slow hard | Often **slower** |
| Smooth Bezier turn | Longer | Higher average speed | Often **faster** |

A longer, smoother path can finish sooner if average velocity stays higher.

---

## How curvature affects speed

Lateral acceleration on a curve:

\[
a_\mathrm{lat} = v^2 \cdot |\kappa|
\]

Safe max speed for a given friction / lateral limit:

\[
v_\mathrm{max} = \sqrt{\frac{a_\mathrm{lat,max}}{|\kappa|}}
\]

where `a_lat,max = RobotModel.getMaxLateralAcceleration()`  
(`frictionCoefficient × g` after SysId).

- **Straight** (`κ ≈ 0`): no curvature limit → robot top speed  
- **Gentle curve** (small `|κ|`): high `v_max`  
- **Tight curve** (large `|κ|`): low `v_max`

`VelocityConstraint` then takes:

```
min(robotTopSpeed, curvatureLimit, segmentCap?)
```

---

## When a line outperforms a Bezier

- Nearly straight move  
- Need precise end pose / short distance  
- Little curvature benefit  

Lines keep `|κ| ≈ 0` → full model speed.

## When a Bezier outperforms a line

- Must change direction (corner)  
- Line chain forces a near-stop or hard yaw  
- Smooth Bezier spreads curvature → higher average `v`  

Trade distance for continuous speed.

---

## PathSpec.Segment.maxVelocity

| Value | Meaning |
|-------|---------|
| `0` | Dynamic only: `min(robot, curvature)` |
| `> 0` | Cap: `min(userCap, robot, curvature)` |

Example — precision approach:

```java
new PathSpec.Segment(points, HeadingMode.HOLD_START, 20.0) // never faster than 20 in/s
```

Example — full dynamic cruise:

```java
new PathSpec.Segment(points, HeadingMode.TANGENT, 0.0)
```

---

## RobotModel (single source of truth)

Do not duplicate limits. Use:

```java
RobotModel.MotionConstraints c = robot.robotModel.constraints();
c.maxVelocity();
c.maxAcceleration();
c.maxDeceleration();
c.frictionCoefficient();
c.maxLateralAcceleration();
```

Tune via SysId OpModes, then `RobotConfiguration.createRobotModel()`.

---

## Telemetry (`FollowerOutput`)

| Field | Meaning |
|-------|---------|
| `velocityLimit` | Active ceiling from `VelocityConstraint` |
| `curvatureLimitReason` | `ROBOT_TOP_SPEED` / `CURVATURE` / `SEGMENT_CAP` |
| `velocityReference` | End-of-path profiled target this tick |
| `accelerationReference` | Tangential accel target this tick |
| `curvature` | Signed κ (1/in) |

---

## Self-check examples

Run `VelocityConstraintExamples.runSelfCheck()` (throws if expectations fail):

1. Straight line → ≈ robot top speed, reason `ROBOT_TOP_SPEED`  
2. Tight Bezier κ → lower speed, reason `CURVATURE`  
3. Segment cap → never above cap, reason `SEGMENT_CAP` when binding  
4. Different `RobotModel` friction / top speed → different limits on same κ  
