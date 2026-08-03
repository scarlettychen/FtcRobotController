# BrainSTEM autonomous guide

Everything your team normally edits is under this `brainstem` folder:

- `RobotConfiguration.java` — motor names/directions, Pinpoint offsets, Pedro constants/model
- `RobotModel.java` — **primary path tuning** (kS/kV/kA, mass, accel limits)
- `BrainSTEMRobot.java` — hardware fields, Pinpoint ownership, pose sync, robot loop
- `subsystems/` — subsystem implementations
- `follower/` — portable PathSpec / PathFollower (classic Pedro behind adapter)
- `auto/*OpMode.java` — FTC lifecycle

Pedro Pathing follows paths with model feedforward + light PID correction. Ivy is TeamCode-only for mechanisms.

## 0. How pose / distance is measured

`BrainSTEMRobot` owns a goBILDA **Pinpoint** localizer. Each `robot.update()`:

1. `pinpoint.update()` — reads dead wheels + IMU
2. copies Pinpoint pose/velocity into Pedro's `ExternalPoseLocalizer`
3. ticks the bridge and subsystems

Pedro path following then uses that pose to know how far the robot has traveled.
Tune Pinpoint offsets/directions in `RobotConfiguration.createPinpointConstants()`.
Hardware map name defaults to `odo` (set in `RobotConfiguration.createPinpointConstants()`).

## 1. Configure the robot

Edit `RobotConfiguration`:

- `createMecanumConstants()` — motor names and directions
- `createPinpointConstants()` — Pinpoint hardware name, pod offsets, encoder directions
- `createFollowerConstants()` — leave alone (light correction only)
- `createRobotModel()` — **tune here**: mass, limits, kS/kV/kA

Edit `RobotModel` to tune:

- `mass`, `wheelRadius`, `motorFreeSpeed`, `gearRatio`, `motorEfficiency`
- `frictionCoefficient`
- `maxAcceleration`, `maxDeceleration`, `maxLateralAcceleration`
- `maxVelocityOverride`
- `maxAngularVelocity`, `maxAngularAcceleration`
- `kS`, `kV`, `kA`
- CRUISE / LOADED / PRECISION velocity and acceleration scales

Path following uses classic Pedro with **dynamic velocity limits** from
`VelocityConstraint` (curvature + RobotModel). See `docs/DYNAMIC_VELOCITY.md`.
Do not retune Pedro PID for normal autos. Cruise power comes from `RobotModel.feedforwardPower`.

## 2. Add a subsystem

1. Create a class in `brainstem/subsystems/`.
2. Implement `Component`:

```java
public final class Shooter implements Component {
    public Shooter(HardwareMap hardwareMap) {
        // map motors/servos
    }

    public void shootClose() { }
    public boolean atSpeed() { return true; }
    public void stop() { }

    @Override public void reset() { stop(); }
    @Override public void update() { }
    @Override public String test() { return "Shooter"; }
}
```

3. Add a public field in `BrainSTEMRobot`, construct it, and call `addSubsystem(shooter)`.
4. Add named commands in `RobotActions`:

```java
public Command shooterOnClose() {
    return run(() -> robot.shooter.shootClose());
}

public Command waitForShooter() {
    return waitUntil(() -> robot.shooter.atSpeed());
}

public Command shooterOnAndReady() {
    return runThenWait(
            () -> robot.shooter.shootClose(),
            () -> robot.shooter.atSpeed()
    );
}
```

`run(...)` changes mechanism state once; it does not keep calling the method and does not
turn the mechanism off. Stop mechanisms explicitly in another command and in OpMode/robot stop.

## 3. Add coordinates

Pose arrays always use `{xInches, yInches, headingDegrees}`:

```java
public double[] score = xyz(-39, -39, -137);
public double[] pickup = xyz(-12, -58, -90);
```

Add every new named field to `auto/poses/RobotPoses`, then fill it in for Blue/Red subclasses.
Blue and Red tables can be independent.
Use `PoseConverter.useFTCCoordinates()` (already called by `PedroGuide`) so arrays are interpreted
in FTC field coordinates.

Waypoints and markers are different:

- waypoints are field poses that shape `pathDrive(...)` geometry
- markers are callbacks fired once at a path completion fraction from 0 to 1
- with time-optimal following, marker completion spans the whole path-chain arc length

## 4. Add named drive commands

Add these to `RobotActions`. Autos should call the names, not raw coordinates.

```java
public Command driveToScore() {
    return lineDrive(() -> {
        precision();
        return poses().close1Shooting;
    });
}

public Command collectCycle() {
    return pathDrive(
            () -> {
                loaded();
                return poses().collect1Pre;
            },
            () -> poses().firstSpikeEnd
    );
}
```

The `Supplier<double[]>` lambdas are deferred until command initialization. Therefore alliance
selection happens before coordinates are read.

### Drive builders available inside `RobotActions`

- `lineDrive(double[] pose, Marker...)`
- `lineDrive(x, y, headingDegrees, Marker...)`
- `lineDrive(Supplier<double[]> pose, Marker...)` — preferred for alliance poses
- `bezierDrive(double[]... poses)` — curved path through control/end poses
- `bezierDrive(Pose... poses)`
- `bezierDrive(Supplier<double[]>... poses)` — deferred/alliance-safe
- `pathDrive(double[]... waypoints)` — line-chain through poses
- `pathDrive(Marker[], double[]... waypoints)`
- `pathDrive(Supplier<double[]>... waypoints)` — preferred for alliance poses
- `pathDrive(Marker[], Supplier<double[]>... waypoints)`
- `turnTo(headingDegrees)`

### Motion contexts

Call these while resolving a named drive:

- `cruise()` — full speed
- `loaded()` — carrying game elements
- `precision()` — slower scoring/alignment

They change scaling in the TeamCode `RobotModel`.

## 5. Command helpers

Available inside `RobotActions`:

- `run(Runnable)` — one-shot state change
- `waitSeconds(seconds)` — wall-clock delay
- `waitUntil(BooleanSupplier)` — finishes when condition is true
- `runThenWait(start, finished)` — run once, then wait until ready
- `sequence(Command...)` — commands one after another
- `parallel(Command...)` — all together; finishes when <em>all</em> finish
- `driveWith(drive, alongside...)` — path + other actions together; finishes when the
  <em>path</em> finishes (mechanisms can run during the drive without blocking it)
- `alongWith(...)` — alias for `parallel`
- `conditional(condition, onTrue, onFalse)` — branch once at initialize
- `retry(supplier, success, maxAttempts)` — fresh command per attempt until success
- `validate(condition, onSuccess, onFailure)` — validation branch with PASS/FAILED log
- `waitUntilValidated(condition, timeoutSeconds)` — wait or TIMEOUT, then continue

Low-level `FunctionalCommand` helpers are also available:

- `Commands.instant(action)`
- `ControlFlow.waitSeconds(seconds)`
- `Commands.waitUntil / Command.build()`
- constructor `(onInit, onExecute, finished, onEnd)`

### High-level resilient actions (use these in match autos)

Prefer named helpers over raw `retry` / `validate` trees. Recovery stays **local** to the
action that can fail; attempt counts and timeouts are fixed so behavior stays deterministic.

| Helper | Behavior |
|--------|----------|
| `tryCollect()` | `retry(() -> collect(), hasGamePiece, 2)` |
| `tryScore()` | `validate(hasGamePiece, score(), recoverIntake())` |
| `safeAlign()` | `retry(() -> align(), isAligned, 2)` |
| `recoverLocalization()` | brief settle + validate Pinpoint is still usable |

Wire sensor stubs on `RobotActions`: `hasGamePiece()`, `isShooterAtSpeed()`, `isAligned()`,
`isLocalizationReasonable()`.

## 5b. Auton composition examples

### 1) Normal deterministic auton (no branching)

```java
@Override
public void run() {
    run(sequence(
            parallel(bot.shooterTurnOnClose(), bot.driveToGoal()),
            bot.waitSeconds(0.2),
            bot.moveSpindexer360(),
            bot.collectFirstSpike(),
            bot.driveToGoal()
    ));
}
```

Fixed order, fixed poses — same path every run.

### 2) Retrying a failed intake

```java
@Override
public void run() {
    run(sequence(
            bot.driveOffLine(),
            bot.tryCollect(),          // up to 2 collect attempts until hasGamePiece
            bot.driveToGoal(),
            bot.tryScore()
    ));
}
```

`tryCollect()` owns its own recovery budget. The rest of the auton does not grow a decision tree.

Equivalent explicit form (prefer the helper in match code):

```java
bot.retry(() -> bot.collect(), bot::hasGamePiece, 2);
```

### 3) Validating a scoring action

```java
@Override
public void run() {
    run(sequence(
            bot.tryCollect(),
            bot.safeAlign(),
            bot.tryScore()   // requires piece; waits for shooter speed inside score()
    ));
}
```

Inside `tryScore()` / `score()`:

- no piece → local `recoverIntake()` (not a global “mode”)
- piece present → drive to shoot, `waitUntilValidated(isShooterAtSpeed, 1.5)`, fire only on PASS

Logs look like:

```text
Validation:
    PASS
```

or `FAILED` / `TIMEOUT`.
## 6. Write an OpMode auton (Ivy)

Keep sequencing in the OpMode. No AutoMode class — fewer files to sift when something breaks.

```java
@Autonomous(name = "My Auto")
public class MyAutoOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        RobotActions bot = new RobotActions(robot);
        bot.getDrive().setExternalLoop(true);

        // alliance select during init...
        waitForStart();
        if (isStopRequested()) return;

        robot.setStartPose(bot.poses().start);
        Command root = bot.sequence(
                bot.driveToGoal(),
                bot.waitSeconds(0.2) // use ControlFlow.waitSeconds via a helper if needed
        );
        // Prefer:
        // Command root = sequential(bot.driveToGoal(), waitMs(200));

        Scheduler.reset();
        Scheduler.schedule(root);
        while (opModeIsActive() && Scheduler.isScheduled(root)) {
            robot.update();
            Scheduler.execute();
            telemetry.update();
        }
        Scheduler.cancel(root);
        Scheduler.reset();
        robot.follower.breakFollowing();
    }
}
```

Named actions live in `RobotActions`. Composition helpers live in `ControlFlow` or Ivy
`Groups` / `Commands`. Pedro only supplies `PedroDrive` path commands.

## 7. Own the auton from an OpMode

```java
BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
RobotActions actions = new RobotActions(robot);
actions.setAlliance(isRed);

MyAuto auto = new MyAuto(robot, actions);
auto.setAlliance(isRed);
auto.setExternalLoop(true);

// Useful during INIT so telemetry/localization starts at this auto's own pose.
robot.setStartPose(auto.getStartPose());

waitForStart();
auto.start();

while (opModeIsActive() && !auto.isFinished()) {
    // Pinpoint is already read inside robot.update().
    // Only call robot.syncPose(...) if another localizer (e.g. RR) owns pose instead.
    robot.update();
    auto.update();
    telemetry.update();
}

auto.stop();
```

When an external localizer owns pose, initialize/reset that localizer to the same
`auto.getStartPose()` before the loop.

## 8. Schedule one command directly

```java
Command move = drive.forwardDrive(48);
Scheduler.reset();
Scheduler.schedule(move);
while (opModeIsActive() && Scheduler.isScheduled(move)) {
    robot.update();
    Scheduler.execute();
}
Scheduler.cancel(move);
Scheduler.reset();
robot.follower.breakFollowing();
```

Ivy: `Scheduler.schedule` / `execute` / `cancel` / `reset` / `isScheduled`.
Compositions: `Groups.sequential`, `parallel`, `deadline`, or `ControlFlow.*`.

## 9. BrainSTEMRobot functions

- constructor `(hardwareMap, telemetry, opMode)` — one only; starts at field origin
- `setStartPose(double[])` — match autos only (`{x,y,headingDegrees}`)
- `setAlliance(red)`
- `addSubsystem(Component)` / `getSubsystems()`
- `update()` / `reset()`

## 10. PedroDrive low-level functions

Normally wrap these in `RobotActions`. They remain useful when creating a new named action:

- `getFollower()`, `getPose()`, `isBusy()`, `update()`, `pathCompletion()`
- `holdEnd(boolean)` — classic Pedro hold-at-end
- `setExternalLoop(boolean)`
- `setStartPose(x,y,headingDegrees)` / `setStartPose(double[])`
- `lineDrive(...)`, `bezierDrive(...)`, `pathDrive(...)`, `turnTo(...)`

Prefer `brainstem/follower/PathSpec` + `PathFollower` for all autos (via `OpmodeCommands`).
UI planners should emit PathSpec JSON (`PathSpec.fromJson` / `toJson`).

Paths are baked in command `initialize()`, not while the OpMode constructs the sequence. The
start is the follower's current pose at that moment; destinations are absolute field positions.

## 11. Safety rules

- Do not command Pedro Mecanum and Road Runner MecanumDrive at the same time.
- Always have correct motor names/directions and a working pose source.
- Explicitly stop subsystem motors; one-shot commands do not auto-stop them.
- Always cancel/stop and call `breakFollowing()` when an OpMode ends early.
- Tune `RobotModel` kS/kV/kA (used by `PedroFollowerAdapter`) before heavy classic PID retuning.
- Match autos: call `tryCollect` / `tryScore` / `safeAlign` / `recoverLocalization`, not raw
  `retry`/`validate` trees. Keep recovery inside the failing action.
- Use `robot.setStartPose(double[])` for absolute Pinpoint stamps (not repeated
  `pinpoint.setStartPose`, which rebases and corrupts XY).
- Fixed retry counts and timeouts keep resilient actions deterministic.
