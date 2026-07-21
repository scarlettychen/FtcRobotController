# BrainSTEM autonomous guide

Everything your team normally edits is under this `brainstem` folder:

- `RobotConfiguration.java` — motor names/directions, Pinpoint offsets, Pedro constants/model
- `RobotModel.java` — physical/time-optimal tuning
- `BrainSTEMRobot.java` — hardware fields, Pinpoint ownership, pose sync, robot loop
- `subsystems/` — subsystem implementations
- `auto/RobotActions.java` — named drive and subsystem commands
- `auto/poses/` — field coordinates
- `auto/*Auto.java` — autonomous sequences
- `auto/*OpMode.java` — FTC lifecycle and loop ownership

The Pedro library contains reusable framework code. Do not add team subsystem commands or
match coordinates there.

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
- `createFollowerConstants()` — Pedro follower settings (mostly relevant to classic following)
- `createRobotModel()` — creates the team-owned time-optimal model
- `configurePredictiveFollower()` — TeamCode feedback gains layered over model feedforward

Edit `RobotModel` to tune:

- `mass`, `wheelRadius`, `motorFreeSpeed`, `gearRatio`, `motorEfficiency`
- `frictionCoefficient`
- `maxAcceleration`, `maxDeceleration`, `maxLateralAcceleration`
- `maxVelocityOverride`
- `maxAngularVelocity`, `maxAngularAcceleration`
- `kS`, `kV`, `kA`
- CRUISE / LOADED / PRECISION velocity and acceleration scales

Time-optimal following is enabled by default. Tune `RobotModel` first. Classic Pedro PID
settings are primarily relevant when `drive.useTimeOptimal(false)` is selected.

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
public AutoCommand shooterOnClose() {
    return run(() -> robot.shooter.shootClose());
}

public AutoCommand waitForShooter() {
    return waitUntil(() -> robot.shooter.atSpeed());
}

public AutoCommand shooterOnAndReady() {
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
public AutoCommand driveToScore() {
    return lineDrive(() -> {
        precision();
        return poses().close1Shooting;
    });
}

public AutoCommand collectCycle() {
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
- `sequence(AutoCommand...)` — commands one after another
- `parallel(AutoCommand...)` — all commands together; finishes when all finish
- `conditional(condition, onTrue, onFalse)` — branch once at initialize
- `retry(supplier, success, maxAttempts)` — fresh command per attempt until success
- `validate(condition, onSuccess, onFailure)` — validation branch with PASS/FAILED log
- `waitUntilValidated(condition, timeoutSeconds)` — wait or TIMEOUT, then continue

Low-level `FunctionalCommand` helpers are also available:

- `FunctionalCommand.instant(action)`
- `FunctionalCommand.waitSeconds(seconds)`
- `FunctionalCommand.runUntil(executeEachLoop, finished)`
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
## 6. Write an AutoMode auton

Each auton owns its start pose. Override `getStartPose()`:

```java
public final class MyAuto extends AutoMode {
    private static final double[] BLUE_START = AlliancePoses.xyz(-65, -41.75, 0);
    private static final double[] RED_START = AlliancePoses.xyz(-65, 41.75, 0);
    private final RobotActions bot;

    public MyAuto(BrainSTEMRobot robot, RobotActions bot) {
        super(robot.follower, bot);
        this.bot = bot;
    }

    @Override
    public double[] getStartPose() {
        return isRed() ? RED_START : BLUE_START;
    }

    @Override
    public void run() {
        run(sequence(
                parallel(bot.shooterTurnOnClose(), bot.driveToGoal()),
                bot.waitSeconds(0.2),
                bot.moveSpindexer360(),
                bot.collectFirstSpike()
        ));
    }
}
```

`AutoMode.start()` applies `getStartPose()` before building/scheduling commands. This lets every
auton have a different start without putting an auton inside `BrainSTEMRobot`.

`AutoMode` functions:

- `setAlliance(red)`
- `setExternalLoop(true)` — use when BrainSTEM supplies pose/loop ownership
- `getStartPose()` — override per auton
- `start()` — apply start, build root sequence, schedule it
- `update()` — call every active loop
- `isFinished()`
- `stop()` — cancel and break following
- inside `run()`: `run(sequence(...))`, `parallel(...)`, `schedule(...)`, `conditional(...)`,
  `retry(...)`, `validate(...)`, `waitUntilValidated(...)`

## 7. Own the auton from an OpMode

```java
BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
RobotActions actions = PedroGuide.createActions(robot);
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

For tests or short routines:

```java
AutoScheduler scheduler = new AutoScheduler();
AutoCommand command = actions.driveToGoal();

waitForStart();
scheduler.schedule(command);

while (opModeIsActive() && scheduler.isRunning()) {
    robot.update();
    scheduler.run();
    telemetry.update();
}

scheduler.cancel();
robot.follower.breakFollowing();
```

`AutoScheduler` functions:

- `schedule(command)` — ends/replaces any current command
- `run()` — initialize once, execute each loop, end when finished
- `cancel()`
- `isRunning()`
- `isFinished()`

## 9. BrainSTEMRobot functions

- constructors — create hardware/follower/model only; no auton
- `addSubsystem(Component)`
- `getSubsystems()`
- `setAlliance(red)` — robot flag only; also set alliance on actions/auto
- `setStartPose(double[])` — apply an auton-specific `{x,y,headingDegrees}` pose
- `syncPose(x,y,headingRad)`
- `syncPose(x,y,headingRad,vx,vy,omega)`
- `syncPose(...,localizationConfidence)`
- `update()` — bridge then every subsystem
- `updateWithTelemetry()`
- `reset()`

## 10. PedroDrive low-level functions

Normally wrap these in `RobotActions`. They remain useful when creating a new named action:

- `getFollower()`, `getPose()`, `isBusy()`, `update()`
- `useTimeOptimal(boolean)` — default true
- `holdEnd(boolean)` — classic follower only
- `setExternalLoop(boolean)`
- `setStartPose(x,y,headingDegrees)` / `setStartPose(double[])`
- `lineDrive(...)`, `bezierDrive(...)`, `pathDrive(...)`, `turnTo(...)`

Paths are baked in command `initialize()`, not while the OpMode constructs the sequence. The
start is the follower's current pose at that moment; destinations are absolute field positions.

## 11. Safety rules

- Do not command Pedro Mecanum and Road Runner MecanumDrive at the same time.
- Always have correct motor names/directions and a working pose source.
- Explicitly stop subsystem motors; one-shot commands do not auto-stop them.
- Always cancel/stop and call `breakFollowing()` when an OpMode ends early.
- Tune the TeamCode `RobotModel` before reaching for classic Pedro PID tuners while TO is on.
- Match autos: call `tryCollect` / `tryScore` / `safeAlign` / `recoverLocalization`, not raw
  `retry`/`validate` trees. Keep recovery inside the failing action.
- Use `robot.setStartPose(double[])` for absolute Pinpoint stamps (not repeated
  `pinpoint.setStartPose`, which rebases and corrupts XY).
- Fixed retry counts and timeouts keep resilient actions deterministic.
