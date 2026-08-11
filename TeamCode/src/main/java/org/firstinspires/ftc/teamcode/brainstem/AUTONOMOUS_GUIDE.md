# BrainSTEM autonomous guide

Everything your team normally edits is under this `brainstem` folder:

- `RobotConfiguration.java` — motor names/directions, Pinpoint offsets, Pedro constants/model
- `RobotModel.java` — **primary path tuning** (kS/kV/kA, mass, accel limits)
- `BrainSTEMRobot.java` — hardware fields, Pinpoint ownership, pose sync, robot loop
- `subsystems/` — subsystem implementations
- `follower/` — portable PathSpec / PathFollower (classic Pedro behind adapter)
- `auto/CommandAutoOpMode.java` — base class for match autos (init/run/cleanup loop)
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
4. Add helpers in `OpmodeCommands` (or call the subsystem from an Ivy `Commands.instant`).

```java
public static Command shooterOnClose(Shooter shooter) {
    return Commands.instant(shooter::shootClose).requiring(shooter);
}
```

One-shot commands change state once; they do not keep calling the method and do not
auto-stop. Stop mechanisms explicitly in another command and when the OpMode ends.

## 3. Coordinates

Pose arrays are `{xInches, yInches, headingDegrees}` in {@link FieldCoords}:
center origin, walls ±72, **0° = +Y** (into the field), CCW+.

```java
double[] score = FieldCoords.xyz(-39, -39, -137);
double[] pickup = FieldCoords.xyz(-12, -58, -90);
```

Keep poses as `public static double[]` fields on the auto (see `RedLineScoreAuto`).
For Blue/Red variants, either mirror Y in code (`FieldCoords.xyz(x, -y, ...)`) or make
two small autos that both extend the same `CommandAutoOpMode` subclass with `isRed()` flipped.

## 4. Drive (the simple API)

You need three things:

1. `PathFollower drive = robot.createPathFollower()`
2. A `PathSpec` (hand-built or loaded JSON)
3. `OpmodeCommands.followPath(drive, spec)` — or the shortcuts below

### Build a path

```java
// one line — keep current heading
PathSpec.to(here, goal, PathSpec.HeadingMode.HOLD_START)

// one line — rotate to goal heading
PathSpec.to(here, goal, PathSpec.HeadingMode.HOLD)

// line-chain through poses
PathSpec.through("cycle", start, pickup, score)

// bezier
PathSpec.curve("arc", PathSpec.HeadingMode.TANGENT, null, a, b, c, d)

// robot-relative
PathSpec.forward(here, 24)
PathSpec.strafe(here, 12)   // +left

// speed cap (0 = dynamic VelocityConstraint)
PathSpec.through("slow", start, goal).maxVelocity(20)
```

### Follow it (Ivy)

```java
OpmodeCommands.followPath(drive, path)

// shortcuts (bake from live pose when the command starts):
OpmodeCommands.lineTo(drive, GOAL)     // keep heading
OpmodeCommands.driveTo(drive, GOAL)    // apply GOAL heading
OpmodeCommands.driveForward(drive, 24)
OpmodeCommands.strafeLeft(drive, 12)
```

### Load from a planner UI

```java
PathSpec path = PathSpec.fromRaw(resources, R.raw.my_path);
// or hot-reload: PathSpec.fromFile("my_path.json")  // /sdcard/FIRST/paths/
OpmodeCommands.followPath(drive, path);
```

Export contract: `docs/PATH_PLANNER_INTEGRATION.md`.

### Motion scales

On `robot.robotModel`: `cruise()`, `loaded()`, `precision()` — call before a drive if you want slower/faster cruise for that section.

## 5. Mechanism commands

Use `OpmodeCommands` for intake / lift (see that class).
Compose with Ivy:

- `Commands.instant(...)`
- `Commands.waitUntil(...)`
- `Groups.sequential` / `parallel` / `race`

## 6. Match OpMode skeleton

Extend `CommandAutoOpMode` — it owns robot/follower construction, the init-pose
telemetry loop, the `Scheduler` run loop, and shutdown (motors + vision). You only
provide alliance, an optional start pose, and the command tree:

```java
@Autonomous(name = "My Auto")
public class MyAutoOpMode extends CommandAutoOpMode {
    public static double[] START = FieldCoords.xyz(-24, -63, 0);
    public static double[] GOAL = FieldCoords.xyz(-20, 20, 0);

    @Override
    protected boolean isRed() {
        return true;
    }

    @Override
    protected double[] startPose() {
        return START;
    }

    @Override
    protected Command buildAuto() {
        return Groups.sequential(
                OpmodeCommands.lineTo(drive, GOAL),
                OpmodeCommands.strafeLeft(drive, 1.5)
        );
    }

    @Override
    protected void addRunTelemetry() {
        telemetry.addData("field", FieldCoords.format(robot.getFieldPose()));
        telemetry.addData("busy", drive.isBusy());
    }
}
```

`robot` and `drive` are protected fields set up before `buildAuto()` runs. See
`RedLineScoreAuto` for a full example (parallel groups, ball collection, telemetry).

Only reach for a raw `LinearOpMode` if an auto genuinely doesn't fit the
build→run→cleanup shape (e.g. `DriveForwardOpMode`, which drives the raw
`PathFollower` for a SysId-style demo).

## 7. BrainSTEMRobot

- `createPathFollower()` → `PathFollower`
- `setStartPose({x,y,headingDeg})` — absolute Pinpoint stamp (do not call `pinpoint.setStartPose` repeatedly)
- `setAlliance(red)` / `update()` / `reset()`

## 8. Safety

- Do not command Pedro Mecanum and another drivetrain at the same time.
- Cancel the follower and zero motors when an OpMode ends early.
- Tune `RobotModel` (kS/kV/kA, accel limits) before heavy Pedro PID retuning.
- Dynamic speed limits: `docs/DYNAMIC_VELOCITY.md`.
