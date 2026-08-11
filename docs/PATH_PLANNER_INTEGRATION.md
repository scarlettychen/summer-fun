# Path planner UI integration

BrainSTEM does **not** consume trapezoidal time-sampled trajectories from a web
simulator. The robot follows geometry + dynamic velocity limits
(`VelocityConstraint` + `RobotModel`). UI planners should export **path geometry**,
not a timed state array.

```
AutoBuilder / simulator
        │  JSON (waypoints or segments)
        ▼
PathSpec.parse(...)          ← TeamCode
        ▼
PathFollower.startPath(...)  ← Pedro behind adapter
```

See also `docs/DYNAMIC_VELOCITY.md` and `brainstem/AUTONOMOUS_GUIDE.md` §10.

---

## What the UI should export

Prefer one of:

1. **Waypoint document** (matches AutoBuilder-style editors) — easiest for canvas UIs  
2. **Native PathSpec** (`segments` + Bezier `controlPoints`) — closest to Pedro

Both are accepted by `PathSpec.parse(String)` / `PathPlannerImport.parse(String)`.

Do **not** require `trajectory.states` (time, x, y, heading, velocity) on the robot.
Use that array only inside the browser simulator (`AutoSimulator`). On export, map
waypoints → PathSpec JSON — that's the only format the robot loads.

---

## Coordinate system (required)

BrainSTEM `FieldCoords`:

| Property | Value |
|----------|--------|
| Origin | Field **center** |
| Units | **Inches** (planner may emit meters — set `"units"`) |
| Walls | ±72 in |
| Heading 0° | +Y (into the field from the −Y wall) |
| Heading sense | Degrees, **counter-clockwise** |

Planner document fields:

```json
"units": "inches" | "meters" | "cm",
"origin": "center" | "corner"
```

- `"origin": "corner"` treats (0,0) as the −X/−Y corner and converts to center origin.
- Speeds (`maxLinearSpeed` / `maxVelocity`) use the same length unit per second.

---

## Waypoint document (`brainstem.path.v1`)

```json
{
  "format": "brainstem.path.v1",
  "name": "collect-cycle",
  "units": "inches",
  "origin": "center",
  "waypoints": [
    {
      "x": -12,
      "y": -58,
      "headingDegrees": -90,
      "outgoing": { "x": -12, "y": -40 },
      "maxLinearSpeed": 0,
      "headingMode": "HOLD",
      "passPosition": true
    },
    {
      "x": -39,
      "y": -39,
      "headingDegrees": -137,
      "incoming": { "x": -28, "y": -45 },
      "maxLinearSpeed": 40,
      "headingMode": "HOLD"
    }
  ]
}
```

### Field mapping from typical AutoBuilder params

| UI field | PathSpec / import |
|----------|-------------------|
| `x`, `y` | Waypoint position (converted via `units` / `origin`) |
| `heading` / `headingDegrees` | Hold heading when `headingMode` is `HOLD` |
| `controlNext` / `outgoing` | Cubic out handle (absolute, or `{relative:true,x,y}` / `dx`/`dy`) |
| `controlPrev` / `incoming` | Cubic in handle |
| `maxLinearSpeed` / `maxVelocity` | Segment cap in/s (`0` = dynamic only) |
| `passPosition` / `passThrough` | Informational — PathChain already continues without stopping |
| `headingMode` | `HOLD_START` \| `HOLD` \| `TANGENT` |
| `tangentHeading` / `followTangent` | Forces `TANGENT` |

Segment construction between waypoints A → B:

- Both handles → cubic `[A, A.out, B.in, B]`
- One handle → quadratic `[A, handle, B]`
- None → line `[A, B]`

`maxVelocity` on the segment = B's speed, else A's, else `0`.

---

## Native PathSpec JSON

```json
{
  "format": "brainstem.path.v1",
  "name": "line-demo",
  "units": "inches",
  "origin": "center",
  "segments": [
    {
      "headingMode": "HOLD",
      "maxVelocity": 0,
      "controlPoints": [
        { "x": 0, "y": 0, "headingDegrees": 0 },
        { "x": 24, "y": 0, "headingDegrees": 0 }
      ]
    }
  ]
}
```

2 control points → Pedro `BezierLine`; 3+ → `BezierCurve`.

---

## Where to plug this into an AutoBuilder UI

| UI concern | Do this |
|------------|---------|
| Call site instead of `generateTrajectory` for **robot export** | Build waypoint JSON (or native segments) and download / copy; keep `generateTrajectory` for **simulator preview only** |
| `ftcOpmodeGenerator.js` | Emit `PathSpec.parse("...")` or write `.json` for `res/raw` / `/sdcard/FIRST/paths/` — not a custom Pedro PathBuilder dump |
| Canvas → meters | Convert in the exporter (`units: "meters"`) or convert to inches before export; robot always follows inches |
| Alliance mirror | Mirror in the UI (`mirrorWaypointForRed` etc.), then export already-mirrored FieldCoords |

### Robot-side loaders

```java
PathSpec path = PathSpec.fromRaw(resources, R.raw.sample_collect_path);
PathSpec path = PathSpec.fromFile("my_auto.json"); // /sdcard/FIRST/paths/
Command cmd = OpmodeCommands.followPath(drive, path);
```

Sample OpMode: `brainstem/auto/FollowPlannerPathOpMode.java` (starts `@Disabled`).

---

## What the UI should *not* send to the robot

- Time-sampled `states[]` as the path of record  
- Pedro-specific Java (`PathBuilder`, `BezierCurve`, `Follower`) — keep Pedro inside `follower/`  
- Road Runner / corner-origin numbers without `"origin": "corner"`  

Robot timing comes from feedforward + `VelocityConstraint`, not from the planner's trapezoid.
