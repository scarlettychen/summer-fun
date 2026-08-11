# Async robot CSV logger

`brainstem.logging.Logger` records match state to `/sdcard/FIRST/logs/*.csv`
on a **background thread**. The robot loop only fills a pooled `LogEntry` and
returns — never waits on flash.

```
robot thread                    writer thread
────────────                    ─────────────
borrow LogEntry
fill fields
offer → queue  ──────────────►  drain batch
                                 append CSV
                                 flush ~250ms
                                 return to pool
```

If the queue is full, the sample is **dropped** (counted) so control stays real-time.

## OpMode usage

```java
Logger logger = new Logger();
logger.start("Tele");

while (opModeIsActive()) {
    double t0 = time;
    robot.update();
    // … control …
    logger.update(robot, (time - t0) * 1000.0);  // non-blocking
}

logger.stop();  // flush + close
```

## Extending the schema

1. Add a `public double` on `LogEntry`
2. Add the name to `LogEntry.CSV_HEADER`
3. Append it in `LogEntry.appendCsv`
4. Clear it in `LogEntry.reset`
5. Fill it in `Logger.fillFromRobot` (or manually after `borrow()`)

Match-only columns without a rebuild of the core schema:

```java
int i = logger.registerExtra("my_signal"); // before start()
entry.setExtra(i, value);
```

## Analyze in Python

```python
import pandas as pd
df = pd.read_csv("Tele_1710000000000.csv")
df.plot(x="timestamp_ms", y=["x_in", "y_in", "battery_v"])
```

See `LoggerDemoOpMode` for a full example. Turret columns are `NaN` until a turret subsystem exists.
