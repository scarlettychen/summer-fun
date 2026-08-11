# Verified commands — a build guide

Package: `brainstem/utils/verificationSystem/`

## The problem this solves

A normal Ivy `Command` (everything in `OpmodeCommands`) assumes that if the
code ran, the outcome happened: `intake.setPower(1)` → "intake is running."
On a real robot that's not always true — a ball can jam, a lift can stall
against a hard stop, a servo can be disconnected. Nothing in a plain Command
checks whether the *physical* result actually occurred.

`VerifiedCommand` wraps a normal action with a loop: run it, check with a
sensor whether it actually worked, and if not, try a corrective step and
retry — a bounded number of times, not forever. That's the whole system.

## The flow

```
VerifiedCommand
      │
      ├── action.start(), then action.execute() every tick
      │   until action.isFinished() → action.end(false)
      │
      ↓
   Verifier                  ← checked every tick
      │
      ├── SUCCESS ─────────→ ActionResult.success(...)   [done]
      │
      ├── UNCERTAIN ───────→ (do nothing, check again next tick)
      │
      └── FAILED
             ↓
       FailureContext        ← built once per failed attempt
             │
             ├──────────────→ FailureLogger.log(...)
             ↓
       RecoveryAction runs to completion (skip if none configured)
             ↓
       RetryPolicy.shouldRetry(...)
        │                  │
       true                false
        │                  │
        └─→ action.start() ↓
            again        ActionResult.failure(...)  [done]
```

Nothing here talks to anything two steps away — `Verifier` never sees
`RetryPolicy`, `RecoveryAction` never sees `Verifier`. `VerifiedCommand` is
the only class that knows the whole sequence. That's deliberate: it's what
lets you swap any one piece without touching the others.

## What's in the package, and why nothing more is

Nine small files, each one thing:

| File | Role |
|---|---|
| `Action` | Neutral `start/execute/isFinished/end` contract — what gets verified. |
| `Verifier` | Checks the physical outcome; returns SUCCESS / FAILED / UNCERTAIN. |
| `VerificationResult` | The answer type `Verifier` returns. |
| `RecoveryAction` | One corrective step to run after a failure (optional — pass null for none). |
| `RetryPolicy` | Yes/no: try again after this failure? |
| `FailureLogger` | One log call per failed attempt (optional). |
| `ExecutionContext` | Attempt number + elapsed time for the current run. |
| `FailureContext` | What failed, on which attempt. |
| `ActionResult` | Final outcome handed back to the caller. |
| `VerifiedCommand` | The orchestrator — the only class that ties the rest together. |

Plus ready-to-use defaults so a first `VerifiedCommand` doesn't require
writing anything new:

- **`SimpleVerificationResult`** — the obvious `VerificationResult`: just a
  `VerificationType`. Use the static constants `SUCCESS` / `FAILED` /
  `UNCERTAIN` from a `Verifier`.
- **`MaxAttemptsRetryPolicy`** — retry up to N times, no more to configure.
- **`NoOpRecoveryAction`** — does nothing, finishes immediately; pass this
  (or just `null`) when no corrective step is needed.
- **`TelemetryFailureLogger`** — one `telemetry.addData(...)` line per
  failed attempt.

A few things that earlier drafts of this system had and were cut, on the
principle that this package should have exactly one way to do each job:

- **A `RecoveryPolicy` that picks between several `RecoveryAction`s.**
  `RecoveryAction.begin(context)` already receives the `ExecutionContext`,
  which already has `attemptNumber()` — that's enough for a single
  `RecoveryAction` to behave differently per attempt itself (see the worked
  example below). A separate policy object choosing between action objects
  was a second way to express the same branch.
- **A retry-delay knob on `RetryPolicy`.** A delay before retrying is just a
  `RecoveryAction` that waits a few ticks before `isFinished()` returns
  true — no need for a second delay mechanism on the retry decision itself.
- **A generic `target`/blackboard on `ExecutionContext`.** Every
  Action/Verifier/RecoveryAction you write is a closure built at the call
  site where you already hold the subsystem reference (see the worked
  example) — there's nothing to hand back that you don't already have.
- **`CommandAdapter`, a framework-picker facade.** It existed to prepare for
  FTCLib support that doesn't exist yet — this SDK depends on exactly one
  command framework (Ivy) today. Call `IvyCommandAction`/`IvyCommandAdapter`
  directly; if FTCLib is ever added, write its mirror-image pair and call
  that directly too. A facade for a choice that's never actually made at
  runtime is pure indirection.

`VerifiedCommandExamples.runSelfCheck()` exercises the whole state machine
(immediate success, uncertain-then-success, fail → recover → retry →
succeed, and exhausting retries) with synthetic actions/verifiers — no
hardware required. Run it from the Driver Station via the **"Verified
Command Self-Check"** TeleOp
(`teleop/testing/VerifiedCommandSelfCheckOpMode.java`), or read it directly
as a reference for how the pieces fit together before wiring up a real one.

## Building a `VerifiedCommand`

Either the plain constructor or the fluent `Builder` works; `Builder` reads
a little better when several fields are being set at once:

```java
VerifiedCommand raiseLift = VerifiedCommand.builder()
        .action(action)                                 // your Action — see below
        .verifier(context -> lift.atTarget()
                ? SimpleVerificationResult.SUCCESS
                : SimpleVerificationResult.UNCERTAIN)
        .retryPolicy(new MaxAttemptsRetryPolicy(3))
        .recovery(NoOpRecoveryAction.INSTANCE)
        .failureLogger(new TelemetryFailureLogger(telemetry, "lift"))
        .build();
```

`recovery` and `failureLogger` are both optional — omit them (or pass
`null` via the plain 3-arg constructor) for a bare execute/verify/retry loop
with no corrective step and no logging.

## Worked example (to ground the abstractions)

Concrete case: raising `FourBarLinkage` to `SCORE_HIGH` and confirming it
actually got there, instead of trusting that `setState()` was called.

```java
Action raise = new Action() {
    @Override public void start() { lift.setState(LinkState.SCORE_HIGH); }
    @Override public void execute() { }
    @Override public boolean isFinished() { return true; } // instant — PID lives in lift.update()
    @Override public void end(boolean interrupted) { }
};

Verifier atHighPosition = context -> {
    if (lift.atTarget()) return SimpleVerificationResult.SUCCESS;
    return context.elapsedMillis() > 1500
            ? SimpleVerificationResult.FAILED
            : SimpleVerificationResult.UNCERTAIN;
};

RecoveryAction breakStaticFriction = new RecoveryAction() {
    private long startedAt;
    @Override public void begin(ExecutionContext context) {
        startedAt = System.currentTimeMillis();
        if (context.attemptNumber() > 1) {
            lift.nudge(); // only bother on the 2nd+ failure — 1st may just need a beat
        }
    }
    @Override public void update() { }
    @Override public boolean isFinished() { return System.currentTimeMillis() - startedAt > 150; }
};

VerifiedCommand raiseLift = VerifiedCommand.builder()
        .action(raise)
        .verifier(atHighPosition)
        .retryPolicy(new MaxAttemptsRetryPolicy(3))
        .recovery(breakStaticFriction)
        .failureLogger(new TelemetryFailureLogger(telemetry, "lift"))
        .build();
```

- **Action**: instant — `start()` calls `lift.setState(...)`, `isFinished()`
  is `true` immediately since the PID loop lives in `FourBarLinkage.update()`,
  called every robot loop regardless. No need for `IvyCommandAction` here
  since it's not already a Command.
- **Verifier**: `lift.atTarget()` → SUCCESS; otherwise UNCERTAIN until 1500ms
  since attempt start (`ExecutionContext.elapsedMillis()`), then FAILED. That
  timeout logic lives inside the Verifier, not in `VerifiedCommand`.
- **RecoveryAction**: a single action that checks `context.attemptNumber()`
  itself to escalate — 1st failure just waits a beat, 2nd+ actually nudges.
- **RetryPolicy**: `MaxAttemptsRetryPolicy(3)` — 3 attempts total.
- **FailureLogger**: `TelemetryFailureLogger` for match-time visibility; pipe
  into `brainstem/logging/Logger` too if you want it on record after the
  match (register an extra column via `Logger.registerExtra(...)` and fill
  it from the latest `ActionResult`/`FailureContext`).

Same pattern applies to `IntakeBeamBreak` (verify `getBallCount()`
incremented after starting the intake) or any other sensor-backed subsystem.

## Framework interop: Ivy (and, eventually, FTCLib)

`VerifiedCommand` does not implement Ivy's `Command` directly. It implements
the neutral `Action` interface instead — `start()` / `execute()` /
`isFinished()` / `end(interrupted)` — and two small adapter classes cross
the boundary in each direction:

| `Action` (neutral) | Ivy `Command` | FTCLib `Command` |
|---|---|---|
| `start()` | `start()` | `initialize()` |
| `execute()` | `execute()` | `execute()` |
| `isFinished()` | `done()` | `isFinished()` |
| `end(boolean interrupted)` | `end(EndCondition)` | `end(boolean interrupted)` |

- **`IvyCommandAction implements Action`** — wraps an existing Ivy `Command`
  so it can be passed in as the thing a `VerifiedCommand` verifies. Use this
  when the action you want checked is already written as a normal command
  (a whole path-following segment, a `Commands.instant(...)`, etc.) instead
  of a one-shot setter.
- **`IvyCommandAdapter implements Command`** — wraps any `Action` (normally
  a finished `VerifiedCommand`) so it can be scheduled with
  `Scheduler`/`Groups.sequential/parallel/race` right next to ordinary
  commands in `OpmodeCommands`:

```java
Command cmd = new IvyCommandAdapter(verifiedCommand, lift); // lift = requirement
Scheduler.schedule(cmd);
```

Because requirements/priority/interrupt-behavior are Ivy-specific scheduling
concerns, they live on `IvyCommandAdapter`'s constructor, not on `Action`
itself — keeps the neutral contract genuinely neutral.

This SDK depends on exactly one command framework today, so that's the only
adapter pair that exists. If/when FTCLib is added, write
`FtcLibCommandAction`/`FtcLibCommandAdapter` as its mirror image (the table
above makes that close to a copy-paste) and call those directly at whatever
call sites need FTCLib — no framework-picking facade needed for a choice
that's fixed at each call site anyway.

## Fitting it into autos

Wrap the finished `VerifiedCommand` in `new IvyCommandAdapter(verifiedCommand,
lift)` (pass whatever `Component`s it drives as requirements) and it drops
straight into existing `Groups.sequential(...)` / `parallel(...)` trees in
`OpmodeCommands` next to ordinary commands — no separate scheduler, no new
run loop. Keep a reference to the `VerifiedCommand` itself (not just the
adapter) so the caller can read `verifiedCommand.getResult()` once it's
finished, to branch on success/failure — e.g. skip a dependent step, or just
log it and move on. `Action`/`Command` have no return value, so this getter
is the only way the caller finds out what happened.

## Guardrails worth building in from the start

- **Bound UNCERTAIN.** An unresolved UNCERTAIN loops forever unless the
  Verifier itself times out into FAILED. `VerifiedCommand` should not need to
  know about timeouts — that's the Verifier's job.
- **Bound retries.** `RetryPolicy` must eventually return `false`, or a
  jammed mechanism retries for the rest of the match.
- **Idempotent actions.** Whatever action `VerifiedCommand` re-runs on retry
  must be safe to call more than once (setting a target state is; spawning a
  new subsystem object is not).
- **Non-blocking everywhere.** Same rule as the rest of this codebase
  (vision, logging): no `Thread.sleep`, no blocking I/O, inside `execute()`,
  `Verifier.verify()`, or `RecoveryAction.update()`. Everything ticks once
  per robot loop.
