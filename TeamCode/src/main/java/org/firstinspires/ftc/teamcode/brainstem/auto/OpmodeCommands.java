package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.brainstem.follower.PathFollower;
import org.firstinspires.ftc.teamcode.brainstem.follower.PathSpec;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.FourBarLinkage;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Intake;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Limelight;

public final class OpmodeCommands {
    private OpmodeCommands() {}

    public static Command turnOnIntake(Intake intake, FourBarLinkage lift) {

        if (lift.state == FourBarLinkage.LinkState.DOWN){
            return setIntake(intake, Intake.IntakeState.IN);
        } else {
            return setIntake(intake, Intake.IntakeState.OFF);
        }

    }
    public static Command turnOnIntakeEx(Intake intake) {
        return setIntake(intake, Intake.IntakeState.OUT);
    }

    public static Command turnOffIntake(Intake intake) {
        return setIntake(intake, Intake.IntakeState.OFF);
    }

    public static Command reverseIntake(Intake intake) {
        return setIntake(intake, Intake.IntakeState.OUT);
    }

    public static Command setIntake(Intake intake, Intake.IntakeState state) {
        return Commands.instant(() -> intake.setIntakeState(state)).requiring(intake);
    }



    public static Command setLiftDown(FourBarLinkage lift) {
        return setLift(lift, FourBarLinkage.LinkState.DOWN);
    }

    public static Command setLiftLow(FourBarLinkage lift) {
        return setLift(lift, FourBarLinkage.LinkState.SCORE_LOW);
    }

    public static Command setLiftHigh(FourBarLinkage lift) {
        return setLift(lift, FourBarLinkage.LinkState.SCORE_HIGH);
    }

    public static Command setLift(FourBarLinkage lift, FourBarLinkage.LinkState state) {
        return Commands.instant(() -> lift.setState(state)).requiring(lift);
    }

    public static Command holdLift(FourBarLinkage lift, FourBarLinkage.LinkState state) {
        return Command.build()
                .setStart(() -> lift.setState(state))
                .setExecute(() -> lift.setState(state))
                .setDone(() -> lift.getState() == state && lift.atTarget())
                .requiring(lift);
    }

    public static Command holdLiftHigh(FourBarLinkage lift) {
        return holdLift(lift, FourBarLinkage.LinkState.SCORE_HIGH);
    }

    public static Command holdLiftLow(FourBarLinkage lift) {
        return holdLift(lift, FourBarLinkage.LinkState.SCORE_LOW);
    }

    public static Command holdLiftDown(FourBarLinkage lift) {
        return holdLift(lift, FourBarLinkage.LinkState.DOWN);
    }

    public static Command waitLiftAtTarget(FourBarLinkage lift) {
        return Commands.waitUntil(lift::atTarget).requiring(lift);
    }

    public static Command waitLiftAtTarget(
            FourBarLinkage lift, FourBarLinkage.LinkState expected) {
        return Commands.waitUntil(
                () -> lift.getState() == expected && lift.atTarget()
        ).requiring(lift);
    }



    public static Command followPath(PathFollower drive, PathSpec spec) {
        return followPath(drive, () -> spec);
    }

    public static Command followPath(PathFollower drive, java.util.function.Supplier<PathSpec> bake) {
        return Command.build()
                .setStart(() -> drive.startPath(bake.get()))
                .setExecute(drive::update)
                .setDone(() -> !drive.isBusy())
                .setEnd(end -> { if (drive.isBusy()) drive.cancel(); })
                .requiring(drive);
    }

    public static Command followPathJson(PathFollower drive, String pathJson) {
        return followPath(drive, PathSpec.parse(pathJson));
    }

    public static Command driveTo(PathFollower drive, double[] fieldPose) {
        return followPath(drive, () -> PathSpec.to(
                drive.getFieldPose(), fieldPose, PathSpec.HeadingMode.HOLD));
    }

    public static Command driveTo(PathFollower drive, double x, double y, double headingDegrees) {
        return driveTo(drive, new double[]{x, y, headingDegrees});
    }

    public static Command lineTo(PathFollower drive, double[] fieldPose) {
        return followPath(drive, () -> PathSpec.to(
                drive.getFieldPose(), fieldPose, PathSpec.HeadingMode.HOLD_START));
    }

    public static Command lineTo(PathFollower drive, double x, double y, double headingDegrees) {
        return lineTo(drive, new double[]{x, y, headingDegrees});
    }

    public static Command driveForward(PathFollower drive, double inches) {
        return followPath(drive, () -> PathSpec.forward(drive.getFieldPose(), inches));
    }

    public static Command driveBack(PathFollower drive, double inches) {
        return driveForward(drive, -Math.abs(inches));
    }

    public static Command strafe(PathFollower drive, double inches) {
        return followPath(drive, () -> PathSpec.strafe(drive.getFieldPose(), inches));
    }

    public static Command strafeLeft(PathFollower drive, double inches) {
        return strafe(drive, Math.abs(inches));
    }

    public static Command strafeRight(PathFollower drive, double inches) {
        return strafe(drive, -Math.abs(inches));
    }

    public static Command collectBallsThenBackOff(
            PathFollower drive, Limelight limelight, Intake intake, double backInches,
            double timeoutSeconds) {
        return Groups.sequential(
                smartCollect(drive, limelight, intake, timeoutSeconds),
                Groups.race(driveBack(drive, backInches), Commands.waitMs(2500))
        );
    }

    public static Command smartCollect(
            PathFollower drive, Limelight limelight, Intake intake, double timeoutSeconds) {
        final BallChaseController chase = new BallChaseController(limelight);
        final long[] startMs = {0};
        final boolean[] finished = {false};
        final double[] lastForward = {0.15};
        final long timeoutMs = Math.max(1L, (long) (timeoutSeconds * 1000.0));

        Command chaseCmd = Command.build()
                .setStart(() -> {
                    drive.startManualDrive();
                    chase.reset();
                    startMs[0] = System.currentTimeMillis();
                    finished[0] = false;
                    lastForward[0] = 0.15;
                    intake.setIntakeState(Intake.IntakeState.IN);
                })
                .setExecute(() -> {
                    if (finished[0]) return;
                    long elapsed = System.currentTimeMillis() - startMs[0];
                    if (elapsed >= timeoutMs) {
                        finished[0] = true;
                        drive.setManualDrive(0, 0, 0);
                        drive.update();
                        return;
                    }
                    Limelight.BallDetection ball = chase.track();
                    if (ball == null) {
                        double creep = Math.max(0.15, Math.min(0.30, lastForward[0]));
                        drive.setManualDrive(creep, 0, 0);
                        drive.update();
                        return;
                    }
                    double range = chase.rangeInches(ball);
                    double tx = ball.txDeg - Limelight.CHASE_TX_OFFSET_DEG;
                    double forward = chase.forwardTo(range);

                    boolean near = !Double.isNaN(range) && range <= Limelight.CHASE_STRAFE_RANGE_IN;
                    double turn;
                    double strafe = 0.0;
                    if (near) {
                        strafe = Math.max(-Limelight.CHASE_MAX_STRAFE,
                                Math.min(Limelight.CHASE_MAX_STRAFE, -Limelight.CHASE_KP_STRAFE * tx));
                        turn = chase.turnTo(tx) * Limelight.CHASE_NEAR_TURN_SCALE;
                        forward = Math.max(forward, Limelight.CHASE_MIN_FORWARD);
                    } else {
                        turn = chase.turnTo(tx);
                        if (Math.abs(tx) < 15.0) {
                            forward = Math.max(forward, Limelight.CHASE_MIN_FORWARD * 0.7);
                        }
                    }
                    lastForward[0] = forward;
                    drive.setManualDrive(forward, strafe, turn);
                    drive.update();
                })
                .setDone(() -> finished[0]
                        || (startMs[0] != 0 && System.currentTimeMillis() - startMs[0] >= timeoutMs))
                .setEnd(end -> {
                    finished[0] = true;
                    drive.setManualDrive(0, 0, 0);
                    drive.update();
                    drive.cancel();
                    intake.setIntakeState(Intake.IntakeState.OFF);
                })
                .requiring(drive, limelight, intake);

        return Groups.race(chaseCmd, Commands.waitMs(timeoutMs));
    }

    public static Command raiseAndScore(
            Intake intake, FourBarLinkage lift, FourBarLinkage.LinkState scoreHeight) {
        return Groups.sequential(
                Commands.instant(() -> {
                    intake.setIntakeState(Intake.IntakeState.OFF);
                    lift.setState(scoreHeight);
                }).requiring(intake, lift),
                waitLiftAtTarget(lift, scoreHeight)
        );
    }

    public static Command raiseAndScoreHigh(Intake intake, FourBarLinkage lift) {
        return raiseAndScore(intake, lift, FourBarLinkage.LinkState.SCORE_HIGH);
    }

    public static Command raiseAndScoreLow(Intake intake, FourBarLinkage lift) {
        return raiseAndScore(intake, lift, FourBarLinkage.LinkState.SCORE_LOW);
    }

    public static Command driveRaiseAndScore(
            PathFollower drive,
            Intake intake,
            FourBarLinkage lift,
            double[] fieldPose,
            FourBarLinkage.LinkState scoreHeight
    ) {
        return Groups.parallel(
                driveTo(drive, fieldPose),
                raiseAndScore(intake, lift, scoreHeight)
        );
    }

    public static Command driveRaiseAndScoreHigh(
            PathFollower drive, Intake intake, FourBarLinkage lift, double[] fieldPose) {
        return driveRaiseAndScore(drive, intake, lift, fieldPose, FourBarLinkage.LinkState.SCORE_HIGH);
    }

    public static Command driveRaiseAndScoreLow(
            PathFollower drive, Intake intake, FourBarLinkage lift, double[] fieldPose) {
        return driveRaiseAndScore(drive, intake, lift, fieldPose, FourBarLinkage.LinkState.SCORE_LOW);
    }

    public static Command resetAll(Intake intake, FourBarLinkage lift) {
        return Commands.instant(() -> {
            intake.setIntakeState(Intake.IntakeState.OFF);
            lift.setState(FourBarLinkage.LinkState.DOWN);
        }).requiring(intake, lift);
    }

    public static Command resetAndCollect(Intake intake, FourBarLinkage lift) {
        return Commands.instant(() -> {
            lift.setState(FourBarLinkage.LinkState.DOWN);
            intake.setIntakeState(Intake.IntakeState.IN);
        }).requiring(intake, lift);
    }
}
