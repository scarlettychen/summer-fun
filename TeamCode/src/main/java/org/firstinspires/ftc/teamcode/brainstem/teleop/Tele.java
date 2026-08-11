package org.firstinspires.ftc.teamcode.brainstem.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.FieldCoords;
import org.firstinspires.ftc.teamcode.brainstem.auto.OpmodeCommands;
import org.firstinspires.ftc.teamcode.brainstem.follower.PathFollower;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.FourBarLinkage;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Intake;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Drive;
import org.firstinspires.ftc.teamcode.brainstem.utils.AntiTipping;
import org.firstinspires.ftc.teamcode.brainstem.utils.GamepadTracker;

@Configurable
public abstract class Tele extends LinearOpMode {

    public static double SMART_COLLECT_TIMEOUT_S = 15.0;
    public static double[] START = {-24, -63, 0};
    public static double TRIGGER_DEADBAND = 0.3;

    public static boolean ANTI_TIP_ENABLED = true;
    public static double ANTI_TIP_KP = 0.12;
    public static double ANTI_TIP_THRESHOLD_DEG = 12.0;

    public static double ANTI_TIP_EXIT_DEG = 4.0;
    public static double ANTI_TIP_MAX_POWER = 0.9;

    public static double ANTI_TIP_MIN_POWER = 0.4;

    private enum TriggerFlow { OFF, IN, OUT }

    private final boolean red;

    protected BrainSTEMRobot robot;
    protected PathFollower pathFollower;
    protected GamepadTracker gp1;
    protected GamepadTracker gp2;
    protected AntiTipping antiTip;

    private Command active;
    private boolean suppressStickDrive;

    private TriggerFlow lastTriggerFlow;

    protected Tele(boolean red) {
        this.red = red;
    }

    protected void run(Command command) {
        run(command, false);
    }

    protected void run(Command command, boolean takesDrive) {
        cancelCommand();
        if (command == null) {
            return;
        }
        active = command;
        suppressStickDrive = takesDrive;
        Scheduler.schedule(command);
    }

    protected void cancelCommand() {
        if (active != null) {
            Scheduler.cancel(active);
            active = null;
        }
        suppressStickDrive = false;
        Scheduler.reset();
    }

    private boolean commandRunning() {
        return active != null && Scheduler.isScheduled(active);
    }

    @Override
    public void runOpMode() {
        robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        robot.setAlliance(red);
        robot.setStartPose(START);

        pathFollower = robot.createPathFollower();

        GoBildaPinpointDriver odo = robot.pinpoint.getPinpoint();
        antiTip = new AntiTipping(
                () -> odo.getPitch(AngleUnit.DEGREES),
                () -> odo.getRoll(AngleUnit.DEGREES),
                ANTI_TIP_KP,
                ANTI_TIP_THRESHOLD_DEG,
                ANTI_TIP_EXIT_DEG,
                ANTI_TIP_MAX_POWER,
                ANTI_TIP_MIN_POWER);

        robot.update();
        antiTip.tare();

        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);

        telemetry.addLine("Tele");
        telemetry.addData("alliance", red ? "RED" : "BLUE");
        telemetry.addData("start", "(%.0f, %.0f, %.0f°)", START[0], START[1], START[2]);
        telemetry.addLine("Y smart collect | X cancel | RT intake | LT extake | RB score | LB eject");
        telemetry.addData("anti-tip", ANTI_TIP_ENABLED ? "ON (tared)" : "off");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.setStartPose(START);
        robot.update();
        if (antiTip != null) {
            antiTip.tare();
        }
        lastTriggerFlow = null;

        while (opModeIsActive()) {
            robot.update();
            gp1.update();
            gp2.update();

            if (gp1.isFirstRightBumper()) {
                lastTriggerFlow = null;
                if (robot.lift.getState() == FourBarLinkage.LinkState.SCORE_HIGH) {
                    run(OpmodeCommands.resetAll(robot.intake, robot.lift));
                } else {
                    run(OpmodeCommands.raiseAndScoreHigh(robot.intake, robot.lift));
                }
            }

            if (gp1.isFirstY()) {
                lastTriggerFlow = null;
                run(OpmodeCommands.smartCollect(
                        pathFollower,
                        robot.limelight,
                        robot.intake,
                        SMART_COLLECT_TIMEOUT_S), true);
            }

            if (gp1.isFirstX()) {
                lastTriggerFlow = null;
                cancelCommand();
                pathFollower.cancel();
                robot.drive.setMotorPowers(0, 0, 0, 0);
            }

            if (gp1.isFirstB() || gamepad2.bWasPressed()) {
                lastTriggerFlow = null;
                run(OpmodeCommands.resetAll(robot.intake, robot.lift));
            }

            if (gp1.isFirstLeftBumper()) {
                lastTriggerFlow = null;
                run(OpmodeCommands.reverseIntake(robot.intake));
            }

            if (gp1.isFirstDpadDown()) {
                lastTriggerFlow = null;
                run(OpmodeCommands.raiseAndScoreLow(robot.intake, robot.lift));
            }
            if (gp1.isFirstDpadUp()) {
                lastTriggerFlow = null;
                run(OpmodeCommands.raiseAndScoreHigh(robot.intake, robot.lift));
            }

            if (!commandRunning()) {
                TriggerFlow want;
                if (gamepad1.right_trigger > TRIGGER_DEADBAND) {
                    want = TriggerFlow.IN;
                } else if (gamepad1.left_trigger > TRIGGER_DEADBAND || gamepad2.aWasPressed()) {
                    want = TriggerFlow.OUT;
                } else {
                    want = TriggerFlow.OFF;
                }
                if (want != lastTriggerFlow) {
                    boolean releaseToOff = want == TriggerFlow.OFF
                            && (lastTriggerFlow == TriggerFlow.IN || lastTriggerFlow == TriggerFlow.OUT);
                    boolean applyOn = want == TriggerFlow.IN || want == TriggerFlow.OUT;
                    if (applyOn || releaseToOff) {
                        lastTriggerFlow = want;
                        Command flow;
                        switch (want) {
                            case IN:
                                flow = OpmodeCommands.setIntake(robot.intake, Intake.IntakeState.IN);
                                break;
                            case OUT:
                                flow = OpmodeCommands.setIntake(robot.intake, Intake.IntakeState.OUT);
                                break;
                            default:
                                flow = OpmodeCommands.setIntake(robot.intake, Intake.IntakeState.OFF);
                                break;
                        }
                        Scheduler.schedule(flow);
                    } else {

                        lastTriggerFlow = TriggerFlow.OFF;
                    }
                }
            } else {
                lastTriggerFlow = null;
            }

            Scheduler.execute();
            if (!commandRunning()) {
                suppressStickDrive = false;
            }

            AntiTipping.DriveCorrection tip = AntiTipping.DriveCorrection.NONE;
            if (ANTI_TIP_ENABLED && antiTip != null) {
                antiTip.setKP(ANTI_TIP_KP);
                antiTip.setMaxCorrectionPower(ANTI_TIP_MAX_POWER);
                antiTip.setMinCorrectionPower(ANTI_TIP_MIN_POWER);
                antiTip.setTippingThreshold(ANTI_TIP_THRESHOLD_DEG, ANTI_TIP_EXIT_DEG);
                tip = antiTip.calculate();
            }

            if (!suppressStickDrive) {
                if (ANTI_TIP_ENABLED && antiTip != null && antiTip.isTipping()) {

                    robot.drive.driveArcade(tip.forward, tip.strafe, 0);
                } else {
                    robot.drive.driveFromSticks(
                            gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                }
            }

            double[] field = robot.getFieldPose();
            telemetry.addData("alliance", red ? "RED" : "BLUE");
            telemetry.addData("cmd", commandRunning() ? "RUNNING" : "off");

            telemetry.addData("field", FieldCoords.format(field));
            if (ANTI_TIP_ENABLED && antiTip != null) {
                telemetry.addData("anti-tip", antiTip.isTipping() ? "CORRECTING" : "ok");
                telemetry.addData("tip err p/r", "%.1f° / %.1f°",
                        antiTip.getPitchErrorDegrees(), antiTip.getRollErrorDegrees());
                if (antiTip.isTipping()) {
                    telemetry.addData("tip corr", "f=%.2f s=%.2f", tip.forward, tip.strafe);
                }
            }
            telemetry.update();
        }
        Scheduler.reset();
        cancelCommand();
        pathFollower.cancel();
        robot.drive.setMotorPowers(0, 0, 0, 0);
        robot.stopVision();
    }
}
