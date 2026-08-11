package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;

@TeleOp(name = "SysId Friction μ", group = "SysId")
public class SysIdFrictionOpMode extends LinearOpMode {

    private static final double G = 386.09;
    private static final double FULL_STICK = 0.85;
    private static final double MIN_DT = 0.008;

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        robot.setStartPose(new double[]{0, 0, 0});

        telemetry.addLine("SysId: Friction coefficient μ");
        telemetry.addLine("STRAFE: full left/right until slip → best for RobotModel");
        telemetry.addLine("Y = cut power + COAST (float) after a forward run");
        telemetry.addLine("A=reset  B=brake stop");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.setStartPose(new double[]{0, 0, 0});
        robot.update();
        setBrake(robot, true);

        ElapsedTime clock = new ElapsedTime();
        double lastT = 0;
        double lastVx = 0;
        double lastVy = 0;
        boolean haveSample = false;
        boolean coasting = false;

        double peakLatAccel = 0;
        double peakCoastDecel = 0;

        while (opModeIsActive()) {
            robot.update();

            if (gamepad1.a) {
                peakLatAccel = 0;
                peakCoastDecel = 0;
                haveSample = false;
                coasting = false;
                setBrake(robot, true);
            }
            if (gamepad1.b) {
                coasting = false;
                setBrake(robot, true);
                robot.drive.setMotorPowers(0, 0, 0, 0);
            }
            if (gamepad1.y) {

                coasting = true;
                setBrake(robot, false);
                robot.drive.setMotorPowers(0, 0, 0, 0);
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x * 0.75;

            if (!coasting) {
                setBrake(robot, true);
                robot.drive.driveArcade(y, x, rx);
            } else {
                robot.drive.setMotorPowers(0, 0, 0, 0);
                double speed = Math.hypot(
                        robot.pinpoint.getVelocity().getX(),
                        robot.pinpoint.getVelocity().getY());
                if (speed < 2.0) {
                    coasting = false;
                    setBrake(robot, true);
                }
            }

            double heading = robot.pinpoint.getPose().getHeading();
            double fieldVx = robot.pinpoint.getVelocity().getX();
            double fieldVy = robot.pinpoint.getVelocity().getY();

            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double robotForward = fieldVx * cos + fieldVy * sin;
            double robotStrafe = -fieldVx * sin + fieldVy * cos;

            double t = clock.seconds();
            double dt = t - lastT;
            if (haveSample && dt >= MIN_DT) {
                double ax = (fieldVx - lastVx) / dt;
                double ay = (fieldVy - lastVy) / dt;

                double lastStrafe = -lastVx * sin + lastVy * cos;
                double aLat = (robotStrafe - lastStrafe) / dt;
                double aMag = Math.hypot(ax, ay);

                boolean strafingHard = Math.abs(x) >= FULL_STICK && Math.abs(y) < 0.35;
                if (strafingHard) {
                    peakLatAccel = Math.max(peakLatAccel, Math.abs(aLat));
                }
                if (coasting && aMag > 0.5) {

                    double speed = Math.hypot(fieldVx, fieldVy);
                    if (speed > 4.0) {
                        peakCoastDecel = Math.max(peakCoastDecel, aMag);
                    }
                }

                lastT = t;
                lastVx = fieldVx;
                lastVy = fieldVy;
            } else if (!haveSample) {
                lastT = t;
                lastVx = fieldVx;
                lastVy = fieldVy;
                haveSample = true;
            }

            double muStrafe = peakLatAccel / G;
            double muCoast = peakCoastDecel / G;

            double suggestedMu = muStrafe > 0.05 ? muStrafe * 0.85 : 0;

            telemetry.addLine("--- live ---");
            telemetry.addData("mode", coasting ? "COAST" : "DRIVE");
            telemetry.addData("fwd / strafe in/s", "%.1f / %.1f", robotForward, robotStrafe);
            telemetry.addData("stick x (strafe)", "%.2f", x);
            telemetry.addLine("--- peaks ---");
            telemetry.addData("peak |a_lat| in/s^2", "%.1f", peakLatAccel);
            telemetry.addData("μ strafe (a/g)", "%.3f  → frictionCoefficient(%.3f)", muStrafe, suggestedMu);
            telemetry.addData("peak coast |a|", "%.1f  (μ≈%.3f)", peakCoastDecel, muCoast);
            telemetry.addData("→ maxLatAccel", "%.1f in/s^2", suggestedMu * G);
            telemetry.addLine("A reset · Y coast · B brake · prefer strafe μ");
            telemetry.update();
        }

        setBrake(robot, true);
        robot.drive.setMotorPowers(0, 0, 0, 0);
    }

    private static void setBrake(BrainSTEMRobot robot, boolean brake) {

        DcMotor.ZeroPowerBehavior behavior =
                brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        try {
            robot.hardwareMap.get(DcMotor.class, "FL").setZeroPowerBehavior(behavior);
            robot.hardwareMap.get(DcMotor.class, "FR").setZeroPowerBehavior(behavior);
            robot.hardwareMap.get(DcMotor.class, "BL").setZeroPowerBehavior(behavior);
            robot.hardwareMap.get(DcMotor.class, "BR").setZeroPowerBehavior(behavior);
        } catch (Exception ignored) {

        }
    }
}
