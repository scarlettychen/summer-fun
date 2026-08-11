package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.RoadRunnerCoordinates;

@Configurable
@TeleOp(name = "Localization Test", group = "Test")
public class LocalizationTest extends LinearOpMode {

    public static double startX = 0;
    public static double startY = 0;
    public static double startHeadingDeg = 0;

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        robot.setStartPose(new double[]{startX, startY, startHeadingDeg});

        telemetry.addLine("Localization Test");
        telemetry.addLine("0°=+Y (into field) CCW+. forward @ 0° → +y only (x flat)");
        telemetry.addLine("if x AND y move → pod dirs/offsets wrong");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.setStartPose(new double[]{startX, startY, startHeadingDeg});
        robot.update();

        Pose startField = robot.pinpoint.getPose()
                .getAsCoordinateSystem(RoadRunnerCoordinates.INSTANCE);
        double x0 = startField.getX();
        double y0 = startField.getY();

        while (opModeIsActive()) {
            robot.update();

            robot.drive.driveFromSticks(
                    gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            Pose pedro = robot.pinpoint.getPose();
            Pose field = pedro.getAsCoordinateSystem(RoadRunnerCoordinates.INSTANCE);
            Pose pedroView = pedro.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            double dx = field.getX() - x0;
            double dy = field.getY() - y0;

            telemetry.addData("rr x", "%.2f  (d=%+.2f)", field.getX(), dx);
            telemetry.addData("rr y", "%.2f  (d=%+.2f)", field.getY(), dy);
            telemetry.addData("rr heading (deg)", "%.1f", Math.toDegrees(field.getHeading()));
            telemetry.addData("pedro x/y/h", "(%.1f, %.1f, %.0f°)",
                    pedroView.getX(),
                    pedroView.getY(),
                    Math.toDegrees(pedroView.getHeading()));
            telemetry.addLine("---");
            telemetry.addLine("forward only: |dy| should stay small vs |dx|");
            telemetry.addLine("±few inches noise = normal; big curve = offsets/dirs");
            telemetry.update();
        }

        robot.drive.setMotorPowers(0, 0, 0, 0);
    }
}
