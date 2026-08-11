package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.FieldCoords;
import org.firstinspires.ftc.teamcode.brainstem.logging.Logger;

@TeleOp(name = "Logger Demo", group = "Test")
public class LoggerDemoOpMode extends LinearOpMode {

    private final Logger logger = new Logger();

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);

        int pathBusyCol = logger.registerExtra("path_busy");

        logger.start("LoggerDemo");

        telemetry.addLine("Logging to /sdcard/FIRST/logs/");
        telemetry.addData("file", logger.getLogFile() != null ? logger.getLogFile().getName() : "?");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            logger.stop();
            robot.stopVision();
            return;
        }

        ElapsedTime loopTimer = new ElapsedTime();
        while (opModeIsActive()) {
            loopTimer.reset();

            robot.update();

            robot.drive.driveFromSticks(
                    gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            double loopMs = loopTimer.milliseconds();
            org.firstinspires.ftc.teamcode.brainstem.logging.LogEntry entry = logger.borrow();
            if (entry != null) {
                Logger.fillFromRobot(entry, robot, loopMs);
                entry.setExtra(pathBusyCol, 0);
                logger.update(entry);
            }

            logger.addTelemetry(telemetry);
            telemetry.addData("field", FieldCoords.format(robot.getFieldPose()));
            telemetry.addData("loop ms", "%.1f", loopMs);
            telemetry.update();
        }

        logger.stop();
        robot.drive.setMotorPowers(0, 0, 0, 0);
        robot.stopVision();
    }
}
