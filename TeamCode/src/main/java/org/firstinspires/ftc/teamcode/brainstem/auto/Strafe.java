package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.brainstem.FieldCoords;

@Configurable
@Autonomous(name = "wierd auton", group = "Auto")
public class Strafe extends CommandAutoOpMode {
    public static double DRIVE_FORWARD = -86;
    public static double STRAFE_LEFT = 12;

    @Override
    protected boolean isRed() {
        return true;
    }

    @Override
    protected Command buildAuto() {
        return Groups.sequential(
                Groups.parallel(
                        OpmodeCommands.driveForward(drive, DRIVE_FORWARD),
                        OpmodeCommands.raiseAndScoreHigh(robot.intake, robot.lift)
                ),
                OpmodeCommands.strafeRight(drive, STRAFE_LEFT),
                Commands.waitMs(500),
                OpmodeCommands.driveForward(drive, -1.5),
                OpmodeCommands.reverseIntake(robot.intake),
                Commands.waitMs(1500),
                OpmodeCommands.driveForward(drive, 5),
                Commands.waitMs(1500),
                OpmodeCommands.resetAll(robot.intake, robot.lift)
        );
    }

    @Override
    protected void addInitTelemetry() {
        telemetry.addLine("wierd auton");
    }

    @Override
    protected void addRunTelemetry() {
        telemetry.addData("field", FieldCoords.format(robot.getFieldPose()));
        telemetry.addData("lift", "%s pos=%d atTarget=%s",
                robot.lift.getState(), robot.lift.getPosition(), robot.lift.atTarget());
        telemetry.addData("busy", drive.isBusy());
    }
}
