package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.brainstem.FieldCoords;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Limelight;

@Configurable
@Autonomous(name = "Red Line Score", group = "Auto")
public class RedLineScoreAuto extends CommandAutoOpMode {

    public static double[] START = FieldCoords.xyz(-24, -72 + 9, 0);
    public static double[] GOAL = FieldCoords.xyz(-20, 20, 0);
    public static double[] BALLS = FieldCoords.xyz(-12, -12, 180);
    public static double STRAFE_LEFT_IN = 1.5;

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
                Groups.parallel(
                        OpmodeCommands.lineTo(drive, GOAL),
                        OpmodeCommands.setLiftHigh(robot.lift)
                ),
                OpmodeCommands.strafeLeft(drive, STRAFE_LEFT_IN),
                OpmodeCommands.reverseIntake(robot.intake),
                Groups.parallel(
                        OpmodeCommands.driveTo(drive, BALLS),
                        OpmodeCommands.resetAndCollect(robot.intake, robot.lift)
                ),
                OpmodeCommands.collectBallsThenBackOff(
                        drive, robot.limelight, robot.intake, Limelight.COLLECT_BACK_OFF_IN, 10)
        );
    }

    @Override
    protected void addInitTelemetry() {
        telemetry.addLine("Red Line Score");
        telemetry.addLine("FieldCoords: 0°=+Y (into field)  CCW+  walls±72");
        telemetry.addData("start", FieldCoords.format(START));
        telemetry.addData("goal", FieldCoords.format(GOAL));
        telemetry.addData("balls", FieldCoords.format(BALLS));
        telemetry.addData("strafe left in", STRAFE_LEFT_IN);
        telemetry.addData("field now", FieldCoords.format(robot.getFieldPose()));
    }

    @Override
    protected void addRunTelemetry() {
        telemetry.addData("field", FieldCoords.format(robot.getFieldPose()));
        telemetry.addData("balls tgt", FieldCoords.format(BALLS));
        telemetry.addData("lift", "%s pos=%d atTarget=%s",
                robot.lift.getState(), robot.lift.getPosition(), robot.lift.atTarget());
        telemetry.addData("busy", drive.isBusy());
    }
}
