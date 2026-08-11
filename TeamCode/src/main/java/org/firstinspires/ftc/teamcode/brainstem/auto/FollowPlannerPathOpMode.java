package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.FieldCoords;
import org.firstinspires.ftc.teamcode.brainstem.follower.FollowerOutput;
import org.firstinspires.ftc.teamcode.brainstem.follower.PathFollower;
import org.firstinspires.ftc.teamcode.brainstem.follower.PathSpec;

@Disabled
@Autonomous(name = "Follow Planner Path", group = "Pedro")
public class FollowPlannerPathOpMode extends LinearOpMode {

    public static final boolean USE_RC_FILE = false;
    public static final String RC_FILE_NAME = "sample_collect_path.json";

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        PathFollower drive = robot.createPathFollower();

        PathSpec path = USE_RC_FILE
                ? PathSpec.fromFile(RC_FILE_NAME)
                : PathSpec.fromRaw(hardwareMap.appContext.getResources(), R.raw.sample_collect_path);

        telemetry.addLine("path: " + path.name + " (" + path.segments.size() + " segs)");
        telemetry.addLine("Place robot at first waypoint before START");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        PathSpec.Waypoint start = path.segments.get(0).controlPoints.get(0);
        double heading = start.headingDegrees != null ? start.headingDegrees : 0;
        robot.setStartPose(FieldCoords.xyz(start.x, start.y, heading));

        drive.startPath(path);
        while (opModeIsActive() && !drive.isFinished()) {
            robot.update();
            FollowerOutput out = drive.update();
            telemetry.addData("field", FieldCoords.format(drive.getFieldPose()));
            telemetry.addData("pathDone", "%.2f", out.pathCompletion);
            telemetry.addData("vLimit", "%.1f %s", out.velocityLimit, out.curvatureLimitReason);
            telemetry.update();
        }
        drive.cancel();
    }
}
