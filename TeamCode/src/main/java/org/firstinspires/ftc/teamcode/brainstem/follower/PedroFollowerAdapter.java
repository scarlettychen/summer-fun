package org.firstinspires.ftc.teamcode.brainstem.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.brainstem.RoadRunnerCoordinates;
import org.firstinspires.ftc.teamcode.brainstem.RobotModel;
import org.firstinspires.ftc.teamcode.brainstem.utils.SafeSensor;

final class PedroFollowerAdapter implements PathFollower {

    private final Follower pedroFollower;
    private final RobotModel robotModel;
    private final SafeSensor<Pose> safePose;
    private boolean manualDrive;
    private PathSpec activeSpec;

    PedroFollowerAdapter(Follower pedroFollower, RobotModel robotModel) {
        this.pedroFollower = pedroFollower;
        this.robotModel = robotModel;
        Pose initial = pedroFollower.getPose();
        this.safePose = new SafeSensor<>(
                "pedroPose",
                pedroFollower::getPose,
                initial != null ? initial : new Pose());
    }

    @Override
    public void startPath(PathSpec spec) {
        manualDrive = false;
        activeSpec = spec;
        PathChain chain = PathSpecConverter.toPedroPathChain(spec, pedroFollower);
        pedroFollower.followPath(chain, false);
    }

    @Override
    public FollowerOutput update() {
        safePose.read();

        Path currentPath = pedroFollower.getCurrentPath();
        double curvature = currentPath != null ? currentPath.getClosestPointCurvature() : 0;

        VelocityConstraint.Result limit =
                VelocityConstraint.getMaxVelocity(curvature, robotModel, currentSegmentMaxVelocity());

        if (robotModel != null) {
            if (!manualDrive && pedroFollower.isBusy()) {
                robotModel.setPathVelocityCeiling(limit.maxVelocity);
            } else {
                robotModel.clearPathVelocityCeiling();
            }
        }

        pedroFollower.update();

        Vector translational = pedroFollower.getTranslationalError();
        return new FollowerOutput(
                pedroFollower.getPathCompletion(),
                translational != null ? translational.getMagnitude() : 0,
                limit.maxVelocity,
                limit.reason.name());
    }

    private double currentSegmentMaxVelocity() {
        if (activeSpec == null || activeSpec.segments.isEmpty()) {
            return 0;
        }
        int lastIdx = activeSpec.segments.size() - 1;
        int idx = Math.max(0, Math.min(lastIdx, (int) Math.round(pedroFollower.getCurrentPathNumber())));
        return activeSpec.segments.get(idx).maxVelocity;
    }

    @Override
    public boolean isFinished() {
        return !manualDrive && !pedroFollower.isBusy();
    }

    @Override
    public void cancel() {
        manualDrive = false;
        activeSpec = null;
        clearVelocityCeiling();
        pedroFollower.breakFollowing();
    }

    @Override
    public double[] getFieldPose() {
        Pose field = safePose.read().getAsCoordinateSystem(RoadRunnerCoordinates.INSTANCE);
        return new double[]{field.getX(), field.getY(), Math.toDegrees(field.getHeading())};
    }

    @Override
    public void startManualDrive() {
        manualDrive = true;
        activeSpec = null;
        clearVelocityCeiling();
        pedroFollower.startTeleopDrive();
    }

    @Override
    public void setManualDrive(double forward, double strafe, double turn) {
        pedroFollower.setTeleOpDrive(forward, strafe, turn, true);
    }

    private void clearVelocityCeiling() {
        if (robotModel != null) {
            robotModel.clearPathVelocityCeiling();
        }
    }
}
