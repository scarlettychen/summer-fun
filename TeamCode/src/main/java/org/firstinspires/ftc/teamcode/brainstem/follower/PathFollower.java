package org.firstinspires.ftc.teamcode.brainstem.follower;

public interface PathFollower {

    void startPath(PathSpec spec);

    FollowerOutput update();

    boolean isFinished();

    default boolean isBusy() {
        return !isFinished();
    }

    void cancel();

    double[] getFieldPose();

    void startManualDrive();

    void setManualDrive(double forward, double strafe, double turn);
}
