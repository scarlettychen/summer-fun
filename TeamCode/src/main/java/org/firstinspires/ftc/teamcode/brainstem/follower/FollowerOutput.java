package org.firstinspires.ftc.teamcode.brainstem.follower;

public final class FollowerOutput {

    public final double pathCompletion;
    public final double crossTrackError;
    public final double velocityLimit;
    public final String curvatureLimitReason;

    public FollowerOutput(
            double pathCompletion,
            double crossTrackError,
            double velocityLimit,
            String curvatureLimitReason) {
        this.pathCompletion = pathCompletion;
        this.crossTrackError = crossTrackError;
        this.velocityLimit = velocityLimit;
        this.curvatureLimitReason =
                curvatureLimitReason == null ? VelocityConstraint.LimitReason.NONE.name() : curvatureLimitReason;
    }
}
