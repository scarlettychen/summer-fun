package org.firstinspires.ftc.teamcode.brainstem.follower;

import org.firstinspires.ftc.teamcode.brainstem.RobotModel;

public final class VelocityConstraint {

    public enum LimitReason {

        ROBOT_TOP_SPEED,

        CURVATURE,

        SEGMENT_CAP,

        NONE
    }

    public static final class Result {
        public final double maxVelocity;
        public final LimitReason reason;
        public final double curvature;
        public final double robotTopSpeed;
        public final double curvatureLimitedSpeed;
        public final double segmentCap;

        public Result(
                double maxVelocity,
                LimitReason reason,
                double curvature,
                double robotTopSpeed,
                double curvatureLimitedSpeed,
                double segmentCap) {
            this.maxVelocity = maxVelocity;
            this.reason = reason;
            this.curvature = curvature;
            this.robotTopSpeed = robotTopSpeed;
            this.curvatureLimitedSpeed = curvatureLimitedSpeed;
            this.segmentCap = segmentCap;
        }
    }

    private static final double CURVATURE_EPS = 1e-6;

    private VelocityConstraint() {}

    public static Result getMaxVelocity(
            double curvature, RobotModel model, double segmentMaxVelocity) {
        if (model == null) {
            return new Result(0, LimitReason.NONE, curvature, 0, Double.POSITIVE_INFINITY, segmentMaxVelocity);
        }

        double robotTop = Math.max(0.0, model.motorLimitedVelocityIgnoringPathCeiling());
        double absK = Math.abs(curvature);
        double curvLimit = Double.POSITIVE_INFINITY;
        if (absK > CURVATURE_EPS) {
            double aLatMax = Math.max(model.getMaxLateralAcceleration(), 1e-6);
            curvLimit = Math.sqrt(aLatMax / absK);
        }

        double segmentCap = segmentMaxVelocity > 0 ? segmentMaxVelocity : Double.POSITIVE_INFINITY;

        double v = robotTop;
        LimitReason reason = LimitReason.ROBOT_TOP_SPEED;

        if (curvLimit < v) {
            v = curvLimit;
            reason = LimitReason.CURVATURE;
        }
        if (segmentCap < v) {
            v = segmentCap;
            reason = LimitReason.SEGMENT_CAP;
        }

        return new Result(v, reason, curvature, robotTop, curvLimit, segmentCap);
    }

    public static double getMaxVelocity(double curvature, RobotModel model) {
        return getMaxVelocity(curvature, model, 0).maxVelocity;
    }
}
