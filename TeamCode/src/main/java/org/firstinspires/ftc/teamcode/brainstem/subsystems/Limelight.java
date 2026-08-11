package org.firstinspires.ftc.teamcode.brainstem.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Component;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.brainstem.FieldCoords;
import org.firstinspires.ftc.teamcode.brainstem.RoadRunnerCoordinates;
import org.firstinspires.ftc.teamcode.brainstem.vision.Detection;
import org.firstinspires.ftc.teamcode.brainstem.vision.VisionPose;
import org.firstinspires.ftc.teamcode.brainstem.vision.VisionResult;
import org.firstinspires.ftc.teamcode.brainstem.vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.brainstem.vision.pipelines.LimelightPipeline;

@Configurable
public class Limelight implements Component {

    public static double CAMERA_HEIGHT_IN = 10.0;
    public static double CAMERA_PITCH_DEG = -20.0;
    public static double CAMERA_FORWARD_IN = 3.0;
    public static double CAMERA_LEFT_IN = -1.5;
    public static double MIN_AREA = 0.001;
    public static double MIN_CONFIDENCE = 0.3;
    public static double MAX_RANGE_IN = 72.0;
    public static double MIN_RANGE_IN = 6.0;
    public static String CLASS_FILTER = "";

    public static double CHASE_KP_TX = 0.018;
    public static double CHASE_KI_TX = 0.0;
    public static double CHASE_KD_TX = 0.001;
    public static double CHASE_KP_RANGE = 0.035;
    public static double CHASE_KI_RANGE = 0.0;
    public static double CHASE_KD_RANGE = 0.0;
    public static double CHASE_KP_STRAFE = 0.025;
    public static double CHASE_MAX_FORWARD = 0.85;
    public static double CHASE_MAX_TURN = 0.45;
    public static double CHASE_MAX_STRAFE = 0.45;
    public static double CHASE_MIN_FORWARD = 0.28;
    public static double CHASE_TX_TOL_DEG = 4.0;
    public static double CHASE_STOP_RANGE_IN = 3.0;
    public static double CHASE_STRAFE_RANGE_IN = 20.0;
    public static double CHASE_NEAR_TURN_SCALE = 0.2;
    public static double CHASE_TX_OFFSET_DEG = 0.0;
    public static int CHASE_LOST_FRAMES = 15;
    public static double MAX_CHASE_TIME_MS = 5000.0;

    public static int COLLECT_COUNT = 5;
    public static double COLLECT_DWELL_MS = 400.0;
    public static double COLLECT_BACK_OFF_IN = 12.0;

    public static double WALL_MIN_X = -70.0;
    public static double WALL_MAX_X = 70.0;
    public static double WALL_MIN_Y = -70.0;
    public static double WALL_MAX_Y = 70.0;

    public static double RED_MIN_Y = 0.0;
    public static double RED_MAX_Y = WALL_MAX_Y;
    public static double RED_MIN_X = WALL_MIN_X;
    public static double RED_MAX_X = WALL_MAX_X;

    public static double BLUE_MIN_Y = WALL_MIN_Y;
    public static double BLUE_MAX_Y = 0.0;
    public static double BLUE_MIN_X = WALL_MIN_X;
    public static double BLUE_MAX_X = WALL_MAX_X;

    public static boolean isOutOfBounds(double x, double y, boolean red) {
        if (red) {
            return x < RED_MIN_X || x > RED_MAX_X || y < RED_MIN_Y || y > RED_MAX_Y;
        }
        return x < BLUE_MIN_X || x > BLUE_MAX_X || y < BLUE_MIN_Y || y > BLUE_MAX_Y;
    }

    private final Telemetry telemetry;
    private final VisionSubsystem vision;

    private BallDetection closest;

    public static final class BallDetection {
        public final String className;
        public final double confidence;
        public final double txDeg;
        public final double tyDeg;
        public final double area;

        public BallDetection(
                String className, double confidence, double txDeg, double tyDeg, double area) {
            this.className = className;
            this.confidence = confidence;
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.area = area;
        }
    }

    public Limelight(VisionSubsystem vision, Telemetry telemetry) {
        this.vision = vision;
        this.telemetry = telemetry;
    }

    public VisionSubsystem getVision() {
        return vision;
    }

    public BallDetection getClosestBall() {
        return closest;
    }

    public boolean hasBall() {
        return closest != null;
    }

    public double[] estimateClosestBallFieldPose(Pose robotPedroPose) {
        if (closest == null || robotPedroPose == null) {
            return null;
        }
        return estimateBallFieldPose(robotPedroPose, closest);
    }

    public double[] estimateBallFieldPose(Pose robotPedroPose, BallDetection ball) {
        double range = estimateRangeInches(ball.tyDeg);
        if (Double.isNaN(range)) {
            return null;
        }
        range = Math.max(MIN_RANGE_IN, Math.min(MAX_RANGE_IN, range));

        double txRad = Math.toRadians(ball.txDeg);
        double forwardFromCamera = range * Math.cos(txRad);
        double rightFromCamera = range * Math.sin(txRad);
        double forward = CAMERA_FORWARD_IN + forwardFromCamera;
        double left = CAMERA_LEFT_IN - rightFromCamera;

        Pose field = robotPedroPose.getAsCoordinateSystem(RoadRunnerCoordinates.INSTANCE);
        double hCcw = FieldCoords.ccwRadians(field.getHeading());
        double cos = Math.cos(hCcw);
        double sin = Math.sin(hCcw);
        double x = field.getX() + forward * cos - left * sin;
        double y = field.getY() + forward * sin + left * cos;
        return new double[]{x, y, Math.toDegrees(field.getHeading())};
    }

    public double estimateRangeInches(double tyDeg) {
        return LimelightPipeline.estimateRangeInches(tyDeg);
    }

    public double estimateClosestRangeInches() {
        if (closest == null) {
            return Double.NaN;
        }
        return estimateRangeInches(closest.tyDeg);
    }

    @Override
    public void reset() {
        closest = null;
    }

    @Override
    public void update() {
        VisionResult result = vision.getLatestResult();
        Detection best = result.bestDetection();
        if (best == null) {
            closest = null;
            telemetry.addData("balls", 0);
            return;
        }
        closest = new BallDetection(
                best.className, best.confidence, best.txDegrees, best.tyDegrees, best.area);
        telemetry.addData("balls", result.detections.size());
        telemetry.addData("closest", "%s area=%.3f tx=%.1f",
                closest.className, closest.area, closest.txDeg);
        VisionPose rel = result.robotRelativePose;
        if (rel != null) {
            telemetry.addData("ball body", "fwd=%.1f left=%.1f", rel.xInches, rel.yInches);
        }
    }

    @Override
    public String test() {
        return "Limelight";
    }
}
