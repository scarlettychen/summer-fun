package org.firstinspires.ftc.teamcode.brainstem.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Component;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.brainstem.utils.HardwareNames;
import org.firstinspires.ftc.teamcode.brainstem.utils.MotionProfiler;
import org.firstinspires.ftc.teamcode.brainstem.utils.PIDController;

@Configurable
public class HorizontalSlide implements Component {

    public static double TESTING_POWER = 0.3;

    public static int MOTOR_SIGN = 1;

    public static int HIGH_POS = 510;
    public static int LOW_POS = 177;
    public static int DOWN_POS = 40;

    public static int ERROR_THRESHOLD = 20;

    public static double PARACHUTE_POWER = -0.005;

    public static double kP = 0.007, kI = 0.0, kD = 0.0005;
    public static double kG = 0.2;
    public static double kS = 0.1;
    public static double kV = 0.0;

    public static double MAX_VEL = 1500;
    public static double MAX_ACCEL = 4000;

    private final DcMotorEx right;
    private final PIDController pid;
    private final MotionProfiler profiler;
    private final ElapsedTime clock = new ElapsedTime();
    private final Telemetry telemetry;

    private double targetPosition = 0.0;
    private double lastPower = 0.0;
    private MotionProfiler.MotionState profiled = new MotionProfiler.MotionState(0, 0, 0);

    public enum LinkState {
        DOWN,
        SCORE_LOW,
        SCORE_HIGH,
        TESTING,
        OFF
    }

    public LinkState state = LinkState.DOWN;

    public HorizontalSlide(HardwareMap hardwareMap, Telemetry tel) {
        this.telemetry = tel;

        right = hardwareMap.get(DcMotorEx.class, HardwareNames.liftRight);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMotorEnable();

        pid = new PIDController(kP, kI, kD);
        pid.setOutputLimit(1.0);

        profiler = new MotionProfiler(MAX_ACCEL, MAX_VEL, MotionProfiler.ProfileMode.DIST);

        state = LinkState.DOWN;
        targetPosition = DOWN_POS;
    }

    public LinkState getState() {
        return state;
    }

    public void setState(LinkState newState) {
        boolean changed = this.state != newState;
        this.state = newState;

        switch (newState) {
            case SCORE_LOW:
                targetPosition = LOW_POS;
                if (changed) {
                    pid.reset();
                    profiler.setTarget(targetPosition, getPosition(), clock.seconds());
                }
                break;
            case SCORE_HIGH:
                targetPosition = HIGH_POS;
                if (changed) {
                    pid.reset();
                    profiler.setTarget(targetPosition, getPosition(), clock.seconds());
                }
                break;
            case DOWN:
                targetPosition = DOWN_POS;
                break;
            case OFF:
                break;
            case TESTING:
                setPower(0);
                break;
        }
    }

    @Override
    public void reset() {}

    @Override
    public void update() {
        pid.setGains(kP, kI, kD);
        profiler.setConstraints(MAX_ACCEL, MAX_VEL);

        int current = right.getCurrentPosition();

        switch (state) {
            case SCORE_LOW:
                targetPosition = LOW_POS;
                goToTarget(current);
                break;
            case SCORE_HIGH:
                targetPosition = HIGH_POS;
                goToTarget(current);
                break;
            case DOWN:

                if (current > DOWN_POS + ERROR_THRESHOLD) {
                    setPower(PARACHUTE_POWER);
                } else {
                    setPower(0);
                }
                break;
            case TESTING:
                setPower(TESTING_POWER);
                break;
            case OFF:
                setPower(0);
                break;
        }

        telemetry.addData("lift state", state);
        telemetry.addData("lift pos/tgt", "%d / %.0f", current, targetPosition);
        telemetry.addData("lift profile vel/accel", "%.0f / %.0f", profiled.velocity, profiled.acceleration);
        telemetry.addData("lift power", lastPower);
        telemetry.addData("lift atTarget", atTarget());
    }

    @Override
    public String test() {
        return "";
    }

    public void goToTarget(int current) {
        // profiler.update() reports the max velocity/accel the trapezoid allows given how far
        // we've actually traveled, not a lead position (that always equals `current`), so PID
        // still closes the loop on the real target; the profile only adds a shaped velocity
        // feedforward that ramps up on the way out and tapers back to 0 approaching the target.
        profiled = profiler.update(current, clock.seconds());

        double fb = pid.calculate(current, targetPosition);
        double err = targetPosition - current;

        double ff = kG + kS * Math.signum(err) + kV * profiled.velocity;
        setPower(fb + ff);
    }

    private void setPower(double pow) {
        lastPower = Range.clip(MOTOR_SIGN * pow, -1.0, 1.0);
        right.setPower(lastPower);
    }

    public boolean atTarget() {
        return Math.abs(desiredTarget() - getPosition()) < ERROR_THRESHOLD;
    }

    private double desiredTarget() {
        switch (state) {
            case SCORE_HIGH:
                return HIGH_POS;
            case SCORE_LOW:
                return LOW_POS;
            case DOWN:
                return DOWN_POS;
            default:
                return targetPosition;
        }
    }

    public int getPosition() {
        return right.getCurrentPosition();
    }

    public double getTargetPosition() {
        return desiredTarget();
    }

    public int getRightPosition() {
        return right.getCurrentPosition();
    }

    public double getLastPower() {
        return lastPower;
    }
}
