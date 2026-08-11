package org.firstinspires.ftc.teamcode.brainstem.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DifferentialWristBall {

    private static Telemetry telemetry;
    private static ServoImplEx leftServo;
    private static ServoImplEx rightServo;

    private static double PIVOT_OFFSET = 0;
    private static double PIVOT_SIGN = 0;
    private static double TURN_SIGN = 0;
    private static double SERVO_RANGE_DEG = 0;

    public enum WristState {
        PICK_UP,
        TILT_90,
        DOWN,
        TESTING
    }

    public WristState state = WristState.DOWN;

    public DifferentialWristBall(HardwareMap hardwareMap, Telemetry tel) {
        this.telemetry = tel;

        leftServo = hardwareMap.get(ServoImplEx.class, "servo one");
        rightServo = hardwareMap.get(ServoImplEx.class, "servo two");

        leftServo.setPwmRange(new PwmControl.PwmRange(0, 0)); //TODO: TUNE
        rightServo.setPwmRange(new PwmControl.PwmRange(0, 0)); //TODO: TUNE

//        leftServo.setDirection(ServoImplEx.Direction.REVERSE);  TODO: check if this.
    }

    public WristState getState() {
        return state;
    }

    public void update() {
        switch(state) {
            case PICK_UP:
                break;
            case DOWN:
                break;
            case TILT_90:
                break;
            case TESTING:
                // PIVOT:
                // Positive = wrist pivots UP
                // Negative = wrist pivots DOWN
                //
                // TURN:
                // Positive = wrist turns CLOCKWISE
                // Negative = wrist turns COUNTERCLOCKWISE


                leftServo.setPosition(0.55);

                telemetry.addLine("TESTING: LEFT SERVO");
                telemetry.addData("Left Position", 0.55);

//                rightServo.setPosition(0.55);
//
//                telemetry.addLine("TESTING: RIGHT SERVO");
//                telemetry.addData("Right Position", 0.55);


                // turn and pivot signs:

//                double pivotTest = 0.00;
//                double turnTest = 0.10;
//
//                double left = 0.50 + pivotTest + turnTest;
//                double right = 0.50 + pivotTest - turnTest;
//
//                leftServo.setPosition(left);
//                rightServo.setPosition(right);
//
//                telemetry.addLine("=== TURN TEST ===");
//                telemetry.addData("Pivot command", pivotTest);
//                telemetry.addData("Turn command", turnTest);
//                telemetry.addData("Left", left);
//                telemetry.addData("Right", right);




                break;
        }
    }




    public void setWrist(double pivot, double turn) {

        // PIVOT:
        // Positive = pivot UP from DOWN position
        // Negative = pivot DOWN from DOWN position
        //
        // TURN:
        // Positive = turn CLOCKWISE from DOWN position
        // Negative = turn COUNTERCLOCKWISE from DOWN position

        double leftPos =
                        PIVOT_SIGN * pivot
                        + TURN_SIGN * turn;

        double rightPos =
                        PIVOT_SIGN * pivot
                        - TURN_SIGN * turn;

        leftServo.setPosition(
                Range.clip(leftPos, 0.0, 1.0)
        );

        rightServo.setPosition(
                Range.clip(rightPos, 0.0, 1.0)
        );
    }
}
