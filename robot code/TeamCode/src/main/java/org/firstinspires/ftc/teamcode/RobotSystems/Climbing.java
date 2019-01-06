package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

public class Climbing {
    public enum Angle {
        DRIVE_POS(50),
        COLLECT(180),
        GO_TO_CLIMB(90),
        CLIMB(30),
        PUT(70);
        float pos;
        final double ticksPerDegree =93.588;

        public int getTicks() {
            return ((int) (ticksPerDegree * pos));
        }

        Angle(float ang) {
            this.pos = ang;
        }
    }

    public enum Height {

        DRIVE_POS(0),
        COLLECT(20),
        GO_TO_CLIMB(30),
        CLIMB(10),
        PUT(33);

        float pos;
        final double ticksPerCm = 281.942;

        public int getTicks() {
            return ((int) (ticksPerCm * pos));
        }

        Height(float pos) {
            this.pos = pos;
        }
    }

    private final double MOVING_SPEED = 0.8;
    private final double HANG_OPEN_POS = 0.3;
    private final double HANG_CLOSE_POS = 0;

    private DcMotor liftMotor;
    private Servo hangServo;
    private DcMotor angleMotor;
    private OpMode opMode;
    private DigitalChannel liftTouch;
    private DigitalChannel angleTouch;
    private boolean angleTouchIsPrevActive;
    private boolean angleTouchIsActive;
    private boolean liftTouchIsPrevActive;
    private boolean liftTouchIsActive;
    private double angleJoystickValue = 0;
    private double angleJoystickPrevValue = 0;
    private double liftJoystickValue = 0;
    private double liftJoystickPrevValue = 0;

    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        angleMotor = hardwareMap.get(DcMotor.class, "angleMotor");
        hangServo = hardwareMap.get(Servo.class, "hangServo");
        angleTouch = hardwareMap.get(DigitalChannel.class, "angle_touch");
        liftTouch = hardwareMap.get(DigitalChannel.class, "lift_touch");

        liftMotor.setPower(0);
        angleMotor.setPower(0);

        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        angleMotor.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftTouch.setMode(DigitalChannel.Mode.INPUT);
        angleTouch.setMode(DigitalChannel.Mode.INPUT);
        lockServo();

    }

    public void teleOpMotion(Gamepad operator) {
        angleTouchIsActive = !angleTouch.getState();
        liftTouchIsActive = !liftTouch.getState();

        angleJoystickValue = -operator.left_stick_y;
        liftJoystickValue = -operator.right_stick_y;

        if (angleTouchIsActive && !angleTouchIsPrevActive) {
            angleMotor.setPower(0);
            angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (liftTouchIsActive && !liftTouchIsPrevActive) {
            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (!liftMotor.isBusy() && liftMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (!angleMotor.isBusy() && angleMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            angleMotor.setPower(0);
            angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (operator.dpad_down) {
            moveLift(Height.DRIVE_POS);
            moveAngle(Angle.DRIVE_POS);
        }

        if (operator.dpad_left) {
            moveLift(Height.CLIMB);
            moveAngle(Angle.CLIMB);
        }


        if (operator.dpad_up) {
            moveLift(Height.GO_TO_CLIMB);
            moveAngle(Angle.GO_TO_CLIMB);
            openServo();
        }

        if (operator.right_trigger > 0.7) {
           moveLift(Height.COLLECT);
           moveAngle(Angle.COLLECT);
        }

        if (operator.left_trigger > 0.7) {
            moveLift(Height.PUT);
            moveAngle(Angle.PUT);
        }

        if (Math.abs(liftJoystickValue) > 0.1) {
            liftMoveManual(liftJoystickValue);
        } else if (Math.abs(liftJoystickPrevValue) > 0.1) {
            liftMotor.setPower(0);
        }

        if (Math.abs(angleJoystickValue) > 0.1) {
            angleMoveManual(angleJoystickValue);
        } else if (Math.abs(angleJoystickPrevValue) > 0.1) {
            angleMotor.setPower(0);
        }
        if (operator.a) {
            openServo();
        }

        if (operator.x) {
            lockServo();
        }


        opMode.telemetry.addLine("climbing: \n").addData("lift motor power: ", liftMotor.getPower())
                .addData(" Position: ", liftMotor.getCurrentPosition() + "\n")
                .addData("Angle motor power: ", angleMotor.getPower())
                .addData(" Position: ", angleMotor.getCurrentPosition() + "\n")
                .addData("Servo position: ", hangServo.getPosition() + "\n")
                .addData("liftTouch: " , liftTouchIsActive)
                .addData(" angleTouch: ", angleTouchIsActive);

        angleTouchIsPrevActive = angleTouchIsActive;
        liftTouchIsPrevActive = liftTouchIsActive;

        angleJoystickPrevValue = angleJoystickValue;
        liftJoystickPrevValue = liftJoystickValue;
    }


    public void moveLift(Height height) {
        liftMotor.setTargetPosition(height.getTicks());

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setPower(Math.abs(MOVING_SPEED));

    }

    private void liftMoveManual(double motorPower) {
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (liftTouchIsActive && motorPower < 0) {
            return;
        }
        liftMotor.setPower(motorPower * 0.8);
    }

    public void moveAngle(Angle Angle) {
        angleMotor.setTargetPosition(Angle.getTicks());
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angleMotor.setPower(Math.abs(MOVING_SPEED));
    }

    private void angleMoveManual(double motorPower) {
        angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(angleTouchIsActive && motorPower < 0) {
            return;
        }
        angleMotor.setPower(motorPower * 1);

    }

    public void lockServo() {
        hangServo.setPosition(HANG_CLOSE_POS);
    }

    public void openServo() {
        hangServo.setPosition(HANG_OPEN_POS);
    }
    public void waitForFinish(DcMotor motor){

        while(motor.isBusy() && ((LinearOpMode) opMode).opModeIsActive()){
            opMode.telemetry.addData(motor.getDeviceName() + ": ",motor.getCurrentPosition());
            opMode.telemetry.update();
        }

    }

    public void land() {
        /*moveAngle(Climbing.Angle.STARTPOS);
        waitForFinish(angleMotor);
        moveLift(Height.MEDIUM);
        waitForFinish(liftMotor);
        moveAngle(Angle.GO_TO_CLIMB);
        waitForFinish(angleMotor);*/
        moveLift(Climbing.Height.GO_TO_CLIMB);
        waitForFinish(liftMotor);
        moveAngle(Climbing.Angle.GO_TO_CLIMB);
        waitForFinish(angleMotor);
        openServo();
        ((LinearOpMode) opMode).sleep(300);
        moveAngle(Angle.DRIVE_POS);
        waitForFinish(angleMotor);

        angleMotor.setPower(0);
        liftMotor.setPower(0);

    }

    public void moveliftAuto(Height height) {
        moveLift(height);
        waitForFinish(liftMotor);
        liftMotor.setPower(0);
    }

    public void moveAngleAuto(Angle angle) {
        moveAngle(angle);
        waitForFinish(angleMotor);
        angleMotor.setPower(0);
    }

    public DcMotor getLiftMotor() {
        return liftMotor;
    }

    public DcMotor getAngleMotor() {
        return angleMotor;
    }
    public boolean AngleTouchPressed(){
        return !angleTouch.getState();
    }
    public boolean LiftTouchPressed(){
        return !liftTouch.getState();
    }

    public Servo getHangServo() {
        return hangServo;
    }
}
