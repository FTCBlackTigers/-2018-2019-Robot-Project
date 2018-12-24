package org.firstinspires.ftc.teamcode.RobotSystems;

import android.graphics.Path;

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
        DOWN(0),
        STARTPOS(30),
        COLLECT(200),
        CLIMB(95),
        PUT(90);
        float pos;
        final double ticksPerDegrees = 58.9;

        public int getTicks() {
            return ((int) (ticksPerDegrees * pos));
        }

        Angle(float ang) {
            this.pos = ang;
        }
    }

    public enum Height {
        MIN(0),
        MEDIUM(10),
        MAX(29);
        float pos;
        final double ticksPerCm = 277.105;

        public int getTicks() {
            return ((int) (ticksPerCm * pos));
        }

        Height(float pos) {
            this.pos = pos;
        }
    }

    private final double MOVING_SPEED = 0.5;
    private final double HANG_OPEN_POS = 0.25;
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



        if (!liftMotor.isBusy()) {
            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (!angleMotor.isBusy()) {
            angleMotor.setPower(0);
            angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (operator.dpad_down) {
            moveLift(Height.MIN);
        }

        if (operator.dpad_right) {
           moveLift(Height.MEDIUM);
        }

        if (operator.dpad_up) {
            moveLift(Height.MAX);
        }

        if (operator.right_trigger > 0.7) {
           moveAngle(Angle.CLIMB);
        }

        if (operator.left_trigger > 0.7) {
           moveAngle(Angle.COLLECT);
        }

        if (-operator.right_stick_y > 0.1 || -operator.right_stick_y < -0.1) {
            liftMoveManual(-operator.right_stick_y);
        }

        if (-operator.left_stick_y > 0.1 || -operator.left_stick_y < -0.1) {
            angleMoveManual(-operator.left_stick_y);
        }
        if (operator.a) {
            openServo();
        }

        if (operator.x) {
            lockServo();
        }
        opMode.telemetry.addLine("climbing: \n").addData("lift motor power: ", liftMotor.getPower())
                .addData(" Position: ", liftMotor.getCurrentPosition() + "\n")
                .addData("Height motor power: ", angleMotor.getPower())
                .addData("Servo position: ", hangServo.getPosition() + "\n")
                .addData(" Position: ", angleMotor.getCurrentPosition() + "\n")
                .addData("liftTouch : " , liftTouchIsActive)
                .addData(" angleTouch : ", angleTouchIsActive);


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

        angleTouchIsPrevActive = angleTouchIsActive;
        liftTouchIsPrevActive = liftTouchIsActive;
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
        //if(!(Height.MAX.getTicks() <= liftMotor.getCurrentPosition() && motorPower > 0) && !(Height.MIN.getTicks() >= liftMotor.getCurrentPosition() && motorPower < 0)){
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
        angleMotor.setPower(motorPower*0.6);

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
        moveAngle(Climbing.Angle.STARTPOS);
        waitForFinish(angleMotor);
        moveLift(Climbing.Height.MEDIUM);
        waitForFinish(liftMotor);
        moveAngle(Climbing.Angle.CLIMB);
        waitForFinish(angleMotor);
        moveLift(Climbing.Height.MAX);
        waitForFinish(liftMotor);
        moveAngle(Climbing.Angle.CLIMB);
        waitForFinish(angleMotor);
        openServo();
        ((LinearOpMode) opMode).sleep(300);
        moveAngle(Angle.STARTPOS);
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

}
