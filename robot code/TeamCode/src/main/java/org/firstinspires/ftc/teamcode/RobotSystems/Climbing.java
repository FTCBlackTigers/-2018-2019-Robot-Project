package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Climbing {
    //TODO:set motor Encoder positions and check @ticksPerCm
    public enum Height {
        DOWN(0),
        UP(50);
        float pos;
        final double ticksPerDegrees = 70.93;

        public int getTicks() {
            return ((int) (ticksPerDegrees * pos));
        }

        Height(float ang) {
            this.pos = ang;
        }
    }

    //TODO:set motor Encoder positions and check @ticksPerCm
    public enum Angle {
        MIN(0),
        MEDIUM(5),
        MAX(33f);
        float pos;
        final double ticksPerCm = 276.27;

        public int getTicks() {
            return ((int) (ticksPerCm * pos));
        }

        Angle(float pos) {
            this.pos = pos;
        }
    }

    //TODO: update the values down
    private final double HANG_OPEN_POS = 0;
    private final double HANG_CLOSE_POS = 0.5;
    private final double MOVING_SPEED = 0.5;

    private DcMotor liftMotor;
    private DcMotor angleMotor;
    private Servo hangServo;
    private OpMode opMode;
    private DigitalChannel liftTouch;
    private DigitalChannel angleTouch;

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
        angleMotor.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftTouch.setMode(DigitalChannel.Mode.INPUT);
        angleTouch.setMode(DigitalChannel.Mode.INPUT);

        lockServo();

    }

    public void teleOpMotion(Gamepad operator) {
        if (!liftMotor.isBusy()) {
            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (!angleMotor.isBusy()) {
            angleMotor.setPower(0);
            angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (operator.dpad_down) {
//            moveLift(Angle.MIN);
        }

        if (operator.dpad_right) {
//           moveLift(Angle.MEDIUM);
        }

        if (operator.dpad_up) {
//            moveLift(Angle.MAX);
        }

        if (operator.right_trigger > 0.7) {
//            moveAngle(Height.UP);
        }

        if (operator.left_trigger > 0.7) {
//            moveAngle(Height.DOWN);
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

        opMode.telemetry.addLine("climbing: \n").addData("lift motor: ", liftMotor.getPower())
                .addData(" Position: ", liftMotor.getCurrentPosition() + "\n")
                .addData("Height motor power: ", angleMotor.getPower())
                .addData(" Position: ", angleMotor.getCurrentPosition() + "\n")
                .addData("Servo position: ", hangServo.getPosition());


        if (!liftTouch.getState()) {
            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            opMode.telemetry.addData("liftTouch", "Is Not Pressed");
        }

        //TODO: Touch Sensor
//        if ((angleTouch.getState() && !   liftTouch.getState()) || (!angleTouch.getState() && liftTouch.getState())) {
//            angleMotor.setPower(0);
//            liftMotor.setPower(0);
//            angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
    }


    public void moveLift(Angle height) {
        liftMotor.setTargetPosition(height.getTicks());

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setPower(Math.abs(MOVING_SPEED));

    }

    public void lockServo() {
        hangServo.setPosition(HANG_CLOSE_POS);
    }

    public void openServo() {
        hangServo.setPosition(HANG_OPEN_POS);
    }

    private void liftMoveManual(double motorPower) {
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //if(!(Angle.MAX.getTicks() <= liftMotor.getCurrentPosition() && motorPower > 0) && !(Angle.MIN.getTicks() >= liftMotor.getCurrentPosition() && motorPower < 0)){
        liftMotor.setPower(motorPower);
    }


    public void moveAngle(Height Angle) {
        angleMotor.setTargetPosition(Angle.getTicks());
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angleMotor.setPower(Math.abs(MOVING_SPEED));
    }

    private void angleMoveManual(double motorPower) {
        angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //if(!(Height.UP.getTicks() <= angleMotor.getCurrentPosition() && motorPower > 0) && !(Height.DOWN.getTicks() >= angleMotor.getCurrentPosition() && motorPower < 0)){
        angleMotor.setPower(motorPower);
        //}
    }
}
