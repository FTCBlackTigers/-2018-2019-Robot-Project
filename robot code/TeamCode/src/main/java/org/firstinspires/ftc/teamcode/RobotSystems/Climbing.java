package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Climbing {
    //TODO:set motor Encoder positions and check @ticksPerCm
    enum Angle{
        DOWN(-10),
        UP(50);
        float pos;
        final double ticksPerDegrees = 1;
        public int getTicks() {
            return ((int) (ticksPerDegrees * pos));
        }
        Angle(float ang){
            this.pos = ang;
        }
    }

    //TODO:set motor Encoder positions and check @ticksPerCm
    enum Height {
        MIN(-10),
        MEDIUM(50),
        MAX(100);
        float pos;
        final double ticksPerCm = 1;
        public int getTicks(){
            return ((int) (ticksPerCm * pos));
        }
        Height(float pos) {
           this.pos = pos;
        }
    }

    //TODO: update the values down
    private final double HANG_OPEN_POS = 0;
    private final double HANG_CLOSE_POS = 1;
    private final double MOVING_SPEED = 0.5;

    private DcMotor liftMotor;
    private DcMotor angleMotor;
    private Servo hangServo;
    private OpMode opMode;

    public  void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");
        angleMotor = hardwareMap.get(DcMotor.class,"angleMotor");
        hangServo = hardwareMap.get(Servo.class,"hangServo");

        liftMotor.setPower(0);
        angleMotor.setPower(0);

        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        angleMotor.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            moveLift(Height.MIN);
        }

        if (operator.dpad_right) {
            moveLift(Height.MEDIUM);
        }

        if (operator.dpad_up) {
            moveLift(Height.MAX);
        }

        if(operator.right_trigger > 0.7){
           moveAngle(Angle.UP);
        }

        if(operator.left_trigger > 0.7){
            moveAngle(Angle.DOWN);
        }

        if(-operator.right_stick_y > 0.1 || -operator.right_stick_y  <  -0.1){
            liftMoveManual(-operator.right_stick_y);
        }

        if(-operator.left_stick_y > 0.1 || -operator.left_stick_y  <  -0.1){
            angleMoveManual(-operator.left_stick_y);
        }

        if(operator.a){
            openServo();
        }

        if(operator.x){
            lockServo();
        }

        opMode.telemetry.addLine("climbing: \n").addData("lift motor: ",liftMotor.getPower())
                .addData(" Position: ",liftMotor.getCurrentPosition() + "\n")
                .addData("Angle motor power: ",angleMotor.getPower())
                .addData(" Position: ",angleMotor.getCurrentPosition() + "\n")
                .addData("Servo position: ",hangServo.getPosition());



    }



    private void moveLift(Height height){
        liftMotor.setTargetPosition(height.getTicks());

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setPower(Math.abs(MOVING_SPEED));
    }

    private void lockServo(){
        hangServo.setPosition(HANG_CLOSE_POS);
    }

    private void openServo(){
        hangServo.setPosition(HANG_OPEN_POS);
    }

    private void liftMoveManual(double motorPower){
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(!(Height.MAX.getTicks() <= liftMotor.getCurrentPosition() && motorPower > 0) && !(Height.MIN.getTicks() >= liftMotor.getCurrentPosition() && motorPower < 0)){
            liftMotor.setPower(motorPower);
        }
    }


    private void moveAngle(Angle angle){
        angleMotor.setTargetPosition(angle.getTicks());

        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        angleMotor.setPower(Math.abs(MOVING_SPEED));
    }

    private void angleMoveManual(double motorPower){
        angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(!(Angle.UP.getTicks() <= angleMotor.getCurrentPosition() && motorPower > 0) && !(Angle.DOWN.getTicks() >= angleMotor.getCurrentPosition() && motorPower < 0)){
            angleMotor.setPower(motorPower);
        }
    }
}
