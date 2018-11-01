package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Climbing {
    enum Angle{ //TODO:set motor Encoder positions and check @tucksPerCm
        DOWN(0),
        UP(0);
        float pos;
        final double ticksPerCm = 0;
        public int getTicks() {
            return ((int) (ticksPerCm * pos));
        }
        Angle(float ang){
            this.pos = pos;
        }
    }
    enum Height { //TODO:set motor Encoder positions and check @ticksPerCm
        MIN(0),
        MEDIUM(0),
        MAX(0);
        float pos;
        final double ticksPerCm = 0;
        public int getTicks(){
            return ((int) (ticksPerCm * pos));
        }
        Height(float pos) {
           this.pos = pos;
        }
    }

    //TODO: update the values down
    private final double HANG_OPEN_POS = 0;
    private final double HANG_CLOSE_POS = 0;
    private final double MOVING_SPEED = 0;

    private DcMotor liftMotor;
    private DcMotor angleMotor;
    private Servo hangServo;
    private TouchSensor touchSensor;
    private OpMode opMode;

    public  void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");
        angleMotor = hardwareMap.get(DcMotor.class,"angleMotor");
        hangServo = hardwareMap.get(Servo.class,"hangServo");
        touchSensor = hardwareMap.get(TouchSensor.class,"touchSensor");

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
    public void teleOpMotion(Gamepad gamepad) {
        if(!liftMotor.isBusy()){
            liftMotor.setPower(0);
        }
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


}
