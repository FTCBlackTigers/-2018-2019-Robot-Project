package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    enum Minerals{
        GOLD, SILVER, UNKNOWN;
    }
    /*
     * TODO: change values
     * */
    private final double COLLECTION_SPEED = 0.5;
    private final double RELEASE_SPEED = 0.2;
    private final double LEFT_SERVO_OPEN_POS = 0.5;
    private final double RIGHT_SERVO_OPEN_POS = 0.5;
    private final double LEFT_SERVO_CLOSE_POS = 0.5;
    private final double RIGHT_SERVO_CLOSE_POS = 0.5;

    private OpMode OpMode;

    private DcMotor collectMotor;
    private Servo leftServo, rightServo;
    private NormalizedColorSensor leftColorSensor, rightColorSensor;
    private Minerals searchMineral;

    private boolean buttonPressed;
    private boolean releaseModeIsActive;


    public void init(HardwareMap hardwareMap, OpMode opMode){
        this.OpMode = opMode;
        collectMotor = hardwareMap.get(DcMotor.class, "collectMotor");
        collectMotor.setDirection(DcMotor.Direction.FORWARD);
        collectMotor.setPower(0);
        collectMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        closeLeftGate();
        closeRightGate();

        leftColorSensor = hardwareMap.get(NormalizedColorSensor.class, "leftColorSensor");
        rightColorSensor = hardwareMap.get(NormalizedColorSensor.class, "rightColorSensor");

        searchMineral = Minerals.GOLD;
    }

    public void teleOpMotion(Gamepad driver, Gamepad operator) {
        if(driver.right_bumper || operator.right_bumper) {
            this.collect();
        }

        if((buttonPressed && !driver.right_bumper) || (buttonPressed && !operator.right_bumper) ){
            closeRightGate();
            closeLeftGate();
            this.stopMotor();
        }
        if(!releaseModeIsActive && !(driver.right_bumper || operator.right_bumper)) {
            this.stopMotor();
            closeRightGate();
            closeLeftGate();
        }
        if(driver.left_bumper || operator.left_bumper){
            this.release();
        }
        if(operator.y) {
            setSearchMineral(Minerals.GOLD);
        }
        if(operator.b) {
            setSearchMineral(Minerals.SILVER);
        }
        releaseModeIsActive = driver.left_bumper || operator.left_bumper;
        buttonPressed = driver.right_bumper || operator.right_bumper;

    }

    private void openLeftGate() {
        leftServo.setPosition(LEFT_SERVO_OPEN_POS);
    }

    private void openRightGate() {
        rightServo.setPosition(RIGHT_SERVO_OPEN_POS);
    }

    private void closeLeftGate() {
        leftServo.setPosition(LEFT_SERVO_CLOSE_POS);
    }

    private void closeRightGate() {
        rightServo.setPosition(RIGHT_SERVO_CLOSE_POS);
    }

    private void release(){
        openLeftGate();
        openRightGate();
        collectMotor.setPower(RELEASE_SPEED);
    }

    private void collect() {
        collectMotor.setPower(COLLECTION_SPEED);
        this.leftOutput();
        this.rightOutput();
    }

    //TODO:write a code to identity a mineral
    private Minerals getLeftMineral() {
        return Minerals.UNKNOWN;
    }

    private Minerals getRightMineral() {
        return Minerals.UNKNOWN;
    }


    private void leftOutput(){
        if(getLeftMineral() == Minerals.UNKNOWN) {
            closeLeftGate();
        }
        else if (this.getLeftMineral() != searchMineral) {
                    openLeftGate();
        }
    }

    private void rightOutput(){
        if(getRightMineral() == Minerals.UNKNOWN) {
            closeRightGate();
        }
        else if (this.getRightMineral() != searchMineral) {
            openRightGate();
        }
    }
    private void stopMotor(){collectMotor.setPower(0);}
    public void setSearchMineral(Minerals mineral){
        this.searchMineral = mineral;
    }

}



