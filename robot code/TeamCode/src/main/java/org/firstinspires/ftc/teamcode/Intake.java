package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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
    private final double LEFT_SERVO_OPEN_POS = 1;
    private final double RIGHT_SERVO_OPEN_POS = 1;
    private final double LEFT_SERVO_CLOSE_POS = 0.5;
    private final double RIGHT_SERVO_CLOSE_POS = 0.5;

    private OpMode opMode;

    private DcMotor collectMotor;
    private Servo leftServo, rightServo;
    private NormalizedColorSensor leftColorSensor, rightColorSensor;
    private Minerals searchMineral;

    private boolean collectModeIsPrevActive;
    private boolean releaseModeIsPrevActive;


    public void init(HardwareMap hardwareMap, OpMode opMode){
        this.opMode = opMode;
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

        if((collectModeIsPrevActive && !driver.right_bumper) || (collectModeIsPrevActive && !operator.right_bumper) ){
            closeRightGate();
            closeLeftGate();
            this.stopMotor();
        }
        if(releaseModeIsPrevActive && !(driver.right_bumper || operator.right_bumper)) {
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

        opMode.telemetry.addLine("intake: \n" )
                .addData("collectMotorPower: ", collectMotor.getPower()+"\n")
                .addData("rightServoPos: ", rightServo.getPosition())
                .addData("leftServoPos: ", leftServo.getPosition()+"\n")
                .addData("search mineral: ", searchMineral+"\n")
                .addData("releaseModeIsPrevActive: ", releaseModeIsPrevActive)
                .addData("collectModeIsPrevActive: ", collectModeIsPrevActive +"\n");

        releaseModeIsPrevActive = driver.left_bumper || operator.left_bumper;
        collectModeIsPrevActive = driver.right_bumper || operator.right_bumper;


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

    private Minerals getLeftMineral() {
        NormalizedRGBA colors = leftColorSensor.getNormalizedColors();
        if(colors.red > 0.080 && colors.red < 0.12 && colors.green > 0.080 && colors.green < 0.12 && colors.blue > 0.060 && colors.blue < 0.1 ){
            return Minerals.SILVER;
        }
        if(colors.red > 0.03 && colors.red < 0.50 && colors.green > 0.02 && colors.green < 0.04 && colors.blue > 0.01 && colors.blue < 0.02 ){
            return Minerals.GOLD;
        }
        return Minerals.UNKNOWN;
    }

    private Minerals getRightMineral() {
        NormalizedRGBA colors = rightColorSensor.getNormalizedColors();
        if(colors.red > 0.080 && colors.red < 0.12 && colors.green > 0.080 && colors.green < 0.12 && colors.blue > 0.060 && colors.blue < 0.1 ){
            return Minerals.SILVER;
        }
        if(colors.red > 0.03 && colors.red < 0.50 && colors.green > 0.02 && colors.green < 0.04 && colors.blue > 0.01 && colors.blue < 0.02 ){
            return Minerals.GOLD;
        }
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



