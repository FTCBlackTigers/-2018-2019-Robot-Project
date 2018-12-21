package org.firstinspires.ftc.teamcode.RobotSystems;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class Intake {
    enum Minerals{
        GOLD, SILVER, UNKNOWN;
    }
    /*
     * TODO: correct values
     * */
    private final double COLLECTION_SPEED = 0.8;
    private final double RELEASE_SPEED = 0.4;

    private final double LEFT_SERVO_OPEN_POS = 0.8;
    private final double RIGHT_SERVO_OPEN_POS = 0;
    private final double LEFT_SERVO_CLOSE_POS = 0.15;
    private final double RIGHT_SERVO_CLOSE_POS = 0.7;

    private OpMode opMode;

    private DcMotor collectMotor;
    public Servo leftServo, rightServo;
    private ColorSensor leftColorSensor, rightColorSensor;
    private Minerals searchMineral;

    private boolean collectModeIsPrevActive;
    private boolean releaseModeIsPrevActive;
    private boolean releaseModeIsActive;
    private boolean collectModeIsActive;


    public void init(HardwareMap hardwareMap, OpMode opMode){
        this.opMode = opMode;
        collectMotor = hardwareMap.get(DcMotor.class, "collectMotor");
        collectMotor.setDirection(DcMotor.Direction.FORWARD);
        collectMotor.setPower(0);
        collectMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftColorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        searchMineral = Minerals.GOLD;

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        closeLeftGate();
        closeRightGate();
        if (leftColorSensor instanceof SwitchableLight && rightColorSensor instanceof  SwitchableLight) {
        ((SwitchableLight)leftColorSensor).enableLight(false);
        ((SwitchableLight)rightColorSensor).enableLight(false);
    }}

    public void teleOpMotion(Gamepad driver, Gamepad operator) {

        releaseModeIsActive = driver.left_bumper || operator.left_bumper;
        collectModeIsActive = driver.right_bumper || operator.right_bumper;

        if(collectModeIsActive) {
            this.collect();
        }
        else if (releaseModeIsActive) {
            this.release();
        }

        if(collectModeIsPrevActive && !collectModeIsActive) {
            closeRightGate();
            closeLeftGate();
            this.stopMotor();
        }
        if(releaseModeIsPrevActive && !releaseModeIsActive) {
            this.stopMotor();
            closeRightGate();
            closeLeftGate();
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

        releaseModeIsPrevActive = releaseModeIsActive;
        collectModeIsPrevActive = collectModeIsActive;


    }

    private void openLeftGate() {
        leftServo.setPosition(LEFT_SERVO_OPEN_POS);
        LeftScounter = 0;
        LeftGcounter = 0;

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

    public void release(){
        openLeftGate();
        openRightGate();
        collectMotor.setPower(RELEASE_SPEED);
    }

   public void collect() {
        collectMotor.setPower(COLLECTION_SPEED);
        this.leftOutput();
        //this.rightOutput();
    }
        private int LeftScounter = 0;
        private int LeftGcounter = 0;
    private Minerals getLeftMineral() {
        float[] hsvValuse = new float[3];
        Color.RGBToHSV(leftColorSensor.red() * 255, leftColorSensor.green() * 255, leftColorSensor.blue()* 255, hsvValuse);
        opMode.telemetry.addLine("leftColorSensor: ").addData("Hue: ", hsvValuse[0]);

        //TODO: check the color values;
        if(hsvValuse[2] < 4000 && hsvValuse[2] > 1000){
            LeftScounter++;
        }else {
            LeftScounter = 0;
        }
        if(LeftScounter >= 3) {
            return Minerals.SILVER;
        }


        if(hsvValuse[2]< 900 && hsvValuse[2] > 360){
            LeftGcounter++;
        }else{
            LeftGcounter = 0;
        }
        if(LeftGcounter >= 3){
            return Minerals.GOLD;
        }

        return Minerals.UNKNOWN;
    }

    private Minerals getRightMineral() {
        float[] hsvValuse = new float[3];
        Color.RGBToHSV(rightColorSensor.red() * 255, rightColorSensor.green() * 255, rightColorSensor.blue()* 255, hsvValuse);
        opMode.telemetry.addLine("rightColorSensor: ").addData("Hue: ", hsvValuse[0]);

        //TODO: check the color values;
        if(false){
            return Minerals.SILVER;
        }
        if(false){
            return Minerals.GOLD;
        }
        return Minerals.UNKNOWN;
    }


    private void leftOutput(){
        //TODO:FIX
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
    public void stopMotor(){collectMotor.setPower(0);}
    public void setSearchMineral(Minerals mineral){
        this.searchMineral = mineral;
    }

}



