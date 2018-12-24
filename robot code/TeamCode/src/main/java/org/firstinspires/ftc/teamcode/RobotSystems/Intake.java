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

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    enum Minerals{
        GOLD, SILVER, UNKNOWN;
    }

    private final double COLLECTION_SPEED = 0.8;
    private final double RELEASE_SPEED = 0.4;
    private final double TEAM_MARKER_SPEED = -0.1;
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
        }
    }

    public void teleOpMotion(Gamepad driver, Gamepad operator) {

        releaseModeIsActive = driver.left_bumper || operator.left_bumper;
        collectModeIsActive = driver.right_bumper || operator.right_bumper;

        if(collectModeIsActive) {
            this.collect();
        }
        else if (releaseModeIsActive) {
            this.release();
        }

        /*if (driver.dpad_up){
            putTeamMarker();
        }else stopMotor();*/

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
        this.rightOutput();
    }

    public void putTeamMarker(){
        collectMotor.setPower(TEAM_MARKER_SPEED);
        openRightGate();
        openLeftGate();
    }

    private Minerals getLeftMineral() {
        float[] hsvValues = new float[3];
        Color.RGBToHSV(leftColorSensor.red() * 255, leftColorSensor.green() * 255, leftColorSensor.blue()* 255, hsvValues);
        opMode.telemetry.addLine("leftColorSensor: ").addData("Value: ", hsvValues[2]);

        if(hsvValues[2] >= 1550) {
            return Minerals.SILVER;
        }
        if (hsvValues[2] >= 300 && hsvValues[2] <= 550) {
            return Minerals.GOLD;
        }
        return Minerals.UNKNOWN;
    }

    private Minerals getRightMineral() {
        float[] hsvValues = new float[3];
        Color.RGBToHSV(rightColorSensor.red() * 255, rightColorSensor.green() * 255, rightColorSensor.blue()* 255, hsvValues);
        opMode.telemetry.addLine("rightColorSensor: ").addData("Value: ", hsvValues[2]);

        if(hsvValues[2] >= 1550) {
            return Minerals.SILVER;
        }
        if (hsvValues[2] >= 300 && hsvValues[2] <= 550) {
            return Minerals.GOLD;
        }
        return Minerals.UNKNOWN;
    }


    private void leftOutput(){
        Minerals imat= getLeftMineral();
        if(imat == Minerals.UNKNOWN) {
            closeLeftGate();
        }
        else if (imat != searchMineral) {
            openLeftGate();
        }
    }

    private void rightOutput(){
        Minerals imat= getRightMineral();
        if(imat == Minerals.UNKNOWN) {
            closeRightGate();
        }
        else if (imat != searchMineral) {
            openRightGate();
        }
    }

    public void stopMotor(){
        collectMotor.setPower(0);
        closeRightGate();
        closeLeftGate();
    }

    public void setSearchMineral(Minerals mineral){
        this.searchMineral = mineral;
    }

}



