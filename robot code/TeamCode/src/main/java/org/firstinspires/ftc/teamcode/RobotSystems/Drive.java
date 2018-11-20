package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BT_Gyro;

public class Drive {
    enum Direction {
        FORWARD, BACKWARD;
    }

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 19.2;    //TODO: check the DRIVE_GEAR_REDUCTION
    static final double WHEEL_DIAMETER_CM = 4.0;    //TODO: check the WHEEL_DIAMETER_CM
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.141592654);
    static final double      AUTO_TURN_SPEED              = 0.3;
    static final double     THRESHOLD = 0.1;
    static final double     P_TURN_COEFF            = 0.1;
    private OpMode opMode;

    private BT_Gyro gyro = new BT_Gyro();
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        gyro.init(hardwareMap);

        //TODO: check motors direction;
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void teleOpMotion(Gamepad gamepad) {
        tankDrive(-gamepad.left_stick_y, -gamepad.right_stick_y);
        opMode.telemetry.addLine("Drive: ").
                addData("left motor power: ", leftDrive.getPower()).
                addData("right motor power: ", rightDrive.getPower());

    }

    private void tankDrive(double powerLeftDrive, double powerRightDrive) {
        leftDrive.setPower(powerLeftDrive);
        rightDrive.setPower(powerRightDrive);
    }

    public void turnByGyroAbsolut (double degrees, double timeMs) {
        double rightSpeed, leftSpeed;
        double steer;
        double error = getError(degrees);
        double t;
        double stopTime = opMode.getRuntime() + timeMs;
        // keep looping while we are still active, and not on heading.
        while((Math.abs(error) > THRESHOLD) && (opMode.getRuntime() < stopTime)) {
            while (Math.abs(error) > THRESHOLD ) {
                // Update telemetry & Allow time for other processes to run.
                steer = getSteer(error, P_TURN_COEFF);
                rightSpeed = AUTO_TURN_SPEED * steer;
                if(rightSpeed > 0 && rightSpeed<0.1)
                    rightSpeed= 0.1;
                else if(rightSpeed < 0 &&rightSpeed>-0.1)
                    rightSpeed= -0.1;
                //give the rightDrive the opposite speed in order to rotate
                leftSpeed = -rightSpeed;

                leftDrive.setPower(leftSpeed);
                rightDrive.setPower(rightSpeed);
                error = getError(degrees);
                opMode.telemetry.addData("Error: ", error);
                opMode.telemetry.update();
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            t = opMode.getRuntime();
            while (opMode.getRuntime() < t + 300){
                error = getError(degrees);
                opMode.telemetry.addData("Error: ", error);
                opMode.telemetry.update();
            }
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void turnByGyroRelative(double degrees,double time){
        turnByGyroAbsolut(this.gyro.getAngle()+degrees,time);
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void trxDrive(Gamepad driver) {
         if (driver.a && driver.right_trigger != 0){
            leftDrive.setPower(driver.right_trigger);
            rightDrive.setPower(-driver.right_trigger);
        }else if (driver.a && driver.left_trigger != 0) {
             leftDrive.setPower(-driver.left_trigger);
             rightDrive.setPower(driver.left_trigger);
         }else if (driver.right_trigger != 0)
        {
            leftDrive.setPower(driver.right_trigger);
            rightDrive.setPower(driver.right_trigger);
        }else if (driver.left_trigger != 0){
            leftDrive.setPower(-driver.left_trigger);
            rightDrive.setPower(-driver.left_trigger);
        }else{
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
    }


    public void driveByEncoder(double distanceCm, double speed, Direction direction, double timeMs) {
        if (opMode instanceof LinearOpMode) {
            int newLeftTarget = 0;
            int newRightTarget = 0;

            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (direction == Direction.FORWARD) {
                newLeftTarget = (int) (distanceCm * COUNTS_PER_CM);
                newRightTarget = (int) (distanceCm * COUNTS_PER_CM);
            } else if (direction == Direction.BACKWARD) {
                newLeftTarget = (int) (distanceCm * COUNTS_PER_CM * -1);
                newRightTarget = (int) (distanceCm * COUNTS_PER_CM * -1);
            }

            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            double stopTime = opMode.getRuntime() + timeMs;
            while (((LinearOpMode) opMode).opModeIsActive() &&
                    (opMode.getRuntime() < stopTime) &&
                    (rightDrive.isBusy() && leftDrive.isBusy())) {
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}

