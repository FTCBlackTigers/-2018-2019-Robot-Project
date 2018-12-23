package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Pixy.PixyBlock;
import org.firstinspires.ftc.teamcode.Pixy.PixyCam;

public class Drive {
    public enum Direction {
        FORWARD, BACKWARD;
    }


    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 40;
    static final double WHEEL_DIAMETER_CM = 10.16;
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.141592654);
    static final double AUTO_TURN_SPEED = 0.2;
    static final double THRESHOLD = 5;
    static final double P_TURN_COEFF = 0.075;
    private final int PIXY_MIDDLE_PIXEL = 200;
    private final int PIXY_PIXEL_THRESHHOLDD = 10;

    private OpMode opMode;
    private BT_Gyro gyro = new BT_Gyro();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    PixyCam pixyCam;

    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        gyro.init(hardwareMap);
        pixyCam = hardwareMap.get(PixyCam.class, "pixy");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void teleOpMotion(Gamepad driver) {
        if (driver.dpad_up) {
            leftDrive.setPower(0.2);
            rightDrive.setPower(0.2);
        } else if (driver.dpad_down) {
            leftDrive.setPower(-0.2);
            rightDrive.setPower(-0.2);
        } else tankDrive(-driver.left_stick_y, -driver.right_stick_y);

        opMode.telemetry.addLine("Drive: ").
                addData("left motor power: ", leftDrive.getPower()).
                addData("right motor power: ", rightDrive.getPower())
                .addData("left motor pos: ", leftDrive.getCurrentPosition())
                .addData("right motor pos: ", rightDrive.getCurrentPosition());

    }


    private void tankDrive(double powerLeftDrive, double powerRightDrive) {
        leftDrive.setPower(powerLeftDrive * 0.5);
        rightDrive.setPower(powerRightDrive * 0.5);
    }

    public void turnByGyroAbsolut(double degrees, double timeMs) {
        double rightSpeed, leftSpeed;
        double steer;
        double error = getError(degrees);
        double time;
        double stopTime = opMode.getRuntime() + timeMs;
        // keep looping while we are still active, and not on heading.
        while ((Math.abs(error) > THRESHOLD) && (opMode.getRuntime() < stopTime)) {
            while (Math.abs(error) > THRESHOLD) {
                // Update telemetry & Allow time for other processes to run.
                steer = getSteer(error, P_TURN_COEFF);
                rightSpeed = AUTO_TURN_SPEED * steer;
                if (rightSpeed > 0 && rightSpeed < 0.1)
                    rightSpeed = 0.1;
                else if (rightSpeed < 0 && rightSpeed > -0.1)
                    rightSpeed = -0.1;
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
            time = opMode.getRuntime();
            /*while (opMode.getRuntime() < t + 300){
                error = getError(degrees);
                opMode.telemetry.addData("Error: ", error);
                opMode.telemetry.update();
            }*/
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void turnByGyroRelative(double degrees, double time) {
        turnByGyroAbsolut(this.gyro.getAngle() + degrees, time);
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getAngle();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void trxDrive(Gamepad driver) {
        if (driver.a && driver.right_trigger != 0) {
            leftDrive.setPower(driver.right_trigger);
            rightDrive.setPower(-driver.right_trigger);
        } else if (driver.a && driver.left_trigger != 0) {
            leftDrive.setPower(-driver.left_trigger);
            rightDrive.setPower(driver.left_trigger);
        } else if (driver.right_trigger != 0) {
            leftDrive.setPower(driver.right_trigger);
            rightDrive.setPower(driver.right_trigger);
        } else if (driver.left_trigger != 0) {
            leftDrive.setPower(-driver.left_trigger);
            rightDrive.setPower(-driver.left_trigger);
        } else {
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
                opMode.telemetry.addData("leftPos", leftDrive.getCurrentPosition());
                opMode.telemetry.addData("rightPos", rightDrive.getCurrentPosition());
                opMode.telemetry.update();
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //MY BLOCKS OF GOD

    // BLOCK - CM TO ROTATION
    public int cmToRotations(double cm) {
        return (int) (cm / WHEEL_DIAMETER_CM);
    }

    // BLOCK - MOVE FORWARD
    public void moveForward(double cm, double power) {
        leftDrive.setTargetPosition(cmToRotations(cm));
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightDrive.setTargetPosition(cmToRotations(cm));
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    // BLOCK - ROTATE
    public void rotate(double angle, double power) {
        double rotations = (40 * angle) / WHEEL_DIAMETER_CM;
        leftDrive.setTargetPosition(cmToRotations(rotations));
        rightDrive.setTargetPosition(cmToRotations(rotations));

        leftDrive.setPower(-power);
        rightDrive.setPower(power);
    }

    public void moveByPixy() {
        PixyBlock block = pixyCam.getBiggestBlocks(1).get(2);
        opMode.telemetry.addLine(block.toString());
        opMode.telemetry.update();
        int error = block.x - PIXY_MIDDLE_PIXEL;
        while (((LinearOpMode) opMode).opModeIsActive() && Math.abs(error) >= PIXY_PIXEL_THRESHHOLDD && !block.isEmpty()) {
            leftDrive.setPower(0.2 * Math.signum(error));
            rightDrive.setPower(0.2 * -Math.signum(error));
            error = block.x - PIXY_MIDDLE_PIXEL;
            opMode.telemetry.addLine(block.toString());
            opMode.telemetry.update();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}




