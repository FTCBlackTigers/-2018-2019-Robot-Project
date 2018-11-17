package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {
    enum Direction {
        FORWARD, BACKWARD;
    }

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 19.2;    //TODO: check the DRIVE_GEAR_REDUCTION
    static final double WHEEL_DIAMETER_CM = 4.0;    //TODO: check the WHEEL_DIAMETER_CM
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.141592654);

    private OpMode opMode;

    //TODO: add gyro;
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

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
