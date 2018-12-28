package org.firstinspires.ftc.teamcode.Autonomous.AutoBlue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotSystems.Climbing;
import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.RobotSystems.Robot;

@Autonomous(name = "BlueCreaterSampling", group = "Tests")
public class BlueCreaterSampling extends LinearOpMode{
    private Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        waitForStart();
        robot.climbing.land();
        robot.drive.turnByGyroAbsolut(40, 1500);
        robot.drive.Sampling();
        telemetry.addData("gyro angle: ", robot.drive.getAngle());
        telemetry.update();
        sleep(1500);
        robot.drive.driveToSampling();
        robot.climbing.moveLift(Climbing.Height.MEDIUM);
        robot.drive.turnByGyroAbsolut(65, 1500);
        robot.drive.driveByEncoder(70, 0.3, Drive.Direction.BACKWARD, 2000 );
        robot.drive.turnByGyroAbsolut(120, 1000);
        robot.drive.driveByEncoder(55, 0.3, Drive.Direction.BACKWARD, 2000);
        robot.drive.turnByGyroAbsolut(140, 1000);
        robot.drive.driveByEncoder(40, 0.3, Drive.Direction.BACKWARD, 2000);
        robot.intake.release();
        sleep(2500);
        robot.intake.stopMotor();
        //robot.drive.driveByEncoder(120, 0.5, Drive.Direction.FORWARD, 2000);
    }

}
