package org.firstinspires.ftc.teamcode.Autonomous.AutoBlue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.RobotSystems.Robot;

@Autonomous(name = "BlueCreaterSampling", group = "Tests")
public class BlueCreaterSampling extends LinearOpMode{
    private Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap , this);
        waitForStart();
        robot.climbing.land();
        robot.drive.turnByGyroAbsolut(40, 1500);
        robot.drive.Sampling();
        robot.drive.driveByEncoder(65, 0.3, Drive.Direction.BACKWARD, 5000);
        robot.drive.driveByEncoder(65, 0.3, Drive.Direction.FORWARD, 5000);
        robot.drive.turnByGyroAbsolut(60, 1000);
        robot.drive.driveByEncoder(70, 0.3, Drive.Direction.BACKWARD, 2000);
        robot.drive.turnByGyroRelative(60, 1000);
        robot.drive.driveByEncoder(70, 0.5, Drive.Direction.BACKWARD, 2000);
        robot.intake.release();
        sleep(500);
        robot.intake.stopMotor();
        robot.drive.driveByEncoder(120, 0.5, Drive.Direction.FORWARD, 2000);

    }
}
