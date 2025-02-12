package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Prototyping.LogCreator;

public class Robot  {
    public Drive drive = new Drive();
    public Intake intake = new Intake();
    public Climbing climbing = new Climbing();

    public void init(HardwareMap hardwareMap, OpMode opMode) {
        drive.init(hardwareMap, opMode);
        intake.init(hardwareMap, opMode);
        climbing.init(hardwareMap, opMode);
    }

    public void teleop(Gamepad driver, Gamepad operator){
        drive.teleOpMotion(driver);
        intake.teleOpMotion(driver, operator);
        climbing.teleOpMotion(operator);
    }

}
