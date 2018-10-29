package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot  {
    private Drive drive;
    private Intake intake;

    public void init(HardwareMap hardwareMap, OpMode opMode) {
        drive.init(hardwareMap, opMode);
        intake.init(hardwareMap, opMode);
    }

    public void teleop(Gamepad driver, Gamepad operator){
        drive.teleOpMotion(driver);
        intake.teleOpMotion(driver, operator);

    }

}
