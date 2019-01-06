/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Prototyping;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import java.io.*;
import java.util.Date;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.RobotConfigNameable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.RobotSystems.Robot;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

public class LogCreator {
    Date date = new Date();
    private final String PATH = "/sdcard/FIRST/log.txt";
    private Robot robot = new Robot();
    private static java.io.File file;
    FileOutputStream fileOutputStream;
    PrintStream printStream;


    public void createFile() throws FileNotFoundException {
        try {
            file = new java.io.File(PATH);
            fileOutputStream = new FileOutputStream(file);
            printStream = new PrintStream(fileOutputStream);
        } catch (FileNotFoundException x) {
            throw x;
        }
    }

    public void writeToFile(String str) {
        try {
            printStream.write(str.getBytes());
        } catch (IOException x) {
        }

    }

    public void closeFile() {
        try {
            printStream.close();
            fileOutputStream.close();
        } catch (IOException x) {

        }


    }

    /*public void logCrate(HardwareMap hardwareMap) {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        try {
            fileOutputStream = new FileOutputStream(file);
            printStream = new PrintStream(fileOutputStream);

            printStream.write("Time | gyro pos | leftMotor pos,Power | rightMotorpos,Power | liftMotor Pos,Power | angleMotor pos,Power | liftTouch | angleTouch | RcolorSensor | LcolorSensor | RightServo pos | LeftServo pos | hang servo pos | ptentiometer voltage |\n".getBytes());

            //while (this.opModeIsActive()){
                printStream.write((date.getMinutes() + ":" + date.getSeconds() + "|").getBytes());
                printStream.write((robot.drive.getAngle()+ "|").getBytes());
                printStream.write((robot.drive.getLeftMotor().getCurrentPosition() + "," + robot.drive.getLeftMotor().getPower()+ "|").getBytes());
                printStream.write((robot.drive.getrightMotor().getCurrentPosition() + "," + robot.drive.getrightMotor().getPower()+ "|").getBytes());
                printStream.write((robot.climbing.getLiftMotor().getCurrentPosition() + "," + robot.climbing.getAngleMotor().getPower()+ "|").getBytes());
                printStream.write((robot.climbing.LiftTouchPressed()+ "|").getBytes());
                printStream.write((robot.climbing.AngleTouchPressed()+ "|").getBytes());
                printStream.write((robot.intake.rightColor()+ "|").getBytes());
                printStream.write((robot.intake.leftColor()+ "|").getBytes());
                printStream.write((robot.intake.rightServo.getPosition() + "|").getBytes());
                printStream.write((robot.intake.leftServo.getPosition() + "|").getBytes());
                printStream.write((robot.climbing.getHangServo().getPosition() + "|").getBytes());
                printStream.write("\n".getBytes());
            //}

            printStream.close();
            fileOutputStream.close();

        } catch (IOException e) {
        } finally {

        }

    }*/
}
