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

package org.firstinspires.ftc.teamcode.Autonomous.Creater;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.teamcode.RobotSystems.Climbing;
import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.RobotSystems.Robot;

/**
 * doing Landing, aTeam Marker and Parking.
 * PTS=65
 * In case of broken robot
 * Starting from BlueCreaterEm
 */
@Autonomous(name = "BlueCreaterEm", group = "Tests")
@Disabled
public class BlueCreaterEm extends LinearOpMode {
  private Robot robot = new Robot();
  private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() throws InterruptedException {
    robot.init(hardwareMap , this);
    waitForStart();
    robot.climbing.land();

    robot.drive.driveByEncoder(20, 0.3, Drive.Direction.BACKWARD, 2000);
    robot.climbing.moveliftAuto(Climbing.Height.DRIVE_POS);
    robot.drive.turnByGyroRelative(95, 2000);
    robot.drive.driveByEncoder(90, 0.3, Drive.Direction.BACKWARD, 2000);
    robot.drive.turnByGyroRelative(40, 2000);
    robot.drive.driveByEncoder(60, 0.3, Drive.Direction.BACKWARD, 2000);
    robot.intake.release();
    sleep(500);
    robot.intake.stopMotor();
    robot.drive.driveByEncoder(100,0.5, Drive.Direction.FORWARD,1000);
    robot.drive.driveByEncoder(10,0.2, Drive.Direction.FORWARD,1000);
    }
}