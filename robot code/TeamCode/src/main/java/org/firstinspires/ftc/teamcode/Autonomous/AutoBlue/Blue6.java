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

package org.firstinspires.ftc.teamcode.Autonomous.AutoBlue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotSystems.Climbing;
import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.RobotSystems.Robot;

/**
 * doing 2 Sampling,Team Marker and Parking
 * PTS=75
 * Starting from blueCrater
 */
@Autonomous(name = "Blue6", group = "Tests")
@Disabled
public class Blue6 extends LinearOpMode {

  private Robot robot = new Robot();
  private ElapsedTime runtime = new ElapsedTime();


  @Override
  public void runOpMode() throws InterruptedException {
    //TODO:Check values with Shalev(the god)
    robot.init(hardwareMap , this);
    waitForStart();
    //TODO: Use pixy to do sampling
    //robot.drive.moveByPixy();
    //Sampling
    robot.drive.driveByEncoder(30, 0.5, Drive.Direction.FORWARD, 2500);
    robot.drive.turnByGyroAbsolut(90,1500);//FIX
    //TurnLeft
    robot.drive.driveByEncoder(60, 0.5, Drive.Direction.FORWARD, 2500);//FIX
   //drive forward
    robot.drive.turnByGyroAbsolut(90,15001);//FIX
    //Rotate left
    robot.drive.driveByEncoder(60, 0.5, Drive.Direction.FORWARD, 2500);//FIX
    //drive forward
    robot.drive.turnByGyroAbsolut(90,2000);//FIX
    //Rotate to the Sampling
    //TODO: Use pixy to do sampling
    //Correct our angle
    robot.drive.driveByEncoder(55, 0.5, Drive.Direction.FORWARD, 2300);
    //driving to the depot
    robot.intake.release(); //released the Team Marker
    wait(1500);
    robot.drive.turnByGyroAbsolut(180,1500); // Rotate to the crater
    robot.drive.driveByEncoder(210,0.7, Drive.Direction.FORWARD, 3500);
    // driving to the crater in order to park


  }
}