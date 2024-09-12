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

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Launcher Test Control", group = "Concept")
@Disabled
public class Launcher_Test_Control extends LinearOpMode {


    static final double MAX_POS     =  .27;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    double MaxAngle = 90;
    double MinAngle = 20;


    // Define class members
    private Servo LaunchServo;
    private double currentangle =  0;
    private double currentposition = 0;

    double  startposition = (MAX_POS - MIN_POS) / 2; // Start at halfway position


    // Setup Motor
private DcMotorEx LaunchMotor = null;



    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        LaunchServo = hardwareMap.get(Servo.class, "launchservo");
        LaunchMotor=hardwareMap.get(DcMotorEx.class,"launchmotor");

        LaunchMotor.setDirection(DcMotorSimple.Direction.REVERSE);

   LaunchServo.setPosition(startposition);
        currentposition = LaunchServo.getPosition();


        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){       
            // Servo control using gamepad left bumper and trigger
            if (gamepad1.left_bumper) {
                LaunchServo.setPosition(LaunchServo.getPosition() + 0.01);
            } else if (gamepad1.left_trigger > 0) {
                LaunchServo.setPosition(LaunchServo.getPosition() - 0.01);


            }
            sleep(100);

            // Motor control using gamepad X button
            if (gamepad1.x) {
                // Start the motor when X is pressed
                LaunchMotor.setPower(1.0);
            } else {
                // Stop the motor when X is released
                LaunchMotor.setPower(0.0);
            }

            // You can add more controls or logic as needed
            currentposition = LaunchServo.getPosition();
            currentangle = (((currentposition/(MAX_POS-MIN_POS))*MaxAngle-MinAngle))+MinAngle;

                    telemetry.addData("Current servo position", currentposition);
                    telemetry.addData("Current Angle", currentangle);
                    telemetry.update();
        
        }
    }






        }



