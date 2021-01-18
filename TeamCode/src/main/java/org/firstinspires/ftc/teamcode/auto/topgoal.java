package org.firstinspires.ftc.teamcode.auto;/* Copyright (c) 2017 FIRST. All rights reserved.
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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "topgoal", group = "Iterative Opmode")
public class topgoal extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Durations

    double start = 3.5;

    Robot robot = new Robot();


    public void rotate() {
        robot.setDrivePower(0, 0, .5);

    }

    public void strafeRight() {
        robot.frontRight.setPower(-.3);
        robot.backRight.setPower(.3);
        robot.frontLeft.setPower(.3);
        robot.backLeft.setPower(-.3);
    }

    public double duration(double duration) {
        start += duration;
        return start;
    }


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        MotorConfigurationType motorConfigurationType = robot.shooterMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        robot.shooterMotor.setMotorType(motorConfigurationType);

        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.wobbleClaw.setPosition(0);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        robot.shooterMotor.setPower(.58);


    }

    @Override
    public void loop() {

        //Shoots
        if (runtime.seconds() > start && runtime.seconds() < duration(.5)) {
            robot.intakeServo.setPower(1);

        } else if (runtime.seconds() > start && runtime.seconds() < duration(.5)) {
            robot.intakeServo.setPower(0);

        }
        if (runtime.seconds() > start && runtime.seconds() < duration(1)) {
            robot.intakeServo.setPower(1);

        }

        //

        if (runtime.seconds() > start && runtime.seconds() < duration(3.1)) {
            robot.shooterMotor.setPower(0);
            robot.intakeServo.setPower(0);
            robot.frontRight.setPower(-.4);
            robot.backRight.setPower(-.4);
            robot.frontLeft.setPower(-.38);
            robot.backLeft.setPower(-.38);
        }
        if (runtime.seconds() > start && runtime.seconds() < duration(2.8)) {
            robot.driveAll(0);
            rotate();
        }

        if (runtime.seconds() > start && runtime.seconds() < duration(1)) {
            robot.setDrivePower(0, 0, 0);
            robot.driveAll(0);
            robot.wobbleArm.setPosition(.35);

        }
        if (runtime.seconds() > start && runtime.seconds() < duration(1.1)) {
            robot.wobbleClaw.setPosition(.8);
        }


    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
