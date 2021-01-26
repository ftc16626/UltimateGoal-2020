package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;


@TeleOp(name = "Teleop", group = "Linear Opmode")
public class MainTeleop extends LinearOpMode {

    double cruise = 0;
    boolean clawOpen;
    Robot robot = new Robot();

    public void intakeControl() {

        if (gamepad2.a && !robot.intakeLimit.isPressed()) { //Rotates intake servos

            robot.intakeMotor.setPower(1);
            robot.intakeServo.setPower(1);

        } else if (gamepad2.right_trigger > 0 && gamepad2.a) { //Allows intake to move rings while using the manual control
            robot.intakeMotor.setPower(1);
            robot.intakeServo.setPower(1);

        } else if (gamepad2.b && gamepad2.a) { //Allows intake to move rings while holding b
            robot.intakeMotor.setPower(1);
            robot.intakeServo.setPower(1);

        } else if (gamepad2.x && gamepad2.a) { //Allows intake to move rings while holding x
            robot.intakeMotor.setPower(1);
            robot.intakeServo.setPower(1);

        } else if (gamepad2.y) { //Reverses intake
            robot.intakeMotor.setPower(-1);
            robot.intakeServo.setPower(-1);

        } else { //Stops moving intake when no buttons are being used
            robot.intakeMotor.setPower(0);
            robot.intakeServo.setPower(0);

        }
    }

    public void driveControl() {

        if (gamepad1.right_trigger > 0) {
            robot.setDrivePower(gamepad1.left_stick_y * .3, -gamepad1.left_stick_x * .3, gamepad1.right_stick_x * .3);
        } else {
            robot.setDrivePower(gamepad1.left_stick_y, -gamepad1.left_stick_x * 1.5, gamepad1.right_stick_x);
        }


    }

    public void driveControlTwo() { //"Flips" what the front of the robot is in terms of what is forward and back while driving

        if (gamepad1.right_trigger > 0) {
            robot.setDrivePower(-gamepad1.left_stick_y * .3, gamepad1.left_stick_x * .3, gamepad1.right_stick_x * .3);
        } else {
            robot.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.5, gamepad1.right_stick_x);
        }


    }

    public void shootControl() {
        if (gamepad2.b) {
            //Faster shooter speed for topgoal
            robot.shooterMotor.setPower(.55 + cruise);
        } else if (gamepad2.x) {
            //Slower shooter speed for powershot
            robot.shooterMotor.setPower(.5 + cruise);
        } else {
            //Allows manual control over speed (Not very useful though)
            robot.shooterMotor.setPower(gamepad2.right_trigger);

        }

    }


    public void cruiseControl() {


    }


    public void wobbleArmControl() {
/*
        //All the way up
        if (gamepad2.left_stick_y != 0) {
            robot.wobbleArm.setPosition(1);
        }

        //intermediate
        if (gamepad2.dpad_right) {
            robot.wobbleArm.setPosition(.8);
        }
        //Down
        if (gamepad2.dpad_left) {
            robot.wobbleArm.setPosition(.2);
        }

 */
        //All the way up
        if (gamepad2.left_stick_y != 0) {
            robot.wobbleArm.setTargetPosition(160);
            robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleArm.setPower(.1);
        }

        //intermediate
        if (gamepad2.dpad_right) {
            robot.wobbleArm.setTargetPosition(444);
            robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleArm.setPower(.1);
        }
        //Down
        if (gamepad2.dpad_left) {
            robot.wobbleArm.setTargetPosition(818);
            robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleArm.setPower(.1);
            if (robot.wobbleArm.getCurrentPosition() == 810)
                robot.wobbleArm.setPower(0);
        }

    }

    public void wobbleClawControl() {
        if (gamepad2.right_bumper) {
            robot.wobbleClaw.setPosition(0);
        }
        if (gamepad2.left_bumper) {
            robot.wobbleClaw.setPosition(.4);
        }

    }


    @Override
    public void runOpMode() {

        //Init phase of opmode before START is pressed
        robot.init(hardwareMap);

        //Allows shooter motor to reach its max speed
        MotorConfigurationType motorConfigurationType = robot.shooterMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        robot.shooterMotor.setMotorType(motorConfigurationType);

        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cruise = 0;


        waitForStart();

        //Code to run after START is pressed
        while (!isStopRequested()) {

            if (!gamepad1.a) {
                if (gamepad1.left_trigger > 0) {
                    driveControlTwo(); //Normal drive where the shooter is the back
                } else
                    driveControl(); //Normal drive where the shooter is the front
            } else if (gamepad1.a) {
                robot.setDrivePower(0, 0, 0);
            }
            if (gamepad1.x) {
                robot.setDrivePower(0, 0, -10);
            }
            if (gamepad1.b) {
                robot.setDrivePower(0, 0, 10);
            }
            if (gamepad1.y) {
                robot.setDrivePower(0, 0, 15);
            }

            shootControl();
            cruiseControl();
            wobbleArmControl();
            wobbleClawControl();


            intakeControl();
            telemetry.addData("Claw", clawOpen);
            if (gamepad2.right_bumper)
                telemetry.addData("Button", "pressed");
            telemetry.update();


        }
    }
}
