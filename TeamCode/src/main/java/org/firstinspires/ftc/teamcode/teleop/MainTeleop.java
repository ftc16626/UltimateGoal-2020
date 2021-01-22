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
    Robot robot = new Robot();

    public void intakeControl() {

        if (gamepad2.a && !robot.intakeLimit.isPressed()) {

            robot.intakeMotor.setPower(1);
            robot.intakeServo.setPower(1);

        } else if (gamepad2.right_trigger > 0 && gamepad2.a) {
            robot.intakeMotor.setPower(1);
            robot.intakeServo.setPower(1);

        } else if (gamepad2.b && gamepad2.a) {
            robot.intakeMotor.setPower(1);
            robot.intakeServo.setPower(1);

        } else if (gamepad2.x && gamepad2.a) {
            robot.intakeMotor.setPower(1);
            robot.intakeServo.setPower(1);

        } else if (gamepad2.y) {
            robot.intakeMotor.setPower(-1);
            robot.intakeServo.setPower(-1);

        } else {
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

    public void driveControlTwo() {

        if (gamepad1.right_trigger > 0) {
            robot.setDrivePower(-gamepad1.left_stick_y * .3, gamepad1.left_stick_x * .3, gamepad1.right_stick_x * .3);
        } else {
            robot.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.5, gamepad1.right_stick_x);
        }


    }

    public void shootControl() {
        if (gamepad2.b) {

            robot.shooterMotor.setPower(.55 + cruise);
        } else if (gamepad2.x) {

            robot.shooterMotor.setPower(.5 + cruise);
        } else {
            robot.shooterMotor.setPower(gamepad2.right_trigger);
        }

    }

    public void cruiseControl() {
        if (gamepad2.dpad_up) {
            cruise += .2;
        }
        if (gamepad2.dpad_down) {
            cruise -= .2;
        }
    }

    public void wobbleArmControl() {

        /*
        if (gamepad2.dpad_right) {
            robot.wobbleArm.setPosition(.35);

        }
        
         */
        //Up
        if (gamepad2.dpad_right) {
            robot.wobbleArm.setPosition(.5);
        }
        //Down
        if (gamepad2.dpad_left) {
            robot.wobbleArm.setPosition(1);
        }


    }

    public void wobbleClawControl() {
        if (gamepad2.right_bumper) {
            robot.wobbleClaw.setPosition(0);
        }
        if (gamepad2.left_bumper) {
            robot.wobbleClaw.setPosition(1);
        }

    }


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        MotorConfigurationType motorConfigurationType = robot.shooterMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        robot.shooterMotor.setMotorType(motorConfigurationType);

        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cruise = 0;
        waitForStart();

        while (!isStopRequested()) {

            if (!gamepad1.a) {
                if (gamepad1.left_trigger > 0) {
                    driveControlTwo();
                } else
                    driveControl();
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


        }
    }
}
