package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;


@TeleOp(name = "Teleop", group = "Linear Opmode")
public class MainTeleop extends LinearOpMode {

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

    public void shootControl() {
        if (gamepad2.b) {

            robot.shooterMotor.setPower(.7);
        } else if (gamepad2.x) {
            robot.shooterMotor.setPower(.6);
        } else {
            robot.shooterMotor.setPower(gamepad2.right_trigger);
        }
    }

    public void wobbleArmControl() {

        while (gamepad2.left_trigger > 0) {
            robot.wobbleArm.setPosition(gamepad2.left_trigger);
        }
        if (gamepad2.dpad_down) {
            robot.wobbleArm.setPosition(.5);
        }
        if (gamepad2.dpad_left) {
            robot.wobbleArm.setPosition(.6);
        }
        if (gamepad2.dpad_up) {
            robot.wobbleArm.setPosition(1);
        }


    }

    public void wobbleClawControl() {
        if (gamepad2.right_bumper) {
            robot.wobbleClaw.setPosition(1);
        }
        if (gamepad2.left_bumper) {
            robot.wobbleClaw.setPosition(0);
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        MotorConfigurationType motorConfigurationType = robot.shooterMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        robot.shooterMotor.setMotorType(motorConfigurationType);

        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();


        while (!isStopRequested()) {

            driveControl();
            shootControl();
            wobbleArmControl();
            wobbleClawControl();
            intakeControl();


        }
    }
}
