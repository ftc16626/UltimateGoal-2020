package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;


@TeleOp(name = "Teleop", group = "Linear Opmode")
public class MainTeleop extends LinearOpMode {

    Robot robot = new Robot();


    public void intake() {

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

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {


            driveControl();
            shootControl();
            intake();


        }
    }
}
