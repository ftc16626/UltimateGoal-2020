package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Disabled
@TeleOp(name = "Tank", group = "Tool")
public class TankTest extends LinearOpMode {

    Robot robot = new Robot();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();


        while (!isStopRequested()) {
            if (gamepad2.a)
                robot.shooterMotor.setPower(.2);
            if (gamepad2.b)
                robot.shooterMotor.setPower(.4);
            if (gamepad2.y)
                robot.shooterMotor.setPower(.45);
            if (gamepad2.x)
                robot.shooterMotor.setPower(.55);
            if (gamepad2.right_trigger > 0) {
                robot.intakeServo.setPower(1);
                robot.intakeMotor.setPower(1);
            } else {
                robot.intakeServo.setPower(0);
                robot.intakeMotor.setPower(0);
            }

        }

    }

}


