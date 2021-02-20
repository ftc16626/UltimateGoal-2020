package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Disabled
@TeleOp(name = "LimitTest", group = "Tool")
public class LimitTest extends LinearOpMode {

    Robot robot = new Robot();




    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while (!isStopRequested()) {



            if (gamepad2.a)
                robot.wobbleClaw.setPosition(0);
            if (gamepad2.b)
                robot.wobbleClaw.setPosition(.5);
            if (gamepad2.y)
                robot.wobbleClaw.setPosition(1);
            if (gamepad2.x)
                robot.wobbleClaw.setPosition(0);

            telemetry.addData("Servo Position", String.valueOf(robot.wobbleClaw.getPosition()));
            telemetry.addData("Motor Position", String.valueOf(robot.wobbleArm.getCurrentPosition()));
            telemetry.addData("Servo Position", String.valueOf(robot.wobbleClaw.getPosition()));
            telemetry.update();
        }
    }
}
