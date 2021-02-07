package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Robot;


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



            telemetry.addData("Motor Position", String.valueOf(robot.wobbleArm.getCurrentPosition()));
            telemetry.addData("Servo Position", String.valueOf(robot.wobbleClaw.getPosition()));
            telemetry.update();
        }
    }
}
