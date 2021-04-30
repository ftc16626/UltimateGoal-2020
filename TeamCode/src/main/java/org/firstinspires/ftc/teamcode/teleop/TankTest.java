package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Robot;


@TeleOp(name = "Tank", group = "Tool")
public class TankTest extends LinearOpMode {

    Robot robot = new Robot();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();


        while (!isStopRequested()) {
            telemetry.addData("encoder", robot.wobbleArm.getCurrentPosition());
            telemetry.update();

        }

    }

}


