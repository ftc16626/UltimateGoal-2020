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

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {


            robot.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.5, gamepad1.right_stick_x);


        }
    }
}
