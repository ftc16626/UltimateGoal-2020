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


@TeleOp(name = "Tank", group = "Tool")
public class TankTest extends LinearOpMode {

    Robot robot = new Robot();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.setDriveUsingEncoders();
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (!isStopRequested()) {


            if (gamepad1.left_stick_y > 0) {
                robot.frontRight.setPower(1);
                robot.wobbleArm.setPower(.1);

            }
            telemetry.addData("FrontLeft encoder", String.valueOf(robot.frontLeft.getCurrentPosition()));
            telemetry.addData("BackLeft encoder", String.valueOf(robot.backLeft.getCurrentPosition()));
            telemetry.addData("FrontRight encoder", String.valueOf(robot.frontRight.getCurrentPosition()));
            telemetry.addData("BackRight", String.valueOf(robot.backRight.getCurrentPosition()));
            telemetry.addData("Arm", String.valueOf(robot.wobbleArm.getCurrentPosition()));
            telemetry.update();
        }
    }
}
