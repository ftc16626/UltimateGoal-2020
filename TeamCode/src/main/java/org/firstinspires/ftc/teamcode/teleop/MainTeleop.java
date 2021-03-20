package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import static org.firstinspires.ftc.teamcode.VeloPIDTuner.MOTOR_GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.VeloPIDTuner.MOTOR_TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.hardware.Robot.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.hardware.Robot.CLAW_OPENED;
import static org.firstinspires.ftc.teamcode.hardware.Robot.MOTOR_MAX_RPM;

@TeleOp(name = "Teleop", group = "Teleop")
@Config
public class MainTeleop extends LinearOpMode {

    Robot robot = new Robot();
    private double shooterVelo = 0.0;
    private ElapsedTime timer;

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

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

        if (gamepad1.right_trigger > 0) { //Makes robot drive slower
            robot.setDrivePower(gamepad1.left_stick_y * .3, -gamepad1.left_stick_x * .3, gamepad1.right_stick_x * .3);
        } else { //Allows robot to drive upto max speed
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
            setVelocity((DcMotorEx) robot.shooterMotor, (.5 * rpmToTicksPerSecond(MOTOR_MAX_RPM)));
            printVelocities((DcMotorEx) robot.shooterMotor, (.5 * rpmToTicksPerSecond(MOTOR_MAX_RPM)));
            //robot.shooterMotor.setPower(.5); //Speed before change was .5

        } else if (gamepad2.x) {
            //Slower shooter speed for powershot
            setVelocity((DcMotorEx) robot.shooterMotor, (.45 * rpmToTicksPerSecond(MOTOR_MAX_RPM)));
            printVelocities((DcMotorEx) robot.shooterMotor, (.45 * rpmToTicksPerSecond(MOTOR_MAX_RPM)));
            //robot.shooterMotor.setPower(.45); //Speed before change was .45

        } else {
            //Allows manual control over speed (Not very useful though)
            setVelocity((DcMotorEx) robot.shooterMotor, (gamepad2.right_trigger * rpmToTicksPerSecond(MOTOR_MAX_RPM)));
            printVelocities((DcMotorEx) robot.shooterMotor, (gamepad2.right_trigger * rpmToTicksPerSecond(MOTOR_MAX_RPM)));

        }

    }

    public void wobbleArmControl() {

        if (gamepad2.dpad_left) {
            robot.wobbleArm.setPower(-.3);
        } else if (gamepad2.dpad_right) {
            robot.wobbleArm.setPower(.3);
        } else if (gamepad2.dpad_up) {
            robot.wobbleArm.setTargetPosition(600);
            robot.wobbleArm.setPower(.3);
            telemetry.addData("Motor Position", String.valueOf(robot.wobbleArm.getCurrentPosition()));
            telemetry.update();
        } else {
            robot.wobbleArm.setPower(0);
        }


    }

    public void wobbleClawControl() {
        if (gamepad2.right_bumper) {
            robot.wobbleClaw.setPosition(CLAW_CLOSED);
        }
        if (gamepad2.left_bumper) {
            robot.wobbleClaw.setPosition(CLAW_OPENED);
        }

    }

    private void setVelocity(DcMotorEx motor, double power) {
        Log.i("power", Double.toString(power));
        motor.setVelocity(power);
    }

    private void printVelocities(DcMotorEx motor, double target) {
        telemetry.addData("targetVelocity", target);

        double motorVelo = motor.getVelocity();
        telemetry.addData("velocity", motorVelo);
        telemetry.addData("error", target - motorVelo);
    }

    @Override
    public void runOpMode() {

        //INIT phase of opmode before START is pressed
        robot.init(hardwareMap);
        robot.wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        //Code to loop after START is pressed
        while (!isStopRequested()) {

            if (gamepad1.left_trigger > 0) {
                driveControlTwo(); //Normal drive where the shooter is the back
            } else
                driveControl(); //Normal drive where the shooter is the front

            shootControl();

            wobbleClawControl();

            intakeControl();

            wobbleArmControl();

            telemetry.update();

        }
    }

}
