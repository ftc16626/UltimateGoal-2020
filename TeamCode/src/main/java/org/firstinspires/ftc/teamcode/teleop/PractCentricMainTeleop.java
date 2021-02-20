package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import static org.firstinspires.ftc.teamcode.teleop.PoseStorage.currentPose;


@TeleOp(name = "CentricTeleop", group = "Teleop")
@Config
public class PractCentricMainTeleop extends LinearOpMode {


    Robot robot = new Robot();


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


    public void shootControl() {
        if (gamepad2.b) {
            //Faster shooter speed for topgoal
            robot.shooterMotor.setPower(.525);
        } else if (gamepad2.x) {
            //Slower shooter speed for powershot
            robot.shooterMotor.setPower(.5);
        } else {
            //Allows manual control over speed (Not very useful though)
            robot.shooterMotor.setPower(gamepad2.right_trigger);

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
            robot.wobbleClaw.setPosition(Robot.CLAW_CLOSED);
        }
        if (gamepad2.left_bumper) {
            robot.wobbleClaw.setPosition(Robot.CLAW_OPENED);
        }

    }


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        drive.setPoseEstimate(new Pose2d(-63.0, -16.0, Math.toRadians(180)) );

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        //Init phase of opmode before START is pressed
        robot.init(hardwareMap);
        robot.wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();


        //Code to run after START is pressed
        while (!isStopRequested()) {
            Pose2d poseEstimate = null;

            if (gamepad1.right_trigger > 0) {
                poseEstimate = drive.getPoseEstimate();

                // Create a vector from the gamepad x/y inputs
                // Then, rotate that vector by the inverse of that heading
                Vector2d input = new Vector2d(
                        gamepad1.left_stick_x *.5,
                        -gamepad1.left_stick_y * .5
                ).rotated(-poseEstimate.getHeading());

                // Pass in the rotated input + right stick value for rotation
                // Rotation is not part of the rotated input thus must be passed in separately
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x * .5
                        )
                );

                // Update everything. Odometry. Etc.
                drive.update();
            }
            else {
                poseEstimate = drive.getPoseEstimate();

                // Create a vector from the gamepad x/y inputs
                // Then, rotate that vector by the inverse of that heading
                Vector2d input = new Vector2d(
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y
                ).rotated(-poseEstimate.getHeading());

                // Pass in the rotated input + right stick value for rotation
                // Rotation is not part of the rotated input thus must be passed in separately
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x
                        )
                );

                // Update everything. Odometry. Etc.
                drive.update();
            }


            shootControl();

            wobbleClawControl();

            intakeControl();

            wobbleArmControl();

            if (gamepad1.left_stick_button) {
                poseEstimate = drive.getPoseEstimate();

                // Create a vector from the gamepad x/y inputs
                // Then, rotate that vector by the inverse of that heading
                Vector2d input = new Vector2d(
                        gamepad1.left_stick_x *1.5,
                        -gamepad1.left_stick_y * 1.5
                ).rotated(-poseEstimate.getHeading());

                // Pass in the rotated input + right stick value for rotation
                // Rotation is not part of the rotated input thus must be passed in separately
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x * 1.5
                        )
                );

                // Update everything. Odometry. Etc.
                drive.update();
            }


        }
    }
}
