package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name = "ParkOne Red", group = "Autonomous Red")
public class ParkOneRed extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        //Setup roadrunner trajectory
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);
        Trajectory one = drive.trajectoryBuilder(new Pose2d(-63.0, -36.0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(10.0, -28.0, 0.0))
                .build();

        drive.setPoseEstimate(new Pose2d(-63.0, -36.0, Math.toRadians(180)));
        //
        robot.wobbleClaw.setPosition(0);

        waitForStart();


        robot.shooterMotor.setPower(.58);


        sleep(5000);


        robot.intakeServo.setPower(1);

        sleep(1000);

        robot.intakeServo.setPower(0);

        sleep(2000);

        robot.intakeServo.setPower(1);

        sleep(4000);

        robot.shooterMotor.setPower(0);
        robot.intakeServo.setPower(0);


        drive.followTrajectory(one);

        robot.wobbleArm.setPower(.5);

        sleep(1000);
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(.4);

        sleep(500);

    }
}
