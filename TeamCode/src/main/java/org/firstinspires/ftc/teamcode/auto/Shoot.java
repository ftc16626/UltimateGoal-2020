package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Shoot", group = "Linear Opmode")
public class Shoot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory one = drive.trajectoryBuilder(new Pose2d(-63.0, -36.0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(10.0, -36.0, 0.0))
                .build();


        drive.setPoseEstimate(new Pose2d(-63.0, -36.0, Math.toRadians(180)));
        waitForStart();

        drive.followTrajectory(one);


    }
}
