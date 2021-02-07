package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "ParkAll Red", group = "Autonomous Red")
public class ParkAllRed extends LinearOpMode {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "webcam"; // insert webcam name from configuration if using webcam
    Robot robot = new Robot();
    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        robot.wobbleClaw.setPosition(0);


        waitForStart();

        sleep(3000);
        String height = height = String.valueOf(pipeline.getHeight());
        telemetry.addData("[Ring Stack] >>", height);
        telemetry.update();

        if (height.equals("ONE")) {  //Zone B

            auto2(-18.0, -28.0, Math.toRadians(180));
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Trajectory two = drive.trajectoryBuilder(new Pose2d(-18.0, -28.0, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(10.0, -28.0, 0.0))
                    .build();
            drive.setPoseEstimate(new Pose2d(-18.0, -28.0, Math.toRadians(180)));
            drive.followTrajectory(two);


            sleep(2000);
            robot.wobbleArm.setPower(.5);
            sleep(1000);
            robot.wobbleArm.setPower(0);
            robot.wobbleClaw.setPosition(.4);

        }


        if (height.equals("FOUR")) {  //Zone C
            auto2(40.0, -55.0, 0.0);
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Trajectory two = drive.trajectoryBuilder(new Pose2d(40.0, -55.0, 0.0))
                    .lineToLinearHeading(new Pose2d(0.0, -55.0, 0.0))
                    .build();
            drive.setPoseEstimate(new Pose2d(40, -55.0, 0.0));
            drive.followTrajectory(two);


        }

        if (height.equals("ZERO")) { //Zone A
            { //Zone A
                SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
                Trajectory two = drive.trajectoryBuilder(new Pose2d(-63.0, -36.0, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(-5.0, -63.0, 0.0))
                        .build();
                drive.setPoseEstimate(new Pose2d(-63.0, -36.0, Math.toRadians(180)));


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

                drive.followTrajectory(two);
                robot.wobbleArm.setPower(.5);

                sleep(1000);
                robot.wobbleArm.setPower(0);
                robot.wobbleClaw.setPosition(.4);

                sleep(500);
            }

        }


    }

    private void auto(double x, double y, double h) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory one = drive.trajectoryBuilder(new Pose2d(-63.0, -36.0, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(x, y), h)
                .build();

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

    private void auto2(double x, double y, double h) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory one = drive.trajectoryBuilder(new Pose2d(-63.0, -36.0, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(x, y, h))
                .build();

        drive.setPoseEstimate(new Pose2d(-63.0, -36.0, Math.toRadians(180)));


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






