package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.teleop.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "IMPROVED", group = "Autonomous Red")
public class ParkAllRedImproved extends LinearOpMode {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final int HORIZON = 100; // horizon value to tune
    private static final boolean DEBUG = false; // if debug is wanted, change to true
    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "webcam"; // insert webcam name from configuration if using webcam

    private final double POWER_SHOT_ONE = 6.0; //X coordinate of PS ONE
    private final double POWER_SHOT_TWO = 15.5; //X coordinate of PS TWO
    private final double POWER_SHOT_THREE = 23.5; //X coordinate of PS THREE

    Robot robot = new Robot();
    Pose2d startPose = new Pose2d(-59, -17, 0.0); //Starting position of robot

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.wobbleClaw.setPosition(Robot.CLAW_CLOSED);

        //Runs the code to search for rings with camera
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

        robot.wobbleClaw.setPosition(Robot.CLAW_CLOSED); //Closes claw again?
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        String height = "UNKNOWN"; //height equals unknown until it senses the rings
        while (!isStarted()) { //Continually displays whatever height is (0, 1, or 4)
            height = String.valueOf(pipeline.getHeight());
            telemetry.addData("[Ring Stack] >>", height);
            telemetry.update();
        }

        waitForStart();
        //Code to run once after start is pressed

        if (height.equals("ZERO")) //Zone A
            zero();


        if (height.equals("ONE")) //Zone B
            one();


        if (height.equals("FOUR")) //Zone C
            four();

    }

    private void zero() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        robot.shooterMotor.setPower(.45);

        Trajectory one = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-2.0, -35.0))

                .build();

        Trajectory two = drive.trajectoryBuilder(one.end())
                .lineToSplineHeading(new Pose2d(3.0, POWER_SHOT_ONE, Math.toRadians(180)))
                .build();

        Trajectory twoshottwo = drive.trajectoryBuilder(two.end())
                .strafeRight(POWER_SHOT_TWO - POWER_SHOT_ONE)
                .build();

        Trajectory twoshotthree = drive.trajectoryBuilder(twoshottwo.end())
                .strafeRight(POWER_SHOT_THREE - POWER_SHOT_TWO)
                .build();


        Trajectory three = drive.trajectoryBuilder(two.end())
                .splineToLinearHeading(new Pose2d(-40.0, 11, Math.toRadians(270)), Math.toRadians(180))
                .build();

        Trajectory four = drive.trajectoryBuilder(three.end())

                .splineToConstantHeading(new Vector2d(21.0, -8.0), 0.0)


                .build();

        Trajectory five = drive.trajectoryBuilder(four.end())

                .back(3.0)


                .build();


        drive.followTrajectory(one);
        robot.wobbleArm.setTargetPosition(645);
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.wobbleArm.setPower(.15);
        while (robot.wobbleArm.isBusy()) {
            telemetry.addData("waiting", "...");
            telemetry.update();
        }
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_OPENED);


        drive.followTrajectory(two);
        while (robot.intakeLimit.isPressed()) {
            robot.intakeServo.setPower(1);
            robot.intakeMotor.setPower(.5);
        }
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);
        while (!robot.intakeLimit.isPressed()) {
            robot.intakeServo.setPower(1);
            robot.intakeMotor.setPower(.5);
        }
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);

        telemetry.addData("FIRST RING", "FINISHED");
        telemetry.update();


        drive.followTrajectory(twoshottwo);
        while (robot.intakeLimit.isPressed()) {
            robot.intakeServo.setPower(1);
            robot.intakeMotor.setPower(.5);
        }
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);
        while (!robot.intakeLimit.isPressed()) {
            robot.intakeServo.setPower(1);
            robot.intakeMotor.setPower(.5);
        }
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);


        telemetry.addData("SECOND RING", "FINISHED");
        telemetry.update();


        drive.followTrajectory(twoshotthree);
        while (robot.intakeLimit.isPressed()) {
            robot.intakeServo.setPower(1);
            robot.intakeMotor.setPower(.5);
        }
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);

        telemetry.addData("THRID RING", "FINISHED");
        telemetry.update();


        drive.followTrajectory(three);
        robot.wobbleClaw.setPosition(Robot.CLAW_CLOSED);
        robot.wobbleArm.setTargetPosition(0);
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.wobbleArm.setPower(.15);
        while (robot.wobbleArm.isBusy()) {
            telemetry.addData("waiting again", "...");
            telemetry.update();
        }
        robot.wobbleArm.setPower(0);


        drive.followTrajectory(four);
        robot.wobbleArm.setTargetPosition(645);
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.wobbleArm.setPower(.15);
        while (robot.wobbleArm.isBusy()) {
            telemetry.addData("waiting", "...");
            telemetry.update();
        }
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_OPENED);
        drive.followTrajectory(five);

        PoseStorage.currentPose = drive.getPoseEstimate();


    }

    private void one() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        robot.shooterMotor.setPower(.45);

        Trajectory one = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(16.0, -12.0))

                .build();

        Trajectory two = drive.trajectoryBuilder(one.end())
                .lineToSplineHeading(new Pose2d(-2.0, POWER_SHOT_ONE, Math.toRadians(180)))
                .build();

        Trajectory twoshottwo = drive.trajectoryBuilder(two.end())
                .strafeRight(POWER_SHOT_TWO - POWER_SHOT_ONE)
                .build();

        Trajectory twoshotthree = drive.trajectoryBuilder(twoshottwo.end())
                .strafeRight(POWER_SHOT_THREE - POWER_SHOT_TWO)
                .build();


        Trajectory three = drive.trajectoryBuilder(two.end())
                .splineToLinearHeading(new Pose2d(-40.0, 9.5, Math.toRadians(270)), Math.toRadians(180))
                .build();

        Trajectory four = drive.trajectoryBuilder(three.end())

                .splineToConstantHeading(new Vector2d(41.0, 24.5), 0.0)

                .build();

        Trajectory five = drive.trajectoryBuilder(four.end())

                .strafeRight(26.0)

                .build();


        drive.followTrajectory(one);
        robot.wobbleArm.setTargetPosition(645);
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.wobbleArm.setPower(.15);
        while (robot.wobbleArm.isBusy()) {
            telemetry.addData("waiting", "...");
            telemetry.update();
        }
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_OPENED);


        drive.followTrajectory(two);
        while (robot.intakeLimit.isPressed()) {
            robot.intakeServo.setPower(1);
            robot.intakeMotor.setPower(.5);
        }
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);
        while (!robot.intakeLimit.isPressed()) {
            robot.intakeServo.setPower(1);
            robot.intakeMotor.setPower(.5);
        }
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);

        telemetry.addData("FIRST RING", "FINISHED");
        telemetry.update();


        drive.followTrajectory(twoshottwo);
        while (robot.intakeLimit.isPressed()) {
            robot.intakeServo.setPower(1);
            robot.intakeMotor.setPower(.5);
        }
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);
        while (!robot.intakeLimit.isPressed()) {
            robot.intakeServo.setPower(1);
            robot.intakeMotor.setPower(.5);
        }
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);


        telemetry.addData("SECOND RING", "FINISHED");
        telemetry.update();


        drive.followTrajectory(twoshotthree);
        while (robot.intakeLimit.isPressed()) {
            robot.intakeServo.setPower(1);
            robot.intakeMotor.setPower(.5);
        }
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);

        telemetry.addData("THRID RING", "FINISHED");
        telemetry.update();


        drive.followTrajectory(three);
        robot.wobbleClaw.setPosition(Robot.CLAW_CLOSED);
        robot.wobbleArm.setTargetPosition(0);
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.wobbleArm.setPower(.15);
        while (robot.wobbleArm.isBusy()) {
            telemetry.addData("waiting again", "...");
            telemetry.update();
        }
        robot.wobbleArm.setPower(0);


        drive.followTrajectory(four);
        robot.wobbleArm.setTargetPosition(645);
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.wobbleArm.setPower(.15);
        while (robot.wobbleArm.isBusy()) {
            telemetry.addData("waiting", "...");
            telemetry.update();
        }
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_OPENED);


        robot.wobbleArm.setTargetPosition(0);
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.wobbleArm.setPower(.15);
        while (robot.wobbleArm.isBusy()) {
            telemetry.addData("waiting again", "...");
            telemetry.update();
        }
        robot.wobbleArm.setPower(0);
        drive.followTrajectory(five);

        PoseStorage.currentPose = drive.getPoseEstimate();


    }

    private void four() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        robot.shooterMotor.setPower(.45);

        Trajectory one = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(45.0, -36.0))

                .build();

        Trajectory two = drive.trajectoryBuilder(one.end())
                .lineToSplineHeading(new Pose2d(-2.0, POWER_SHOT_ONE, Math.toRadians(180)))
                .build();

        Trajectory twoshottwo = drive.trajectoryBuilder(two.end())
                .strafeRight(POWER_SHOT_TWO - POWER_SHOT_ONE)
                .build();

        Trajectory twoshotthree = drive.trajectoryBuilder(twoshottwo.end())
                .strafeRight(POWER_SHOT_THREE - POWER_SHOT_TWO)
                .build();


        Trajectory three = drive.trajectoryBuilder(two.end())

                .splineToLinearHeading(new Pose2d(-41.0, 11, Math.toRadians(270)), Math.toRadians(180))

                .build();

        Trajectory four = drive.trajectoryBuilder(three.end())

                .splineToConstantHeading(new Vector2d(67, 0.5), 0.0)

                .build();

        Trajectory five = drive.trajectoryBuilder(four.end())

                .strafeRight(48.0)

                .build();


        drive.followTrajectory(one);
        robot.wobbleArm.setTargetPosition(645);
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.wobbleArm.setPower(.15);
        while (robot.wobbleArm.isBusy()) {
            telemetry.addData("waiting", "...");
            telemetry.update();
        }
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_OPENED);


        drive.followTrajectory(two);
        while (robot.intakeLimit.isPressed()) {
            robot.intakeServo.setPower(1);
            robot.intakeMotor.setPower(.5);
        }
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);
        while (!robot.intakeLimit.isPressed()) {
            robot.intakeServo.setPower(1);
            robot.intakeMotor.setPower(.5);
        }
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);

        telemetry.addData("FIRST RING", "FINISHED");
        telemetry.update();


        drive.followTrajectory(twoshottwo);
        while (robot.intakeLimit.isPressed()) {
            robot.intakeServo.setPower(1);
            robot.intakeMotor.setPower(.5);
        }
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);
        while (!robot.intakeLimit.isPressed()) {
            robot.intakeServo.setPower(1);
            robot.intakeMotor.setPower(.5);
        }
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);


        telemetry.addData("SECOND RING", "FINISHED");
        telemetry.update();


        drive.followTrajectory(twoshotthree);
        while (robot.intakeLimit.isPressed()) {
            robot.intakeServo.setPower(1);
            robot.intakeMotor.setPower(.5);
        }
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);


        telemetry.addData("THRID RING", "FINISHED");
        telemetry.update();

        drive.followTrajectory(three);
        robot.wobbleClaw.setPosition(Robot.CLAW_CLOSED);
        robot.wobbleArm.setTargetPosition(0);
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.wobbleArm.setPower(.15);
        while (robot.wobbleArm.isBusy()) {
            telemetry.addData("waiting again", "...");
            telemetry.update();
        }
        robot.wobbleArm.setPower(0);


        drive.followTrajectory(four);
        robot.wobbleArm.setTargetPosition(645);
        robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.wobbleArm.setPower(.50);
        while (robot.wobbleArm.isBusy()) {
            telemetry.addData("waiting", "...");
            telemetry.update();
        }
        robot.wobbleArm.setPower(0);

        robot.wobbleClaw.setPosition(Robot.CLAW_OPENED);

        drive.followTrajectory(five);

        PoseStorage.currentPose = drive.getPoseEstimate();


    }

}






