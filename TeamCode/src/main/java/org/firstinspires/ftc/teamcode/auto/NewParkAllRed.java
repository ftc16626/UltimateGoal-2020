package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.teleop.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "NEW ParkAll Red", group = "Autonomous Red")
public class NewParkAllRed extends LinearOpMode {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "webcam"; // insert webcam name from configuration if using webcam

    private final double POWER_SHOT_ONE = -1.0;
    private final double POWER_SHOT_TWO = 10.0;
    private final double POWER_SHOT_THREE = 20.0;
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

        //sleep(3000);
        String height = "UNKNOWN";
        while (!isStarted()) {
            height = String.valueOf(pipeline.getHeight());
            telemetry.addData("[Ring Stack] >>", height);
            telemetry.update();
        }

        waitForStart();


        if (height.equals("ZERO")) //Zone A
            zero();


        if (height.equals("ONE")) //Zone B
            one();


        if (height.equals("FOUR")) //Zone C
            four();

        else
            zero();


    }

    private void zero() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-63.0, -16.0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        robot.shooterMotor.setPower(.5);
        Trajectory zero = drive.trajectoryBuilder(startPose)
                .strafeLeft(4.0)
                .build();
        Trajectory one = drive.trajectoryBuilder(zero.end())
                .back(5.0)
                .splineTo(new Vector2d(-2.0, POWER_SHOT_ONE), 0.0)
                .build();

        Trajectory two = drive.trajectoryBuilder(one.end())
                .lineToConstantHeading(new Vector2d(-2.0, POWER_SHOT_TWO))
                .build();
//TwoHalf Not necessary?
        Trajectory twohalf = drive.trajectoryBuilder(two.end())
                .lineToConstantHeading(new Vector2d(-2.0, POWER_SHOT_THREE))
                .build();

        Trajectory three = drive.trajectoryBuilder(twohalf.end())
                .back(35.0)
                .build();

        Trajectory four = drive.trajectoryBuilder(three.end())
                .lineToConstantHeading(new Vector2d(35.0, -41.0))
                .build();

        Trajectory five = drive.trajectoryBuilder(four.end())
                .lineToConstantHeading(new Vector2d(35.0, -32.0))
                .splineToConstantHeading(new Vector2d(-32.0, -14.0), Math.toRadians(180))
                .build();

        Trajectory six = drive.trajectoryBuilder(five.end())
                .lineToSplineHeading(new Pose2d(-25.0, -32.0, 0.0))
                .build();

        Trajectory seven = drive.trajectoryBuilder(six.end())
                .lineToConstantHeading(new Vector2d(-12.0, -30.0))
                .build();

        Trajectory eight = drive.trajectoryBuilder(seven.end())
                .lineToConstantHeading(new Vector2d(-15.0, 0.0))
                .build();

        Trajectory nine = drive.trajectoryBuilder(eight.end())
                .forward(25.0)
                .build();

        drive.followTrajectory(zero);
        drive.followTrajectory(one); //-----------

        //Shoot First Power shot
        robot.intakeServo.setPower(1);
        sleep(800);
        robot.intakeServo.setPower(0);
        //

        drive.followTrajectory(two); //-----------
        //Shoot Second Power Shot
        robot.intakeServo.setPower(1);
        sleep(2000);
        robot.intakeServo.setPower(0);
        //
        drive.followTrajectory(twohalf);
        //Shoot Third Power Shot
        robot.intakeServo.setPower(1);
        robot.intakeMotor.setPower(1);
        sleep(2000);
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(1);
        //
        drive.followTrajectory(three); //-----------

        drive.followTrajectory(four); //-----------

        //Drop Arm and open claw
        robot.wobbleArm.setPower(.5);
        sleep(800);
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_OPENED);
        sleep(800);
        //

        //Then lift arm
        robot.wobbleArm.setPower(-.5);
        sleep(800);
        robot.wobbleArm.setPower(0);
        //

        drive.followTrajectory(five); //-----------

        //Drop Arm and close claw
        robot.wobbleArm.setPower(.5);
        sleep(1000);
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_CLOSED);
        sleep(800);
        //
        //Then lift arm
        robot.wobbleArm.setPower(-.5);
        sleep(1000);
        robot.wobbleArm.setPower(0);
        //
        drive.followTrajectory(six); //-----------

        drive.followTrajectory(seven); //-----------
        //Drop Arm and open claw
        robot.wobbleArm.setPower(.5);
        sleep(1000);
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_OPENED);


/*
        drive.followTrajectory(eight); //-----------
        drive.followTrajectory(nine); //-----------

 */


        drive.setPoseEstimate(PoseStorage.currentPose);

    }

    private void one() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-63.0, -16.0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        robot.shooterMotor.setPower(.5);
        Trajectory zero = drive.trajectoryBuilder(startPose)
                .strafeRight(35.0)
                .build();
        Trajectory one = drive.trajectoryBuilder(zero.end())
                .back(5.0)
                .splineTo(new Vector2d(-3.0, POWER_SHOT_ONE + 1), 0.0)
                //Shoot First Power shot
                .build();

        Trajectory two = drive.trajectoryBuilder(one.end())
                .lineToConstantHeading(new Vector2d(-3.0, POWER_SHOT_TWO + 1))
                //Shoot Second Power Shot
                .build();

        Trajectory twohalf = drive.trajectoryBuilder(two.end())
                .lineToConstantHeading(new Vector2d(-3.0, POWER_SHOT_THREE))
                .build();


        Trajectory three = drive.trajectoryBuilder(twohalf.end())
                .back(55.0)
                .build();

        Trajectory four = drive.trajectoryBuilder(three.end())
                .lineToConstantHeading(new Vector2d(55.0, -15.0))
                //Drop Arm and open claw
                //Then lift arm
                .build();


        Trajectory five = drive.trajectoryBuilder(four.end())
                .lineToConstantHeading(new Vector2d(55.0, 5.0))
                .splineToConstantHeading(new Vector2d(-13.0, -16.0), Math.toRadians(180))
                .build();

        Trajectory six = drive.trajectoryBuilder(five.end())
                .lineToConstantHeading(new Vector2d(-35.0, -13.0))
                //Intake and intake motor
                .build();

        Trajectory seven = drive.trajectoryBuilder(six.end())
                .lineToSplineHeading(new Pose2d(0.0, -11.0, 0.0))
                //drop arm close claw
                .build();

        Trajectory eight = drive.trajectoryBuilder(six.end())
                .splineToConstantHeading(new Vector2d(-3.0, -10), Math.toRadians(225))
                //Shoot third power shot
                .build();

        Trajectory nine = drive.trajectoryBuilder(eight.end())
                .lineToSplineHeading(new Pose2d(10.0, -36.0, 0.0))
                //Lift claw
                .build();

        drive.followTrajectory(zero);

        drive.followTrajectory(one); //-----------
        //Shoot First Power shot
        robot.intakeServo.setPower(1);
        sleep(800);
        robot.intakeServo.setPower(0);
        //

        drive.followTrajectory(two); //-----------
        //Shoot Second Power Shot
        robot.intakeServo.setPower(1);
        sleep(2000);
        robot.intakeServo.setPower(0);
        //


        drive.followTrajectory(twohalf);
        //Shoot Third Power Shot
        robot.intakeServo.setPower(1);
        robot.intakeMotor.setPower(1);
        sleep(2000);
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);
        //


        drive.followTrajectory(three); //-----------
        drive.followTrajectory(four); //-----------
        //Drop Arm and open claw
        robot.wobbleArm.setPower(.5);
        sleep(800);
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_OPENED);
        sleep(800);
        //

        //Then lift arm
        robot.wobbleArm.setPower(-.5);
        sleep(800);
        robot.wobbleArm.setPower(0);
        //

        drive.followTrajectory(five); //-----------
        //Intake and intake motor
        /*
        robot.intakeMotor.setPower(1);
        robot.intakeServo.setPower(1);
        robot.shooterMotor.setPower(.525);

         */

        drive.followTrajectory(six); //-----------
        robot.intakeMotor.setPower(0);
        robot.intakeServo.setPower(0);
        //Drop Arm and close claw
        robot.wobbleArm.setPower(.5);
        sleep(1000);
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_CLOSED);
        sleep(800);
        //
        //Then lift arm
        robot.wobbleArm.setPower(-.5);
        sleep(2000);
        robot.wobbleArm.setPower(0);
        //



        drive.followTrajectory(seven); //-----------
        //Drop Arm and open claw
        robot.wobbleArm.setPower(.5);
        sleep(800);
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_OPENED);
        //
/*
        drive.followTrajectory(eight); //-----------


        drive.followTrajectory(nine); //-----------
        //Drop Arm and open claw
        robot.wobbleArm.setPower(.5);
        sleep(800);
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_OPENED);
        //

 */

        drive.setPoseEstimate(PoseStorage.currentPose);


    }

    private void four() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-63.0, -16.0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        robot.shooterMotor.setPower(.5);
        Trajectory zero = drive.trajectoryBuilder(startPose)
                .strafeRight(35.0)
                .build();
        Trajectory one = drive.trajectoryBuilder(zero.end())
                .back(5.0)
                .splineTo(new Vector2d(-3.0, POWER_SHOT_ONE + 1), 0.0)

                .build();

        Trajectory two = drive.trajectoryBuilder(one.end())
                .lineToConstantHeading(new Vector2d(-3.0, POWER_SHOT_TWO + 1))

                .build();

        Trajectory twohalf = drive.trajectoryBuilder(two.end())
                .lineToConstantHeading(new Vector2d(-3.0, POWER_SHOT_THREE))
                .build();


        Trajectory three = drive.trajectoryBuilder(twohalf.end())
                .back(55.0) //Needs to be more in order to reach Zone C
                .build();

        Trajectory four = drive.trajectoryBuilder(three.end())
               // .lineToConstantHeading(new Vector2d(55.0, -15.0))
                .lineToSplineHeading(new Pose2d(55.0, -15.0, Math.toRadians(90.0)))
                .build();


        Trajectory five = drive.trajectoryBuilder(four.end())
                //.lineToConstantHeading(new Vector2d(55.0, 5.0))
                .lineToSplineHeading(new Pose2d(55.0, 5.0, Math.toRadians(180.0)))
                .splineToConstantHeading(new Vector2d(-13.0, -16.0), Math.toRadians(180))
                .build();

        Trajectory six = drive.trajectoryBuilder(five.end())
                .lineToConstantHeading(new Vector2d(-35.0, -13.0))
                .build();

        Trajectory seven = drive.trajectoryBuilder(six.end())
                .lineToSplineHeading(new Pose2d(0.0, -11.0, 0.0))
                //drop arm close claw
                .build();

        Trajectory eight = drive.trajectoryBuilder(six.end())
                .splineToConstantHeading(new Vector2d(-3.0, -10), Math.toRadians(225))
                //Shoot third power shot
                .build();

        Trajectory nine = drive.trajectoryBuilder(eight.end())
                .lineToSplineHeading(new Pose2d(10.0, -36.0, 0.0))
                //Lift claw
                .build();

        drive.followTrajectory(zero);

        drive.followTrajectory(one); //-----------
        //Shoot First Power shot
        robot.intakeServo.setPower(1);
        sleep(800);
        robot.intakeServo.setPower(0);
        //

        drive.followTrajectory(two); //-----------
        //Shoot Second Power Shot
        robot.intakeServo.setPower(1);
        sleep(2000);
        robot.intakeServo.setPower(0);
        //


        drive.followTrajectory(twohalf);
        //Shoot Third Power Shot
        robot.intakeServo.setPower(1);
        robot.intakeMotor.setPower(1);
        sleep(2000);
        robot.intakeServo.setPower(0);
        robot.intakeMotor.setPower(0);
        //


        drive.followTrajectory(three); //-----------
        drive.followTrajectory(four); //-----------
        //Drop Arm and open claw
        robot.wobbleArm.setPower(.5);
        sleep(800);
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_OPENED);
        sleep(800);
        //

        //Then lift arm
        robot.wobbleArm.setPower(-.5);
        sleep(800);
        robot.wobbleArm.setPower(0);
        //

        drive.followTrajectory(five); //-----------


        drive.followTrajectory(six); //-----------
        robot.intakeMotor.setPower(0);
        robot.intakeServo.setPower(0);
        //Drop Arm and close claw
        robot.wobbleArm.setPower(.5);
        sleep(1000);
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_CLOSED);
        sleep(800);
        //
        //Then lift arm
        robot.wobbleArm.setPower(-.5);
        sleep(2000);
        robot.wobbleArm.setPower(0);
        //



        drive.followTrajectory(seven); //-----------
        //Drop Arm and open claw
        robot.wobbleArm.setPower(.5);
        sleep(800);
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_OPENED);
        //
/*
        drive.followTrajectory(eight); //-----------


        drive.followTrajectory(nine); //-----------
        //Drop Arm and open claw
        robot.wobbleArm.setPower(.5);
        sleep(800);
        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(Robot.CLAW_OPENED);
        //

 */

        drive.setPoseEstimate(PoseStorage.currentPose);


    }

}






