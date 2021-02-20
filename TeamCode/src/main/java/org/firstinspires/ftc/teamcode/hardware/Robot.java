package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

public class Robot {


    public static final double CLAW_CLOSED = .65;
    public static final double CLAW_OPENED = 1;

    public TouchSensor intakeLimit;
    public TouchSensor armLimit1;
    public TouchSensor armLimit2;

    //public Servo wobbleArm;
    public DcMotor wobbleArm;
    public Servo wobbleClaw;

    public CRServo intakeServo;
    public DcMotor intakeMotor;
    public DcMotor shooterMotor;

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;







    public void init(HardwareMap hardwareMap) {
        //Initialize Hardware
        intakeLimit = hardwareMap.get(TouchSensor.class, "intakeLimit");
        armLimit1 = hardwareMap.get(TouchSensor.class, "armLimit1");
        armLimit2 = hardwareMap.get(TouchSensor.class, "armLimit2");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        //wobbleArm = hardwareMap.get(Servo.class, "wobbleArm");
        wobbleArm = hardwareMap.get(DcMotor.class, "wobbleArm");
        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");


        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        //Because of the way the motors are mounted the left side needs to be flipped
        frontLeft.setDirection(REVERSE);
        backLeft.setDirection(REVERSE);

        //Sets all drive motors to zero to make sure they don't drive off unintentionally during initialization
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        //Sets Shooter Motor to run with encoders
        MotorConfigurationType motorConfigurationType = shooterMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shooterMotor.setMotorType(motorConfigurationType);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void setDriveUsingEncoders() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void driveAll(double power) { //Quick way of setting all of the motors to run at certain power
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }


    public void setDrivePower(double y, double x, double rx) { //Code to drive robot in holonomic fashion

        double frontLeftPower = y + x + rx;
        double backLeftPower = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower = y + x - rx;

        if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
                Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(backRightPower), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;


        }
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);


    }

    }

