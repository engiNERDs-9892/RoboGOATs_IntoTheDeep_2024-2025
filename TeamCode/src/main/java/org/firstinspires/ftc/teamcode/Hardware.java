package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


public class Hardware {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;

    public DcMotor winch = null;
    public DcMotor intake = null;
    public DcMotor transfer = null;
    public DcMotor lift = null;

    public Servo hook = null;
    public Servo stripper = null;
    public Servo escapementFinger = null;
    public Servo launcherRelease = null;
    public Servo droneAngle = null;

    public VCNL4000 rightDistance = null;
    public VCNL4000 leftDistance = null;
    public TouchSensor firstPixelDetector = null;
    public TouchSensor secondPixelDetector = null;

    HardwareMap hwMap = null;

    public Hardware() {

    }

    public double stripperFirstRelease = 0.40;
    public double stripperSecondRelease = 0.1;
    public double stripperOpen = 0.85;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        leftDrive = hwMap.get(DcMotor.class, "fl");
        rightDrive = hwMap.get(DcMotor.class, "fr");
        rightBackDrive = hwMap.get(DcMotor.class, "br");
        leftBackDrive = hwMap.get(DcMotor.class, "bl");

        winch = hwMap.get(DcMotor.class, "winch");
        intake = hwMap.get(DcMotor.class, "intake");
        transfer = hwMap.get(DcMotor.class, "transfer");
        lift = hwMap.get(DcMotor.class, "lift");

        hook = hwMap.get(Servo.class, "hook");
        stripper = hwMap.get(Servo.class, "stripper");
        escapementFinger = hwMap.get(Servo.class, "finger");
        launcherRelease = hwMap.get(Servo.class, "release");
        droneAngle = hwMap.get(Servo.class, "angle");

        firstPixelDetector = hwMap.get(TouchSensor.class, "bb1");
        secondPixelDetector = hwMap.get(TouchSensor.class, "bb2");

        rightDistance = hwMap.get(VCNL4000.class, "rdist");


        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        winch.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}