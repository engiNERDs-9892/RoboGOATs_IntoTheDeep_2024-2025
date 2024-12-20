package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class Test2 extends LinearOpMode {

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorHS;
    private DcMotor motorVS;
    private Servo servoArm;
    private Servo servoIn;
    private Servo servoBucket;



    @Override
    public void runOpMode() {
        isStarted();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setWeightedDrivePower(new Pose2d(0, 0.15, 0));
        sleep(30000);
    }

}
