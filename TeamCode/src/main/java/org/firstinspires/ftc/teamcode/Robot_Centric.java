package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.List;

@TeleOp
public class Robot_Centric extends LinearOpMode {

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
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorHS = hardwareMap.dcMotor.get("motorHS");
        motorVS = hardwareMap.dcMotor.get("motorVS");
        servoArm = hardwareMap.servo.get("servoArm");
        servoIn = hardwareMap.servo.get("servoIn");
        servoBucket = hardwareMap.servo.get("servoBucket");


        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorHS.setPower(0);
        motorVS.setPower(0);
        servoArm.setPosition(.46);
        servoBucket.setPosition(.3);

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorHS.setDirection(DcMotor.Direction.REVERSE);
        motorVS.setDirection(DcMotor.Direction.FORWARD);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        // run until the end of the match (driver presses STOP) uwu
        while (opModeIsActive()) {
            double max;



            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Lift", motorVS.getCurrentPosition());
            telemetry.update();

            if (gamepad1.right_trigger != 0) {
                motorFL.setPower(.3 * leftFrontPower);
                motorBL.setPower(.3 * leftBackPower);
                motorFR.setPower(.3 * rightFrontPower);
                motorBR.setPower(.3 * rightBackPower);

            }   else if (gamepad1.left_trigger != 0) {
                motorFL.setPower(1 * leftFrontPower);
                motorBL.setPower(1 * leftBackPower);
                motorFR.setPower(1 * rightFrontPower);
                motorBR.setPower(1 * rightBackPower);
            }
          else{
                motorFL.setPower(.8 * leftFrontPower);
                motorBL.setPower(.8 * leftBackPower);
                motorFR.setPower(.8 * rightFrontPower);
                motorBR.setPower(.8 * rightBackPower);
            }
            double HSpower = gamepad2.right_stick_y;
            double VSpower = gamepad2.left_stick_y;

            motorHS.setPower(HSpower * -.5);
            motorVS.setPower(VSpower * .8);

             if(gamepad2.x){
                servoBucket.setPosition(.3);
            }
            if(gamepad2.b){
                servoBucket.setPosition(0);
            }
            if(gamepad2.dpad_right){
                servoArm.setPosition(-.3);
            }
            if(gamepad2.dpad_up){
                servoArm.setPosition(.2);
            }
            if(gamepad2.dpad_left){
                servoArm.setPosition(.53);
            }
            if(gamepad2.right_bumper){
                servoIn.setPosition(-1);
            } else if(gamepad2.left_bumper){
                servoIn.setPosition(.9);
            } else {
                servoIn.setPosition(0.5);
            }

        }
    }


    /*
     * Sample tracking wheel localizer implementation assuming the standard configuration:
     *
     *    /--------------\
     *    |     ____     |
     *    |     ----     |
     *    | ||        || |
     *    | ||        || |
     *    |              |
     *    |              |
     *    \--------------/
     *
     */
    @Config
    public class PinpointLocalizer implements Localizer {

        private Encoder leftEncoder, rightEncoder, frontEncoder;

        private List<Integer> lastEncPositions, lastEncVels;
        GoBildaPinpointDriver odo;
        public PinpointLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {


            lastEncPositions = lastTrackingEncPositions;
            lastEncVels = lastTrackingEncVels;


            odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
            odo.setOffsets(-69, 187);
            odo.setOffsets(-165.1, 82.6);//-6 14
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            odo.resetPosAndIMU();
            // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)


        }


        @NonNull
        @Override
        public Pose2d getPoseEstimate() {
            Pose2D pose = odo.getPosition();
            return new Pose2d(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.RADIANS));
        }

        @Override
        public void setPoseEstimate(@NonNull Pose2d pose2d) {
            Pose2D pose = new Pose2D(DistanceUnit.INCH, pose2d.getX(), pose2d.getY(), AngleUnit.RADIANS, pose2d.getHeading());
            odo.setPosition(pose);
        }

        @Nullable
        @Override
        public Pose2d getPoseVelocity() {
            Pose2D pose = odo.getVelocity();
            return new Pose2d(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.RADIANS));
        }

        @Override
        public void update() {
            odo.update();
        }

        //@Override
        //public
    }

}
