package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
        servoArm.setPosition(.55);
        servoBucket.setPosition(.3);

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorHS.setDirection(DcMotor.Direction.REVERSE);
        motorVS.setDirection(DcMotor.Direction.FORWARD);


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

            if (gamepad1.left_trigger != 0) {
                motorFL.setPower(.3 * leftFrontPower);
                motorBL.setPower(.3 * leftBackPower);
                motorFR.setPower(.3 * rightFrontPower);
                motorBR.setPower(.3 * rightBackPower);

            }   else if (gamepad1.right_trigger != 0) {
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
            motorVS.setPower(VSpower * .5);

            if(gamepad2.x){
                servoBucket.setPosition(.3);
            }
            if(gamepad2.b){
                servoBucket.setPosition(0);
            }
            if(gamepad2.dpad_right){
                servoArm.setPosition(0);
            }
            if(gamepad2.dpad_up){
                servoArm.setPosition(.2);
            }
            if(gamepad2.dpad_left){
                servoArm.setPosition(.55);
            }
            if(gamepad2.right_bumper){
                servoIn.setPosition(1);
            } else if(gamepad2.left_bumper){
                servoIn.setPosition(.4);
            } else {
                servoIn.setPosition(0.5);
            }

        }
    }
}
