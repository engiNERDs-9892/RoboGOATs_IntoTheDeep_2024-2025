package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Starter_Autonomous extends LinearOpMode {

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorHS;
    private DcMotor motorVS;
    private Servo servoArm;
    private Servo servoIn;
    private Servo servoBucket;

    int in=45;

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
        servoArm.setPosition(.53);
        servoBucket.setPosition(.3);

        //"foot" tis was typeth via foot

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorHS.setDirection(DcMotor.Direction.REVERSE);
        motorVS.setDirection(DcMotor.Direction.FORWARD);

        motorVS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        Move(directions.FORWARDS,9,.8);
        Move(directions.LEFT,48,.8);
        Move(directions.CLOCKWISE,10,.4);
        Move(directions.BACKWARDS,4,.5);
        HS_Out(.4,200);
        VS_Up(1,-5500);
        sleep(1000);
        servoBucket.setPosition(0);
        sleep(1000);
        servoBucket.setPosition(.3);
        VS_Up(1,0);
        HS_Out(-.4,200);
        Move(directions.FORWARDS,24,.8);
        Move(directions.COUNTERCLOCKWISE,33,.8);
        Move(directions.RIGHT,25,.6);
        Move(directions.FORWARDS, 10, .3);
        Move(directions.BACKWARDS, 9, .8);
        servoArm.setPosition(.3);
        servoIn.setPosition(-1);
        HS_Out(.5,1800);
        servoIn.setPosition(.5);
        sleep(200);
        servoIn.setPosition(-1);
        sleep(200);
        HS_Out(-.5,1800);
        servoIn.setPosition(.5);
        servoArm.setPosition(.2);
        Move(directions.LEFT,25,.8);
        Move(directions.CLOCKWISE,33,.8);
        Move(directions.BACKWARDS,17,.8);
        servoArm.setPosition(.53);
        servoIn.setPosition(.6);
        sleep(1000);
        VS_Up(1,-5500);
        servoIn.setPosition(.5);
        servoBucket.setPosition(0);


        //no, please don't pour coffee on meeee
        //coffee craft



    }




/////////////////////////   FUNCTIONS  ////////////////////////////////////////

    private void HS_Out(double speed,int time) {
        motorHS.setPower(speed);
        sleep(time);
        motorHS.setPower(0);
    }

    private void VS_Up( double speed, int target) {
        motorVS.setTargetPosition(target);
        motorVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorVS.setPower(speed);
        while(motorVS.isBusy());



    }

    private void Move(directions direction, int target, double speed) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // This sets the direction for the motor for the wheels to drive forward
        if (direction == directions.FORWARDS) {
            motorFL.setDirection(DcMotor.Direction.REVERSE);
            motorFR.setDirection(DcMotor.Direction.FORWARD);
            motorBL.setDirection(DcMotor.Direction.REVERSE);
            motorBR.setDirection(DcMotor.Direction.FORWARD);
        }

        // Sets the motor direction to move Backwards
        else if (direction == directions.BACKWARDS) {
            motorFL.setDirection(DcMotor.Direction.FORWARD);
            motorFR.setDirection(DcMotor.Direction.REVERSE);
            motorBL.setDirection(DcMotor.Direction.FORWARD);
            motorBR.setDirection(DcMotor.Direction.REVERSE);
        }

        // Sets the motor direction to move to the Left ( Note * Port = Left)
        else if (direction == directions.RIGHT) {
            motorFL.setDirection(DcMotor.Direction.REVERSE);
            motorFR.setDirection(DcMotor.Direction.REVERSE);
            motorBL.setDirection(DcMotor.Direction.FORWARD);
            motorBR.setDirection(DcMotor.Direction.FORWARD);
        }

        // Sets the motor direction to move to the Right (Note * Starboard = Right)
        else if (direction == directions.LEFT) {
            motorFL.setDirection(DcMotor.Direction.FORWARD);
            motorFR.setDirection(DcMotor.Direction.FORWARD);
            motorBL.setDirection(DcMotor.Direction.REVERSE);
            motorBR.setDirection(DcMotor.Direction.REVERSE);
        }
        else if (direction == directions.CLOCKWISE) {
            motorFL.setDirection(DcMotor.Direction.REVERSE);
            motorFR.setDirection(DcMotor.Direction.REVERSE);
            motorBL.setDirection(DcMotor.Direction.REVERSE);
            motorBR.setDirection(DcMotor.Direction.REVERSE);
        }
        else if (direction == directions.COUNTERCLOCKWISE) {
            motorFL.setDirection(DcMotor.Direction.FORWARD);
            motorFR.setDirection(DcMotor.Direction.FORWARD);
            motorBL.setDirection(DcMotor.Direction.FORWARD);
            motorBR.setDirection(DcMotor.Direction.FORWARD);
        }

        // Gives it a position to run to
        motorFL.setTargetPosition(target * in);
        motorFR.setTargetPosition(target * in);
        motorBL.setTargetPosition(target * in);
        motorBR.setTargetPosition(target * in);

        // tells it to go to the position that is set
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // the motor speed for Wheels
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);



        // While loop keeps the code running until motors reach the desired position
        while (opModeIsActive() && ((motorFL.isBusy() || motorFR.isBusy()))) {
        }
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    } //closes movement function
    enum directions {
        FORWARDS,
        BACKWARDS,
        LEFT,
        RIGHT,
        CLOCKWISE,
        COUNTERCLOCKWISE
    }//closes directions
}
