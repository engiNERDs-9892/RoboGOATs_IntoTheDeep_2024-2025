package org.firstinspires.ftc.team417_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team417_2020.Resources.Toggler;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Competition Auto")
public class AutoCompetition extends MasterAutonomous {

    int allianceSide = 1;
    Toggler rightBumper = new Toggler();

    // Constants
    static final int MINIMUM_RING_AREA = 1800;
    static final int PARKING_Y_POSITION = -58;
    static final int A_C_TARGET_ZONE_X_POSITION = 12;
    static final int A_TARGET_ZONE_Y_POSITION = -68;
    static final int B_TARGET_ZONE_Y_POSITION = -86;
    static final int C_TARGET_ZONE_Y_POSITION = -110;

    RingDetectionOpenCV ringDetector = new RingDetectionOpenCV();
    OpenCvCamera webcam;
    int numRings;

    @Override
    public void runOpMode() throws InterruptedException {
        autoInitializeRobot();
        initializeRingCounter();

        telemetry.addLine("waiting for start");

        while (!opModeIsActive()) {
            countRings();
            pickAllianceSide();
            telemetry.update();
        }


        waitForStart();

        //strafe away from wall and pivot
        move(-6, 0, 0.9);
        pivot(-90, 0.7);
        // 0 rings -> Zone A
        // 1 rings -> Zone B
        // 4 rings -> Zone C
        deliverWobbleGoal();

        /*
        //Returning to get second wobble goal
        //move left
        move(-20, A_TARGET_ZONE_Y_POSITION, 0.9);
        //move forward
        move(-20, 24, 0.9);
        pivot(180,0.7);
        //wobbleGoalGrabber
        pivot(180,0.7);
        move(0, 0, 0.9);

        deliverWobbleGoal();
        */
        //grab
        //pivot
        //move - tbd

        //deliverWobbleGoal


        // Park over white line
        move(-30, PARKING_Y_POSITION, 0.9);
        // navigate to other goal
        move(-30, -18, 0.9);
        move(-20, -18, 0.9);
        pivot(88, 0.9);


        // move toward wobble goal
        move(-20, -22, -0.3);


        // lift arm
        sleep(300);
        wobbleGoalGrabber.setPosition(WOBBLE_GOAL_GRABBER_OUT);
        runMotorToPosition(motorWobbleGoalArm, 120, 0.2);
        motorWobbleGoalArm.setPower(0.005);
        sleep(300);
        // close grabber
        wobbleGoalGrabber.setPosition(WOBBLE_GOAL_GRABBER_IN);
        // back up
        move(-20, -18, 0.7);
        motorWobbleGoalArm.setPower(0.0);
        // lift arm up
        //runMotorToPosition(motorWobbleGoalArm, 500, 0.9);
        //go to starting position
        pivot(-90, 0.7);
        move(0, 0, 0.7);

        deliverWobbleGoal();

        //parking on white line
        move(-13, -68, 0.9);
    }

    // todo program Wobble Goal arm
    public void OpenWobbleGoalGrabber() {
        wobbleGoalGrabber.setPosition(WOBBLE_GOAL_GRABBER_OUT);



    }

    // Uses gamepad to pick alliance side
    public int pickAllianceSide() {
        boolean isRedSide = rightBumper.toggle(gamepad1.right_bumper);
        if (isRedSide) {
            allianceSide = 1;
            telemetry.addLine("--------RED ALLIANCE--------");
        } else {
            allianceSide = -1;
            telemetry.addLine("--------BLUE ALLIANCE--------");
        }
        return allianceSide;
    }

    // Determines number of rings based on area and aspect ratio of orange rectangle
    public int countRings() {
        telemetry.addData("max rectangle", ringDetector.maxRect.toString());

        // area has to be greater than 1800, otherwise there are 0 rings
        if (ringDetector.maxRect.area() < MINIMUM_RING_AREA) {
            numRings = 0;
        }
        else if (ringDetector.maxRect.width / ringDetector.maxRect.height > 2.5) {
            numRings = 1;
        } else {
            numRings = 4;
        }
        telemetry.addData("Number of rings", numRings);

        // Use gamepad to adjust U threshold for filtering orange in different lighting conditions
        if (gamepad1.a) {
            ringDetector.uThreshold ++;
            sleep(200);
        } else if (gamepad1.b) {
            ringDetector.uThreshold --;
            sleep(200);
        }
        telemetry.addData("U Threshold", ringDetector.uThreshold);

        return numRings;
    }

    // Set up webcam streaming for counting Rings
    public void initializeRingCounter() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
        });
        webcam.setPipeline(ringDetector);
    }

    public void deliverWobbleGoal() throws InterruptedException {
        switch (numRings) {
            case 0:
                // Nagivate to Zone A
                move(A_C_TARGET_ZONE_X_POSITION, -24, 0.9);
                move(A_C_TARGET_ZONE_X_POSITION, A_TARGET_ZONE_Y_POSITION, 0.9);
                OpenWobbleGoalGrabber();
                move(A_C_TARGET_ZONE_X_POSITION, (A_TARGET_ZONE_Y_POSITION + 15), 0.9);
                break;

            case 1:
                // Navigate to Zone B
                move(A_C_TARGET_ZONE_X_POSITION, -24, 0.9);
                move(-13, B_TARGET_ZONE_Y_POSITION, 0.9);
                OpenWobbleGoalGrabber();
                break;

            case 4:
                // Navigate to Zone C
                move(A_C_TARGET_ZONE_X_POSITION, -24, 0.9);
                move(A_C_TARGET_ZONE_X_POSITION, C_TARGET_ZONE_Y_POSITION, 0.9);
                OpenWobbleGoalGrabber();
                break;

        }


    }




}


