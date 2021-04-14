package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED_WG_7_HG", group = "Philos")
public class AutoRed_More_Rings extends Autonomous_Methods {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initializing robot config and vision setup
        initRobot();
        visionUtility.init();
        int numberOfRings = 0;
        resetAngle();
        waitForStart();

        while (opModeIsActive()) {
            long runTime = System.currentTimeMillis();
            while ((System.currentTimeMillis() - runTime) < 1000) {
                numberOfRings = visionUtility.getRingStack(this);
                if (numberOfRings > 0) {
                    break;
                }
                telemetry.addData("stackCount:", numberOfRings);
                telemetry.update();
            }
            visionUtility.tfodDeactivate();

            if (numberOfRings > 0) {
                if (numberOfRings == 1) {
                    strafeLeft(0.7, 18.5, true, false);
//                    forward(0.5, 44);
                    forward(0.7, 42, true, false);
                    strafeRight(0.5, 6, true, false);
                    arm(0.5, 1325);
                    grabOpen();
                    sleep(200);
                    arm(0.2, -300);
                    backward(0.8, 12.5, true, false);
                    shooter(0.6);
                    strafeRight(0.5, 10.5, true, false);
                    trigger();
                    trigger();
                    sleep(100);
                    trigger();
                    intake(0.55);
                    PIDrotate(0, 0.8);
                    backward(0.3, 11, true, false);
                    forward(0.5, 8, true, false);
                    trigger();
                    trigger();
                    intake(0);
                    strafeLeft(0.7, 19.7, true, false);
                    backward(0.7, 25.5, true, false);
                    arm(0.6, 300);
                    grabClose();
                    sleep(200);
                    forward(0.7, 42, true, false);
                    strafeRight(0.5, 6, true, false);
                    arm(0.5, 1325);
                    grabOpen();
                    sleep(200);
                    arm(0.2, -300);
                    backward(0.8, 22.5, true, false);


                } else if (numberOfRings == 4) {

                    strafeLeft(0.85, 15, false, false);
                    forward(0.85, 56, true, false);
                    strafeRight(0.7, 17, true, false);
                    arm(0.8, 1325);
                    sleep(500);
                    grabOpen();
                    sleep(500);
                    arm(0.6, -300);
                    shooter(0.6);
                    backward(0.7, 25, true, false);
                    // strafeLeft(0.5,3.5);
                    strafeLeft(0.7, 6, true, false);
                    PIDrotate(0, 0.8);
                    trigger();
                    trigger();
                    trigger();
                    intake(0.55);
                    backward(0.8, 4, true, false);
                    backward(0.1, 5, true, true);
                    forward(0.8, 7, true, true);
                    PIDrotate(0, 0.8);
                    trigger();
                    trigger();
                    trigger();
                    backward(0.8, 6, true, false);
                    backward(0.1, 6, true, true);
                    forward(0.8, 8, true, true);
                    trigger();
                    trigger();
                    forward(0.9, 3, false, false);
                    intake(0);


                }
            } else {
                shooter(1);
                forward(0.7, 26, true, false);
                strafeLeft(0.5, 2, true, false);
                trigger();
                trigger();
                trigger();
                shooter(0);
                forward(0.65, 2.5, true, false);
                strafeRight(0.65, 4.5, true, false);
                arm(0.6, 1325);
                grabOpen();
                arm(0.6, -300);
                strafeLeft(0.7, 19.7, true, false);
                backward(0.7, 25.5, true, false);
                arm(0.6, 300);
                grabClose();
                sleep(200);
                forward(0.7, 29, true, false);
                strafeRight(0.7, 18, true, false);
                grabOpen();
                sleep(200);
                strafeLeft(0.6, 6, false, false);
                grabOpen();


            }
            break;
        }
    }

    }



