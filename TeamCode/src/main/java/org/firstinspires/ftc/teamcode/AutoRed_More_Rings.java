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
                    strafeLeft(0.9, 18.5, true, false);
//                    forward(0.5, 44);
                    forward(0.9, 40, true, false);
                    strafeRight(0.6, 7, true, false);
                    arm(0.9, 1325);
                    grabOpen();
                    sleep(200);
                    arm(0.9, -440);
                    backward(0.9, 11, true, false);
                    shooter(0.72);
                    strafeRight(0.7, 8, true, false);
                    intake(1);
                    trigger();
                    sleep(100);
                    trigger();
                    sleep(100);
                    trigger();
                    sleep(100);
                    trigger();
                    backward(0.3, 13, true, false);
                    forward(0.6, 11, true, false);
                    trigger();
                    trigger();
                    intake(0);
                    shooter(0);
                    strafeLeft(0.9, 13, true, false);
                    backward(0.9, 26, false, false);
                    arm(0.7, 305);
                    forward(0.3, 4, false, false);
                    grabClose();
                    sleep(200);
                    forward(0.9, 36, false, false);
                    strafeRight(0.9, 8, false, false);
                    //arm(0.5, 1325);
                    grabOpen();
                    sleep(200);
                    arm(0.8, -330);
                    backward(1, 10, false, false);


                } else if (numberOfRings == 4) {

                    strafeLeft(0.85, 17, true, false);
                    forward(0.85, 52, true, false);
                    sleep(50);
                    strafeRight(0.6, 19, true, false);
                    shooter(0.71);
                    arm(0.8, 1325);
                    grabOpen();
                    arm(0.8, -1100);
                    backward(0.9, 22.5, true, false);
                    // strafeLeft(0.5,3.5);
                    strafeLeft(0.6, 4, true, false);
                    PIDrotate(0, 1);
                    sleep(100);
                    trigger();
                    trigger();
                    trigger();
                    trigger();
                    intake(0.9);
                    backward(0.6, 6.5, false, false);

                    backward(0.1, 3, false, true);
                    PIDrotate(0, 0.9);
                    forward(0.9, 7, true, true);

                    trigger();
                    trigger();
                    trigger();
                    backward(0.6, 8, false, false);
                    backward(0.1, 6, false, true);
                    PIDrotate(0, 0.9);
                    forward(0.9, 9.5, true, true);

                    trigger();
                    trigger();
                    trigger();
                    forward(0.9, 9, false, false);
                    intake(0);


                }
            } else {
                shooter(0.71);
                forward(0.7, 28, true, false);
                strafeLeft(0.4, 2, true, false);
                sleep(200);
                trigger();
                sleep(100);
                trigger();
                sleep(100);
                trigger();
                shooter(0);
                forward(0.65, 3, true, false);
                strafeRight(0.65, 6.5, true, false);
                arm(0.6, 1325);
                grabOpen();
                arm(0.6, -320);
                strafeLeft(0.7, 20.5, true, false);
                backward(0.7, 30.5, true, false);
                arm(0.6, 310);
                forward(0.1, 3.3, true, false);
                grabClose();
                sleep(200);
                forward(0.7, 29, true, false);
                strafeRight(0.7, 24, true, false);
                grabOpen();
                arm(0.6, -900);
                sleep(200);
                strafeLeft(0.6, 6, false, false);
                grabOpen();


            }
            break;
        }
    }

    }



