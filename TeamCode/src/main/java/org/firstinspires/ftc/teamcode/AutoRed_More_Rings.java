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
                    strafeLeft(0.9, 17, true, false);
//                    forward(0.5, 44);
                    forward(0.9, 44, true, false);
                    strafeRight(0.6, 8, true, false);
                    shooter(0.63);
                    arm(0.9, 1325);
                    grabOpen();
                    arm(0.9, -440);
                    backward(0.7, 17, true, false);
                    strafeRight(0.7, 5, true, false);
                    intake(0.2, 1);
                    PIDrotate(0, 1);
                    sleep(200);
                    trigger();
                    sleep(200);
                    trigger();
                    sleep(200);
                    trigger();
                    backward(0.6, 11, true, false);
                    forward(0.6, 11, true, false);
                    PIDrotate(0, 0.8);
                    trigger();
                    trigger();
                    intake(0, 0);
                    shooter(0);
                    strafeLeft(0.9, 13.5, true, false);
                    backward(0.9, 24, true, false);
                    arm(0.7, 330);
                    forward(0.3, 2, false, false);
                    grabClose();
                    sleep(200);
                    forward(0.9, 39.5, false, false);
                    strafeRight(0.7, 6, false, false);
                    sleep(100);
                    //arm(0.5, 1325);
                    grabOpen();
                    sleep(200);
                    arm(0.7, -330);
                    backward(1, 8, false, false);


                } else if (numberOfRings == 4) {

                    strafeLeft(0.7, 17, true, false);
                    forward(0.85, 52, true, false);
                    sleep(50);
                    strafeRight(0.6, 19, true, false);
                    shooter(0.65);
                    arm(0.8, 1325);
                    grabOpen();
                    arm(0.8, -1100);
                    backward(0.9, 23.5, true, false);
                    // strafeLeft(0.5,3.5);
                    intake(0.3, 1);
                    strafeLeft(0.6, 5, true, false);
                    sleep(200);
                    PIDrotate(0, 0.8);
                    trigger();
                    trigger();
                    trigger();
                    trigger();
                    backward(0.6, 5.5, true, false);

                    backward(0.3, 6, true, true);
                    PIDrotate(0, 0.9);
                    forward(0.9, 9.5, true, true);
                    sleep(100);
                    PIDrotate(0, 1);
                    trigger();
                    trigger();
                    trigger();
                    backward(0.6, 19, true, false);
                    PIDrotate(0, 0.9);
                    forward(0.9, 17.5, true, true);
                    sleep(200);
                    PIDrotate(0, 1);
                    trigger();
                    trigger();
                    sleep(100);
                    trigger();
                    forward(0.9, 6, false, false);
                    intake(0, 0);


                }
            } else {
                shooter(0.65);
                forward(0.7, 28, true, false);
                strafeLeft(0.4, 4, true, false);
                sleep(500);
                trigger();
                sleep(500);
                trigger();
                sleep(500);
                trigger();
                shooter(0);
                forward(0.65, 2, true, false);
                strafeRight(0.65, 6.5, true, false);
                arm(0.6, 1325);
                grabOpen();
                arm(0.6, -320);
                strafeLeft(0.7, 20.5, true, false);
                backward(0.7, 29.5, true, false);
                arm(0.6, 310);
                forward(0.1, 4, true, false);
                grabClose();
                sleep(200);
                forward(0.7, 30.5, true, false);
                strafeRight(0.7, 18, true, false);
                grabOpen();
                arm(0.6, -950);
                sleep(200);
                strafeLeft(0.6, 6, false, false);
                grabOpen();


            }
            break;
        }
    }

    }



