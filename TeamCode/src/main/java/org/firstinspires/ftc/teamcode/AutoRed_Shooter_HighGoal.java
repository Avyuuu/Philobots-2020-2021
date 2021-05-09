package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED_WG_HG1", group = "Philos")
public class AutoRed_Shooter_HighGoal extends Autonomous_Methods {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initializing robot config and vision setup
        initRobot();
        visionUtility.init();
        int numberOfRings = 0;
        boolean extraRings = true;

        waitForStart();

        while (opModeIsActive()) {
            long runTime = System.currentTimeMillis();
            while ((System.currentTimeMillis() - runTime) < 3000) {
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

                    strafeLeft(0.7, 5, true, false);
                    forward(0.7, 44, true, false);
                    strafeRight(0.7, 6, true, false);
                    arm(0.9, 1325);
                    grabOpen();
                    sleep(500);
                    shooter(0.65);
                    backward(0.7, 22, true, false);
                    strafeRight(0.3, 8, true, false);
                    intake(0.2, 1);
                    PIDrotate(0, 1);
                    sleep(200);
                    trigger();
                    sleep(200);
                    trigger();
                    sleep(200);
                    trigger();

                    if (extraRings == true) {
                        backward(0.6, 11, true, false);
                        forward(0.6, 11, true, false);
                        trigger();
                        forward(0.6, 6, true, false);

                    } else {
                        intake(0, 0);
                        forward(0.5, 6, false, false);
                    }

                } else if (numberOfRings == 4) {
                    strafeLeft(0.3, 5, true, false);
                    forward(0.4, 43, true, false);
                    strafeRight(0.3, 16.5, true, false);
                    arm(0.7, 1325);
                    grabOpen();
                    sleep(200);
                    shooter(0.65);
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
                    if (extraRings == true) {
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
                    } else {
                        intake(0, 0);
                        forward(0.9, 2, false, false);
                    }

                }
            } else {
                shooter(0.65);
                strafeLeft(0.7, 9, true, false);
                forward(0.7, 27.5, true, false);
                PIDrotate(-18, 0.7);
                trigger();
                sleep(100);
                trigger();
                sleep(100);
                trigger();
                forward(0.7, 13, true, false);
                strafeRight(0.8, 20, false, false);
                PIDrotate(-45, 0.6);
                arm(0.7, 1325);
                grabOpen();
                sleep(200);
                arm(0.8, -440);
                PIDrotate(0, 0.8);
                strafeLeft(0.7, 20, true, false);
                backward(0.6, 9, false, false);

            }
            break;
        }

    }
}


