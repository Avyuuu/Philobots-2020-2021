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

                    strafeLeft(0.3, 14, true);
//                    forward(0.5, 44);
                    shooter(0.55);
                    forward(0.6, 27, true);
//                    strafeRight(0.3, 6);
//                    arm(0.2);
                    PIDrotate(0, 1);
                    trigger();
                    PIDrotate(6.5, 1);
                    trigger();
                    PIDrotate(-6.5, 1);
                    trigger();
                    PIDrotate(0, 1);
                    shooter(0);
                    forward(0.7, 15, false);
                    strafeRight(0.5, 4, true);
                    arm(0.2);
                    forward(0.5, 3.5, false);
                    strafeLeft(0.5, 4, false);
                    backward(0.8, 16, true);
                    strafeRight(0.5, 11.5, true);

//                    strafeLeft(0.4,4);
//                    shooter(0.675);
//                    backward(0.3, 22);
//                    strafeRight(0.3, 12.5);
//                    sleep(200);
//                    trigger();
//                    trigger();
//                    trigger();
                    shooter(0.65);
                    intake(0.8);
                    backward(0.3, 15, true);
                    forward(0.5, 12, true);
                    trigger();
                    trigger();
                    forward(0.8, 5, false);
                    intake(0);

                } else if (numberOfRings == 4) {

                    forward(0.6, 14, false);
                    backward(0.6, 4, true);
                    strafeLeft(0.5, 14, false);
                    forward(0.6, 44, false);
                    strafeRight(0.6, 16, true);
                    arm(0.2);
                    //sleep(1000);
                    forward(0.6, 2, false);
                    // strafeLeft(0.5,3.5);
                    strafeLeft(0.6, 4, false);
                    shooter(0.675);
                    backward(0.6, 28, true);
                    trigger();
                    intake(0.8);
                    sleep(100);
                    trigger();
                    sleep(300);
                    trigger();
                    //sleep(200);
                    //strafeLeft(0.2, 1.5);
                    backward(0.5, 5, true);
                    forward(0.4, 3, false);
                    sleep(1300);
                    trigger();
                    trigger();
                    backward(0.3, 5, true);
                    sleep(200);
                    forward(0.3, 3, false);
                    backward(0.3, 7, true);
                    intake(0.55);
                    forward(0.3, 10, false);
                    PIDrotate(0, 0.8);
                    trigger();
                    trigger();
                    trigger();
                    trigger();
                    forward(0.7, 8, false);
                    intake(0);
                }
            } else {
                forward(0.5, 32, true);
                strafeRight(0.3, 2, false);
                arm(0.2);
                sleep(500);
                forward(0.3, 4, false);
                strafeLeft(0.3, 15.5, true);
                shooter(0.58);
                backward(0.4, 8, true);
                PIDrotate(0, 0.8);
                trigger();
                PIDrotate(6.5, 0.8);
                trigger();
                PIDrotate(-6.5, 0.8);
                trigger();
                forward(0.3, 6, false);

            }
            break;
        }

    }
}