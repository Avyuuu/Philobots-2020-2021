//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//@Autonomous(name = "RED_WG_PS_HG1", group = "Philos")
//public class AutoRed_Shooter_HighGoal extends Autonomous_Methods{
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Initializing robot config and vision setup
//        initRobot();
//        visionUtility.init();
//        int numberOfRings = 0;
//
//        waitForStart();
//
//        while(opModeIsActive()) {
//            long runTime = System.currentTimeMillis();
//            while ((System.currentTimeMillis() - runTime) < 3000) {
//                numberOfRings = visionUtility.getRingStack(this);
//                if(numberOfRings > 0){
//                    break;
//                }
//                telemetry.addData("stackCount:", numberOfRings);
//                telemetry.update();
//            }
//            visionUtility.tfodDeactivate();
//
//
//            if (numberOfRings > 0) {
//                if (numberOfRings == 1) {
//
//                    strafeLeft(0.5, 15);
//                    forward(0.4, 44);
//                    strafeRight(0.3, 6);
//                    arm(0.2);
//                    sleep(700);
//                    forward(0.4, 3.5);
//                    strafeLeft(0.4,4);
//                    shooter(0.7);
//                    backward(0.3, 22);
//                    strafeRight(0.3, 13);
//                    sleep(1500);
//                    trigger();
//                    trigger();
//                    trigger();
//                    intake(0.75);
//                    backward(0.3, 15);
//                    forward(0.4, 15);
//                    trigger();
//                    forward(0.3, 6);
//
//                } else if (numberOfRings == 4) {
//                    forward(0.6, 14);
//                    backward(0.5, 3);
//                    strafeLeft(0.3, 14);
//                    forward(0.4, 43);
//                    strafeRight(0.3, 16.5);
//                    arm(0.2);
//                    sleep(200);
//                    shooter(0.65);
//                    forward(0.5,2);
//                    strafeLeft(0.5,3.5);
//                    backward(0.5, 28);
//                    trigger();
//                    sleep(200);
//                    trigger();
//                    sleep(200);
//                    trigger();
//                    sleep(200);
//                    intake(0.8);
//                    backward(0.1, 3.5);
//                    sleep(500);
//                    forward(0.5, 4);
//                    sleep(1000);
//                    trigger();
//                    backward(0.5, 13);
//                    forward(0.5,9);
//                    sleep(500);
//                    trigger();
//                    sleep(300);
//                    trigger();
//                    sleep(300);
//                    trigger();
//                    forward(0.7, 7);
//                }
//            } else {
//                forward(0.5, 32);
//                strafeRight(0.3, 2);
//                arm(0.2);
//                sleep(500);
//                forward(0.3, 4);
//                strafeLeft(0.3, 4);
//                shooter(0.65);
//                backward(0.4, 11);
//                strafeRight(0.3, 2);
//                sleep(500);
//                trigger();
//                trigger();
//                trigger();
//                shooter(0);
//                forward(0.3, 7);
//
//            }
//            break;
//        }
//
//    }
//}
//
//
