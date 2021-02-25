//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//import org.firstinspires.ftc.teamcode.Library.VisionUtility;
//
//
//@Autonomous(name = "AutoRedLeftWG 1", group = "Philos")
//public class AutoRedLeft extends Autonomous_Methods {
//
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
//                    forward(0.4, 48);
//                    arm(0.2);
//                    sleep(500);
//                    forward(0.4, 3);
//                    strafeLeft(0.4,4);
//                    backward(0.3, 22);
//                    strafeRight(0.4,6);
//                    shooter(0.625);
//                    sleep(1500);
//                    trigger();
//                    strafeLeft(0.4, 5);
//                    trigger();
//                    strafeLeft(0.4,5);
//                    trigger();
//                    shooter(0);
//
//
//                } else if (numberOfRings == 4) {
//                    shooter(0.60);
//                    forward(0.45, 24);
//                    strafeLeft(0.4, 7);
//                    sleep(1500);
//                    trigger();
//                    strafeLeft(0.3, 3.5);
//                    trigger();
//                    strafeLeft(0.3, 3.5);
//                    trigger();
//                    shooter(0);
////                    forward(0.4, 33.5);
////                    strafeRight(0.4, 20);
////                    arm(0.3);
////
//
//                }
//            } else {
//                forward(0.5, 32);
//                strafeRight(0.2, 3);
//                arm(0.2);
//                sleep(250);
//                forward(0.3, 3);
//                strafeLeft(0.3, 3);
//                backward(0.25, 11);
//                strafeRight(0.3, 6);
//                shooter(0.675);
//                sleep(1500);
//                strafeLeft(0.3, 5);
//                trigger();
//                strafeLeft(0.3, 5);
//                trigger();
//                shooter(0);
//
//            }
//            break;
//        }
//
//    }
//}
//
