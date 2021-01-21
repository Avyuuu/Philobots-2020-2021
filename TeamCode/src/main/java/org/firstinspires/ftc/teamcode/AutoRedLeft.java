package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Library.VisionUtility;


@Autonomous(name = "AutoRedLeftWG", group = "Philos")
public class AutoRedLeft extends Autonomous_Methods {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initializing robot config and vision setup
        initRobot();
        visionUtility.init();
        int numberOfRings = 0;

        waitForStart();

        while(opModeIsActive()) {
            long runTime = System.currentTimeMillis();
            while ((System.currentTimeMillis() - runTime) < 3000) {
                numberOfRings = visionUtility.getRingStack(this);
                if(numberOfRings > 0){
                    break;
                }
                telemetry.addData("stackCount:", numberOfRings);
                telemetry.update();
            }
            visionUtility.tfodDeactivate();


            if (numberOfRings > 0) {
                if (numberOfRings == 1) {

                    strafeLeft(0.5, 15);
                    forward(0.4, 48);
                    arm(0.2);
                    sleep(500);
                    forward(0.4, 3);
                    strafeLeft(0.4,4);
                    backward(0.3, 22);
                    strafeRight(0.4,6);
                    shooter(0.625);
                    sleep(1500);
                    trigger();
                    strafeLeft(0.4, 5);
                    trigger();
                    strafeLeft(0.4,5);
                    trigger();
                    shooter(0);

                } else if (numberOfRings == 4) {
                    strafeLeft(0.4, 9);
                    forward(0.45, 54);
                    strafeRight(0.4, 14);
                    arm(0.3);
                    forward(0.5,3);
                    strafeLeft(0.4,3.5);
                    backward(0.4, 31);
                    strafeLeft(0.3, 9.5);
                    shooter(0.6);
                    sleep(1500);
//                    trigger();
//                    trigger();
//                    trigger();
                    trigger();
                    strafeLeft(0.3, 3.5);
                    trigger();
                    strafeLeft(0.3, 3.5);
                    trigger();
                    shooter(0);
                }
            } else {
                forward(0.5, 36);
                arm(0.2);
                sleep(250);
                forward(0.3, 3);
                strafeLeft(0.3, 4);
                backward(0.25, 11);
                strafeLeft(0.3, 9);
                shooter(0.575);
                sleep(1500);
                trigger();
                strafeLeft(0.3, 5);
                trigger();
                strafeLeft(0.3, 5);
                trigger();
                shooter(0);

            }
            break;
        }

    }
}

