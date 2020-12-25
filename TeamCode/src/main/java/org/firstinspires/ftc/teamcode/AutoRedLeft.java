package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Library.VisionUtility;


@Autonomous(name = "AutoRedLeft", group = "Philos")
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
            while ((System.currentTimeMillis() - runTime) < 5000) {
                numberOfRings = visionUtility.getRingStack(this);
                if((System.currentTimeMillis()- runTime) >2000 && numberOfRings == 0)
                    forward(0.3, 4);
                if(numberOfRings > 0){
                    break;
                }
                telemetry.addData("stackCount:", numberOfRings);
                telemetry.update();
            }
            visionUtility.tfodDeactivate();


            if (numberOfRings > 0) {
                if (numberOfRings == 1) {

                    strafeLeft(0.5, 12);
                    forward(0.4, 57);
                    strafeRight(0.4, 9);
                    arm(0.3);
                    forward(0.4, 7);
                    backward(0.3, 25);
                    strafeRight(0.7, 16);
                    sleep(1000); //shoot();

                } else if (numberOfRings == 4) {
                    strafeLeft(0.7, 13);
                    forward(0.7, 83.5);
                    strafeRight(0.5, 19.5);
                    sleep(1000);//drop wobble goal
                    backward(0.7, 35);
                    strafeRight(0.5, 2);
                    sleep(1000); //shoot();
                }
            } else {
                forward(0.5, 24);
                strafeRight(0.3, 5);
                arm(30);
                sleep(4000);
                forward(0.3, 8);
                strafeLeft(0.4, 4);
                backward(0.5, 12);
                sleep(1000);//shoot();
            }
            break;
        }

    }
}

