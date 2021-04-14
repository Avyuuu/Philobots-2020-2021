package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "Test", group = "Philos")
public class AutoRedLeft extends Autonomous_Methods {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initializing robot config and vision setup
        initRobot();
        int numberOfRings = 0;

        waitForStart();

        while (opModeIsActive()) {
            testEncoders();
            sleep(600000);
            break;
        }
    }
}

