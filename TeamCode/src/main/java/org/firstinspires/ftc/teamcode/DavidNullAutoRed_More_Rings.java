package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED_WG_7_HG", group = "Philos")
public class DavidNullAutoRed_More_Rings extends DavidAutonomous_Methods {
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
            numberOfRings = 1;
            if (numberOfRings > 0) {
                if (numberOfRings == 1) {
                    //                    //nullOuterTapeAuto("red",false,false,false);
                    //                    //autoRedMoreRingsFour();
                    //                    //PIDrotate(270,0.4);
                    turn(0.5,90, "clockwise",false);
                } else if (numberOfRings == 4) {
                    nullInnerTapeAuto("blue", true, true, false);
                    //shoots all three rings in possession
                    //gets four rings in stack
                    //park
                }
            } else {
                nullInnerTapeAuto("blue",true,true,true);
                //shoots all three rings in possession
                //gets four rings in stack
                //park
            }

            break;
        }
    }

}


