/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="PracticeOpMode1", group="Linear Opmode")
//@Disabled
public class PracticeOpModeTest extends TeleOp_Methods {

    //OpenCvInternalCamera2 phoneCam;
    //StoneOrientationExample.StoneOrientationAnalysisPipeline pipeline;


    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftFrontPower, leftBackPower;
            double rightFrontPower, rightBackPower;
            double intakePower, liftPower;
            //boolean button_b, button_a;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            /*// POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double shoot = gamepad1.right_trigger;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            shooterPower = Range.clip(shoot + shoot, -1.0,1.0);
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            */

            //this is to use  left joystick to translate robot (y: front/back; x: strafing)
            //           use right joystick to turn
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            double goIn = gamepad2.right_trigger;
            double goOut = gamepad2.left_trigger;
            double lift = gamepad2.left_stick_x;
            //double shoot = gamepad1.right_trigger;

            leftFrontPower = (drive + turn + strafe) * .75;
            rightFrontPower = (drive - turn - strafe) * .75;
            rightBackPower = (drive - turn + strafe) * .75;
            leftBackPower = (drive + turn - strafe) * .75;
            intakePower = Range.clip(goIn - goOut, -0.9, 0.9);
            liftPower = Range.clip(lift, -.5, .5);


            if (gamepad2.dpad_right) {
                trigger.setPosition(shootPosition);
                sleep(100);
                trigger.setPosition(readyPosition);
            }

            //shooterPower.setPower(shoot);

            // Send calculated power to wheels
            /*leftBackDrive.setPower(-leftPower);
            rightBackDrive.setPower(-rightPower);
            leftFrontDrive.setPower(-leftPower);
            rightFrontDrive.setPower(-rightPower);
            frontShooter.setPower(shooterPower);
            */
            back_left.setPower(leftBackPower);
            back_right.setPower(rightBackPower);
            front_left.setPower(leftFrontPower);
            front_right.setPower(rightFrontPower);
            intake.setPower(intakePower);
            frontShooter.setPower(shoot);
            wobblegoal.setPower(liftPower);

            //for shooter function and callibration
            //
            if (gamepad2.b) {
                button_b = true;
            } else if (gamepad2.a) {
                button_a = true;
            }
            if (button_b == true) {
                frontShooter.setPower(shoot += 0.025);
                button_b = false;
                sleep(200);
            } else if (button_a == true) {
                frontShooter.setPower(shoot -= 0.025);
                button_a = false;
                sleep(200);
            }
            if (shoot < 0) {
                shoot = 0;
            } else if (gamepad2.x) {
                shoot = 0;
            } else if (gamepad2.y) {
                shoot = 0.66;
            }

            //IMU align to goal/anything
            else if (gamepad1.left_bumper)
                resetAngle();
            else if (gamepad1.right_bumper) {
                //check(0.4);
                //sleep(250);
                //check(0.1);
                PIDrotate(0,0.8);
                //rotate(0,0.4);
            } else if (gamepad1.dpad_up) {
                forward(0.8, 24);
                returnToTeleOp();
            } else if (gamepad1.dpad_right) {
                diagonals(30,40,0.8);
                returnToTeleOp();
            }
            if (gamepad1.y) {
                automatedPowerShots();
                returnToTeleOp();
            }
            if (gamepad1.b)
                testArm(1200,0.3);
/*
            //other random commands
            //
            if (gamepad1.left_bumper) {
                frontShooter.setPower(0.55);
                sleep(2000);
                strafeRight(12.5, 0.1);
                   trigger.setPosition(shootPosition);
                   sleep(100);
                   trigger.setPosition(readyPosition);
                strafeRight(4.5, 0.1);
                   trigger.setPosition(shootPosition);
                   sleep(100);
                   trigger.setPosition(readyPosition);
                strafeRight(4.5, 0.1);
                   trigger.setPosition(shootPosition);
                   sleep(100);
                   trigger.setPosition(readyPosition);
                   sleep(1000);

                frontShooter.setPower(0);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

*/

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left-back (%.2f), left-front (%.2f), right-back (%.2f), right-front (%.2f), frontShooter (%.2f), intake (%.2f)", leftBackPower, leftFrontPower, rightBackPower, rightFrontPower, shoot, intakePower);
            //telemetry.addData("Servos", "trigger (%.2f)", trigger.getPosition());
            telemetry.addData("Angle: ", "%.2f", getAngle());
            telemetry.addData("correction: ", "%.2f", checkDirection());
            //Log.i("[philos:turnTest]", String.format("GettingAngle %.2f", getAngle()));

            telemetry.update();
        }
    }
}

