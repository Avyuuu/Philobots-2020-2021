package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class TeleOp_Methods extends TeleOp_Members {
    /**
     * copy of runUsingIMUAuto methods
     */
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (globalAngle < -180)
            globalAngle += 360;
        else if (globalAngle > 180)
            globalAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .1;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */

    public void rotate(double degrees, double power) {
        double leftPower, rightPower;

        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // restart imu movement tracking.
        //resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        //power = -power;
        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        //leftMotor.setPower(leftPower);
        //rightMotor.setPower(rightPower);
        back_left.setPower(leftPower);
        front_left.setPower(leftPower);
        back_right.setPower(rightPower);
        front_right.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() > degrees + 3.5) {
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees - 3.5) {
            }

        // turn the motors off.
        //rightMotor.setPower(0);
        //leftMotor.setPower(0);
        back_left.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        front_right.setPower(0);

        // wait for rotation to stop.
        //sleep(50);

        // reset angle tracking on new heading.
        //resetAngle();
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    protected void PIDrotate(double degrees, double power) {
        // restart imu angle tracking.
        //resetAngle();

        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees); //all it does it put degrees into m_setPoint so that it can be used in next method
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                back_left.setPower(power);
                front_left.setPower(power);
                back_right.setPower(-power);
                front_right.setPower(-power);
                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                back_left.setPower(-power);
                front_left.setPower(-power);
                back_right.setPower(power);
                front_right.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                back_left.setPower(-power);
                front_left.setPower(-power);
                back_right.setPower(power);
                front_right.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);

        //rotation = getAngle();

        // wait for rotation to stop.
        sleep(250);

        // reset angle tracking on new heading.
        //resetAngle();
    }

    /**
     * check method for teleOp and possibly autonomous
     *
     * @param power increase power the first check but decrease power the second
     *              add sleep of 50 in between
     */
    public void check(double power) {
        sleep(50);
        if (getAngle() == 0) {
            PIDrotate(0, power);
        } else if (getAngle() > 0) {
            PIDrotate(-getAngle(), power);
        } else if (getAngle() < 0) {
            PIDrotate(-getAngle(), power);
        }
        sleep(250);
    }
    //
    //
    //COMPLETELY USELESS ENCODER MOVEMENTS EXCEPT FOR ENDGAME AUTOMATED POWER SHOTS
    //
    //

    /**
     * Forward is never used but is here to test for correction
     *
     * @param speed
     * @param distance
     */
    public void forward(double speed, double distance) {
        //resetAngle();

        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int counts = (int) ((distance / (4 * Math.PI)) * 1075);
        back_left.setTargetPosition(counts);
        front_left.setTargetPosition(counts);
        back_right.setTargetPosition(counts);
        front_right.setTargetPosition(counts);

        //setting all motors to go forward (positive)

        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setPower(speed);
        front_left.setPower(speed);
        back_right.setPower(speed);
        front_right.setPower(speed);

        while (opModeIsActive() && back_left.isBusy() && front_left.isBusy() && back_right.isBusy() && front_right.isBusy()) {
            double correction = pidDrive.performPID(getAngle());
            back_left.setPower(speed - correction);
            front_left.setPower(speed - correction);
            back_right.setPower(speed + correction);
            front_right.setPower(speed + correction);

            telemetry.addData("Angle: ", "%.2f", getAngle());
            telemetry.addData("correction: ", "%.2f", checkDirection());
        }

        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        front_right.setPower(0);

        //setting all motor powers to 0 (stopping)
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //check(0.3);
    }

    /**
     * Need this method for Power Shot Automation
     *
     * @param speed
     * @param distance
     */
    public void strafeRight(double speed, double distance) {
        //resetAngle();

        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int counts = (int) ((distance / (4 * Math.PI)) * 1075);
        back_left.setTargetPosition(-counts);
        front_left.setTargetPosition(counts);
        back_right.setTargetPosition(counts);
        front_right.setTargetPosition(-counts);

        //setting all motors to go forward (positive)

        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setPower(-speed);
        front_left.setPower(speed);
        back_right.setPower(speed);
        front_right.setPower(-speed);

        while (opModeIsActive() && back_left.isBusy() && front_left.isBusy() && back_right.isBusy() && front_right.isBusy()) {

            double correction = pidDrive.performPID(getAngle());
            back_left.setPower(speed + correction);
            front_left.setPower(speed - correction);
            back_right.setPower(speed + correction);
            front_right.setPower(speed - correction);

            telemetry.addData("Angle: ", "%.2f", getAngle());
            telemetry.addData("correction: ", "%.2f", checkDirection());
        }
        turnOnZeroBehavior();

        //setting all motor powers to 0 (stopping)
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void strafeLeft(double speed, double distance) {
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int counts = (int) ((distance / (4 * Math.PI)) * 1075);
        back_left.setTargetPosition(counts);
        back_right.setTargetPosition(-counts);
        front_right.setTargetPosition(counts);
        front_left.setTargetPosition(-counts);

        //setting all motors to go forward (positive)

        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setPower(speed);
        back_right.setPower(-speed);
        front_right.setPower(speed);
        front_left.setPower(-speed);

        while (opModeIsActive() && back_left.isBusy() && back_right.isBusy() && front_right.isBusy() && front_left.isBusy()) {
            double correction = pidDrive.performPID(getAngle());
            back_left.setPower(speed - correction);
            front_left.setPower(speed + correction);
            back_right.setPower(speed - correction);
            front_right.setPower(speed + correction);

            telemetry.addData("Angle: ", "%.2f", getAngle());
            telemetry.addData("correction: ", "%.2f", checkDirection());
        }

        turnOnZeroBehavior();

        //setting all motor powers to 0 (stopping)
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //check(0.4);
    }
    /**
     * Automated movement of arm to designated parallel position
     * @param counts
     * @param power
     */
    /*
    public void testArm(int counts, double power)
    {
        wobblegoal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobblegoal.setTargetPosition(counts);
        wobblegoal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobblegoal.setPower(power);
        while (opModeIsActive() && wobblegoal.isBusy()) { }
        turnOnZeroBehavior();
        wobblegoal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
     */

    /**
     * Automated 3 power shots from Wall
     */
    public void automatedPowerShots() {
        //resetAngle();
        frontShooter.setPower(0.55);
        strafeLeft(0.4, 16.25);
        PIDrotate(0, 0.8);
        trigger();
        PIDrotate(6.5, 0.8);
        trigger();
        PIDrotate(-6.5, 0.8);
        trigger();
        frontShooter.setPower(0);
    }

    /**
     * to return all drive base motors to RUN USING ENCODERS so it works in teleOp
     */
    public void returnToTeleOp() {
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobblegoal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Initiate the zero behavior / BRAKE behavior when setPower is equal to 0
     */
    public void turnOnZeroBehavior() {
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobblegoal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        front_right.setPower(0);
        wobblegoal.setPower(0);
    }

    public void trigger() {

        trigger.setPosition(0.5);
        sleep(200);
        trigger.setPosition(0.7);
        sleep(400);
    }

    /**
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
    }
}
