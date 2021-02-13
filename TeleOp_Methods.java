package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class TeleOp_Methods extends TeleOp_Members {
    /**
     *copy of runUsingIMUAuto methods
     *
     */
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle ()
    {
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
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

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
     * @param degrees Degrees to turn, + is left - is right
     */

    public void rotate(double degrees, double power)
    {
        double  leftPower, rightPower;

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        power = -power;
        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        //leftMotor.setPower(leftPower);
        //rightMotor.setPower(rightPower);
        leftBackDrive.setPower(leftPower);
        leftFrontDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);
        rightFrontDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees + 3.5) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees - 3.5) {}

        // turn the motors off.
        //rightMotor.setPower(0);
        //leftMotor.setPower(0);
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);

        // wait for rotation to stop.
        //sleep(50);

        // reset angle tracking on new heading.
        resetAngle();
    }
    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void PIDrotate(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

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
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                leftBackDrive.setPower(power);
                rightBackDrive.setPower(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                leftBackDrive.setPower(-power);
                rightBackDrive.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                leftBackDrive.setPower(-power);
                rightBackDrive.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
    /**
     * check method for teleOp and possibly autonomous
     * @param power increase power the first check but decrease power the second
     *              add sleep of 50 in between
     */
    public void check (double power)
    {
        if (getAngle() == 0) {
            rotate(0,power);
        }
        else if (getAngle() > 0) {
            rotate(-getAngle(), power);
        }
        else if (getAngle() < 0) {
            rotate(-getAngle(), power);
        }
        //sleep(50);
    }
    //
    //
    //COMPLETELY USELESS ENCODER MOVEMENTS EXCEPT FOR ENDGAME AUTOMATED POWER SHOTS
    //
    //
    /**
     * Forward is never used but is here to test for correction
     * @param speed
     * @param distance
     */
    public void forward(double speed, double distance)
    {
        resetAngle();

        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int counts = (int) ((distance / (4 * Math.PI)) * 1075);
        leftBackDrive.setTargetPosition(counts);
        leftFrontDrive.setTargetPosition(counts);
        rightBackDrive.setTargetPosition(counts);
        rightFrontDrive.setTargetPosition(counts);

        //setting all motors to go forward (positive)

        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setPower(speed);
        leftFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);

        while (opModeIsActive() && leftBackDrive.isBusy() && leftFrontDrive.isBusy() && rightBackDrive.isBusy() && rightFrontDrive.isBusy()) {
            double correction = checkDirection();
            leftBackDrive.setPower(speed - correction);
            //leftFrontDrive.setPower(speed - correction);
            rightBackDrive.setPower(speed + correction);
            //rightFrontDrive.setPower(speed + correction);
        }

        //setting all motor powers to 0 (stopping)
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    /**
     * Need this method for Power Shot Automation
     * @param speed
     * @param distance
     */
    public void strafeRight(double speed, double distance)
    {
        resetAngle();

        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int counts = (int) ((distance / (4 * Math.PI)) * 1075);
        leftBackDrive.setTargetPosition(counts);
        leftFrontDrive.setTargetPosition(-counts);
        rightBackDrive.setTargetPosition(-counts);
        rightFrontDrive.setTargetPosition(counts);

        //setting all motors to go forward (positive)

        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setPower(speed);
        leftFrontDrive.setPower(-speed);
        rightBackDrive.setPower(-speed);
        rightFrontDrive.setPower(speed);

        while (opModeIsActive() && leftBackDrive.isBusy() && leftFrontDrive.isBusy() && rightBackDrive.isBusy() && rightFrontDrive.isBusy()) {
            double correction = checkDirection();
            leftBackDrive.setPower(speed - correction);
            leftFrontDrive.setPower(speed + correction);
            rightBackDrive.setPower(speed - correction);
            rightFrontDrive.setPower(speed + correction);
        }

        //setting all motor powers to 0 (stopping)
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Automated movement of arm to designated parallel position
     * @param counts
     * @param power
     * @param original
     */
    public void testArm(int counts, double power, int original)
    {
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleArm.setPower(-0.2);
        if(wobbleArm.getCurrentPosition() <= original + 10 && wobbleArm.getCurrentPosition() >= original - 10){
            wobbleArm.setPower(0);
            sleep(1000);
        }
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setTargetPosition(counts);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(power);
    }
    /**
     * Automated 3 power shots from Wall
     */
    public void automatedPowerShots()
    {
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
    }
    /**
     * to return all drive base motors to RUN USING ENCODERS so it works in teleOp
     */
    public void returnToTeleOp()
    {
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /**
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
    }
}
