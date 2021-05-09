package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Library.IMUUtility;
import org.firstinspires.ftc.teamcode.Library.VisionUtility;

public class DavidAutonomous_Methods extends LinearOpMode {

    public Hardware robot = new Hardware();
    public VisionUtility visionUtility;
    private IMUUtility imu;
    PIDController pidRotate, pidDrive;
    Orientation lastAngles = new Orientation();
    double globalAngle;


    public void initRobot() {

        robot.initTeleOpNOIMU(hardwareMap);
        visionUtility = new VisionUtility(hardwareMap, this);

        imu = new IMUUtility(hardwareMap);
        imu.initialize(new BNO055IMU.Parameters());

        telemetry.addLine("wait for gyro calibration"); //telling user to wait for gyro to be calibrated
        telemetry.update();

        //waiting for gyro to calibrate
        while (!imu.isGyroCalibrated()) {

        }

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.02, .0003, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.01, 0, 0.005);

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, 0.3);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        telemetry.addData("gyro status :: ", imu.getCalibrationStatus().toString()); //returning gyro status (ready to start)
        telemetry.update();

    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    protected void PIDrotate(double degrees, double power) {
        // restart imu angle tracking.
        //resetAngle();

        robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                robot.back_left.setPower(power);
                robot.front_left.setPower(power);
                robot.back_right.setPower(-power);
                robot.front_right.setPower(-power);
                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.back_left.setPower(-power);
                robot.front_left.setPower(-power);
                robot.back_right.setPower(power);
                robot.front_right.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.back_left.setPower(-power);
                robot.front_left.setPower(-power);
                robot.back_right.setPower(power);
                robot.front_right.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.back_left.setPower(0);
        robot.back_right.setPower(0);
        robot.front_left.setPower(0);
        robot.front_right.setPower(0);

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

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

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


    public void forward(double speed, double distance, boolean usePidRotate, boolean useIntake) {

        int counts = (int) ((distance / (4 * Math.PI)) * 1200);
        robot.back_left.setTargetPosition(counts);
        robot.back_right.setTargetPosition(counts);
        robot.front_right.setTargetPosition(counts);
        robot.front_left.setTargetPosition(counts);

        //setting all motors to go forward (positive)

        robot.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.back_left.setPower(speed);
        robot.back_right.setPower(speed);
        robot.front_right.setPower(speed);
        robot.front_left.setPower(speed);

        while (opModeIsActive() && robot.back_left.isBusy() && robot.back_right.isBusy() && robot.front_right.isBusy() && robot.front_left.isBusy()) {
            double correction = pidDrive.performPID(getAngle());
            robot.back_left.setPower(speed - correction);
            robot.front_left.setPower(speed - correction);
            robot.back_right.setPower(speed + correction);
            robot.front_right.setPower(speed + correction);

            if (useIntake) {
                intake(0.8);
            }
        }


        //setting all motor powers to 0 (stopping)
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (usePidRotate) {
            sleep(200);
            PIDrotate(0, 1);
        }
        intake(0);
    }

    public void backward(double speed, double distance, boolean usePidRotate, boolean useIntake) {


        //RPM for GoBuilda YEllow Jacket planetary gear motors
        int counts = (int) ((distance / (4 * Math.PI)) * 1200);

        robot.back_left.setTargetPosition(-counts);
        robot.back_right.setTargetPosition(-counts);
        robot.front_right.setTargetPosition(-counts);
        robot.front_left.setTargetPosition(-counts);

        //setting all motors to go forward (positive)

        robot.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.back_left.setPower(-speed);
        robot.back_right.setPower(-speed);
        robot.front_right.setPower(-speed);
        robot.front_left.setPower(-speed);

        while (opModeIsActive() && robot.back_left.isBusy() && robot.back_right.isBusy() && robot.front_right.isBusy() && robot.front_left.isBusy()) {
            double correction = pidDrive.performPID(getAngle());
            robot.back_left.setPower(speed + correction);
            robot.front_left.setPower(speed + correction);
            robot.back_right.setPower(speed - correction);
            robot.front_right.setPower(speed - correction);
            if (useIntake)
                intake(0.8);
        }


        //setting all motor powers to 0 (stopping)
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (usePidRotate) {
            sleep(300);
            PIDrotate(0, 1);
        }
        intake(0);

    }

    public void strafeLeft(double speed, double distance, boolean usePidRotate, boolean useIntake) {

        int counts = (int) ((distance / (4 * Math.PI)) * 1200);
        robot.back_left.setTargetPosition(counts);
        robot.back_right.setTargetPosition(-counts);
        robot.front_right.setTargetPosition(counts);
        robot.front_left.setTargetPosition(-counts);

        robot.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.back_left.setPower(speed);
        robot.back_right.setPower(-speed);
        robot.front_right.setPower(speed);
        robot.front_left.setPower(-speed);

        while (opModeIsActive() && robot.back_left.isBusy() && robot.back_right.isBusy() && robot.front_right.isBusy() && robot.front_left.isBusy()) {
            double correction = pidDrive.performPID(getAngle());
            robot.back_left.setPower(speed - correction);
            robot.front_left.setPower(speed + correction);
            robot.back_right.setPower(speed - correction);
            robot.front_right.setPower(speed + correction);
            if (useIntake) {
                intake(0.8);
            }
        }


        //setting all motor powers to 0 (stopping)
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (usePidRotate) {
            sleep(300);
            PIDrotate(0, 0.8);
        }
        intake(0);
    }

    public void strafeRight(double speed, double distance, boolean usePidRotate, boolean useIntake) {


        int counts = (int) ((distance / (4 * Math.PI)) * 1200);
        robot.back_left.setTargetPosition(-counts);
        robot.back_right.setTargetPosition(counts);
        robot.front_right.setTargetPosition(-counts);
        robot.front_left.setTargetPosition(counts);

        //setting all motors to go forward (positive)

        robot.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.back_left.setPower(-speed);
        robot.back_right.setPower(speed);
        robot.front_right.setPower(-speed);
        robot.front_left.setPower(speed);


        while (opModeIsActive() && robot.back_left.isBusy() && robot.back_right.isBusy() && robot.front_right.isBusy() && robot.front_left.isBusy()) {
            double correction = pidDrive.performPID(getAngle());
            robot.back_left.setPower(speed + correction);
            robot.front_left.setPower(speed - correction);
            robot.back_right.setPower(speed + correction);
            robot.front_right.setPower(speed - correction);
            if (useIntake)
                intake(0.8);
        }

        //setting all motor powers to 0 (stopping)
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (usePidRotate) {
            sleep(300);
            PIDrotate(0, 0.8);
        }
        intake(0);
    }

    public void turn(double power, int angle, String direction, boolean usePidRotate) {

        Orientation startOrientation = imu.resetAndStart(direction);
        robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        double targetAngle;
        double currentAngle;
        double actualPower;
        double stoppingAngle = 0;

        if (direction.equalsIgnoreCase(("Clockwise"))) {
            actualPower = -(power);

            targetAngle = startOrientation.firstAngle - angle;
            currentAngle = startOrientation.firstAngle;

            while ((currentAngle - stoppingAngle) > targetAngle && opModeIsActive()) {

                telemetry.addData("start:", startOrientation.firstAngle);
                telemetry.addData("current:", currentAngle);
                telemetry.addData("target:", targetAngle);

                currentAngle = imu.getAngularOrientation().firstAngle;
                telemetry.addData("currentAngle", currentAngle);
                telemetry.update();
                AngularVelocity v = imu.getAngularVelocity();
                float speed = Math.abs(v.xRotationRate);
                stoppingAngle = Math.abs((0.25f * speed) - 7f);

                telemetry.addData("speed", speed );
                telemetry.addData("stoppingAngle", stoppingAngle);
                telemetry.update();

                Log.i("[philos:turnTest]", String.format("StartingAngle=%f, CurrentAngle=%f, AngularVelocity=%f,targetAngle = %f, StoppingAngle=%f", startOrientation.firstAngle, currentAngle, speed, targetAngle, stoppingAngle));

                robot.front_left.setPower(-(actualPower));
                robot.front_right.setPower(actualPower);
                robot.back_left.setPower(-(actualPower));
                robot.back_right.setPower(actualPower);
                sleep(100);
            }
        } else {
            actualPower = power;

            targetAngle = startOrientation.firstAngle + angle;
            currentAngle = startOrientation.firstAngle;
            while ((currentAngle + stoppingAngle) < targetAngle && opModeIsActive()) {

                telemetry.addData("start:", startOrientation.firstAngle);
                telemetry.addData("current:", currentAngle);
                telemetry.addData("target:", targetAngle);
                telemetry.update();

                currentAngle = imu.getAngularOrientation().firstAngle;
                AngularVelocity v = imu.getAngularVelocity();
                float speed = Math.abs(v.xRotationRate);
                stoppingAngle = Math.abs((0.25f * speed) - 7f); //-8.5609
                Log.i("[philos:turnTest]", String.format("StartingAngle=%f, CurrentAngle=%f, AngularVelocity=%f,targetAngle = %f, StoppingAngle=%f", startOrientation.firstAngle, currentAngle, speed, targetAngle, stoppingAngle));


                robot.front_left.setPower(-(actualPower));
                robot.front_right.setPower(actualPower);
                robot.back_left.setPower(-(actualPower));
                robot.back_right.setPower(actualPower);
            }
        }
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (usePidRotate)
            PIDrotate(0,0.8);
    }


    public void shooter (double power) {
        robot.shooter.setPower(power);
    }

    public void trigger() {

        robot.trigger.setPosition(0.475);
        sleep(200);
        robot.trigger.setPosition(0.72);
        sleep(200);
    }

    public void intake (double power) {

        robot.intake.setPower(power);
        robot.transfer.setPower(0.35);

    }

    public void arm(double power, int counts) {
        robot.wobblegoal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.wobblegoal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wobblegoal.setTargetPosition(counts);
        robot.wobblegoal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.wobblegoal.setPower(power);
        while (opModeIsActive() && robot.wobblegoal.isBusy()) {
            robot.wobblegoal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        robot.wobblegoal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void testEncoders() {

        robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int start_back_right = robot.back_right.getCurrentPosition();
        int start_back_left = robot.back_left.getCurrentPosition();
        int start_front_right = robot.front_right.getCurrentPosition();
        int start_front_left = robot.back_left.getCurrentPosition();

        sleep(10000);
        int end_back_right = robot.back_right.getCurrentPosition();
        int end_back_left = robot.back_left.getCurrentPosition();
        int end_front_right = robot.front_right.getCurrentPosition();
        int end_front_left = robot.back_left.getCurrentPosition();


        telemetry.addData("back_right Encoder Value", (start_back_right - end_back_right));
        telemetry.addData("back_left Encoder Value", (start_back_left - end_back_left));
        telemetry.addData("front_left Encoder Value", (start_front_left - end_front_left));
        telemetry.addData("front_right Encoder Value", (start_front_right - end_front_right));
        telemetry.update();

    }

    public void grabOpen() {

        robot.grab.setPosition(0.7);
    }

    public void grabClose() {

        robot.grab.setPosition(1);
    }
    public void nullleftRedAuto(boolean getSingleRing, boolean getQuadRing, boolean powerShots)
    {
        strafeLeft(0.6,8,false, false); //This segment is for the LEFT RED TAPE
        forward(0.7,28,false,false);
        if(powerShots)
            shooter(0.45); //power for power shots
        else
            shooter(0.65);
        if(powerShots)
        {
            PIDrotate(6,1); trigger();
            PIDrotate(-6,1); trigger();
            PIDrotate(0,1); trigger();
            shooter(0.65);
            strafeRight(0.6,14,true,false);
        }
        else if (!getSingleRing || !getQuadRing) {
        PIDrotate(-18, 1); //shooting the ring at an angle for high goal
        trigger();
        trigger();
        trigger();
        PIDrotate(0, 1);
        shooter(0);
        }
        else {
            if(getSingleRing || getQuadRing) {shooter(0.65);}
            strafeRight(0.6,14,true,true);
            trigger();
            trigger();
            trigger();
            trigger();
            shooter(0);
        }
        /**
         * This is what we are going to do after in case this works better and we need to get the single ring
         *
         */
        if(getSingleRing) {
            intake(1);shooter(0.65);
            backward(0.4, 9, true, true); //getting the single ring
            forward(0.7, 8, true, true);
            trigger();
            trigger();
            trigger();
            if(getQuadRing){
                backward(0.4,20, true, true);
                forward(0.8,19,true,true);
                trigger();trigger();trigger();
            }
        }
        strafeLeft(0.5, 12, false, false); //move out of the way and park
        forward(0.9,6,true, false);
        shooter(0);
    }
    public void nullRightBlueAuto(boolean getSingleRing, boolean getQuadRing, boolean powerShots)
    {
        strafeRight(0.4,8,false, false); //This segment is for the RIGHT BLUE TAPE
        forward(0.7,28,false,false);
        if(powerShots) shooter(0.45); //power for power shots
        else
            shooter(0.65);
        if(powerShots)
        {
            PIDrotate(6,1); trigger();
            PIDrotate(-6,1); trigger();
            PIDrotate(0,1); trigger();
            shooter(0.65);
            strafeLeft(0.6,14,true,false);
        }
        else if (!getSingleRing || !getQuadRing) {
            PIDrotate(18, 1); //shooting the ring at an angle for high goal
            trigger();
            trigger();
            trigger();
            PIDrotate(0, 1);
            shooter(0);
        }
        else {
            if(getSingleRing || getQuadRing) {shooter(0.65);}
            strafeLeft(0.6,14,true,true);
            trigger();
            trigger();
            trigger();
            trigger();
            shooter(0);
        }
        /**
         * This is what we are going to do after in case this works better and we need to get the single ring
         *
         */
        if(getSingleRing) {
            intake(1);shooter(0.65);
            backward(0.4, 9, true, true); //getting the single ring
            forward(0.7, 8, true, true);
            trigger();
            trigger();
            trigger();
            if(getQuadRing){
                backward(0.4,20, true, true);
                forward(0.8,19,true,true);
                trigger();trigger();trigger();
            }
        }
        strafeRight(0.5, 12, false, false); //move out of the way and park
        forward(0.9,6,true, false);
        shooter(0);
    }
    public void nullInnerTapeAuto(String alliance, boolean getSingleRing, boolean getQuadRing, boolean powerShots)
    {
        if (alliance.equals("red")){
            nullleftRedAuto(getSingleRing, getQuadRing, powerShots);
    } else if (alliance.equals("blue")){
            nullRightBlueAuto(getSingleRing, getQuadRing, powerShots);
        }
    }
    public void nullRightRedAuto(boolean getSingleRing, boolean getQuadRing, boolean powerShots)
    {
        strafeRight(0.6,6,false, false); //This segment is for the RIGHT RED TAPE
        if(powerShots)
            shooter(0.45); //power for power shots
        else
            shooter(0.65);
        forward(0.7,28,false,false);
        if(powerShots)
        {
            strafeLeft(0.6,3,false,false);
            PIDrotate(16,1); trigger();
            PIDrotate(16,1); trigger();
            PIDrotate(25,1); trigger();
            shooter(0.65);
            PIDrotate(0,1);
            strafeLeft(0.6,13,true,false);
        }
        else if (!getSingleRing || !getQuadRing) { //This is for 0 ZERO 0 ZERO 0 ZERO rings
            shooter(0.65);
            strafeLeft(0.6,3,false,false);
            PIDrotate(16, 1); //shooting the ring at an angle for high goal
            trigger();
            trigger();
            trigger();
            PIDrotate(0, 1);
            shooter(0); //intake(0);
            sleep(5000);
            strafeLeft(0.5, 13, false, false); //move out of the way and park
        }
        else {
            strafeLeft(0.6,13,true,true);
            trigger();
            trigger();
            trigger();
            trigger();
            shooter(0);
        }
        /**
         * This is what we are going to do after in case this works better and we need to get the single ring
         *
         */
        if(getSingleRing) {
            intake(1);shooter(0.65);
            backward(0.4, 9, true, true); //getting the single ring
            forward(0.7, 8, true, true);
            trigger();
            trigger();
            trigger();
            if(getQuadRing){
                backward(0.4,20, true, true);
                forward(0.8,19,true,true);
                trigger(); trigger(); trigger();
            }
        }
        forward(0.9,6,true, false);
        shooter(0);
    }
    public void nullOuterTapeAuto(String alliance, boolean getSingleRing, boolean getQuadRing, boolean powerShots)
    {
        if (alliance.equals("red")){
            nullRightRedAuto(getSingleRing, getQuadRing, powerShots);
        } else if (alliance.equals("blue")){
            nullRightBlueAuto(getSingleRing, getQuadRing, powerShots);
        }
    }
    public void autoRedMoreRingsFour()
    {
        strafeRight(0.85, 17, true, false);
        forward(0.85, 52, true, false);
        sleep(50);
        strafeLeft(0.6, 19, true, false);
        shooter(0.71);
        PIDrotate(180,1);
        arm(0.8, 1325);
        grabOpen();
        arm(0.8, -1100);
        PIDrotate(-180,1);
        backward(0.9, 22.5, true, false);
        // strafeLeft(0.5,3.5);
        strafeRight(0.6, 4, true, false);
        PIDrotate(0, 1);
        sleep(100);
        trigger();
        trigger();
        trigger();
        trigger();
        intake(0.9);
        backward(0.6, 6.5, false, false);

        backward(0.1, 3, false, true);
        PIDrotate(0, 0.9);
        forward(0.9, 7, true, true);

        trigger();
        trigger();
        trigger();
        backward(0.6, 8, false, false);
        backward(0.1, 6, false, true);
        PIDrotate(0, 0.9);
        forward(0.9, 9.5, true, true);

        trigger();
        trigger();
        trigger();
        forward(0.9, 9, false, false);
        intake(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}


