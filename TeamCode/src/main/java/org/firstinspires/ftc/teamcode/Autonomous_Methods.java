package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Library.IMUUtility;
import org.firstinspires.ftc.teamcode.Library.VisionUtility;

public class Autonomous_Methods extends LinearOpMode {

    private Hardware robot = new Hardware();
    public VisionUtility visionUtility;
    private IMUUtility imu;


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

        telemetry.addData("gyro status :: ", imu.getCalibrationStatus().toString()); //returning gyro status (ready to start)
        telemetry.update();

    }

    public void forward(double speed, double distance) {

        int counts = (int) ((distance / (4 * Math.PI)) * 1075);
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
        }


        //setting all motor powers to 0 (stopping)
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//        int currentPosition = 0;
//        //RPM for GoBuilda YEllow Jacket planetary gear motors
//        int targetEncodedValue  = (int) Math.round((distance / (4 * Math.PI)) * 1150);
//
//        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//
//        while (opModeIsActive() && currentPosition <= targetEncodedValue){
//            currentPosition = robot.back_right.getCurrentPosition();
//            robot.back_left.setPower(speed);
//            robot.back_right.setPower(speed);
//            robot.front_right.setPower(speed);
//            robot.front_left.setPower(speed);
//        }
//
//        //setting all motor powers to 0 (stopping)
//        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void backward(double speed, double distance) {


        //RPM for GoBuilda YEllow Jacket planetary gear motors
        int counts = (int) ((distance / (4 * Math.PI)) * 1150);
//        robot.back_left.setTargetPosition(-counts);
//        robot.back_right.setTargetPosition(-counts);
//        robot.front_right.setTargetPosition(-counts);
//        robot.front_left.setTargetPosition(-counts);
//
//        //setting all motors to go forward (positive)
//
//        robot.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//
//        while (opModeIsActive() && robot.back_right.getCurrentPosition() <= counts){
//            robot.back_left.setPower(-speed);
//            robot.back_right.setPower(-speed);
//            robot.front_right.setPower(-speed);
//            robot.front_left.setPower(-speed);
//        }
//
//            //setting all motor powers to 0 (stopping)
//        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        }


        //setting all motor powers to 0 (stopping)
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void strafeLeft(double speed, double distance) {

        int counts = (int) ((distance / (4 * Math.PI)) * 1150);
        robot.back_left.setTargetPosition(counts);
        robot.back_right.setTargetPosition(-counts);
        robot.front_right.setTargetPosition(counts);
        robot.front_left.setTargetPosition(-counts);

        //setting all motors to go forward (positive)

        robot.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.back_left.setPower(speed);
        robot.back_right.setPower(-speed);
        robot.front_right.setPower(speed);
        robot.front_left.setPower(-speed);

        while (opModeIsActive() && robot.back_left.isBusy() && robot.back_right.isBusy() && robot.front_right.isBusy() && robot.front_left.isBusy()) {
        }


        //setting all motor powers to 0 (stopping)
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void strafeRight(double speed, double distance) {


        int counts = (int) ((distance / (4 * Math.PI)) * 1150);
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
        }

        //setting all motor powers to 0 (stopping)
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void turn(double power, int angle, String direction) {

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
    }


    public void StopAll() {
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void shoot (double power) {

       robot.shooter.setPower(power);

    }

    public void intake (double power) {

        robot.intake.setPower(power);

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}



