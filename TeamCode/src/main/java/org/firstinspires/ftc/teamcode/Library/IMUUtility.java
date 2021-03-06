package org.firstinspires.ftc.teamcode.Library;

//import android.support.annotation.NonNull;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class IMUUtility implements BNO055IMU {


    public BNO055IMU myIMU;  //the original IMU object

    private Orientation startOrientation; //Sets the starting orientation before IMU start moving; it needs to be set by calling resetAndStart
    private String turningDirection; //Sets the turning direction so that MyBoschIMU can use this data to adjust the value in getAngularOrientation

    //Constructor
    public IMUUtility(HardwareMap hardwareMap) {
        myIMU = hardwareMap.get(BNO055IMU.class, "imu");
    }

    @Override
    public boolean initialize(BNO055IMU.Parameters parameters) {
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Start the logging of measured acceleration
        myIMU.initialize(parameters);

        myIMU.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        resetAndStart("");

        return true;
    }
    //Before turning, call this method to keep the starting position in this object
    //Return the starting position (orientation) to the caller so that caller can use startOriention to do some calculation
    public Orientation resetAndStart(String direction) {
        startOrientation = myIMU.getAngularOrientation();
        turningDirection = direction;

        return startOrientation;
    }
    //Based on the start first angle and direction, adjust the first angle
    @Override
    public Orientation getAngularOrientation() {

        Orientation orientation = myIMU.getAngularOrientation();
        if (orientation == null)
            return null;

        if(startOrientation == null)
            return null;

        if (startOrientation.firstAngle > 0 && turningDirection.equalsIgnoreCase("COUNTERCLOCKWISE") && orientation.firstAngle < 0) {
            orientation.firstAngle = orientation.firstAngle + 360;
            Log.i("[Philos:turnTest]", String.format("counterclockwise:%f", orientation.firstAngle));
        }
        else if (startOrientation.firstAngle < 0 && turningDirection.equalsIgnoreCase("CLOCKWISE")  && orientation.firstAngle > 0) {
            orientation.firstAngle = orientation.firstAngle - 360;
            Log.i("[Philos:turnTest]", String.format("clockwise:%f", orientation.firstAngle));
        }
        else if (turningDirection.equalsIgnoreCase("")){
            if (startOrientation.firstAngle > 0 && orientation.firstAngle < -90) {
                orientation.firstAngle = orientation.firstAngle + 360;
            }
            else if (startOrientation.firstAngle < 0 && orientation.firstAngle > 90) {
                orientation.firstAngle = orientation.firstAngle - 360;
            }
        }


        return orientation;
    }



    @Override
    public BNO055IMU.Parameters getParameters() {
        return myIMU.getParameters();
    }

    @Override
    public void close() {
        myIMU.close();
    }

    @Override
    public Orientation getAngularOrientation(AxesReference reference, AxesOrder order, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit) {
        return myIMU.getAngularOrientation(reference, order, angleUnit);
    }

    @Override
    public Acceleration getOverallAcceleration() {
        return myIMU.getOverallAcceleration();
    }

    @Override
    public AngularVelocity getAngularVelocity() {
        return myIMU.getAngularVelocity();
    } // add myIMU Nov 1 to return.

    @Override
    public Acceleration getLinearAcceleration() {
        return null;
    }

    @Override
    public Acceleration getGravity() {
        return null;
    }

    @Override
    public Temperature getTemperature() {
        return null;
    }

    @Override
    public MagneticFlux getMagneticFieldStrength() {
        return null;
    }

    @Override
    public Quaternion getQuaternionOrientation() {
        return null;
    }

    @Override
    public Position getPosition() {
        return myIMU.getPosition();
    }

    @Override
    public Velocity getVelocity() {
        return myIMU.getVelocity();
    }

    @Override
    public Acceleration getAcceleration() {
        return myIMU.getAcceleration();
    }

    @Override
    public void startAccelerationIntegration(Position initialPosition, Velocity initialVelocity, int msPollInterval) {
        myIMU.startAccelerationIntegration(initialPosition, initialVelocity, msPollInterval);
    }

    @Override
    public void stopAccelerationIntegration() {
        myIMU.stopAccelerationIntegration();
    }

    @Override
    public SystemStatus getSystemStatus() {
        return myIMU.getSystemStatus();
    }

    @Override
    public SystemError getSystemError() {
        return myIMU.getSystemError();
    }

    @Override
    public CalibrationStatus getCalibrationStatus() {
        return myIMU.getCalibrationStatus();
    }

    @Override
    public boolean isSystemCalibrated() {
        return myIMU.isSystemCalibrated();
    }

    @Override
    public boolean isGyroCalibrated() {
        return myIMU.isGyroCalibrated();
    }

    @Override
    public boolean isAccelerometerCalibrated() {
        return myIMU.isAccelerometerCalibrated();
    }

    @Override
    public boolean isMagnetometerCalibrated() {
        return myIMU.isMagnetometerCalibrated();
    }

    @Override
    public CalibrationData readCalibrationData() {
        return myIMU.readCalibrationData();
    }

    @Override
    public void writeCalibrationData(CalibrationData data) {

    }

    @Override
    public byte read8(Register register) {
        return 0;
    }

    @Override
    public byte[] read(Register register, int cb) {
        return new byte[0];
    }

    @Override
    public void write8(Register register, int bVal) {

    }

    @Override
    public void write(Register register, byte[] data) {

    }
}