package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class TeleOp_Members extends LinearOpMode {
    public static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 19.2;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 3.7795;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    //Declare OpMode Members
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotorEx back_left = null;
    public DcMotorEx back_right = null;
    public DcMotorEx front_left = null;
    public DcMotorEx front_right = null;
    public DcMotorEx frontShooter = null;
    public DcMotor wobblegoal = null;
    public DcMotorEx intake = null;
    public Servo trigger = null;
    public Servo grab = null;
    //static final double     DRIVE_SPEED             = 0.6;
    //static final double     TURN_SPEED              = 0.5;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, rotation, correction;
    double readyPosition = 0.7, shootPosition = 0.475;
    double shoot = 0;
    boolean button_b = false, button_a = false;
    PIDController pidRotate, pidDrive, pidMovement;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    /**
     * initialize robot so cleaner
     */
    public void initRobot() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        back_left = hardwareMap.get(DcMotorEx.class, "leftRear");
        back_right = hardwareMap.get(DcMotorEx.class, "rightRear");
        front_left = hardwareMap.get(DcMotorEx.class, "leftFront");
        front_right = hardwareMap.get(DcMotorEx.class, "rightFront");
        frontShooter = hardwareMap.get(DcMotorEx.class, "frontShooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        wobblegoal = hardwareMap.get(DcMotor.class, "wobbleArm");
        trigger = hardwareMap.servo.get("trigger");
        grab = hardwareMap.servo.get("grab");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        frontShooter.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        wobblegoal.setDirection(DcMotorSimple.Direction.FORWARD);


        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobblegoal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontShooter.setVelocityPIDFCoefficients(10, .5, 0, 11);
        intake.setVelocityPIDFCoefficients(10, .5, 01, 11);
        back_left.setPositionPIDFCoefficients(5.0);
        back_right.setPositionPIDFCoefficients(5.0);
        front_left.setPositionPIDFCoefficients(5.0);
        front_right.setPositionPIDFCoefficients(5.0);

        //int originalPosition = wobbleArm.getCurrentPosition();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.013, 0.0000, .00);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0.0005, 0.1);

        pidMovement = new PIDController(.012, 0.000, .0002);
        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        //initialize pidMovement
        pidMovement.setSetpoint(0);
        pidMovement.setOutputRange(0, power);
        pidMovement.setInputRange(0, 90);
        pidMovement.enable();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
/*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        // Open async and start streaming inside opened callback
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                pipeline = new StoneOrientationExample.StoneOrientationAnalysisPipeline();
                phoneCam.setPipeline(pipeline);
            }
        });
*/
        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);
        //resetAngle();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //trigger.setPosition(readyPosition);

    }
}
