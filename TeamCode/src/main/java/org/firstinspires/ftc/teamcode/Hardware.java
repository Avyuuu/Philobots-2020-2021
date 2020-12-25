package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Hardware {

    // Motor variable names
    public DcMotor front_left = null;
    public DcMotor front_right = null;
    public DcMotor back_left= null;
    public DcMotor back_right = null;
    public DcMotor shooter = null;
    public DcMotor intake = null;
    public DcMotor wobblegoal = null;


    // Other variable names
    HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();


    public Hardware() {
        hwMap = null;
    }


    public void init(HardwareMap hwMap) {

        // Save reference to Hardware map
        this.hwMap = hwMap;
        period.reset();

        // Define Motors
        front_left = hwMap.dcMotor.get("leftFront");
        front_right= hwMap.dcMotor.get("rightFront");
        back_left = hwMap.dcMotor.get("leftRear");
        back_right = hwMap.dcMotor.get("rightRear");
        shooter = hwMap.dcMotor.get("frontShooter");
        wobblegoal = hwMap.dcMotor.get("wobbleArm");
        intake = hwMap.dcMotor.get("intake");


        // Initialize Motors
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        shooter.setPower(0);
        intake.setPower(0);


    }


    public void initTeleOp(HardwareMap hwMap) {

        // Save reference to Hardware map
        this.hwMap = hwMap;
        period.reset();

        // Define Motors

        front_left = hwMap.dcMotor.get("leftFront");
        front_right= hwMap.dcMotor.get("rightFront");
        back_left = hwMap.dcMotor.get("leftRear");
        back_right = hwMap.dcMotor.get("rightRear");
        shooter = hwMap.dcMotor.get("frontShooter");
        wobblegoal = hwMap.dcMotor.get("wobbleArm");
        intake = hwMap.dcMotor.get("intake");

        // Initialize Motors
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        shooter.setPower(0);
        intake.setPower(0);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }

    public void initTeleOpNOIMU(HardwareMap hwMap) {

        // Save reference to Hardware map
        this.hwMap = hwMap;
        period.reset();

        // Define Motors
        front_left = hwMap.dcMotor.get("leftFront");
        front_right= hwMap.dcMotor.get("rightFront");
        back_left = hwMap.dcMotor.get("leftRear");
        back_right = hwMap.dcMotor.get("rightRear");
        shooter = hwMap.dcMotor.get("frontShooter");
        wobblegoal = hwMap.dcMotor.get("wobbleArm");
        intake = hwMap.dcMotor.get("intake");

        // Initialize Motors
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobblegoal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        wobblegoal.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        shooter.setPower(0);
        wobblegoal.setPower(0);
        intake.setPower(0);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



}
