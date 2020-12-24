package org.firstinspires.ftc.teamcode.Library;

import android.sax.StartElementListener;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class VisionUtility
{
    public VuforiaLocalizer vuforia;

    VuforiaTrackables targetsUltimateGoal;

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_QUAD_ELEMENT = "Quad";
    private static final String LABEL_SINGLE_ELEMENT = "Single";

    public TFObjectDetector tfod;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor


    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    OpenGLMatrix camLocationOnRobot;

    HardwareMap hardwareMap;
    LinearOpMode opMode;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();


    public VisionUtility(HardwareMap h, LinearOpMode op)
    {
        hardwareMap = h;
        opMode = op;
    }

    public void init()
    {

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AeA0/Gv/////AAABmTbgn67KbUjyiURMJ9kjKQqMrU0DOUtGplD8c3d6jHRHE/z1RKFrsBCdxk43yeJRSfi0HykDf+hQlKP+oLHJszg5f+0y07pylSt7QDzchxfv25Wrs7lNBnNZa9eHoihYLa+LDKVDfogyMloCpnUcC7hpZ+rSWHEWHsrL+IUpHTlr37CZNqrknOMHXp9QFhuBT60qi5I4i7XM4JsqBfgldhJ8GDPNdnOfFrsICxC+tj2lVRmfCJf/3y3+D+lFX4KTu8M3ojG51HDuSklTWAH7bN35iM+t+7KFSFgERPpyTAbQnRMhJPhtzXy7wPI3ULJd/mt5XPIiU9EIvMK1W6c3o99voem7ykXpw6iyddfMFRHv";

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsUltimateGoal);


        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }
        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        //tensorflow
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }
    }

    private void initTfod() {

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.setZoom(2.5, 16.0/9);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD_ELEMENT, LABEL_SINGLE_ELEMENT);
    }

    public OpenGLMatrix getRobotLocation()
    {
        /**  Let all the trackable listeners know where the phone is.  */

        targetsUltimateGoal.activate();

        if (opMode.opModeIsActive()) {

            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    opMode.telemetry.addData("Visible Target", trackable.getName());
                    opMode.telemetry.update();
                    targetVisible = true;
                    Log.i("[Philos]:target_vis:", "true");

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();

                    if (robotLocationTransform != null) {
                        opMode.telemetry.addData("Philos XY: ", robotLocationTransform.getColumn(3).get(0) / 25.4f + " " + robotLocationTransform.getColumn(3).get(1) / 25.4f);
                        Log.i("[Philos XY:]", robotLocationTransform.getColumn(3).get(0) / 25.4f + "  " + robotLocationTransform.getColumn(3).get(1) / 25.4f);
                        return robotLocationTransform;
                    }
                }
            }

            opMode.telemetry.update();
        }
        return null;
    }

    public int getRingStack(OpMode opMode) {
        Log.i("[Philos]:", String.format("Tfod null:%b", tfod == null));
        if (tfod != null) {
            List<Recognition> recognitions = tfod.getUpdatedRecognitions();

            Log.i("[Philos]:", String.format("Recognitions null:%b", recognitions == null));
            if (recognitions != null) {
                int i = 0;
                Log.i("[Philos]:", String.format("Recognitions length %d", recognitions.size()));
                for (Recognition recognition : recognitions) {
                        opMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        opMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        opMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                        opMode.telemetry.update();
                        String objRecognized = recognition.getLabel();
                        if(objRecognized.equalsIgnoreCase(LABEL_SINGLE_ELEMENT)) {
                            return 1;
                        }
                        else if(objRecognized.equalsIgnoreCase(LABEL_QUAD_ELEMENT)) {
                            return 4;
                        }
                }
                if (recognitions.size() != 0)
                    opMode.telemetry.addData("[Philos]:","Unrecognized label");
                else
                    opMode.telemetry.addData("[Philos]:","no label detected");
                return 0;
            }

            Log.i("[Philos]:", "no recognition");
            opMode.telemetry.addData("[Philos]:","no recognition object");
            return 0;
        }

        Log.i("[Philos]:", "no tfod");
        opMode.telemetry.addData("[philos]:","no tfod");
        return 0;
    }

    public void tfodDeactivate() {

        if (tfod != null) {
            tfod.shutdown();
        }
    }
}