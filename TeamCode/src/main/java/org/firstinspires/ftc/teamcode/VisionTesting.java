package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "Vision", group = "Philos")
public class VisionTesting extends Autonomous_Methods {

    private static final String TFOD_MODEL = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY = "AeA0/Gv/////AAABmTbgn67KbUjyiURMJ9kjKQqMrU0DOUtGplD8c3d6jHRHE/z1RKFrsBCdxk43yeJRSfi0HykDf+hQlKP+oLHJszg5f+0y07pylSt7QDzchxfv25Wrs7lNBnNZa9eHoihYLa+LDKVDfogyMloCpnUcC7hpZ+rSWHEWHsrL+IUpHTlr37CZNqrknOMHXp9QFhuBT60qi5I4i7XM4JsqBfgldhJ8GDPNdnOfFrsICxC+tj2lVRmfCJf/3y3+D+lFX4KTu8M3ojG51HDuSklTWAH7bN35iM+t+7KFSFgERPpyTAbQnRMhJPhtzXy7wPI3ULJd/mt5XPIiU9EIvMK1W6c3o99voem7ykXpw6iyddfMFRHv";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    private String objRecognized = " ";


    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        initVuforia();
        initTfod();

        if(tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">", "thing is working");
        telemetry.update();
        waitForStart();


        while(opModeIsActive()) {
            if(tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if(updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        objRecognized = recognition.getLabel();
                        if(objRecognized.equalsIgnoreCase("Single") || objRecognized.equalsIgnoreCase("quad"))
                            break;
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                    telemetry.update();
                }
            }
            if(objRecognized.length()>1 )
                break;

        }

        if(tfod != null) {
            tfod.shutdown();
        }

        telemetry.addData("Recognized Object", objRecognized);
        telemetry.update();
        sleep(2000);
        if(objRecognized.equalsIgnoreCase("Quad")) {

            telemetry.addData("Object Detected", objRecognized);
            telemetry.update();
            telemetry.addData("left", 1);
            strafeLeft(0.7, 15);
            telemetry.update();
            telemetry.addData("forward", 2);
            telemetry.update();
            forward(0.7, 37);
            sleep(1000); //drop wobble goal
            telemetry.addData("backward", 2);
            telemetry.update();
            backward(0.7, 25);
            sleep(1000); //shoot();
        }
        else if (objRecognized.equalsIgnoreCase("Single")) {
            telemetry.addData("Object Detected", objRecognized);
            telemetry.update();
            strafeLeft(0.7,15);
            forward(0.7,30);
            sleep(1000);//drop wobble goal
            backward(0.7,30);
            sleep(1000); //shoot();
        } else {
            strafeLeft(0.7, 15);
            forward(0.7, 10);
            sleep(1000);//drop wobble goal
            backward(0.7, 10 );
            sleep(1000);//shoot();
        }

    }

    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod() {

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.setZoom(2.5, 16.0/9);
        tfod.loadModelFromAsset(TFOD_MODEL, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

    }

}