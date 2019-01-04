package org.firstinspires.ftc.teamcode.RobotSystems;
import android.graphics.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationManager;

import java.util.List;
import android.hardware.Camera.Parameters;

import javax.net.ssl.HandshakeCompletedEvent;


public class vRecognation {
    public enum MineralPos{
        LEFT,
        CENTER,
        RIGHT;
    }

    public double lastGoldPos = -1;
    public Recognition lastRecognaition;
    public double topAndBottomAvg = -999;
    private OpMode opMode;
    private HardwareMap hardwareMap;
    /**
     * this parameters discribe our labels that we want to find
     */
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AW/F0cP/////AAABmUpR4+dbt0Negw2nqaCH9Cw2gV4ZxuUmpeJMm7XOTdQVumthQcOeoS9qktHy4EvXtMAFoh7n5KeMiLMDqtKvd1TrbYNUy3f9ST3TMkH2hFYKB6oJMJPB8oelL9Bst/2XJBz0ycMMcKmSsIwyOqwOuHamAlwfT+o7VusfYmY7FPvnXuhn8obCeB5x0hhjwjsBuOz2wnx1us6N5y6on0rdc1DOzC2gI767QLVXAvPyJvMfgtZRGcfzFk0evuSVxrIPCRQBjQYK2s5SsBLZ4sEO4HelibKK5kg0lgT9P1uHSNa8SWX+4wpJm76dFw1nSL7YElieByOTXxycmOdhWaae5qb1lvYYmDiBNFiwfHRDkBRD";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public vRecognation(HardwareMap hardwareMap,OpMode opMode){
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            opMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }



    public double getGoldPos() {
        if (((LinearOpMode) opMode).opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                double goldMineralX = -1;
                double width = -1;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        if (width < recognition.getWidth()){
                            width = recognition.getWidth();
                            lastRecognaition = recognition;
                            topAndBottomAvg = (recognition.getBottom() + recognition.getTop()) / 2;

                        }
                    }
                }

            }
        }
        return topAndBottomAvg;
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public void stopTfod(){
        tfod.shutdown();
    }

}
