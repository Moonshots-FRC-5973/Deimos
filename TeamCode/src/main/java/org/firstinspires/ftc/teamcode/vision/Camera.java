package org.firstinspires.ftc.teamcode.vision;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

public class Camera {
    private static final String MODEL_ASSET = "signal.tflite";
    private static final String MODEL_FILE  = "/sdcard/FIRST/tflitemodels/signal.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    public static final String VUFORIA_KEY = "AS6a0rf/////AAABmQTWiptCrE+LufdlUzBT8weOkDKTan22xYq7kRmbpkAd2B1wy+uBaVuTdp4ngclG4NG6WQ8+8+nnRd+v6OB5Gzm+jMMh02iC+WrML6/2ArgWlM1vh43nEyfKOaOyJ4uZYqKMNAEcXqNLKK2+PdtmQQgiwGhna/VKV/Qdkhwsxt6w+4VGETJJwxT8k+tXTal2DGF5Sr9c69Lz0O0drCDZ2+ZUtuhOn1X+dkVoGxAoqSh/sYiqssxEtfGaf551TQytAXNBpbMgYXNGRSR6WAke2lVC4BxEowhiacPiZDLOZgVrHPc0bJbtN2kIF3OWk/FHj3tuHQ6seHZR4cU/6S7AeP3PaBwnYKbFvg8svUAy3vxD";

    private VuforiaLocalizer vuforia;
    private Telemetry telemetry;
    private TFObjectDetector tfod;


    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(
                hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                )
        );

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        try {
            tfod.loadModelFromAsset(MODEL_ASSET, LABELS);
        } catch(Exception e) {
            tfod.loadModelFromFile(MODEL_FILE, LABELS);
        }
    }

    public final List<Recognition> getRecognitions() {
        List<Recognition> rList = tfod.getUpdatedRecognitions();
        if(rList == null) {
            rList = Collections.emptyList();
        }
        return rList;
    }
}
