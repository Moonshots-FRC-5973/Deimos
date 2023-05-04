package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.firstinspires.ftc.teamcode.vision.pipeline.CVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;

import java.util.ArrayList;
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

    private Telemetry telemetry;
    private TFObjectDetector tfod;
    private OpenCvWebcam webcam;

    private CVPipeline pipeline;

    public CVPipeline getPipeline() {
        return pipeline;
    }

    public int getFrameCount() {
        return webcam.getFrameCount();
    }

    public float getFps() {
        return webcam.getFps();
    }

    public float getTotalFrameTime() {
        return webcam.getTotalFrameTimeMs();
    }

    public float getPipelineTime() {
        return webcam.getPipelineTimeMs();
    }

    public float getOverheadTime() {
        return webcam.getOverheadTimeMs();
    }

    public float getCurrentPipelineMaxFps() {
        return webcam.getCurrentPipelineMaxFps();
    }

    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                )
        );
        pipeline = new CVPipeline(telemetry);

        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.setPipeline(pipeline);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addData("Camera", String.format("Failed to open webcam with code %x", errorCode));
            }
        });

    }
}
