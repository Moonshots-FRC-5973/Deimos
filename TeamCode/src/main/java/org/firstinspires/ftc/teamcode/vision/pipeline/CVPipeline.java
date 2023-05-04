package org.firstinspires.ftc.teamcode.vision.pipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CVPipeline extends OpenCvPipeline {
    private Telemetry telemetry;
    private int conKSize = 11;

    public CVPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat holdingMat1 = new Mat();
        Mat holdingMat2 = new Mat();
        // region Split color channels
        // Split the input image into two subimage channels
        // If we are looking for blue, check red channel,
        // since the cone red value is much lower than the floor's red value
        // and vice versa
        // coi: Column Index as according to the color mode, The input image's color mode is RGB
        Mat redMask = new Mat();
        Mat greenMask = new Mat();
        Mat yellowMask = new Mat();
        Mat blueMask = new Mat();
        Core.extractChannel(input, blueMask, 0);
        Core.extractChannel(input, greenMask, 1);
        Core.extractChannel(input, redMask, 2);
        // endregion

        // region Find yellow color

        Core.inRange(hsv, new Scalar(20, 30, 50), new Scalar(40, 255, 255), yellowMask);
        // endregion
        // region Use a convolution to blur the image
        Mat redCon = new Mat();
        Mat greenCon = new Mat();
        Mat blueCon = new Mat();
        Mat yellowCon = new Mat();

        Imgproc.medianBlur(redMask, redCon, conKSize);
        Imgproc.medianBlur(greenMask, greenCon, conKSize);
        Imgproc.medianBlur(blueMask, blueCon, conKSize);
        Imgproc.medianBlur(yellowMask, yellowCon, conKSize);
        // endregion
        // region Subtract out green channel noise from red and blue
        Mat redSub = new Mat();
        Mat blueSub = new Mat();

        Core.subtract(redCon, greenCon, redSub);
        Core.subtract(blueCon, greenCon, blueSub);
        // endregion
        // region Subtract out blue channel noise from yellow
        Mat yellowSub = new Mat();

        Core.subtract(yellowCon, blueCon, yellowSub);
        // endregion
        // region Remove areas which are both red and blue (purple areas)
        Mat redOutput = new Mat();
        Mat blueOutput = new Mat();
        Mat coloredNoiseMap = new Mat();
        Mat grayNoiseMap = new Mat();
        Mat rgbRed = new Mat();
        Mat rgbBlue = new Mat();
        // Separate grayscale into 3 channels
        Imgproc.cvtColor(redSub, rgbRed, Imgproc.COLOR_GRAY2RGB);
        Imgproc.cvtColor(blueSub, rgbBlue, Imgproc.COLOR_GRAY2RGB);
        // Create a noise map in purple
        Core.multiply(rgbRed, new Scalar(1, 0, 0), redOutput);
        Core.multiply(rgbBlue, new Scalar(0, 0, 1), blueOutput);
        Core.add(blueOutput, redOutput, coloredNoiseMap);

        Imgproc.cvtColor(coloredNoiseMap, grayNoiseMap, Imgproc.COLOR_RGB2GRAY);
        Core.subtract(redSub, grayNoiseMap, redCon);
        Core.subtract(blueSub, grayNoiseMap, blueCon);
        // endregion
        // region Find the image threshold areas
        Mat redThresh = new Mat();
        Mat blueThresh = new Mat();
        Imgproc.threshold(redCon, redThresh, 35, 255, 0);
        Imgproc.threshold(blueCon, blueThresh, 35, 255, 0);
        // endregion
        // region find contours
        Mat redLines = new Mat();
        Mat blueLines = new Mat();
        List<MatOfPoint> redContours = new ArrayList<>();
        List<MatOfPoint> blueContours = new ArrayList<>();
        Mat redHierarchy = new Mat();
        Mat blueHierarchy = new Mat();

        Imgproc.Canny(redThresh, redLines, 100, 300);
        Imgproc.Canny(blueThresh, blueLines, 100, 300);
        Imgproc.findContours(redLines, redContours, redHierarchy, 1, 1);
        Imgproc.findContours(blueLines, blueContours, blueHierarchy, 1, 1);

        if(redContours.size() != 0) {
            Rect biggest = Imgproc.boundingRect(redContours.get(0));

            for(int i = 1; i < redContours.size(); i++) {
                Rect compare = Imgproc.boundingRect(redContours.get(i));
                if(compare.height * compare.width >= biggest.height * biggest.width) {
                    biggest = compare;
                }
            }
        }


        // endregion
        // region draw an output image
        Mat output = new Mat();
        Mat lines = new Mat();
        Imgproc.cvtColor(redLines, rgbRed, Imgproc.COLOR_GRAY2RGB);
        Imgproc.cvtColor(blueLines, rgbBlue, Imgproc.COLOR_GRAY2RGB);
        Core.multiply(rgbRed, new Scalar(0, 0, 1), redOutput);
        Core.multiply(rgbBlue, new Scalar(1, 0, 0), blueOutput);
        Core.add(blueOutput, redOutput, lines);

        return lines;
        // endregion
    }
}
