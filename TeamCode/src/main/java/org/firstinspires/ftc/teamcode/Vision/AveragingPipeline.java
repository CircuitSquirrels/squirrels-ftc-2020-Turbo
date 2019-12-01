package org.firstinspires.ftc.teamcode.Vision;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/*
 * An example image processing pipeline to be run upon receipt of each frame from the camera.
 * Note that the processFrame() method is called serially from the frame worker thread -
 * that is, a new camera frame will not come in while you're still processing a previous one.
 * In other words, the processFrame() method will never be called multiple times simultaneously.
 *
 * However, the rendering of your processed image to the viewport is done in parallel to the
 * frame worker thread. That is, the amount of time it takes to render the image to the
 * viewport does NOT impact the amount of frames per second that your pipeline can process.
 *
 * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
 * frame worker thread. This should not be a problem in the vast majority of cases. However,
 * if you're doing something weird where you do need it synchronized with your OpMode thread,
 * then you will need to account for that accordingly.
 */

public class AveragingPipeline extends TernarySkystonePipeline
{

    // Relative Sampling locations. Values normalized to image size, from [0,1].
    private SampleLocationsNormalized normalizedLocations;

    public SampleLocationsNormalized getNormalizedLocations() {
        return normalizedLocations;
    }

    public void setNormalizedLocations(SampleLocationsNormalized sampleLocationsNormalized) {
        this.normalizedLocations = sampleLocationsNormalized;
    }

    public AveragingPipeline() {
        double height = 0.55;
        SampleLocationsNormalized sampleLocationsNormalized =
                new SampleLocationsNormalized();
        sampleLocationsNormalized.leftPosition = new Point(0.25,height);
        sampleLocationsNormalized.centerPosition = new Point(0.5,height);
        sampleLocationsNormalized.rightPosition = new Point(0.75,height);
        sampleLocationsNormalized.blockSize = new Point(0.12, 0.12);
        sampleLocationsNormalized.backgroundSize = new Point(0.7, 0.12);
        sampleLocationsNormalized.lineThickness = 0.01;
        sampleLocationsNormalized.markerSize = 0.03;
        this.normalizedLocations = sampleLocationsNormalized;

    }

    public AveragingPipeline(SampleLocationsNormalized sampleLocationsNormalized) {
        this.normalizedLocations = sampleLocationsNormalized;
    }

    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    // CV intermediate products
    private Mat YCrCb = new Mat();
    private Mat Cb = new Mat();
    private Mat subMat1;
    private Mat subMat2;
    private Mat subMat3;
    private Mat subMatBackground;
    private int min;
    private int avg1;
    private int avg2;
    private int avg3;
    private int avgBackground;

    public void getStatus() {
        System.out.println(avg1 + " , " + avg2 + " , " + avg3 + " , " + avgBackground);
        //return new ArrayList<>(avg1,avg2,avg3,avgBackground);
    }

    // Sampling pixel locations
    private SampleLocationsPx sampleLocationsPx = new SampleLocationsPx();

    // Output
    private SkystoneRelativeLocation skystoneRelativeLocation = SkystoneRelativeLocation.UNKNOWN;
    private int position = 0; // output position
    public int getPosition() {
        return position;
    }



    @Override
    public Mat processFrame(Mat input)
    {
        // Ensure that pixel locations are scaled to the current input image resolution.
        sampleLocationsPx.scaleSamplingPixelsToImageSizeAndNormalizedLocations(input,normalizedLocations);


        // Convert the image from RGB to YCrCb
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Extract the Cb channel from the image
        Core.extractChannel(YCrCb, Cb, 2);

        // The the sample areas from the Cb channel
        subMat1 = Cb.submat(sampleLocationsPx.leftRect);
        subMat2 = Cb.submat(sampleLocationsPx.centerRect);
        subMat3 = Cb.submat(sampleLocationsPx.rightRect);
        subMatBackground = Cb.submat(sampleLocationsPx.backgroundRect);

        // Average the sample areas
        avg1 = (int)Core.mean(subMat1).val[0];
        avg2 = (int)Core.mean(subMat2).val[0];
        avg3 = (int)Core.mean(subMat3).val[0];
        avgBackground = (int)Core.mean(subMatBackground).val[0];

        // Draw rectangles around the sample areas
        Scalar rectangleColor = new Scalar(0, 0, 255);
        Scalar backgroundColor = new Scalar(255, 0, 0);
        Imgproc.rectangle(input, sampleLocationsPx.leftRect, rectangleColor, sampleLocationsPx.lineThickness);
        Imgproc.rectangle(input, sampleLocationsPx.centerRect, rectangleColor, sampleLocationsPx.lineThickness);
        Imgproc.rectangle(input, sampleLocationsPx.rightRect, rectangleColor, sampleLocationsPx.lineThickness);
        Imgproc.rectangle(input, sampleLocationsPx.backgroundRect, backgroundColor, sampleLocationsPx.lineThickness/3);

        // Figure out which sample zone had the lowest contrast from blue (lightest color)
        min = Math.min(avg1, Math.min(avg2, avg3));

        // Draw a circle on the detected skystone
        Scalar markerColor = new Scalar(255,52,235);
        if(min == avg1) {
            sampleLocationsPx.skystone = sampleLocationsPx.leftPosition;
            position = 1;
        } else if(min == avg2) {
            sampleLocationsPx.skystone = sampleLocationsPx.centerPosition;
            position = 2;
        } else if(min == avg3) {
            sampleLocationsPx.skystone = sampleLocationsPx.rightPosition;
            position = 3;
        } else {
            position = 1;
        }
        Imgproc.circle(input, sampleLocationsPx.skystone, sampleLocationsPx.markerSize, markerColor, -1);

        // Free the allocated submat memory
        subMat1.release();
        subMat1 = null;
        subMat2.release();
        subMat2 = null;
        subMat3.release();
        subMat3 = null;
        subMatBackground.release();
        subMatBackground = null;

        return input;
    }

    @Override
    public SkystoneRelativeLocation getSkystoneRelativeLocation() {
        // TODO: Ensure the enum locations are correct, based on the positions.
        switch (position) {
            case 0:
                skystoneRelativeLocation = SkystoneRelativeLocation.UNKNOWN;
                break;
            case 1:
                skystoneRelativeLocation = SkystoneRelativeLocation.LEFT;
                break;
            case 2:
            default:
                skystoneRelativeLocation = SkystoneRelativeLocation.CENTER;
                break;
            case 3:
                skystoneRelativeLocation = SkystoneRelativeLocation.RIGHT;
                break;
        }

        return skystoneRelativeLocation;
    }
}