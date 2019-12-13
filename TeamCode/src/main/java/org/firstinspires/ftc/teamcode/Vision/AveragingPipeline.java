package org.firstinspires.ftc.teamcode.Vision;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;

/*
 * An image processing pipeline to be run upon receipt of each frame from the camera.
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
    public ArrayList<NormalizedRectangle> scanRegions = new ArrayList<>();
    NormalizedRectangle backgroundRegion = new NormalizedRectangle();
    public NormalizedValue lineThickness = new NormalizedValue(0.01);
    public NormalizedValue markerSize = new NormalizedValue(0.03);


    public AveragingPipeline() {
        double yPosition = 0.55;
        double[] normalizedSize = {0.10,0.08};
        double[] backgroundSize = {0.7,0.12};
        scanRegions.add(new NormalizedRectangle(0.25,yPosition,normalizedSize[0],normalizedSize[1]));
        scanRegions.add(new NormalizedRectangle(0.5,yPosition,normalizedSize[0],normalizedSize[1]));
        scanRegions.add(new NormalizedRectangle(0.75,yPosition,normalizedSize[0],normalizedSize[1]));
        backgroundRegion = new NormalizedRectangle(0.5,yPosition,backgroundSize[0],backgroundSize[1]);
        this.lineThickness = new NormalizedValue(0.005);
        this.markerSize = new NormalizedValue(0.03);
    }

    public AveragingPipeline(ArrayList<NormalizedRectangle> scanRegions, NormalizedRectangle backgroundRegion, NormalizedValue lineThickness, NormalizedValue markerSize) {
        // Warning: these are copied by reference
        this.scanRegions = (ArrayList) scanRegions.clone();
        this.backgroundRegion = (NormalizedRectangle) backgroundRegion.clone();
        this.lineThickness = (NormalizedValue) lineThickness.clone();
        this.markerSize = (NormalizedValue) markerSize.clone();
    }

    // Infer background size from normalized regions.
    public AveragingPipeline(ArrayList<NormalizedRectangle> scanRegions) {
        // Warning: these are copied by reference
        this.scanRegions = (ArrayList<NormalizedRectangle>) scanRegions.clone();
        // Default values for line and marker sizes.
        this.lineThickness = new NormalizedValue(0.005);
        this.markerSize = new NormalizedValue(0.03);

        // Calculate a background rectangle which will overlap the sample regions.
        Double[] minXY = {1.0,1.0};
        Double[] maxXY = {0.0,0.0};
        for(NormalizedRectangle scanRegion: scanRegions) {
            minXY[0] = Math.min(minXY[0],scanRegion.centerXY.getNormalizedX()-scanRegion.sizeXY.getNormalizedX()/2.0);
            maxXY[0] = Math.max(maxXY[0],scanRegion.centerXY.getNormalizedX()+scanRegion.sizeXY.getNormalizedX()/2.0);
            minXY[1] = Math.min(minXY[1],scanRegion.centerXY.getNormalizedY()-scanRegion.sizeXY.getNormalizedY()/2.0);
            maxXY[1] = Math.max(maxXY[1],scanRegion.centerXY.getNormalizedY()+scanRegion.sizeXY.getNormalizedY()/2.0);
        }
        // Calculate average x,y center position:
        double[] averageCenterXY = {(maxXY[0] + minXY[0]) / 2.0, (maxXY[1] + minXY[1]) / 2.0};
        double[] sizeXY = {maxXY[0] - minXY[0],maxXY[1] - minXY[1]};
        this.backgroundRegion = new NormalizedRectangle(averageCenterXY[0], averageCenterXY[1], sizeXY[0], sizeXY[1]);
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
    private Mat subMat;

    private int min;
    private int minIndex;
    private ArrayList<Integer> avgArray = new ArrayList<>();
    private int backgroundAvg = 0;



    public void getStatus() {
        System.out.print("Scans: ");
        for(Integer thisAvg : avgArray) {
            System.out.print(thisAvg.toString() + ", ");
        }
        System.out.println("Background: " + backgroundAvg);
        //return new ArrayList<>(avg1,avg2,avg3,avgBackground);
    }

    public ArrayList<Integer> getData() {
        ArrayList<Integer> tempArray = (ArrayList)avgArray.clone();
        return tempArray;
    }

    public Integer getBackground() {
        return backgroundAvg;
    }

    public ArrayList<Integer> getAllData() {
        ArrayList<Integer> tempArray = (ArrayList)avgArray.clone();
        tempArray.add(backgroundAvg);
        return tempArray;
    }


    public int getMinIndex() {
        return minIndex;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        lastInputImage = input.clone();


        // Convert the image from RGB to YCrCb
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Extract the Cb channel from the image
        Core.extractChannel(YCrCb, Cb, 2);

        // Colors for drawing regions
        Scalar scanRectangleColor = new Scalar(255, 0, 0);
        Scalar backgroundRectangleColor = new Scalar(0, 0, 255);

        ArrayList<Integer> tempAvgArray = new ArrayList<>();
        for(NormalizedRectangle normalizedRectangle: scanRegions) {
            // Select Sample Area
            subMat = Cb.submat(normalizedRectangle.getOpenCVRectangle(input));
            // Average the sample areas and store in array
            tempAvgArray.add((int)Core.mean(subMat).val[0]);
            // Draw rectangles around the sample areas
            Imgproc.rectangle(input, normalizedRectangle.getOpenCVRectangle(input), scanRectangleColor, (int) Math.ceil(lineThickness.getPixelScaledValue(input)));
        }
        avgArray = tempAvgArray;

        // Repeat above steps for the background area
        subMat = Cb.submat(backgroundRegion.getOpenCVRectangle(input));
        backgroundAvg = (int)Core.mean(subMat).val[0];
        Imgproc.rectangle(input, backgroundRegion.getOpenCVRectangle(input), backgroundRectangleColor, (int) Math.ceil(lineThickness.getPixelScaledValue(input)/3.0));

        // Figure out which sample zone had the lowest contrast from blue (lightest color)
        min = Collections.min(avgArray);

        minIndex = 0;
        for(Integer average: avgArray) {
            if(min == average) {
                break;
            } else {
                ++minIndex;
            }
        }

        // Draw a circle on the detected skystone
        Scalar markerColor = new Scalar(255,52,235);

        boolean singleRegion = false;
        if(singleRegion) {
            Imgproc.circle(input, scanRegions.get(minIndex).centerXY.getOpenCvPoint(input),
                    (int) Math.ceil(markerSize.getPixelScaledValue(input)), markerColor, -1);
        } else {
           for(int i = 0; i < avgArray.size(); ++i)  {
              if (avgArray.get(i) > backgroundAvg) {
                  Imgproc.circle(input, scanRegions.get(i).centerXY.getOpenCvPoint(input),
                          (int) Math.ceil(markerSize.getPixelScaledValue(input)), markerColor, -1);
              }
           }
        }


        // Free the allocated submat memory
        subMat.release();
        subMat = null;

        return input;
    }

}