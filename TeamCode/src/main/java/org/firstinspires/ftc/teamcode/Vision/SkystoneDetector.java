package org.firstinspires.ftc.teamcode.Vision;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.Utilities.Color;

import static org.firstinspires.ftc.teamcode.Vision.TernarySkystonePipeline.NormalizedValue;
import static org.firstinspires.ftc.teamcode.Vision.TernarySkystonePipeline.NormalizedRectangle;

import java.util.ArrayList;

/**
 * Initializes a pipeline and starts streaming.
 * Pipeline is focused on image frame, providing relative position information (left, center, right)
 * SkystoneDetector can use teamColor and starting location information to provide more absolute
 * location information for the skystone, such as location index, where 0 is near the field center.
 */
public class SkystoneDetector {

    RobotHardware opmode;
    HardwareMap hardwareMap;
    Color.Ftc teamColor;
    public OpenCvCamera webcam;
    public AveragingPipeline averagingPipeline;

    private final int rows = 640;
    private final int cols = 480;


    public SkystoneDetector(RobotHardware opmode, Color.Ftc teamColor) {
       this.teamColor = teamColor;
       this.opmode = opmode;
       this.hardwareMap = opmode.hardwareMap;
       switch(teamColor) {
           case BLUE:
           default:
               this.averagingPipeline = getAveragingPipelineForBlue();
               break;
           case RED:
               this.averagingPipeline = getAveragingPipelineForRed();
               break;
       }
//       this.averagingPipeline = new AveragingPipeline();
       init(this.averagingPipeline);
    }

    public void init(AveragingPipeline averagingPipeline) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(averagingPipeline);
        // If the resolution specified is not supported by the camera, an error will be thrown.
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);
    }


    static public AveragingPipeline getAveragingPipelineForBlue() {
        double normalizedImageLocationY = 0.53; // Vertical position, from top, [0.0,1.0]
        double[] stone_size_inches_xy = {5.0,3.0}; // Size of the block
        double distance_inches = 25.0;
        double leftStoneOffsetFromRobot_inches = 1.5;
        int[] stoneIndexRangeFromLeft_0To2 = {0,2};
       return getAveragingPipelineFromGeometry(normalizedImageLocationY,stone_size_inches_xy,
               distance_inches, leftStoneOffsetFromRobot_inches,stoneIndexRangeFromLeft_0To2);
    }

    // Index 0 left not included, so index 0 is center, index 1 is right.
    // If neither see the skystone, it must be assumed to be on the left,
    // which is position 0 for red side.
    static public AveragingPipeline getAveragingPipelineForRed() {
        double normalizedImageLocationY = 0.53; // Vertical position, from top, [0.0,1.0]
        double[] stone_size_inches_xy = {5.0,3.0}; // Size of the block
        double distance_inches = 25.0;
        double leftStoneOffsetFromRobot_inches = 7.5;
        int[] stoneIndexRangeFromLeft_0To2 = {0,2};
        return getAveragingPipelineFromGeometry(normalizedImageLocationY,stone_size_inches_xy,
                distance_inches, leftStoneOffsetFromRobot_inches,stoneIndexRangeFromLeft_0To2);
    }




    static public AveragingPipeline getAveragingPipelineFromGeometry(double normalizedImageLocationY,
                         double[] stone_size_inches_xy,  double distance_inches,
                         double leftStoneOffsetFromRobot_inches, int[] stoneIndexRangeFromLeft_0To2) {

//        double[] backgroundSize = {0.7,0.12};
        ArrayList<NormalizedRectangle> scanRegions = new ArrayList<>();

        // Geometry and imageScaling
//        double normalizedImageLocationY = 0.55; // Vertical position, from top, [0.0,1.0]
//        double distance_inches = 25.0;
        double[] normalizedSize = {0.10,0.08};
        double[] fov_xy_degrees = {78.0,78.0};
//        double[] stone_size_inches_xy = {7.5,3.5}; // Size of the block
        normalizedSize = getNormalizedSize(fov_xy_degrees,distance_inches,stone_size_inches_xy);
        double[][] imageLocationXY = new double[3][2];
        double[] relativePositionXY_inches = {10,0}; // Where x is forward,a y is left, from top down view
//        double leftStoneOffsetFromRobot_inches = 4.75;
        double cameraOffsetFromRobot_inches = -5.5;

        for(int i = stoneIndexRangeFromLeft_0To2[0]; i <= stoneIndexRangeFromLeft_0To2[1]; ++i) {
            relativePositionXY_inches[1] = (leftStoneOffsetFromRobot_inches  - 8 * (double) i) - cameraOffsetFromRobot_inches;
            relativePositionXY_inches[0] = distance_inches;
            imageLocationXY[i][0] = getNormalizedPositionX(fov_xy_degrees, relativePositionXY_inches);
            imageLocationXY[i][1] = normalizedImageLocationY;

            // Add current block as a scan region.
            try {
                scanRegions.add(new NormalizedRectangle(imageLocationXY[i], normalizedSize));
            } catch (Exception e) {
                System.out.println(e.getMessage());
                System.out.println("Failed on index: " + i);
                Log.e("SkystoneDetector",e.getMessage() + "  " + "Failed index: " + i);
            }
        }

//        NormalizedRectangle backgroundRegion = new NormalizedRectangle(0.5,normalizedImageLocationY,backgroundSize[0],backgroundSize[1]);
//        NormalizedValue lineThickness = new NormalizedValue(0.005);
//        NormalizedValue markerSize = new NormalizedValue(0.03);
//        return new AveragingPipeline(scanRegions,backgroundRegion,lineThickness,markerSize);
        return new AveragingPipeline(scanRegions);
    }

    public int getSkystoneIndex() {
        SkystoneRelativeLocation skystoneRelativeLocation = getSkystoneRelativeLocation();
        switch(teamColor) {
            case BLUE:
                switch (skystoneRelativeLocation) {
                    case LEFT:
                        return 0;
                    case CENTER:
                        return 1;
                    case RIGHT:
                        return 2;
                    default:
                        return 0;
                }
            case RED:
                switch (skystoneRelativeLocation) {
                    case LEFT:
                        return 2;
                    case CENTER:
                        return 1;
                    case RIGHT:
                        return 0;
                    default:
                        return 0;
                }
            default:
                return 0;
        }
    }

    public SkystoneRelativeLocation getSkystoneRelativeLocation() {
       return  getSkystoneRelativeLocation(averagingPipeline, teamColor);
    }

    // Static to make unit testing easier.
    static public SkystoneRelativeLocation getSkystoneRelativeLocation(AveragingPipeline averagingPipeline, Color.Ftc teamColor) {
        SkystoneRelativeLocation skystoneRelativeLocation = SkystoneRelativeLocation.UNKNOWN;
        Integer background_Cb = averagingPipeline.getBackground();
        ArrayList<Integer> averageDate = averagingPipeline.getData();
        int max_Cb_level = 0;
        int max_Cb_level_index = 0;
        boolean detections = false;
        int loopIndex = 0;
        for(Integer regionBlueAverage: averageDate) {
            if (regionBlueAverage >  background_Cb) {
                detections = true;
            }
            if (regionBlueAverage > max_Cb_level) {
                max_Cb_level = regionBlueAverage;
                max_Cb_level_index = loopIndex;
            }
            ++loopIndex;
        }

        switch(teamColor) {
            case BLUE:
                switch (max_Cb_level_index) {
                    case 0:
                        skystoneRelativeLocation = SkystoneRelativeLocation.LEFT;
                        break;
                    case 1:
                        skystoneRelativeLocation = SkystoneRelativeLocation.CENTER;
                        break;
                    case 2:
                        skystoneRelativeLocation = SkystoneRelativeLocation.RIGHT;
                        break;
                    default:
                        skystoneRelativeLocation = SkystoneRelativeLocation.UNKNOWN;
                        break;
                }
                break;
            case RED:
                if(detections) {
                    switch(max_Cb_level_index) {
                        case 0:
                            skystoneRelativeLocation = SkystoneRelativeLocation.LEFT;
                            break;
                        case 1:
                            skystoneRelativeLocation = SkystoneRelativeLocation.CENTER;
                            break;
                        case 2:
                            skystoneRelativeLocation = SkystoneRelativeLocation.RIGHT;
                            break;
                        default:
                            throw new RuntimeException("Should have a minimum if there is a detection!");
                    }
                } else {
                    // No detections, must be left
                    skystoneRelativeLocation = SkystoneRelativeLocation.LEFT;
                }
                break;
            default:
                skystoneRelativeLocation = SkystoneRelativeLocation.UNKNOWN;
        }

        return skystoneRelativeLocation;
    }

    /*
    *****  Geometric Calculations *****
     */

    /**
     * Returns the normalized xy size of the stone scan region, from [0,1] as a fraction of the image size,
     * based on the camera FOV, the distance to the stone, and the size of the stone.
     * @param fov_degrees_xy Field of view of the camera in degrees.
     * @param distance Distance from camera. Units must be the same as sizeXY.
     * @param sizeXY Size of block. Units must be the same as distance.
     * @return normalizedSizeXY array, both values scaled from [0,1].
     */
    static public double[] getNormalizedSize(double[] fov_degrees_xy, double distance, double [] sizeXY) {
        double[] normalizedSizeXY = {0,0};
        for(int i=0; i<=1; ++i) {
            normalizedSizeXY[i] = 2.0 / fov_degrees_xy[i] * 180/Math.PI * Math.atan(sizeXY[i] / (2.0 * distance));
        }
        return normalizedSizeXY;
    }

    /**
     * Returns normalizedPositionXY array, with values [0,1], indicating the position of the center
     * of the blocks on the camera image, where 0 refers to the minimum pixel location, and 1 refers
     * to the maximum pixel location.  {.5,.5} would reference the center of the image.
     * @param fov_degrees_xy Field of view of the camera in degrees.
     * @param relativePositionXY Block position relative to camera, X out of camera, Y to left. Units irrelevant.
     * @return normalizedPositionXY array, [0,1] position on image, in camera viewport. {0.5,0.5} is center of image.
     */
    static public double getNormalizedPositionX(double[] fov_degrees_xy, double[] relativePositionXY) {
        double normalizedPositionX = 0.5 - Math.atan2(relativePositionXY[1],relativePositionXY[0]) * (180 / Math.PI) /fov_degrees_xy[0];
        return normalizedPositionX;
    }




}


