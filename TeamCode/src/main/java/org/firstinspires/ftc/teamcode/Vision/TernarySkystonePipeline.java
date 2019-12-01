package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvPipeline;

public abstract class TernarySkystonePipeline extends OpenCvPipeline {

    public abstract SkystoneRelativeLocation getSkystoneRelativeLocation();

    public abstract void getStatus();

    /**
     * Data Class for storing the normalized sizes and locations
     * of regions to scan for the skystone.
     * Measurements are normalized to the height and width of the input image.
      */
    public static class SampleLocationsNormalized {

        // Center locations of 3 sample regions
        public Point leftPosition;
        public Point centerPosition;
        public Point rightPosition;
        // Sample region size
        public Point blockSize;
        public Point backgroundSize;
        public double lineThickness = -1;
        public double markerSize = -1;

        public boolean pixelScaleApplied = false;

        SampleLocationsNormalized() {

        }


        SampleLocationsNormalized(Point leftPosition, Point centerPosition, Point rightPosition,
                                  Point blockSize, Point backgroundSize,
                                  double lineThickness, double markerSize) {
            this.leftPosition = leftPosition;
            this.centerPosition = centerPosition;
            this.rightPosition = rightPosition;
            this.blockSize = blockSize;
            this.backgroundSize = backgroundSize;
            this.lineThickness = lineThickness;
            this.markerSize = markerSize;
        }

        /**
         * Validate all data values.
         * None should be null, all should be [0,1],
         * and half the sizes plus any position should not be outside [0,1]
          * @return
         */
        public boolean isValid() {
            boolean notNull;
            boolean withinRange;
            boolean compoundRange;

            // Not null
            notNull =
                    (leftPosition != null) &&
                    (centerPosition != null) &&
                    (rightPosition != null) &&
                    (blockSize != null) &&
                    (backgroundSize != null) &&
                    (lineThickness > 0) &&
                    (markerSize > 0);

            // Verify range of values, including compound ranges.
            withinRange = (leftPosition.x >= 0) && (leftPosition.x <= 1) && (leftPosition.y >= 0) && (leftPosition.y <=1) &&
            (rightPosition.x >= 0) && (rightPosition.x <= 1) && (rightPosition.y >= 0) && (rightPosition.y <=1) &&
            (blockSize.x >= 0) && (blockSize.x <= 1) && (blockSize.y >= 0) && (blockSize.y <=1) &&
            (backgroundSize.x >= 0) && (backgroundSize.x <= 1) && (backgroundSize.y >= 0) && (backgroundSize.y <=1) &&
            lineThickness >= 0 && lineThickness <=1 &&
            markerSize >= 0 && markerSize <=1;


            // Compound sizes
            double minX = Math.min(Math.min(leftPosition.x,centerPosition.x),rightPosition.x);
            double minY = Math.min(Math.min(leftPosition.y,centerPosition.y),rightPosition.y);
            double maxX = Math.max(Math.max(leftPosition.x,centerPosition.x),rightPosition.x);
            double maxY = Math.max(Math.max(leftPosition.y,centerPosition.y),rightPosition.y);
            double maxOffsetX = Math.max(blockSize.x,backgroundSize.x);
            double maxOffsetY = Math.max(blockSize.y,backgroundSize.y);

            compoundRange =
                ((minX - blockSize.x/2) >= 0) &&
                ((maxX + blockSize.x/2) <= 1) &&
                ((minY - blockSize.y/2) >= 0) &&
                ((maxY + blockSize.y/2) <= 1) &&
                ((centerPosition.x - backgroundSize.x/2) >= 0) &&
                ((centerPosition.x + backgroundSize.x/2) <= 1) &&
                ((centerPosition.y - backgroundSize.y/2) >= 0) &&
                ((centerPosition.y + backgroundSize.y/2) <= 1);

            return notNull && withinRange && compoundRange;
        }
    }

    /**
     * Pixel based locations of sample regions
     */
    class SampleLocationsPx {

        private Size imageSizeScaling = new Size(0,0); // Not scaled by default.
        private boolean pixelsScaled = false;

        // New System
        Point leftPosition;
        Point centerPosition;
        Point rightPosition;
        Size blockSize;
        Size backgroundSize;

        // Rectangles for regions of interest
        Rect leftRect;
        Rect centerRect;
        Rect rightRect;
        Rect backgroundRect;

        public Point skystone = new Point();

        public int lineThickness = 1;
        public int markerSize = 10;


        SampleLocationsPx() {
        }

        SampleLocationsPx(Mat input,SampleLocationsNormalized sampleLocationsNormalized) {
            scaleSamplingPixelsToImageSizeAndNormalizedLocations(input,sampleLocationsNormalized);
        }



        public void scaleSamplingPixelsToImageSizeAndNormalizedLocations(Mat input, SampleLocationsNormalized sampleLocationsNormalized) {
            Size currentSize = new Size(input.width(),input.height());
            if(currentSize.equals(imageSizeScaling) && pixelsScaled && sampleLocationsNormalized.pixelScaleApplied) return;
            // else, resize as needed.
            assert(sampleLocationsNormalized.isValid());

            int imageWidth = input.width();
            int imageHeight = input.height();
            int maxLength = Math.max(imageHeight,imageWidth);
            leftPosition = new Point(imageWidth * sampleLocationsNormalized.leftPosition.x, imageHeight * sampleLocationsNormalized.leftPosition.y);
            centerPosition = new Point(imageWidth * sampleLocationsNormalized.centerPosition.x, imageHeight * sampleLocationsNormalized.centerPosition.y);
            rightPosition = new Point(imageWidth * sampleLocationsNormalized.rightPosition.x, imageHeight * sampleLocationsNormalized.rightPosition.y);
            blockSize = new Size(imageWidth * sampleLocationsNormalized.blockSize.x, imageHeight * sampleLocationsNormalized.blockSize.y);
            backgroundSize = new Size(imageWidth * sampleLocationsNormalized.backgroundSize.x, imageHeight * sampleLocationsNormalized.backgroundSize.y);

            leftRect = getCenteredRect(leftPosition,blockSize);
            centerRect = getCenteredRect(centerPosition,blockSize);
            rightRect = getCenteredRect(rightPosition,blockSize);
            backgroundRect = getCenteredRect(centerPosition,backgroundSize);

            this.lineThickness = (int) Math.ceil(maxLength * sampleLocationsNormalized.lineThickness);
            this.markerSize = (int) Math.ceil(maxLength * sampleLocationsNormalized.markerSize);

            pixelsScaled = true;
            imageSizeScaling = new Size(input.width(), input.height());
            sampleLocationsNormalized.pixelScaleApplied = true;
        }

    }

    public Rect getCenteredRect(Point point, Size size) {
        Point topLeftCorner = new Point(point.x - size.width/2, point.y - size.height/2);
        return new Rect(topLeftCorner,size);
    }


}
