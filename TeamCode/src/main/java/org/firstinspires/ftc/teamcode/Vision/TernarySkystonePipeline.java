package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;

public abstract class TernarySkystonePipeline extends OpenCvPipeline {

    public abstract void getStatus();


    public Mat lastInputImage;

    Integer imageNumber = 0;
    public void saveInputImage(String folderPath) {

        File directory = new File(folderPath);
        if(!directory.exists()) {
            directory.mkdir();
        }

        File writeLocation;
        do {
            ++imageNumber;
            writeLocation = new File(directory.getPath() + "/" + "capturedImage_" +
                    imageNumber.toString() + ".jpg");
        } while(writeLocation.exists());
        Imgcodecs.imwrite(writeLocation.getPath(), lastInputImage);
    }

    // Save using settings file folder.
    public void saveInputImage() {
        File writeLocation;
        do {
            ++imageNumber;
            String imageFilename = "capturedImage_" + imageNumber.toString() + ".jpg";
            writeLocation = AppUtil.getInstance().getSettingsFile(imageFilename);
        } while(writeLocation.exists());
        Imgcodecs.imwrite(writeLocation.getPath(), lastInputImage);
    }




    /*
    Functions for handling normalized values, scaling them, and converting them to openCv Rectangles
     */


    /**
     * A single value in the range [0.0,1.0], which is scaled to pixelSize of an image.
     */
    static public class NormalizedValue implements Cloneable{
        private double normalizedValue = 0.0;

        public NormalizedValue() {
        }

        public NormalizedValue(double normalizedValue) {
            // Throw an exception if input value is not between 0.0 and 1.0, inclusive.
            setNormalizedValue(normalizedValue);
        }

        void setNormalizedValue(double normalizedValue) {
            // Throw an exception if input value is not between 0.0 and 1.0, inclusive.
            if(normalizedValue < 0.0 || normalizedValue > 1.0) {
                throw new RuntimeException("NormalizedValue out of bounds: " + String.valueOf(normalizedValue));
            }
            this.normalizedValue = normalizedValue;
        }

        public double getNormalizedValue() {
            return normalizedValue;
        }

        public double getPixelScaledValue(int pixelSizeMax) {
            return normalizedValue * (double) pixelSizeMax;
        }

        public double getPixelScaledValue(Mat input) {
            int pixelSizeMax = Math.max(input.height(),input.width());
            return getPixelScaledValue(pixelSizeMax);
        }

        @Override
        public Object clone()
        {
            try {
                return super.clone();
            } catch (CloneNotSupportedException e) {
                e.printStackTrace();
            }
            return null;
        }
    }

    /**
     * A ordered pair of normalized values, suitable for use as XY
     */
    static public class NormalizedPair implements Cloneable{
        private NormalizedValue x = new NormalizedValue();
        private NormalizedValue y = new NormalizedValue();

        public NormalizedPair() {
        }

        public NormalizedPair(double x, double y) {
            this.x.setNormalizedValue(x);
            this.y.setNormalizedValue(y);
        }

        public NormalizedPair(NormalizedValue x, NormalizedValue y) {
            this.x.setNormalizedValue(x.getNormalizedValue());
            this.y.setNormalizedValue(y.getNormalizedValue());
        }

        public void setX(double x) {
            this.x.setNormalizedValue(x);
        }

        public void setY(double y) {
            this.y.setNormalizedValue(y);
        }

        public void setXY(double x, double y) {
            this.x.setNormalizedValue(x);
            this.y.setNormalizedValue(y);
        }

        public double getNormalizedX() {
            return this.x.getNormalizedValue();
        }

        public double getNormalizedY() {
            return this.y.getNormalizedValue();
        }

        public double getPixelScaledX(int pixelMax) {
            return this.x.getPixelScaledValue(pixelMax);
        }

        public double getPixelScaledX(Mat input) {
            int pixelMax = input.width();
            return this.x.getPixelScaledValue(pixelMax);
        }

        public double getPixelScaledY(int pixelMax) {
            return this.y.getPixelScaledValue(pixelMax);
        }

        public double getPixelScaledY(Mat input) {
            int pixelMax = input.height();
            return this.y.getPixelScaledValue(pixelMax);
        }

        public Point getOpenCvPoint(Mat input) {
            return new Point(getPixelScaledX(input), getPixelScaledY(input));
        }

        public Size getOpenCvSize(Mat input) {
            return new Size(getPixelScaledX(input), getPixelScaledY(input));
        }


        @Override
        public Object clone()
        {
            try {
                return super.clone();
            } catch (CloneNotSupportedException e) {
                e.printStackTrace();
            }
            return null;
        }
    }

    /**
     * Stores the center position and size of a rectangle as normalized values, and returns them
     * either pixel scaled, or as relevant openCV objects.
     */
    static public class NormalizedRectangle implements Cloneable{
        public NormalizedPair centerXY = new NormalizedPair(0.5,0.5);
        public NormalizedPair sizeXY = new NormalizedPair(0.0,0.0);

        public NormalizedRectangle() {}

        public NormalizedRectangle(double center_x_normalized, double center_y_normalized,
                                   double size_x_normalized, double size_y_normalized) {
            centerXY.setXY(center_x_normalized,center_y_normalized);
            sizeXY.setXY(size_x_normalized,size_y_normalized);
            //errorChecking();
        }

        public NormalizedRectangle(double[] center_xy_normalized, double[] size_xy_normalized) {
            this(center_xy_normalized[0], center_xy_normalized[1],size_xy_normalized[0],size_xy_normalized[1]);
        }

        private void errorChecking() {
            assert(centerXY.getNormalizedX() - sizeXY.getNormalizedX()/2.0 >= 0.0);
            assert(centerXY.getNormalizedX() + sizeXY.getNormalizedX()/2.0 <= 1.0);
            assert(centerXY.getNormalizedY() - sizeXY.getNormalizedY()/2.0 >= 0.0);
            assert(centerXY.getNormalizedY() + sizeXY.getNormalizedY()/2.0 <= 1.0);
        }

        public Rect getOpenCVRectangle(Mat input) {
            Rect leftRect = getCenteredRect(centerXY.getOpenCvPoint(input),sizeXY.getOpenCvSize(input));
            return leftRect;
        }

        public Rect getCenteredRect(Point point, Size size) {
            Point topLeftCorner = new Point(point.x - size.width/2, point.y - size.height/2);
            return new Rect(topLeftCorner,size);
        }


        @Override
        public Object clone()
        {
            try {
                return super.clone();
            } catch (CloneNotSupportedException e) {
                e.printStackTrace();
            }
            return null;
        }
    }


}
