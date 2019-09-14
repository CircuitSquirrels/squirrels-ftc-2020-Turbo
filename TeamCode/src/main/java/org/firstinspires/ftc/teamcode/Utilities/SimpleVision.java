package org.firstinspires.ftc.teamcode.Utilities;

import android.graphics.Bitmap;
import android.webkit.WebChromeClient;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Reads Vuforia markers off the camera.
 * Requires the Vuforia key is set in SharedCode/src/main/res/values/vuforia.xml.
 * Example:
 *   private SimpleVision vuforia;
 *
 *   public void init() {
 *       String vuforiaKey = "...";
 *       vuforia = new SimpleVision(vuforiaKey);
 *   }
 *
 *   public void loop() {
 *       RelicRecoveryVuMark vuMark = vuforia.detectMark();
 *       if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
 *           // ... use detected mark.
 *       }
 *   }
 *
 * Thanks to Phillip Tischler  http://pmtischler-ftc-app.readthedocs.io/en/latest/
 */
public class SimpleVision {

    // opMode reference
    private RobotHardware opMode;
    private ElapsedTime tfTimer;
    public static final String TAG = "simpleVision class";

    // The external Vuforia ID localizer.
    private VuforiaLocalizer vuforia;
    private boolean useBackCamera;

    // Vuforia Trackables
    private boolean useVuforiaTrackables;
    private VuforiaTrackables targetsRoverRuckus;
    private List<VuforiaTrackable> allTrackables;
    private VuforiaTrackable blueRover;
    private VuforiaTrackable redFootprint;
    private VuforiaTrackable frontCraters;
    private VuforiaTrackable backSpace;



    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    // Vuforia Nav Geometry Properties
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor


    // TensorFlow Properties
    public boolean tfSupported = true;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private TFObjectDetector tfod;
    public List<Recognition> updatedRecognitions;
    public List<Recognition> pastRecognitions;

    // Webcam Support
    private boolean useWebcam;
    int captureCounter = 0;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    WebcamName webcamName;


    public static enum GoldMineralPosition {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN,
    }

    public static enum MineralIdentificationLocation {
        CENTER,
        BOTTOM,
    }

    public GoldMineralPosition goldMineralPosition = GoldMineralPosition.UNKNOWN;


    /**
     * Creates a Vuforia localizer and starts localization.
     * @param vuforiaLicenseKey The license key to access Vuforia code.
     */
    public SimpleVision(String vuforiaLicenseKey, RobotHardware opMode, boolean useVuforiaMonitor,
                        boolean useTensorFlowMonitor, boolean useBackCamera, boolean useVuforiaTrackables,
                        boolean useWebcam) {
        this.opMode = opMode;
        this.useBackCamera = useBackCamera;
        this.useWebcam = useWebcam;

        this.useVuforiaTrackables = useVuforiaTrackables;
        VuforiaLocalizer.Parameters parameters;
        if (useVuforiaMonitor) {
            // Show camera
            int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else { // Don't use camera monitor
            parameters = new VuforiaLocalizer.Parameters();
        }
        parameters.vuforiaLicenseKey = vuforiaLicenseKey;
         if (useWebcam){ // useWebcam
             try {
                 webcamName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
                 parameters.cameraName = webcamName;
             } catch (Exception e) {
                 useWebcam = false; this.useWebcam = false; // Default to not using webcam.
                 opMode.telemetry.addData("Webcam"," not detected");
             }
        }

        if (!useWebcam) {
            if (useBackCamera) {
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            } else {
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            }
        }
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        if(useWebcam) {
            vuforia.enableConvertFrameToBitmap();
            AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
        }

        if(useVuforiaTrackables) {
            initializeVuforiaTrackingGeometry(parameters);
            activateVuforiaTracking(); // Do we want to start Tracking on initialization?
        }

        // Initialize TensorFlow, if possible.
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(useTensorFlowMonitor);
            activateTensorFlow(); // Do we want to start TF on initialization?
        } else {
            opMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

    }


    public void activateVuforiaTracking() {
        targetsRoverRuckus.activate();
    }

    public OpenGLMatrix getLastLocation() {
        return lastLocation;
    }

    public void activateTensorFlow() {
        if (tfod != null) {
            tfod.activate();
        }
    }

    /*
    *   Initializes vuforia trackables and sets the camera and field geometry.
    * */
    private void initializeVuforiaTrackingGeometry(VuforiaLocalizer.Parameters parameters) {
        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final float CAMERA_FORWARD_DISPLACEMENT  = (float)(0*mmPerInch); //110;   // eg: Camera is 110 mm in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = (float)(6*mmPerInch); //200;   // eg: Camera is 200 mm above ground
        final float CAMERA_LEFT_DISPLACEMENT     = (float)(6*mmPerInch); //0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES,
                        90, 180, 0));
        // For left side Z = 180, for right side Z = 0.
        // Note that reported pitch increases downward, since Y is to the robot's left.

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            //((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

    }




    /**
     * Display the pose of the vuMark, if detected.
     */
    public void updateVuMarkPose() {
        if(useVuforiaTrackables) {

            // check all the trackable target to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    opMode.telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }


            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                opMode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                // Note that reported pitch increases downward, since Y is to the robot's left.
                opMode.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                opMode.telemetry.addData("Visible Target", "none");
            }
            //opMode.telemetry.update();
        }
    }

    public MecanumNavigation.Navigation2D getPositionNav2D() {
        if (lastLocation != null) {
            return new MecanumNavigation.Navigation2D(
                    lastLocation.getTranslation().get(0)/mmPerInch,
                    lastLocation.getTranslation().get(1)/mmPerInch,
                    Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, RADIANS).thirdAngle);
        } else {
                return new MecanumNavigation.Navigation2D(0,0,0);
        }
    }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }


    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod(boolean useTensorFlowMonitor) {
        tfTimer = new ElapsedTime();
        TFObjectDetector.Parameters tfodParameters;
        if (useTensorFlowMonitor) {
            int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        } else {
            tfodParameters = new TFObjectDetector.Parameters();
        }

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


    public void updateTensorFlow(boolean portrait) {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                pastRecognitions = updatedRecognitions;
                tfTimer.reset();
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = portrait ? (int) recognition.getLeft() : (int) recognition.getTop();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = portrait ? (int) recognition.getLeft() : (int) recognition.getTop();
                        } else {
                            silverMineral2X = portrait ? (int) recognition.getLeft() : (int) recognition.getTop();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            setGoldMineralPosition(GoldMineralPosition.LEFT);
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            setGoldMineralPosition(GoldMineralPosition.RIGHT);
                        } else {
                            setGoldMineralPosition(GoldMineralPosition.CENTER);
                        }
                    }
                }
            }
        }


    }

    public void displayTensorFlowDetectionSample() {
        opMode.telemetry.addData("# Object Detected", pastRecognitions.size());

        if (goldMineralPosition == GoldMineralPosition.LEFT) {
            opMode.telemetry.addData("Gold Mineral Position", "Left");
        } else if (goldMineralPosition == GoldMineralPosition.RIGHT) {
            opMode.telemetry.addData("Gold Mineral Position", "Right");
        } else {
            opMode.telemetry.addData("Gold Mineral Position", "Center");
        }
    }

    public void setGoldMineralPosition(GoldMineralPosition goldMineralPosition) {
        if (goldMineralPosition != GoldMineralPosition.UNKNOWN) {
            this.goldMineralPosition = goldMineralPosition;
        }
    }

    public void displayTensorFlowDetections() {
        if (tfod != null) {
            if (pastRecognitions != null && tfTimer.seconds() <= 1) {
                opMode.telemetry.addData("Gold Location:",goldMineralPosition.toString());
                int number_of_recognitions = pastRecognitions.size();
                opMode.telemetry.addData("# Object Detected", number_of_recognitions);
                if (number_of_recognitions > 0) {
                    for (Recognition recognition : pastRecognitions) {
                        opMode.telemetry.addData(recognition.getLabel(),
                        "{Left, Right, Top, Bottom} = %.0f, %.0f, %.0f, %.0f",
                                recognition.getLeft(),recognition.getRight(), recognition.getTop(), recognition.getBottom());
                    }
                }

            }
        }
    }

    public Color.Mineral identifyMineral(MineralIdentificationLocation idLocation) {
        Color.Mineral detectedMineralColor = Color.Mineral.UNKNOWN;

        if (pastRecognitions != null && pastRecognitions.size() > 0) {
            int closestDetectionIndex = 0; // Assume 1st is closest.
            double closestDetectionDistance = 1e6; //
            double currentDistance;

            int imageHeight = pastRecognitions.get(0).getImageHeight();
            int imageWidth = pastRecognitions.get(0).getImageWidth();

            int targetX;
            int targetY;
            double detectionX;
            double detectionY;

            if (idLocation == MineralIdentificationLocation.CENTER) {
                // Find image center
                targetX = imageWidth / 2;
                targetY = imageHeight / 2;
            } else if (idLocation == MineralIdentificationLocation.BOTTOM) {
                // Find bottom center
                targetX = imageWidth /2;
                targetY = imageHeight; // Bottom is max pixel value.
            } else { // Default to image center.
                targetX = imageWidth / 2;
                targetY = imageHeight / 2;
            }

            if (pastRecognitions != null && pastRecognitions.size() > 0 && tfTimer.seconds() <= 1) {
                // Fresh detection exist:
                int detectionIndex = -1;
                for (Recognition recognition : pastRecognitions) {
                    // find distance to target location.
                    detectionIndex++;
                    detectionX = (recognition.getLeft() + recognition.getRight()) / 2;
                    detectionY = (recognition.getTop() + recognition.getBottom()) / 2;
                    currentDistance = Math.sqrt(Math.pow(targetX - detectionX, 2) + Math.pow(targetY - detectionY, 2));
                    if (currentDistance < closestDetectionDistance) {
                        closestDetectionDistance = currentDistance;
                        closestDetectionIndex = detectionIndex;
                    }
                }
                // After finding closest mineral, identify it and return its color.
                String idLabel = pastRecognitions.get(closestDetectionIndex).getLabel();
                if (idLabel == LABEL_GOLD_MINERAL) {
                    detectedMineralColor = Color.Mineral.GOLD;
                } else if (idLabel == LABEL_SILVER_MINERAL) {
                    detectedMineralColor = Color.Mineral.SILVER;
                } else {
                    detectedMineralColor = Color.Mineral.UNKNOWN;
                }

                // Telemetry
                if (detectedMineralColor != Color.Mineral.UNKNOWN) {
                    opMode.telemetry.addData("Found", idLabel);
                    opMode.telemetry.addData("Mineral distance to target", closestDetectionDistance);
                } else {
                    opMode.telemetry.addData("No Mineral Detected in target location", "");
                }

            }

            return  detectedMineralColor;
        }

        return detectedMineralColor;
    }

    /**
     * Sample one frame from the Vuforia stream and write it to a .PNG image file on the robot
     * controller in the /sdcard/FIRST/data directory. The images can be downloaded using Android
     * Studio's Device File Explorer, ADB, or the Media Transfer Protocol (MTP) integration into
     * Windows Explorer, among other means. The images can be useful during robot design and calibration
     * in order to get a sense of what the camera is actually seeing and so assist in camera
     * aiming and alignment.
     */
    void captureFrameToFile() {
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>()
        {
            @Override public void accept(Frame frame)
            {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap != null) {
                    File file = new File(captureDirectory, String.format(Locale.getDefault(), "VuforiaFrame-%d.png", captureCounter++));
                    try {
                        FileOutputStream outputStream = new FileOutputStream(file);
                        try {
                            bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                        } finally {
                            outputStream.close();
                            opMode.telemetry.log().add("captured %s", file.getName());
                        }
                    } catch (IOException e) {
                        RobotLog.ee(TAG, e, "exception in captureFrameToFile()");
                    }
                }
            }
        }));
    }


}
