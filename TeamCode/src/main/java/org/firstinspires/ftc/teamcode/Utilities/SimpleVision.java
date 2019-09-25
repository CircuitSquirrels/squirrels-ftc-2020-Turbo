package org.firstinspires.ftc.teamcode.Utilities;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

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
    private VuforiaTrackables targetsSkystone;
    private List<VuforiaTrackable> allTrackables;


    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    // Vuforia Nav Geometry Properties
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor


    // TensorFlow Properties
    public boolean tfSupported = true;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private TFObjectDetector tfod;
    public List<Recognition> updatedRecognitions;
    public List<Recognition> pastRecognitions;

    // Webcam Support
    private boolean useWebcam;
    int captureCounter = 0;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    WebcamName webcamName;


    public static enum SkystoneSets {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN,
    }

    public static enum MineralIdentificationLocation {
        CENTER,
        BOTTOM,
    }

    public SkystoneSets skystonePositions = SkystoneSets.UNKNOWN;


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
        targetsSkystone.activate();
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
        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkystone);

        VuforiaTrackables stonesAndChips = vuforia.loadTrackablesFromAsset("StonesAndChips");
        VuforiaTrackable redTarget = stonesAndChips.get(0);
        redTarget.setName("RedTarget");  // Stones

        VuforiaTrackable blueTarget  = stonesAndChips.get(1);
        blueTarget.setName("BlueTarget");  // Chips

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(stonesAndChips);

        /**
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="stones" size="247 173"/>
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        /**
         * In order for localization to work, we need to tell the system where each target we
         * wish to use for navigation resides on the field, and we need to specify where on the robot
         * the camera resides. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * For the most part, you don't need to understand the details of the math of how transformation
         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
         * <ol>
         *
         *     <li>You can put two transformations together to produce a third that combines the effect of
         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
         *     then the combined transformation matrix RT which does the rotation first and then the translation
         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
         *     <em>reverse</em> of the chronological order in which they applied.</li>
         *
         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
         *
         *     <li>If you want to break open the black box of a transformation matrix to understand
         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
         *
         * </ol>
         *
         * This example places the "stones" image on the perimeter wall to the Left
         *  of the Red Driver station wall.  Similar to the Red Beacon Location on the Res-Q
         *
         * This example places the "chips" image on the perimeter wall to the Right
         *  of the Blue Driver station.  Similar to the Blue Beacon Location on the Res-Q
         *
         * See the doc folder of this project for a description of the Field Coordinate System
         * conventions.
         *
         * Initially the target is conceptually lying at the origin of the Field Coordinate System
         * (the center of the field), facing up.
         *
         * In this configuration, the target's coordinate system aligns with that of the field.
         *
         * In a real situation we'd also account for the vertical (Z) offset of the target,
         * but for simplicity, we ignore that here; for a real robot, you'll want to fix that.
         *
         * To place the Stones Target on the Red Audience wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it  90 around the field's Z access to face it away from the audience.
         * - Finally, we translate it back along the X axis towards the red audience wall.
         */
        OpenGLMatrix redTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redTarget.setLocationFtcFieldFromTarget(redTargetLocationOnField);
        RobotLog.ii(TAG, "Red Target=%s", format(redTargetLocationOnField));

        /*
         * To place the Stones Target on the Blue Audience wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Finally, we translate it along the Y axis towards the blue audience wall.
         */
        OpenGLMatrix blueTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(0, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueTarget.setLocationFtcFieldFromTarget(blueTargetLocationOnField);
        RobotLog.ii(TAG, "Blue Target=%s", format(blueTargetLocationOnField));

        /**
         * We also need to tell Vuforia where the <em>cameras</em> are relative to the robot.
         *
         * Just as there is a Field Coordinate System, so too there is a Robot Coordinate System.
         * The two share many similarities. The origin of the Robot Coordinate System is wherever
         * you choose to make it on the robot, but typically you'd choose somewhere in the middle
         * of the robot. From that origin, the Y axis is horizontal and positive out towards the
         * "front" of the robot (however you choose "front" to be defined), the X axis is horizontal
         * and positive out towards the "right" of the robot (i.e.: 90deg horizontally clockwise from
         * the positive Y axis), and the Z axis is vertical towards the sky.
         *
         * Similarly, for each camera there is a Camera Coordinate System. The origin of a Camera
         * Coordinate System lies in the middle of the sensor inside of the camera. The Z axis is
         * positive coming out of the lens of the camera in a direction perpendicular to the plane
         * of the sensor. When looking at the face of the lens of the camera (down the positive Z
         * axis), the X axis is positive off to the right in the plane of the sensor, and the Y axis
         * is positive out the top of the lens in the plane of the sensor at 90 horizontally
         * counter clockwise from the X axis.
         *
         * Next, there is Phone Coordinate System (for robots that have phones, of course), though
         * with the advent of Vuforia support for Webcams, this coordinate system is less significant
         * than it was previously. The Phone Coordinate System is defined thusly: with the phone in
         * flat front of you in portrait mode (i.e. as it is when running the robot controller app)
         * and you are staring straight at the face of the phone,
         *     * X is positive heading off to your right,
         *     * Y is positive heading up through the top edge of the phone, and
         *     * Z is pointing out of the screen, toward you.
         * The origin of the Phone Coordinate System is at the origin of the Camera Coordinate System
         * of the front-facing camera on the phone.
         *
         * Finally, it is worth noting that trackable Vuforia Image Targets have their <em>own</em>
         * coordinate system (see {@link VuforiaTrackable}. This is sometimes referred to as the
         * Target Coordinate System. In keeping with the above, when looking at the target in its
         * natural orientation, in the Target Coodinate System
         *     * X is positive heading off to your right,
         *     * Y is positive heading up through the top edge of the target, and
         *     * Z is pointing out of the target, toward you.
         *
         * One can observe that the Camera Coordinate System of the front-facing camera on a phone
         * coincides with the Phone Coordinate System. Further, when a phone is placed on its back
         * at the origin of the Robot Coordinate System and aligned appropriately, those coordinate
         * systems also coincide with the Robot Coordinate System. Got it?
         *
         * In this example here, we're going to assume that we put the camera on the right side
         * of the robot (facing outwards, of course). To determine the transformation matrix that
         * describes that location, first consider the camera as lying on its back at the origin
         * of the Robot Coordinate System such that the Camera Coordinate System and Robot Coordinate
         * System coincide. Then the transformation we need is
         *      * first a rotation of the camera by +90deg along the robot X axis,
         *      * then a rotation of the camera by +90deg along the robot Z axis, and
         *      * finally a translation of the camera to the side of the robot.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 90, 90, 0));
        RobotLog.ii(TAG, "camera=%s", format(robotFromCamera));

        /**
         * Let the trackable listeners we care about know where the camera is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener)redTarget.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener)blueTarget.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);

        /**
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = robotFromCamera          maps   camera coords -> robot coords
         * P = tracker.getPose()        maps   image target coords -> camera coords
         * L = redTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()                 maps   robot coords -> camera coords
         * P.inverted()                 maps   camera coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

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
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
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
                    int skystoneX1 = -1;
                    int skystoneX2 = -1;
                    int stone1 = -1;
                    int stone2 = -1;
                    int stone3 = -1;
                    int stone4 = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                            skystoneX1 = portrait ? (int) recognition.getLeft() : (int) recognition.getTop();
                        } else if (stone1 == -1) {
                            stone1 = portrait ? (int) recognition.getLeft() : (int) recognition.getTop();
                        } else {
                            stone2 = portrait ? (int) recognition.getLeft() : (int) recognition.getTop();
                        }
                    }
                    if (skystoneX1 != -1 && stone1 != -1 && stone2 != -1) {
                        if (skystoneX1 < stone1 && skystoneX1 < stone2) {
                            setSkystonePosition(SkystoneSets.LEFT);
                        } else if (skystoneX1 > stone1 && skystoneX1 > stone2) {
                            setSkystonePosition(SkystoneSets.RIGHT);
                        } else {
                            setSkystonePosition(SkystoneSets.CENTER);
                        }
                    }
                }
            }
        }


    }

    public void displayTensorFlowDetectionSample() {
        opMode.telemetry.addData("# Object Detected", pastRecognitions.size());

        if (skystonePositions == SkystoneSets.LEFT) {
            opMode.telemetry.addData("Skystone Position", "Left");
        } else if (skystonePositions == SkystoneSets.RIGHT) {
            opMode.telemetry.addData("Skystone Position", "Right");
        } else {
            opMode.telemetry.addData("Skystone Position", "Center");
        }
    }

    public void setSkystonePosition(SkystoneSets skystonePosition) {
        if (skystonePosition != SkystoneSets.UNKNOWN) {
            this.skystonePositions = skystonePosition;
        }
    }

    public void displayTensorFlowDetections() {
        if (tfod != null) {
            if (pastRecognitions != null && tfTimer.seconds() <= 1) {
                opMode.telemetry.addData("Gold Location:",skystonePositions.toString());
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
                if (idLabel == LABEL_FIRST_ELEMENT) {
                    detectedMineralColor = Color.Mineral.Skystone;
                } else if (idLabel == LABEL_SECOND_ELEMENT) {
                    detectedMineralColor = Color.Mineral.Stone;
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
