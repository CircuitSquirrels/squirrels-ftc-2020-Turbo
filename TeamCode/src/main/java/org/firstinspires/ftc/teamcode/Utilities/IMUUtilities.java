package org.firstinspires.ftc.teamcode.Utilities;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.Locale;

/**
 * Created by Ashley on 1/18/2018.
 */

public class IMUUtilities {

    // Convenience numbers for when IMUUtilities is actually instantiated.
    public BNO055IMU imu;
    public double heading;
    public double roll;
    public double pitch;
    public double xAccel;
    public double yAccel;
    public double zAccel;

    protected RobotHardware opMode;
    protected Orientation angles;
    protected Acceleration gravity;
    protected Acceleration acceleration;

    protected BNO055IMU.SystemStatus imuSystemStatus;
    protected BNO055IMU.CalibrationStatus imuCalibrationStatus;

    private double lastUpdateSec = 0;
    private double minUpdateDelay = 1.0; // Seconds

    private ImuMode imuMode;

    public enum ImuMode {
        FAST_HEADING_ONLY,
        SLOW_ALL_MEASUREMENTS,
    }


    // Default to Fast, Heading only mode when not specified.
    public IMUUtilities(RobotHardware opMode, String imu_name) {
        this(opMode,imu_name, ImuMode.FAST_HEADING_ONLY);
    }

    public IMUUtilities(RobotHardware opMode, String imu_name, ImuMode imuMode) {
        this.opMode = opMode;
        this.imuMode = imuMode;
        imu = initializeIMU(this.opMode, imu_name, imuMode);
        startIMU(imu);
    }

    private double previousHeading = 0;
    private double headingChange = 0;
    public void updateNow() {

        // If IMU is missing, do nothing.
        if (imu == null) {return;}
        lastUpdateSec = opMode.time;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        roll = angles.secondAngle;
        pitch = angles.thirdAngle;

        // Only grab non-heading info if running in slow mode.
        if(imuMode == ImuMode.SLOW_ALL_MEASUREMENTS) {
            imuSystemStatus = imu.getSystemStatus();
            imuCalibrationStatus = imu.getCalibrationStatus();
            gravity = imu.getGravity();

            acceleration = imu.getLinearAcceleration();
            xAccel = acceleration.xAccel;
            yAccel = acceleration.yAccel;
            zAccel = acceleration.zAccel;
        }

        // Unwrap compensated heading.
        headingCompensation = headingChange > 180 ? headingCompensation - 360 : (headingChange < -180 ? headingCompensation + 360 : headingCompensation);

//       if(headingChange > 180) {
//           headingCompensation -= 360;
//       } else if(headingChange < -180) {
//           headingCompensation += 360;
//       }
    }

    public void update() {
        // If IMU is missing, do nothing.
        if (imu == null) {
            opMode.telemetry.addData("", "IMU Missing");
            Log.e("IMUUtilities", "IMU Missing");
            return;
        }

        // Update rate is limited by minUpdateDelay to prevent too many costly operations.
        // Updating this data is quite expensive.
        if (opMode.time - lastUpdateSec > minUpdateDelay) {
          updateNow();
        }
    }

    // How stale is our data?
    public double dataAge() {
      return opMode.time - lastUpdateSec;
    }

    public void displayTelemetry() {
        // Uses class data from most recent update.
        // If IMU is missing, do nothing.
        if (imu == null) {return;}
        switch (imuMode) {
            case SLOW_ALL_MEASUREMENTS:
                displayIMUTelemetry(imuSystemStatus, imuCalibrationStatus, angles, gravity, acceleration, opMode);
                break;
            case FAST_HEADING_ONLY:
                opMode.telemetry.addLine()
                        .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
                        .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
                        .addData("pitch", formatAngle(angles.angleUnit, angles.thirdAngle));
                break;
        }

        opMode.telemetry.addData("IMU data age", dataAge());
    }

    // Static Functions

    /**
     * Use IMU config name to initialize and return a reference to the imu hardware.
     * @param opMode
     * @param imu_name
     * @return
     */
    static public BNO055IMU initializeIMU(RobotHardware opMode, String imu_name, ImuMode imuMode) {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        if( imuMode == ImuMode.FAST_HEADING_ONLY) {
            parameters.loggingEnabled = false;
        } else {
            parameters.loggingEnabled = true;
        }
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        BNO055IMU imu;
        try {
            imu = opMode.hardwareMap.get(BNO055IMU.class, imu_name);
            imu.initialize(parameters);
        } catch (Exception e) {
            imu = null;
            opMode.telemetry.addData("IMU Missing", imu_name);
        }
        return imu;
    }

    /**
     * Start IMU integration and logging.
     * @param imu
     */
    static public void startIMU (BNO055IMU imu) {
        // If IMU is missing, do nothing.
        if (imu == null) {return;}
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }


    /**
     * Display telemetry data for the IMU
     * @param systemStatus
     * @param calibrationStatus
     * @param angles
     * @param gravity
     * @param acceleration
     * @param opMode
     */
    static public void displayIMUTelemetry(BNO055IMU.SystemStatus systemStatus,
                                           BNO055IMU.CalibrationStatus calibrationStatus,
                                           Orientation angles, Acceleration gravity, Acceleration acceleration,
                                           RobotHardware opMode) {
        opMode.telemetry.addLine().
            addData("status",  systemStatus.toShortString())
            .addData("calib",calibrationStatus.toString());

        opMode.telemetry.addLine()
                .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
                .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
                .addData("pitch", formatAngle(angles.angleUnit, angles.thirdAngle));

        opMode.telemetry.addLine()
                .addData("grvty", gravity.toString())
                .addData("mag", String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel)));

        opMode.telemetry.addLine()
                .addData("X Accel", String.format(Locale.getDefault(), "%.3f", acceleration.xAccel))
                .addData("Y Accel", String.format(Locale.getDefault(), "%.3f", acceleration.yAccel))
                .addData("Z Accel", String.format(Locale.getDefault(), "%.3f", acceleration.zAccel));
    }



    static public void displayIMUTelemetry(final BNO055IMU imu, RobotHardware opMode) {
        // If IMU is missing, do nothing.
        if (imu == null) {return;}
        BNO055IMU.SystemStatus systemStatus = imu.getSystemStatus();
        BNO055IMU.CalibrationStatus calibrationStatus = imu.getCalibrationStatus();
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Acceleration gravity = imu.getGravity();
        Acceleration acceleration = imu.getLinearAcceleration();

        displayIMUTelemetry(systemStatus, calibrationStatus, angles, gravity, acceleration, opMode);
    }


    static public class OrientationAngles {
        public double heading;
        public double roll;
        public double pitch;

        public OrientationAngles(double heading, double roll, double pitch) {
            this.heading = heading;
            this.roll = roll;
            this.pitch = pitch;
        }
    }


    /**
     * Return object with robot heading, roll, and pitch in degrees.
     * @param imu
     * @return OrientationAngles object
     */
    static public OrientationAngles getOrientation(BNO055IMU imu) {
        // If IMU is missing, do nothing.
        if (imu == null) {
            return new OrientationAngles(-999, -999, -999);
        }
        final Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Return heading, roll, pitch
        return new OrientationAngles(angles.firstAngle, angles.secondAngle, angles.thirdAngle);
    }

    /**
     * Redundant method, returns Acceleration object with xAccel, yAccel, zAccell components.
     * @param imu
     * @return Acceleration object
     */
    static public Acceleration getAccelerationComponents(BNO055IMU imu) {
        // If IMU is missing, do nothing.
        if (imu == null) {
            return new Acceleration(DistanceUnit.METER,-999, -999, -999,1000);
        }
        return  imu.getLinearAcceleration();
    }
    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    static protected String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static protected String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    // Calculation helpers
    // Initial and final headings are just used to help comparisons of orientation.
    private double headingCompensation = 0;
    private double initialHeading = 0;
    private double finalHeading = 0;

   public void setCompensatedHeading(double compensatedHeadingDegrees) {
       headingCompensation = compensatedHeadingDegrees - heading;
   }

   public double getCompensatedHeading() {
       return heading + headingCompensation;
   }

   public void setInitialHeading() {
       initialHeading = heading;
   }

   public void setFinalHeading() {
       finalHeading = heading;
   }

   public double getHeadingChange() {
       return finalHeading - initialHeading;
   }


}
