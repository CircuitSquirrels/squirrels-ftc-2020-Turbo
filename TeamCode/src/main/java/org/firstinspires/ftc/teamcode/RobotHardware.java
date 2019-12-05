package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.IMUUtilities;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.SimpleVision;
import org.firstinspires.ftc.teamcode.Utilities.VectorMath;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Vector;

public class RobotHardware extends OpMode {

    // All motors on the robot, in order of MotorName.
    private ArrayList<ExpansionHubMotor> allMotors;

    // Expansion hubs and bulk reads
    private ExpansionHubEx expansionHubDrive;
    private ExpansionHubEx expansionHubArm;
    private RevBulkData bulkDataDrive;
    private RevBulkData bulkDataArm;


    // All servos on the robot, in order of ServoName.
    private ArrayList<Servo> allServos;

    // Names of only the drive motors.
    private ArrayList<MotorName> driveMotorNames;

    // All color sensors on the robot, in order of ColorSensorName.
    private ArrayList<ColorSensor> allColorSensors;

    // Format for displaying decimals.
    public DecimalFormat df;
    public DecimalFormat df_prec;

    // IMU reference
    public BNO055IMU imu;

    // Execution cycle period monitor.
    private ElapsedTime period = new ElapsedTime();
    private Vector<Double> pastPeriods = new Vector<Double>();

    public Controller controller1;
    public Controller controller2;
    public MecanumNavigation mecanumNavigation;
    public AutoDrive autoDrive;
    public IMUUtilities imuUtilities;
    public SimpleVision simpleVision;


    // The motors on the robot, must be the same names defined in the robot's Configuration file.
    public enum MotorName {
        DRIVE_FRONT_LEFT,
        DRIVE_FRONT_RIGHT,
        DRIVE_BACK_LEFT,
        DRIVE_BACK_RIGHT,
        LIFT_WINCH
    }

    public enum DriveMotors {
        DRIVE_FRONT_LEFT,
        DRIVE_FRONT_RIGHT,
        DRIVE_BACK_LEFT,
        DRIVE_BACK_RIGHT
    }

    public enum ArmMotors {
        LIFT_WINCH
    }

    /**
     * Sets the power of the motor.
     *
     * @param motor The motor to modify.
     * @param power The power to set [-1, 1].
     */
    public void setPower(MotorName motor, double power) {
        ExpansionHubMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing", motor.name() + ": " + df.format(power));
        } else {
            m.setPower(power);
        }
    }

    /**
     * Get motor power.
     *
     * @param motor MotorName.
     * @return Motor Power, or zero if it cannot be found.
     */
    public double getPower(MotorName motor) {
        ExpansionHubMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing: ", motor.name());
            return 0;
        } else {
            return m.getPower();
        }
    }

    /**
     * Gets the encoder value of the motor.
     *
     * @param motor MotorName enum value.
     * @return integer encoder position in ticks.
     */
    public int getEncoderValue(MotorName motor) {
        ExpansionHubMotor m = allMotors.get(motor.ordinal());
        if(m != null) {
            for (DriveMotors driveMotors : DriveMotors.values()) {
                if(bulkDataDrive == null) break;
                if(driveMotors.name().equals(motor.name())) {
                    return bulkDataDrive.getMotorCurrentPosition(m);
                }
            }
            for (ArmMotors armMotors : ArmMotors.values()) {
                if(bulkDataArm == null) break;
                if(armMotors.name().equals(motor.name())) {
                    return bulkDataArm.getMotorCurrentPosition(m);
                }
            }
            Log.w("RobotHardware","Not using bulk reads for motor: " + motor.toString());
            return m.getCurrentPosition();
        } else {
            telemetry.addData("Motor Missing: ", motor.name());
            return 0;
        }
    }

    /**
     * Stops all motors.
     */
    public void stopAllMotors() {
        for (MotorName m : MotorName.values()) {
            setPower(m, 0);
        }
    }

    /**
     * Stops all motors and resets the Encoder Positions.
     */
    public void resetAndStopAllMotors() {
        for (MotorName name : MotorName.values()) {
            ExpansionHubMotor motor = allMotors.get(name.ordinal());
            if (motor == null) {
                telemetry.addData("Motor Missing: " ,name.name());
            } else {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }

    /**
     * Set all four drive motors to the same runMode.
     * Options are RUN_WITHOUT_ENCODER, RUN_USING_ENCODER,
     * RUN_TO_POSITION is used with setTargetPosition()
     * STOP_AND_RESET_ENCODER.
     *
     * @param runMode
     */
    protected void setDriveMotorsRunMode(DcMotor.RunMode runMode) {
        for (MotorName motor : driveMotorNames) {
            ExpansionHubMotor m = allMotors.get(motor.ordinal());
            if (m == null) {
                telemetry.addData("Motor Missing: ", motor.name());
            } else {
                m.setMode(runMode);
            }
        }
    }

    /**
     * Set zero power braking behavior for all drive wheels.
     *
     * @param zeroPowerBraking boolean to activate or deactivate zero power braking.
     */
    protected void setDriveMotorsZeroPowerBraking(boolean zeroPowerBraking) {
        DcMotor.ZeroPowerBehavior brakingMode = zeroPowerBraking ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        for (MotorName motor : driveMotorNames) {
            ExpansionHubMotor m = allMotors.get(motor.ordinal());
            if (m == null) {
                telemetry.addData("Motor Missing: ", motor.name());
            } else {
                m.setZeroPowerBehavior(brakingMode);
            }
        }
    }


    /**
     * Sets the drive chain power.
     *
     * @param left  The power for the left two motors.
     * @param right The power for the right two motors.
     */
    public void setDriveForTank(double left, double right) {
        setPower(MotorName.DRIVE_FRONT_LEFT, left);
        setPower(MotorName.DRIVE_BACK_LEFT, left);
        setPower(MotorName.DRIVE_FRONT_RIGHT, right);
        setPower(MotorName.DRIVE_BACK_RIGHT, right);
    }

    /**
     * Apply motor power matching the wheels object.
     *
     * @param wheels Provides all four mecanum wheel powers, [-1, 1].
     */
    public void setDriveForMecanumWheels(Mecanum.Wheels wheels) {
        setPower(MotorName.DRIVE_FRONT_LEFT, wheels.frontLeft);
        setPower(MotorName.DRIVE_BACK_LEFT, wheels.backLeft);
        setPower(MotorName.DRIVE_FRONT_RIGHT, wheels.frontRight);
        setPower(MotorName.DRIVE_BACK_RIGHT, wheels.backRight);
    }

    /**
     * Sets mecanum drive chain power using simplistic calculations.
     *
     * @param leftStickX Unmodified Gamepad leftStickX inputs.
     * @param leftStickY Unmodified Gamepad leftStickY inputs.
     * @param rightStickX Unmodified Gamepad rightStickX inputs.
     * @param rightStickY Unmodified Gamepad rightStickY inputs.
     */
    public void setDriveForSimpleMecanum(double leftStickX, double leftStickY,
                                         double rightStickX, double rightStickY) {
        Mecanum.Wheels wheels = Mecanum.simpleJoystickToWheels (leftStickX, leftStickY, rightStickX, rightStickY);
        setDriveForMecanumWheels(wheels);
    }



    // The servos on the robot, names must be defined in robot Configure file
    public enum ServoName {
        CLAW_LEFT,
        CLAW_RIGHT,
        FOUNDATION,
        AUTO_ARM
    }

    // Servo methods

    /**
     * Sets the angle of the servo.
     *
     * @param servo    The servo to modify.
     * @param position The angle to set [0, 1].
     */
    public void setAngle(ServoName servo, double position) {
        Servo s = allServos.get(servo.ordinal());
        if (s == null) {
            telemetry.addData("Servo Missing", servo.name() + ": " + df.format(position));
        } else {
            s.setPosition(position);
        }
    }

    /**
     * Get the position of a servo.
     *
     * @param servo ServoName enum to check.
     * @return double servo position [0,-1].
     */
    public double getAngle(ServoName servo) {
        Servo s = allServos.get(servo.ordinal());
        if (s == null) {
            telemetry.addData("Servo Missing", servo.name());
            return -1;
        } else {
            return s.getPosition();
        }
    }

    /**
     * Move a servo to a position at a specific speed.
     *
     * @param servo The servo that will be driven.
     * @param targetPos The position that you want the servo to go to; Range [1-0].
     * @param rate The speed at which the servo will move.
     * @return Returns whether or not the servo has arrived to its targetPos.
     */
    public boolean moveServoAtRate(ServoName servo, double targetPos, double rate) {
        boolean isMovementDone;
        double distanceThreshold = 0.05;
        rate = Range.clip(rate,0,10);
        targetPos = Range.clip(targetPos,0,1);
        double currentPosition = getAngle(servo);
        double distance = targetPos - currentPosition;
        double direction = targetPos > currentPosition ? 1 : -1;
        double nextPosition;
        if ( Math.abs(distance) > distanceThreshold ) {
            nextPosition = rate * direction * getLastPeriodSec() + currentPosition;
            isMovementDone = false;
        } else {
            nextPosition = targetPos;
            isMovementDone = true;
        }
        nextPosition = Range.clip(nextPosition,0,1);
        setAngle(servo, nextPosition);

        return isMovementDone;
    }

    /**
     * Initialize the claw vertical with the position defined in Constants.
     */
    public void verticalClaw() {
        setAngle(ServoName.CLAW_LEFT, Constants.LEFT_CLAW_VERTICAL);
        setAngle(ServoName.CLAW_RIGHT, Constants.RIGHT_CLAW_VERTICAL);
    }

    /**
     * Open the claw with the position defined in Constants.
     */
    public void openClaw() {
        setAngle(ServoName.CLAW_LEFT, Constants.LEFT_CLAW_OPEN);
        setAngle(ServoName.CLAW_RIGHT, Constants.RIGHT_CLAW_OPEN);
    }

    /**
     * Close the claw with the position defined in Constants.
     */
    public void closeClaw() {
        setAngle(ServoName.CLAW_LEFT, Constants.LEFT_CLAW_CLOSED);
        setAngle(ServoName.CLAW_RIGHT, Constants.RIGHT_CLAW_CLOSED);
    }

    // The color sensors on the robot.
    public enum ColorSensorName {
//        MINERAL_COLOR,
    }

    /**
     * Gets the color value on the sensor.
     *
     * @param sensor The sensor to read.
     * @param color  The color channel to read intensity.
     */
    public int getColorSensor(ColorSensorName sensor, Color.Channel color) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Color Sensor Missing: ", sensor.name());
            return 0;
        }

        switch (color) {
            case RED:
                return s.red();
            case GREEN:
                return s.green();
            case BLUE:
                return s.blue();
            case ALPHA:
                return s.alpha();
            default:
                return 0;
        }
    }

    /**
     * Check whether or not the sensor is detected on the robot.
     *
     * @param sensor The sensor to check.
     * @return Returns whether or not the sensor exists.
     */
    public boolean colorSensorExists(ColorSensorName sensor) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Sets the LED power for the color sensor.
     *
     * @param sensor  The sensor to set the LED power.
     * @param enabled Whether to turn the LED on.
     */
    public void setColorSensorLedEnabled(ColorSensorName sensor,
                                         boolean enabled) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Color Sensor Missing: ", sensor.name());
        } else {
            s.enableLed(enabled);
        }
    }

    // Possible starting positions.
    public enum StartPosition {
        FIELD_LOADING,
        FIELD_BUILD,
    }

    /**
     * Gets the Vuforia license key.
     */
    protected String getVuforiaLicenseKey() {
        String vuforiaLicenseKey = "AVFkXXL/////AAABmbZECMrRQkzRjDc4Fz5X9EwXnUQbKq9G/gERF/bpOt9TIpneSXsY3Qyv878mUAqY1coOvRiFFj/ZoK+uIs+qK3IHQUwgqJW6y9EhzwRNqcdzEHvrnttZnUJmWjBh0O93lrrC8mGzDw+/wozmT/jr1Peu4qOijWfYhgW2GGtszwl3/u6u9Pca43FcykCY52RsXcsGkM2/8z0Ini3hc/HxrWoKcYnycJf5yXLSxXWJz+vHVdnhu8Wen18HXX4ec9MA+P0psoVuUqtwZGpcXDEXSlk//Z1p9tN3vdBApm2fzdoYfGSqhLnN0gITvIPB3VMpXRUo0Zbb7QXCs0Ydh2EjPNTKlBFNGFewr+7Xhd2XVPcd";
        return vuforiaLicenseKey;
    }

    // IMU Names (Could support multiple REV IMUs)
    public enum IMUNames {
        IMU,
    }

    /**
     * Should be executed at the beginning of loop() function.
     * Adds the most recent period length (in seconds) to a vector.
     * then, calculates the average period length.
     * @return Average period of past execution cycles.
     */
    public double updatePeriodTime(){
        pastPeriods.add(period.seconds());
        period.reset();
        if (pastPeriods.size()>= 200) {
            pastPeriods.remove(0);
        }
        return VectorMath.average(pastPeriods);
    }

    public double getAveragePeriodSec() {
        return VectorMath.average(pastPeriods);
    }

    public double getMaxPeriodSec() {
        return Collections.max(pastPeriods);
    }

    public double getLastPeriodSec() {
        if (pastPeriods.size() != 0) {
            return pastPeriods.lastElement();
        } else {
            return 0;
        }
    }

    /**
     * Initialize all motors, servos, sensors, set motor direction, motor runMode
     * Output whether or not the defined motors, servos, sensors could not be found
     */

    public void init() {

        // Setup expansion hubs for bulk reads.
        expansionHubDrive = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        expansionHubArm = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        allMotors = new ArrayList<ExpansionHubMotor>();
        for (MotorName m : MotorName.values()) {
            try {
                allMotors.add(hardwareMap.get(ExpansionHubMotor.class, m.name()));
            } catch (Exception e) {
                telemetry.addData("Motor Missing", m.name());
                allMotors.add(null);
            }
        }

        // Collect a list of only the drive motors.
        driveMotorNames = new ArrayList<MotorName>();
        driveMotorNames.add(MotorName.DRIVE_FRONT_LEFT);
        driveMotorNames.add(MotorName.DRIVE_FRONT_RIGHT);
        driveMotorNames.add(MotorName.DRIVE_BACK_LEFT);
        driveMotorNames.add(MotorName.DRIVE_BACK_RIGHT);

        resetAndStopAllMotors();
        initializePID(); // Makes current drive PIDF parameters available as K_P, K_I, K_D, K_F.

        // Set motor directions.
        try {
            allMotors.get(MotorName.DRIVE_FRONT_LEFT.ordinal()).setDirection(DcMotor.Direction.FORWARD);
            allMotors.get(MotorName.DRIVE_FRONT_RIGHT.ordinal()).setDirection(DcMotor.Direction.REVERSE);
            allMotors.get(MotorName.DRIVE_BACK_RIGHT.ordinal()).setDirection(DcMotor.Direction.REVERSE);
            allMotors.get(MotorName.DRIVE_BACK_LEFT.ordinal()).setDirection(DcMotor.Direction.FORWARD);
            allMotors.get(MotorName.LIFT_WINCH.ordinal()).setDirection(DcMotorSimple.Direction.FORWARD);
        } catch (Exception e) {
            telemetry.addData("Unable to set motor direction", "");
        }

        // Set drive motors to use encoders
        setDriveMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set drive motors to float instead of brake when power is zero.
        setDriveMotorsZeroPowerBraking(false);

        // Set arm motor to brake
        try {
            allMotors.get(MotorName.LIFT_WINCH.ordinal()).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            allMotors.get(MotorName.LIFT_WINCH.ordinal()).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Unable to set arm motor to zero power brake or encoder use", "");
        }

        allServos = new ArrayList<Servo>();
        for (ServoName s : ServoName.values()) {
            try {
                allServos.add(hardwareMap.get(Servo.class, s.name()));
            } catch (Exception e) {
                telemetry.addData("Servo Missing: ", s.name());
                allServos.add(null);
            }
        }
        // Set servo direction
        try {

        } catch (Exception e) {
            telemetry.addData("Unable to set left servo direction", "");
        }

        allColorSensors = new ArrayList<ColorSensor>();
        for (ColorSensorName s : ColorSensorName.values()) {
            try {
                allColorSensors.add(hardwareMap.get(ColorSensor.class,
                        s.name()));
            } catch (Exception e) {
                telemetry.addData("Color Sensor Missing: ", s.name());
                allColorSensors.add(null);
            }
        }

        // Add Decimal Formats for easy to read telemetry
        df = new DecimalFormat("0.00");
        df_prec = new DecimalFormat("0.0000");

        stopAllMotors();
        period.reset(); // Reset timer
    }

    public void init_loop() {
        bulkDataDrive = expansionHubDrive.getBulkInputData();
        bulkDataArm = expansionHubArm.getBulkInputData();
    }

    public void start() {
        stopAllMotors();
        bulkDataDrive = expansionHubDrive.getBulkInputData();
        bulkDataArm = expansionHubArm.getBulkInputData();
        period.reset(); // Reset timer
    }

    public void loop() {
        updatePeriodTime();
        bulkDataDrive = expansionHubDrive.getBulkInputData();
        bulkDataArm = expansionHubArm.getBulkInputData();
    }

    public void stop() {
        super.stop();

        for (MotorName m : MotorName.values()) {
            setPower(m, 0);
        }
        for (ColorSensorName s : ColorSensorName.values()) {
            setColorSensorLedEnabled(s, false);
        }
    }


    public static double K_P = 2.5;
    public static double K_I = 0.1;
    public static double K_D = 0.2;
    public static double K_F = 0.0;

    public void initializePID() {
        PIDFCoefficients pidfOrig = allMotors.get(driveMotorNames.get(0).ordinal()).getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        K_P = pidfOrig.p;
        K_I = pidfOrig.i;
        K_D = pidfOrig.d;
        K_F = pidfOrig.f;
    }

    public void configureDriveMotorVelocityPID(double K_P, double K_I, double K_D, double K_F) {
        for(MotorName motorName: driveMotorNames) {
            configureMotorVelocityPID(motorName,K_P,K_I,K_D,K_F);
        }
    }

    public void configureMotorVelocityPID(MotorName motorName, double K_P, double K_I, double K_D, double K_F) {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(K_P,K_I,K_D,K_F);
        allMotors.get(motorName.ordinal()).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        this.K_P = K_P;
        this.K_I = K_I;
        this.K_D = K_D;
        this.K_F = K_F;
    }

    public enum ControlParameter {
        P,I,D,F,
    }

    public void changeDriveControlParameterByFactor(ControlParameter controlParameter, double scaleFactor) {
        if(scaleFactor < 0) return; // Error prevention.
        double currentValue = getSingleDriveControlParameter(controlParameter);
        setSingleDriveControlParameter(controlParameter, currentValue * scaleFactor);
    }


    public void setSingleDriveControlParameter(ControlParameter controlParameter, double newValue) {
        if(newValue < 0) return; // Error prevention.
        double P,I,D,F;
        P = this.K_P;
        I = this.K_I;
        D = this.K_D;
        F = this.K_F;
        switch (controlParameter) {
            case P:
                P = newValue;
                break;
            case I:
                I = newValue;
                break;
            case D:
                D = newValue;
                break;
            case F:
                F = newValue;
        }
        configureDriveMotorVelocityPID(P,I,D,F);
    }

    public double getSingleDriveControlParameter(ControlParameter controlParameter) {
        switch (controlParameter) {
            case P:
                return this.K_P;
            case I:
                return this.K_I;
            case D:
                return this.K_D;
            case F:
            default:
                return this.K_F;
        }
    }


    public double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    public double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }

    public int liftArmTicksForLevelFoundationKnob(int level_1to6, boolean withFoundation, boolean withKnob) {
        double liftTicks = (level_1to6 - 1) * 4 * Constants.LIFT_TICKS_PER_INCH;
        if(withFoundation) liftTicks += Constants.LIFT_FOUNDATION_HEIGHT_TICKS;
        if(withKnob) liftTicks += Constants.LIFT_KNOB_HEIGHT_TICKS;
        return (int) liftTicks;
    }

    public double getSkystoneIndex(Waypoints waypoints) {
        double skystone_absolute_x;
        double bot_absolute_x;
        double bot_relative_to_skystone_y;

        if(simpleVision == null) return 0;

        bot_absolute_x = mecanumNavigation.currentPosition.x;
        bot_relative_to_skystone_y = simpleVision.getPositionSkystoneRelativeNav2d().y;
        if (waypoints.getTeamColor() == Color.Ftc.BLUE) {
            skystone_absolute_x = bot_absolute_x - bot_relative_to_skystone_y - 5;
        } else {
            skystone_absolute_x = bot_absolute_x + bot_relative_to_skystone_y + 5;
        }
        return waypoints.skystoneIndexFromX(skystone_absolute_x);
    }
}

