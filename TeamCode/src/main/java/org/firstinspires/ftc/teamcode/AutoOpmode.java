package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utilities.BehaviorSandBox;
import org.firstinspires.ftc.teamcode.Utilities.CSV;
import org.firstinspires.ftc.teamcode.Utilities.Executive;
import org.firstinspires.ftc.teamcode.Utilities.IMUUtilities;
import org.firstinspires.ftc.teamcode.Utilities.InteractiveInit;
import org.firstinspires.ftc.teamcode.Utilities.Mutable;
import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.RobotStateContext;
import org.firstinspires.ftc.teamcode.Utilities.SimpleVision;
import org.firstinspires.ftc.teamcode.Utilities.TimingMonitor;

public class AutoOpmode extends RobotHardware {

    public TimingMonitor timingMonitor;
    protected Color.Ftc robotColor;
    protected StartPosition robotStartPos;
    public Executive.RobotStateMachineContextInterface robotStateContext;
    public Thread thread;

    // Telemetry Recorder
    private CSV csvWriter;
    private CSV controlWriter;
    private boolean writeControls = false;

    //Interactive Init menu
    private InteractiveInit interactiveInit = null;
    public Mutable<Double> AutoDriveSpeed = new Mutable<>(0.5);
    public Mutable<Boolean> DropStones = new Mutable<>(true);
    public Mutable<Boolean> PauseBeforeState = new Mutable<>(true);
    private Mutable<Boolean> RecordTelemetry = new Mutable<>(false);
    public Mutable<Boolean> SimpleAuto = new Mutable<>(true);
    public Mutable<Boolean> ParkInner = new Mutable<>(true);

    @Autonomous(name="auto.Red.Pickup", group="Auto")
    public static class AutoRedPickup extends AutoOpmode {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_LOADING;
            super.init();
        }
    }

    @Autonomous(name="auto.Red.Build", group="Auto")
    public static class AutoRedBuild extends AutoOpmode {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_BUILD;
            super.init();
        }
    }

    @Autonomous(name="auto.Blue.Pickup", group="Auto")
    public static class AutoBluePickup extends AutoOpmode {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_LOADING;
            super.init();
        }
    }

    @Autonomous(name="auto.Blue.Build", group="Auto")
    public static class AutoBlueBuild extends AutoOpmode {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_BUILD;
            super.init();
        }
    }

    @Autonomous(name="auto.Sandbox", group="Test")
    public static class Sandbox extends AutoOpmode {
        @Override public void init() {
            robotColor = Color.Ftc.UNKNOWN;
            robotStartPos = StartPosition.FIELD_LOADING;
            super.init();
        }
    }

    @Override
    public void init() {
        super.init();
        timingMonitor = new TimingMonitor(AutoOpmode.this);
        timingMonitor.disable();
        controller1 = new Controller(gamepad1);
        thread = new Thread(new VisionLoader());
        thread.start();
        // Only initialize the imu if it is going to be used.
        imuUtilities = new IMUUtilities(this,"IMU_1");
        if(!robotColor.equals(Color.Ftc.UNKNOWN)) {
            robotStateContext = new RobotStateContext(AutoOpmode.this, robotColor, robotStartPos);
        } else {
            robotStateContext = new BehaviorSandBox(AutoOpmode.this, Color.Ftc.BLUE, robotStartPos);
        }
        telemetry.addData("Initialization: ", "Successful!");

        // Initialization Menu
        interactiveInit = new InteractiveInit(this);
        interactiveInit.addDouble(AutoDriveSpeed, "DriveSpeed",0.8,1.0,.1,.3,.5, .6, .7);
        interactiveInit.addBoolean(DropStones, "Drop Stones",true, false);
        interactiveInit.addBoolean(SimpleAuto, "Simple Auto: ", true, false);
        interactiveInit.addBoolean(ParkInner, "Park Inner: ", false, true);
        interactiveInit.addBoolean(PauseBeforeState, "Pause Before State", true, false);
        interactiveInit.addBoolean(RecordTelemetry,"Record Telemetry", true, false);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        controller1.update();
        if (simpleVision == null) {
            telemetry.addData("Vision: ", "LOADING...");
        } else {
            telemetry.addData("Vision: ", "INITIALIZED");
            simpleVision.updateVuMarkPose();
        }
        interactiveInit.update();
    }

    @Override
    public void start() {
        super.init();

        // Navigation and control
        mecanumNavigation = new MecanumNavigation(this,Constants.getDriveTrainMecanum());
        mecanumNavigation.initialize(new MecanumNavigation.Navigation2D(0, 0, 0));
        autoDrive = new AutoDrive(this, mecanumNavigation);

        // Ensure starting position at origin, even if wheels turned since initialize.
        mecanumNavigation.update();
        mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0,0,0));
        robotStateContext.init(); //After mecanum init, because state init could reference mecanumNavigation.

        interactiveInit.lock();

        if(RecordTelemetry.get()) {
            csvWriter = new CSV(this);
            csvWriter.open("telemetry.csv");

            if (writeControls) {
                controlWriter = new CSV(this);
                controlWriter.open("controls.csv");
            }
            recordConstantsToFile();
        }
    }

    @Override
    public void loop() {
        timingMonitor.loopStart();
        if(controller1.start()) { timingMonitor.reset();} // Clear with start button
        super.loop();
        timingMonitor.checkpoint("POST super.loop()");
        controller1.update();
        timingMonitor.checkpoint("POST controller.update()");
        mecanumNavigation.update();
        timingMonitor.checkpoint("POST mecanumNavigation.update()");
        robotStateContext.update();
        timingMonitor.checkpoint("POST robotStateMachine.update()");
        if ( imuUtilities != null ) {
            imuUtilities.update();
            imuUtilities.getCompensatedHeading();
            timingMonitor.checkpoint("POST imuUtilities.update()");
        }

        // Conditional Telemetry Recording
        if(RecordTelemetry.get()) {
            if(writeControls) {
                writeControlsToFile();
            }
            writeTelemetryToFile();
            timingMonitor.checkpoint("POST telemetry recorder");
        }

        mecanumNavigation.displayPosition();
        telemetry.addLine();
        telemetry.addData("Period Average (sec)", df_prec.format(getAveragePeriodSec()));
        telemetry.addData("Period Max (sec)", df_prec.format(getMaxPeriodSec()));
        timingMonitor.displayMaxTimes();
        timingMonitor.checkpoint("POST TELEMETRY");

        telemetry.addData("State: ",robotStateContext.getCurrentState());

        try {
            simpleVision.updateVuMarkPose();
            simpleVision.displayFormattedVumarkPose();
            telemetry.addData("Nav2D Absolute", simpleVision.getPositionAbsoluteNav2d());
            telemetry.addData("Nav2D Skystone Relative", simpleVision.getPositionSkystoneRelativeNav2d());
//            telemetry.addData("OpenGL Navigation", simpleVision.getLastAbsoluteLocation().toString());
            simpleVision.updateTensorFlow(true);
            simpleVision.displayTensorFlowDetections();
        } catch(Exception e) {
            telemetry.addData("Vision Not Loaded", "");
        }
        timingMonitor.checkpoint("POST Vision");
        telemetry.addData("Lift Ticks",getEncoderValue(MotorName.LIFT_WINCH));
    }

    @Override
    public void stop() {
        super.stop();
        closeCSV();
    }




    // Initialize vuforia in a separate thread to avoid init() hangups.
    class VisionLoader implements Runnable {
        public void run() {

            //TODO Might need to use trackables, the second to last boolean.
            simpleVision = new SimpleVision(getVuforiaLicenseKey(), AutoOpmode.this,
                    true, false,true,
                    true, SimpleVision.UseWebcamEnum.TRUE, SimpleVision.TensorFlowEnabled.FALSE);
        }
    }



    private void recordConstantsToFile() {
        CSV constantsWriter = new CSV(this);
        constantsWriter.open("constants.csv");
        constantsWriter.addFieldToRecord("drive_wheel_diameter", Constants.DRIVE_WHEEL_DIAMETER_INCHES);
        constantsWriter.addFieldToRecord("wheelbase_width_in", Constants.WHEELBASE_WIDTH_IN);
        constantsWriter.addFieldToRecord("wheelbase_length_in", Constants.WHEELBASE_LENGTH_IN);
        constantsWriter.addFieldToRecord("wheelbase_k", Math.abs(Constants.WHEELBASE_LENGTH_IN/2.0)
                + Math.abs(Constants.WHEELBASE_WIDTH_IN/2.0));
        constantsWriter.addFieldToRecord("drive_wheel_steps_per_rotation", (double)Constants.DRIVE_WHEEL_STEPS_PER_ROT);
        constantsWriter.completeRecord();
        constantsWriter.close();
    }


    private void writeControlsToFile() {
        controlWriter.addFieldToRecord("time",time);

        controlWriter.addFieldToRecord("left_stick_x", controller1.left_stick_x);
        controlWriter.addFieldToRecord("left_stick_y", controller1.left_stick_y);
        controlWriter.addFieldToRecord("right_stick_x", controller1.right_stick_x);
        controlWriter.addFieldToRecord("right_stick_y", controller1.right_stick_y);
        controlWriter.addFieldToRecord("left_trigger", controller1.left_trigger);
        controlWriter.addFieldToRecord("right_trigger", controller1.right_trigger);

        controlWriter.addFieldToRecord("right_stick_button", controller1.rightStickButton() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("left_stick_button", controller1.leftStickButton() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("right_bumper", controller1.rightBumper() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("left_bumper", controller1.leftBumper() ? 1.0 : 0.0);

        controlWriter.addFieldToRecord("a_button", controller1.A() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("b_button", controller1.B() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("x_button", controller1.X() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("y_button", controller1.Y() ? 1.0 : 0.0);

        controlWriter.completeRecord();
    }


    private void writeTelemetryToFile() {
        // setFieldData sets both titles and recordData.
        csvWriter.addFieldToRecord("time",time);
        // Capture all servo positions:
        for (ServoName s : ServoName.values()) {
            csvWriter.addFieldToRecord(s.name(), getAngle(s));
        }
        // Capture all motor encoder values:
        for (MotorName m : MotorName.values()) {
            csvWriter.addFieldToRecord(m.name()+"_ticks", (double)getEncoderValue(m));
        }
        // Capture all motor power levels:
        for (MotorName m : MotorName.values()) {
            csvWriter.addFieldToRecord(m.name()+"_power", getPower(m));
        }
        // Capture mecanumNavigation current position
        csvWriter.addFieldToRecord("x_in",mecanumNavigation.currentPosition.x);
        csvWriter.addFieldToRecord("y_in",mecanumNavigation.currentPosition.y);
        csvWriter.addFieldToRecord("theta_rad",mecanumNavigation.currentPosition.theta);
        if(imuUtilities != null) {
            csvWriter.addFieldToRecord("IMU_heading",imuUtilities.getCompensatedHeading());
        }



        // Add IMU data to current csvWriter record
        //addIMUToRecord(csvWriter);

        // Writes record to file if writer is open.
        csvWriter.completeRecord();

        telemetry.addData("WRITE CONTROLS",writeControls);
        if(writeControls) {
            writeControlsToFile();
        }
    }

    void closeCSV() {
        if(RecordTelemetry.get()) {
            csvWriter.close();
            if (writeControls) {
                controlWriter.close();
            }
        }

    }

    /**
     * Updates the mecanumNavigation heading from the imu heading.
     * Careful, this function forces the IMU to refresh immediately, which
     * causes up to 20ms of latency.
     * Use this once per state in a state machine, or on a timer.
     * Do not call this repeatedly in the main loop!
     */
    public void updateMecanumHeadingFromGyroNow() {
        {
            // Modify current position to account for rotation during descent measured by gyro.
            imuUtilities.updateNow();
            double gyroHeading = imuUtilities.getCompensatedHeading();
            MecanumNavigation.Navigation2D currentPosition = mecanumNavigation.currentPosition.copy();
            currentPosition.theta = degreesToRadians(gyroHeading);
            mecanumNavigation.setCurrentPosition(currentPosition);
        }
    }

    public boolean shouldContinue() {
        return !PauseBeforeState.get() || controller1.AOnce();
    }
}
