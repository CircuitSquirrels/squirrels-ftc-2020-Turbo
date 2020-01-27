package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Utilities.BehaviorSandBox;
import org.firstinspires.ftc.teamcode.FileIO.CSV;
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
import org.firstinspires.ftc.teamcode.Utilities.TimingMonitor;
import org.firstinspires.ftc.teamcode.Vision.AveragingPipeline;
import org.firstinspires.ftc.teamcode.Vision.SkystoneDetector;

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

    // Interactive init options
    public Mutable<Double> DriveSpeed = new Mutable<>(0.5);
    public Mutable<Boolean> DropStones = new Mutable<>(true);
    public Mutable<Boolean> PauseBeforeState = new Mutable<>(false);
    private Mutable<Boolean> RecordTelemetry = new Mutable<>(false);
    public Mutable<Boolean> ConservativeRoute = new Mutable<>(false);
    public Mutable<Boolean> SimpleAuto = new Mutable<>(true);
    public Mutable<Boolean> ParkInner = new Mutable<>(true);
    public Mutable<Boolean> Foundation = new Mutable<>(false);
    public Mutable<Boolean> FoundationPark = new Mutable<>(false);

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

        controller1 = new Controller(gamepad1);

        thread = new Thread(new VisionLoader());
        thread.start();

        imuUtilities = new IMUUtilities(this,"IMU_1");

        if(!robotColor.equals(Color.Ftc.UNKNOWN))
            robotStateContext = new RobotStateContext(AutoOpmode.this, robotColor, robotStartPos);
        else
            robotStateContext = new BehaviorSandBox(AutoOpmode.this, Color.Ftc.BLUE, robotStartPos);


        timingMonitor = new TimingMonitor(AutoOpmode.this);
        // Disable timing monitor to avoid spamming telemetry
        timingMonitor.disable();

        telemetry.addData("Initialization: ", "Successful!");

        interactiveInit = new InteractiveInit(AutoOpmode.this);

        // Initialization Menu
        interactiveInit.addOption(DriveSpeed, "DriveSpeed: ",0.8,1.0,.1,.3,.5, .6, .7)
                .addOption(DropStones, "Drop Stones: ",true, false)
                .addOption(ParkInner, "Park Inner: ", false, true)
                .addOption(Foundation, "Move Foundation: ", false, true)
                .addOption(ConservativeRoute, "Conservative Route: ", false, true)
                .addOption(PauseBeforeState, "Pause Before State: ", true, false)
                .addOption(SimpleAuto, "Simple Auto: ", true, false)
                .addOption(RecordTelemetry,"Record Telemetry: ", true, false)
                .addOption(FoundationPark, "Foundation Park: ", false, true);
    }

    @Override
    public void init_loop() {
        super.init_loop();

        controller1.update();
        interactiveInit.update();
        if (skystoneDetector == null)
            telemetry.addData("Vision: ", "LOADING...");
        else
            telemetry.addData("Vision: ", "INITIALIZED");
    }

    @Override
    public void start() {
        super.start();

        // Navigation and control
        mecanumNavigation = new MecanumNavigation(this, Constants.getDriveTrainMecanum());
        mecanumNavigation.initialize(new MecanumNavigation.Navigation2D(0, 0, 0));
        autoDrive = new AutoDrive(this, mecanumNavigation);

        // Ensure starting position at origin, even if wheels turned since initialize.
        mecanumNavigation.update();
        mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0,0,0));

        robotStateContext.init(); //After mecanum init, because state init could reference mecanumNavigation.

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
        if(controller1.start())
            timingMonitor.reset(); // Clear with start button

        super.loop();
        timingMonitor.checkpoint("POST super.loop()");

        controller1.update();
        timingMonitor.checkpoint("POST controller.update()");

        mecanumNavigation.update();
        timingMonitor.checkpoint("POST mecanumNavigation.update()");

        robotStateContext.update();
        timingMonitor.checkpoint("POST robotStateMachine.update()");

        if (imuUtilities != null) {
            imuUtilities.update();
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
            telemetry.addData("Skystone Position: ", skystoneDetector.getSkystoneRelativeLocation());
            for (int i = 0; i < skystoneDetector.averagingPipeline.getAllData().size() - 1; i++) {
                telemetry.addData("Skystone "+i+"Color: ", skystoneDetector.averagingPipeline.getAllData().get(i));
            }
            telemetry.addData("Skystone Index: ", skystoneDetector.getSkystoneIndex());
        } catch(Exception e) {
            telemetry.addData("Vision Not Loaded", "");
        }
        timingMonitor.checkpoint("POST Vision");
    }

    @Override
    public void stop() {
        super.stop();
        closeCSV();
        saveLiftEncoderPosition();

    }

    private void saveLiftEncoderPosition() {


    }


    // Initialize OpenCV in a separate thread to avoid init() hangups.
    class VisionLoader implements Runnable {
        public void run() {
            skystoneDetector = new SkystoneDetector(AutoOpmode.this, robotColor);
            skystoneDetector.init(new AveragingPipeline());
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
        constantsWriter.addFieldToRecord("drive_wheel_steps_per_rotation", Constants.DRIVE_WHEEL_STEPS_PER_ROT);
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
            csvWriter.addFieldToRecord(m.name()+"_ticks", (double) getEncoderValue(m));
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

    private void closeCSV() {
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
