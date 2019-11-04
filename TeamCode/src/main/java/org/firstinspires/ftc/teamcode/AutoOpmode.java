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
    public MecanumNavigation mecanumNavigation;
    public AutoDrive autoDrive;
    protected Color.Ftc robotColor;
    protected StartPosition robotStartPos;
    public SimpleVision simpleVision;
    public Executive.RobotStateMachineContextInterface robotStateContext;
    public Thread thread;
    public Controller controller;
    public IMUUtilities imuUtilities;

    // Telemetry Recorder
    private CSV csvWriter;
    private CSV controlWriter;
    private boolean writeControls = false;

    //Interactive Init menu
    private InteractiveInit interactiveInit = null;
    private Mutable<Boolean> Simple = new Mutable<>(false);
    private Mutable<Double> AutoDriveSpeed = new Mutable<>(0.5);
    private Mutable<Boolean> RecordTelemetry = new Mutable<>(false);
    private Mutable<Boolean> useIMU = new Mutable<>(false);
    private Mutable<Boolean> earlyFlagDrop = new Mutable<>(false);

    @Autonomous(name="auto.Red.Pickup", group="Auto")
    public static class AutoRedPickup extends AutoOpmode {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_PICKUP;
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
            robotStartPos = StartPosition.FIELD_PICKUP;
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
            robotStartPos = StartPosition.FIELD_BUILD;
            super.init();
        }
    }

    @Override
    public void init() {
        super.init();
        timingMonitor = new TimingMonitor(AutoOpmode.this);
        timingMonitor.disable();
        controller = new Controller(gamepad1);
        thread = new Thread(new VisionLoader());
        thread.start();
        if(!robotColor.equals(Color.Ftc.UNKNOWN)) {
            robotStateContext = new RobotStateContext(AutoOpmode.this, robotColor, robotStartPos);
        } else {
            robotStateContext = new BehaviorSandBox(AutoOpmode.this, Color.Ftc.BLUE, robotStartPos);
        }
        robotStateContext.init();
        telemetry.addData("Initialization:", "Successful!");
        System.out.println("This is a test");
        // Initialization Menu
        interactiveInit = new InteractiveInit(this);
        interactiveInit.addDouble(AutoDriveSpeed, "DriveSpeed",0.8,1.0,.1,.3,.5);
        interactiveInit.addBoolean(RecordTelemetry,"Record Telemetry", true, false);
        interactiveInit.addBoolean(useIMU,"Use IMU", false, true);
        interactiveInit.addBoolean(Simple, "Simple Mode", true, false);
        interactiveInit.addBoolean(earlyFlagDrop, "Drop flag early", false, true);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        controller.update();
        if (simpleVision == null) {
            telemetry.addData("Vision:", "LOADING...");
        } else {
            telemetry.addData("Vision:", "INITIALIZED");
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

        if ( useIMU.get() ) {
            // Only initialize the imu if it is going to be used.
            imuUtilities = new IMUUtilities(this,"IMU_1");
        }
    }

    @Override
    public void loop() {
        timingMonitor.loopStart();
        if(controller.start()) { timingMonitor.reset();} // Clear with start button
        super.loop();
        timingMonitor.checkpoint("POST super.loop()");
        controller.update();
        timingMonitor.checkpoint("POST controller.update()");
        mecanumNavigation.update();
        timingMonitor.checkpoint("POST mecanumNavigation.update()");
        robotStateContext.update();
        timingMonitor.checkpoint("POST robotStateMachine.update()");
        if ( useIMU.get() ) {
            imuUtilities.update();
            imuUtilities.getCompensatedHeading();
            timingMonitor.checkpoint("POST imuUtilities.update()");
            telemetry.addData("descent rotation",imuUtilities.getHeadingChange());
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

        try {
            simpleVision.updateVuMarkPose();
            telemetry.addData("Nav 2D", simpleVision.getPositionNav2D().toString());
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
                    true, false);
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

        controlWriter.addFieldToRecord("left_stick_x",controller.left_stick_x);
        controlWriter.addFieldToRecord("left_stick_y",controller.left_stick_y);
        controlWriter.addFieldToRecord("right_stick_x",controller.right_stick_x);
        controlWriter.addFieldToRecord("right_stick_y",controller.right_stick_y);
        controlWriter.addFieldToRecord("left_trigger",controller.left_trigger);
        controlWriter.addFieldToRecord("right_trigger",controller.right_trigger);

        controlWriter.addFieldToRecord("right_stick_button",controller.rightStickButton() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("left_stick_button",controller.leftStickButton() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("right_bumper",controller.rightBumper() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("left_bumper",controller.leftBumper() ? 1.0 : 0.0);

        controlWriter.addFieldToRecord("a_button",controller.A() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("b_button",controller.B() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("x_button",controller.X() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("y_button",controller.Y() ? 1.0 : 0.0);

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
        if(useIMU.get()) {
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
}
