package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.InteractiveInit;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.Mutable;
import org.firstinspires.ftc.teamcode.Utilities.SimpleVision;
import org.firstinspires.ftc.teamcode.Utilities.TimingMonitor;

@Autonomous(name="FieldLaps", group="Testing")
public class AutoTesting extends RobotHardware {


    public TimingMonitor timingMonitor;
    public MecanumNavigation mecanumNavigation;
    public AutoDrive autoDrive;
    protected Color.Ftc robotColor;
    protected StartPosition robotStartPos;
//    protected RobotStateMachine robotStateMachine;
    public SimpleVision simpleVision;
    private Thread thread;
    public Controller controller;

    //Interactive Init menu
    InteractiveInit interactiveInit = null;
//    public Mutable<Boolean> Simple = new Mutable<>(false);
    public Mutable<Boolean> UsingMiniRobot = new Mutable<>(false);


    // State
    private enum LapState {
        START,
        RED_CRATER,
        BLUE_IMAGE,
        BACK_IMAGE,
        RED_IMAGE,
        FRONT_IMAGE,
        END,
    }
    LapState lapState = LapState.START;
    ElapsedTime stateTimer;


    @Override
    public void init() {
        super.init();
        robotColor = Color.Ftc.RED;
        robotStartPos = StartPosition.FIELD_CRATER;
        stateTimer = new ElapsedTime();


        timingMonitor = new TimingMonitor(AutoTesting.this);
        controller = new Controller(gamepad1);
        thread = new Thread(new VisionLoader());
        thread.start();

        // Finally, construct the state machine.
//        robotStateMachine = new RobotStateMachine(this, robotColor, robotStartPos);
        telemetry.addData("Initialization:", "Successful!");
        // Initialization Menu
        interactiveInit = new InteractiveInit(this);
//        interactiveInit.addBoolean(Simple, "Simple Mode", false, true);
        interactiveInit.addBoolean(UsingMiniRobot, "Using MiniRobot", true, false);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        controller.update();

        if (simpleVision == null) {
            telemetry.addData("Vision:", "LOADING...");
        } else {
            telemetry.addData("Vision:", "INITIALIZED");
        }

        interactiveInit.update();
    }

    @Override
    public void start() {
        super.init();

        // Navigation and control
        if(UsingMiniRobot.get()) {
            mecanumNavigation = new MecanumNavigation(this,Constants.MiniRobot.getDriveTrainMecanum());
        } else {
            mecanumNavigation = new MecanumNavigation(this,Constants.getDriveTrainMecanum());
        }
        mecanumNavigation.initialize(new MecanumNavigation.Navigation2D(0, 0, 0));
        autoDrive = new AutoDrive(this, mecanumNavigation);

        // Ensure starting position at origin, even if wheels turned since initialize.
        mecanumNavigation.update();
        mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0,0,0));
//        robotStateMachine.init();
        // Initialize our state


        interactiveInit.lock();
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
//        robotStateMachine.update();
        updateStateMachine(); // Within class
        timingMonitor.checkpoint("POST robotStateMachine.update()");

        mecanumNavigation.displayPosition();


        telemetry.addData("Current State", lapState);
        telemetry.addLine();
        telemetry.addData("Period Average (sec)", df_prec.format(getAveragePeriodSec()));
        telemetry.addData("Period Max (sec)", df_prec.format(getMaxPeriodSec()));
        timingMonitor.displayMaxTimes();
        timingMonitor.checkpoint("POST TELEMETRY");

        // Vision updates and telemetry
        if (simpleVision != null) {
            simpleVision.updateVuMarkPose();
            telemetry.addData("VuForia Nav2D",simpleVision.getPositionNav2D());
            simpleVision.updateTensorFlow(true);
            simpleVision.displayTensorFlowDetections();
            simpleVision.identifyMineral(SimpleVision.MineralIdentificationLocation.CENTER);
        }

        timingMonitor.checkpoint("POST Vision");
    }




    // Initialize vuforia in a separate thread to avoid init() hangups.
    class VisionLoader implements Runnable {
        public void run() {
            simpleVision = new SimpleVision(getVuforiaLicenseKey(), AutoTesting.this,
                    true, false,false,
                    true, true);
        }
    }


    private void updateStateMachine() {
        boolean arrived = false;
        double speed = 0.5;
        double wallOffset = 60;


        switch (lapState) {
            case START:
                // Set initial mecanumNavigation position.
                mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(-24,-24, degreesToRadians(-45)));

                // Next state logic
                stateTimer.reset();
                lapState = LapState.RED_CRATER;
                break;
            case RED_CRATER:
                arrived = autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(-24,-24, degreesToRadians(-45)),speed);
                if (arrived) {
                    // Next state logic
                    stateTimer.reset();
                    lapState = LapState.RED_IMAGE;
                }
                break;
            case RED_IMAGE:
                arrived = autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(0,-wallOffset, degreesToRadians(0)),speed);
                if (arrived) {
                    // Next state logic
                    stateTimer.reset();
                    lapState = LapState.BACK_IMAGE;
                }
                break;
            case BACK_IMAGE:
                arrived = autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(+wallOffset,0, degreesToRadians(90)),speed);
                if (arrived) {
                    // Next state logic
                    stateTimer.reset();
                    lapState = LapState.BLUE_IMAGE;
                }
                break;
            case BLUE_IMAGE:
                arrived = autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(0,wallOffset, degreesToRadians(180)),speed);
                if (arrived) {
                    // Next state logic
                    stateTimer.reset();
                    lapState = LapState.FRONT_IMAGE;
                }
                break;
            case FRONT_IMAGE:
                arrived = autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(-wallOffset,0, degreesToRadians(270)),speed);
                if (arrived) {
                    // Next state logic
                    stateTimer.reset();
                    lapState = LapState.END;
                }
                break;
            case END:
//                arrived = autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(-24,-24, degreesToRadians(-45)),speed);
//                if (arrived) {
//                    // Next state logic
//                    stateTimer.reset();
//                    lapState = LapState.END;
//                }
                stopAllMotors();
                break;
            default:
                stopAllMotors();
                break;
        }

        if (arrived) {
            updatePositionFromVision();
        }

    }

    private void updatePositionFromVision() {
        if (simpleVision != null) {
            mecanumNavigation.setCurrentPosition(simpleVision.getPositionNav2D());
        }
    }


    private double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    private double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }
}
