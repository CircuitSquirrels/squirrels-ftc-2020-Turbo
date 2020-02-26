// jUnit

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.deadWheels.OdometryConfig;
import org.firstinspires.ftc.teamcode.deadWheels.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.DrivetrainControl.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;
import org.firstinspires.ftc.teamcode.Utilities.RobotStateContext;
import org.junit.Before;
import org.junit.Test;

import java.text.DecimalFormat;

import FakeHardware.FakeAutoOpmode;
import FakeHardware.FakeIMUUtilities;
import FakeHardware.FakeSkystoneDetector;
import static TestUtilities.SimFormat.*;

import static com.google.common.truth.Truth.assertThat;

// Truth
// Mockito
//import org.mockito.Mock;
//import static org.mockito.Mockito.*;
//import org.mockito.Mockito;
//import org.mockito.MockitoAnnotations;
// Teamcode

public class StateMachineTest {

    DecimalFormat df;
    DecimalFormat df_prec;
    


    double simTime = 0.0;

    FakeAutoOpmode opMode;
    MecanumNavigation mecanumNavigation;
    OdometryLocalizer odometryLocalizer;
    AutoDrive autoDrive;
    RobotStateContext robotStateContext;
    FakeIMUUtilities imuUtilities;
    ElapsedTime elapsedTime;

    @Before
    public void setupTest() {
        //        MockitoAnnotations.initMocks(this);
        df = new DecimalFormat("0.00");
        df_prec = new DecimalFormat(("0.00000000"));
        initialize();
    }

    public void initialize() {
        opMode = new FakeAutoOpmode();
        mecanumNavigation = new MecanumNavigation(opMode, Constants.getDriveTrainMecanum());
        opMode.initializeFakeOpMode();

        autoDrive = new AutoDrive(opMode,mecanumNavigation,mecanumNavigation);
        mecanumNavigation.initialize(new Navigation2D(0,0,0));

        odometryLocalizer = new OdometryLocalizer(new OdometryConfig());
        odometryLocalizer.setCurrentPosition(new Navigation2D(0,0,0));
        odometryLocalizer.setEncoderPosition(opMode);

        opMode.setAutoDrive(autoDrive);
        opMode.setMecanumNavigation(mecanumNavigation);
        opMode.setOdometryLocalizer(odometryLocalizer);
        imuUtilities = new FakeIMUUtilities(opMode,"IMU_1");
        opMode.setIMUUtilities(imuUtilities);
        opMode.controller1 = new Controller(opMode.gamepad1);
        FakeSkystoneDetector fakeSkystoneDetector = new FakeSkystoneDetector(opMode);
        opMode.skystoneDetector = fakeSkystoneDetector;
        elapsedTime = new ElapsedTime();

        // Interactive Init settings (Defaults)
        opMode.PauseBeforeState.set(false);
        opMode.setDriveSpeed(1.0);
        opMode.setDropStones(true);
        opMode.setParkInner(false);
        opMode.setSimpleAuto(false);
        opMode.setSkystoneIndex(2);

        robotStateContext = new RobotStateContext(opMode,Color.Ftc.RED, RobotHardware.StartPosition.FIELD_LOADING);
        robotStateContext.init();
        simTime = 0.0;

    }

    @Test
    public void parkOuter_Simple_Index0() {
        initialize();
        // Interactive Init settings
        opMode.PauseBeforeState.set(false);
        opMode.setDriveSpeed(0.7);
        opMode.setDropStones(true);
        opMode.setParkInner(false);
        opMode.setSimpleAuto(true);
        opMode.setSkystoneIndex(0);
        opMode.setMoveFoundation(false);
        opMode.setConservativeMode(true);
        robotStateContext.init(); // Required to apply modifications to interactive init settings.
        simulateStateMachine(true, 2, 40.0, .02);
    }

    @Test
    public void parkOuter_NoDrop_Index1() {
        initialize();
        // Interactive Init settings
        opMode.PauseBeforeState.set(false);
        opMode.setDriveSpeed(0.7);
        opMode.setDropStones(false);
        opMode.setParkInner(false);
        opMode.setSimpleAuto(false);
        opMode.setSkystoneIndex(1);
        opMode.setMoveFoundation(false);
        opMode.setConservativeMode(true);
        robotStateContext.init(); // Required to apply modifications to interactive init settings.
        simulateStateMachine(true, 2, 40.0, .02);
    }

    @Test
    public void parkOuter_DropStones_Index1() {
        initialize();
        // Interactive Init settings
        opMode.PauseBeforeState.set(false);
        opMode.setDriveSpeed(.7);
        opMode.setDropStones(true);
        opMode.setParkInner(false);
        opMode.setSimpleAuto(false);
        opMode.setSkystoneIndex(1);
        opMode.setMoveFoundation(false);
        opMode.setConservativeMode(true);
        robotStateContext.init(); // Required to apply modifications to interactive init settings.
        simulateStateMachine(true, 2, 40.0, .02);
    }

    @Test
    public void parkOuter_DropStones_Index2() {
        initialize();
        // Interactive Init settings
        opMode.PauseBeforeState.set(false);
        opMode.setDriveSpeed(0.7);
        opMode.setDropStones(true);
        opMode.setParkInner(false);
        opMode.setSimpleAuto(false);
        opMode.setSkystoneIndex(2);
        opMode.setMoveFoundation(false);
        opMode.setConservativeMode(true);
        robotStateContext.init(); // Required to apply modifications to interactive init settings.
        simulateStateMachine(true, 2, 40.0, .02);
    }

    @Test
    public void parkInner_MoveFoundation_Index1() {
        initialize();
        // Interactive Init settings
        opMode.PauseBeforeState.set(false);
        opMode.setDriveSpeed(0.7);
        opMode.setDropStones(false);
        opMode.setParkInner(true);
        opMode.setSimpleAuto(false);
        opMode.setSkystoneIndex(1);
        opMode.setMoveFoundation(true);
        opMode.setConservativeMode(true);
        robotStateContext.init(); // Required to apply modifications to interactive init settings.
        simulateStateMachine(true, 2, 40.0, .02);
    }

    // Specify default behavior
    public void simulateStateMachine() {
        simulateStateMachine(false, 2, 40.0, .02);
    }

    public void simulateStateMachine(boolean showTransitionsOnly, int outputUpdatesPerSecond, double simulationEndTime, double simulationStepTime) {
        // Configuration
//        boolean showTransitionsOnly = true;
//        int outputUpdatesPerSecond = 2;
//        double simulationEndTime = 40;
//        double simulationStepTime = 0.02;

        // Initialization

        SimulationTime simulationTime = new SimulationTime();
        simulationTime.setTime(0);
        boolean notStopped = true;
        boolean isNewState = true;
        String previousState = "";
        int formatStateFieldWidth = 53;
        int formatWaypointFieldWidth = 25;

        // Output Heading
        System.out.println(padStringTo(15, "SimTime") +
                padStringTo(formatStateFieldWidth,"| Current States") +
                padStringTo(30,"| Current Position") +
                padStringTo(formatWaypointFieldWidth,"| Last Waypoint Target") +
                padStringTo(17,"| Lift Encoder Ticks"));
        System.out.println("--------------------------------------------------------------------------------------------------------------------------------------");

        // Simulation loop, runs until simulationEndTime or final state is reached.
        while (simulationTime.time() <= simulationEndTime && notStopped) {
            robotStateContext.update(); // Update state simulation

            // Display logic, shows x updates per second AND first update after a state transition.
            isNewState = !previousState.equals(robotStateContext.getCurrentState());
            previousState = robotStateContext.getCurrentState();
            if(isNewState || isDisplayInterval(outputUpdatesPerSecond, simTime,simulationStepTime) && !showTransitionsOnly) {
                System.out.print("SimTime: " + String.format("%5.2f",simTime) + "   ");
                System.out.print(padStringTo(formatStateFieldWidth,robotStateContext.getCurrentState()));
                System.out.print(mecanumNavigation.getCurrentPosition().toString() + "   " + padStringTo(formatWaypointFieldWidth, autoDrive.lastTargetPosition.getLabel()));
                System.out.println("LiftWinch: " + String.format("%5d",opMode.getEncoderValue(RobotHardware.MotorName.LIFT_WINCH)));
                notStopped = !(robotStateContext.getCurrentState().startsWith("Stop_State") || robotStateContext.getCurrentState().startsWith("A_Manual"));

//                Frame2D robotFrame = mecanumNavigation.getRobotFrame();
//                Frame2D gripperFrame = new Frame2D(9,0,0,robotFrame);
//                Waypoints waypoints = new Waypoints(Color.Ftc.RED,1);
//                System.out.println("stone 1 relative to gripper frame: " + (waypoints.stoneLocations.get(1)).getNav2DInLocalFrame(gripperFrame).toString());

            }

            mecanumNavigation.update(); // Update navigation
            // Increment simulation time and FakeRobotHardware state.
            opMode.updateAndIntegrateFakeOpMode(simTime);
            simulationTime.incrementTime(simulationStepTime);
            simTime = simulationTime.time();
        }
        System.out.println();
    }





}
