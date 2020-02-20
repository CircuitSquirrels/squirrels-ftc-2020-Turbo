import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;
import org.firstinspires.ftc.teamcode.Utilities.PositionController;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.concurrent.ScheduledExecutorService;

import FakeHardware.FakeRobotHardware;
import TestUtilities.SimFormat;

import static TestUtilities.SimFormat.padStringTo;
import static TestUtilities.SimFormat.padCenteredStringTo;

public class PositionControllerTest {

    FakeRobotHardware opMode = new FakeRobotHardware();
    MecanumNavigation mecanumNavigation;
    PositionController drive;
    boolean arrived = false;
    ScheduledExecutorService scheduler;
    double simTime = 0;

    @Before
    public void init() {
        opMode.initializeFakeOpMode();
        mecanumNavigation = new MecanumNavigation(opMode, Constants.getDriveTrainMecanum());
        mecanumNavigation.initialize(new MecanumNavigation.Navigation2D(0,0,0));
        drive = new PositionController(opMode, mecanumNavigation);

    }


    @Test
    public void run_square() {
        double stepTime = 0.02;
        double maxTime = 10.0;
        int displaysPerSecond = 10;
        opMode.setAveragePeriodSec(stepTime);
        Navigation2D targetPosition;
        ArrayList<Navigation2D> waypointList = new ArrayList<>();
        int waypointIndex = 0;

        waypointList.add(new Navigation2D(0, 0, Math.toRadians(0)));
        waypointList.add(new Navigation2D(10, 0, Math.toRadians(0)));
        waypointList.add(new Navigation2D(10, 10, Math.toRadians(0)));
        waypointList.add(new Navigation2D(10, 10, Math.toRadians(90)));
        waypointList.add(new Navigation2D(0, 0, Math.toRadians(90)));
        waypointList.add(new Navigation2D(0, 0, Math.toRadians(0)));


        printHeading();
        while (simTime <= maxTime && waypointList.size() > waypointIndex) {

            //Move
            Navigation2D targetWaypoint = waypointList.get(waypointIndex).copy();
            arrived = drive.driveTo(targetWaypoint, 1.0);
            if(arrived) ++waypointIndex;

            // Display
            if(SimFormat.isDisplayInterval(displaysPerSecond,simTime,stepTime)) {
                printStatus();
            }

            // Update
            opMode.updateAndIntegrateFakeOpMode(simTime);
            mecanumNavigation.update();
            simTime += stepTime;
        }
    }


    @Test
    public void run_path() {
        double stepTime = 0.02;
        double maxTime = 10.0;
        int displaysPerSecond = 10;
        opMode.setAveragePeriodSec(stepTime);
        Navigation2D targetPosition;
        ArrayList<Navigation2D> waypointList = new ArrayList<>();
        int waypointIndex = 0;

        waypointList.add(new Navigation2D(0, 0, Math.toRadians(0)));
        waypointList.add(new Navigation2D(30, -50, Math.toRadians(0)));
        waypointList.add(new Navigation2D(35, -50, Math.toRadians(0)));
        waypointList.add(new Navigation2D(30, -50, Math.toRadians(0)));
        waypointList.add(new Navigation2D(10, 50, Math.toRadians(90)));
        waypointList.add(new Navigation2D(30, 50, Math.toRadians(0)));


        printHeading();
        while (simTime <= maxTime && waypointList.size() > waypointIndex) {

            //Move
            Navigation2D targetWaypoint = waypointList.get(waypointIndex).copy();
            arrived = drive.driveTo(targetWaypoint, 1.0);
            if(arrived) ++waypointIndex;

            // Display
            if(arrived || SimFormat.isDisplayInterval(displaysPerSecond,simTime,stepTime)) {
                printStatus();
            }

            // Update
            opMode.updateAndIntegrateFakeOpMode(simTime);
            mecanumNavigation.update();
            simTime += stepTime;
        }
    }


    @Test
    public void run_accelerate() {
        double stepTime = 0.02;
        double maxTime = 6.0;
        int displaysPerSecond = 10;
        opMode.setAveragePeriodSec(stepTime);
        Navigation2D targetPosition;

        printHeading();
        while (simTime <= maxTime) {

            //Move
            if(simTime < 1.0) {
                targetPosition = new Navigation2D(20, 20, Math.toRadians(90));
            } else {
                targetPosition = new Navigation2D(0, 0, Math.toRadians(0));
            }
            arrived = drive.driveTo(targetPosition, 1);

            // Display
            if(SimFormat.isDisplayInterval(displaysPerSecond,simTime,stepTime)) {
                printStatus();
            }

            // Update
            opMode.updateAndIntegrateFakeOpMode(simTime);
            mecanumNavigation.update();
            simTime += stepTime;
        }
    }


    private void printHeading() {
        // Output Heading
        System.out.println(
                padCenteredStringTo(8, "SimTime") + "|" +
                padCenteredStringTo(10,"Arrived") + "|" +
                padCenteredStringTo(30,"Current Position") + "|" +
//                padCenteredStringTo(25,"Robot Frame Power") + "|" +
                padCenteredStringTo(25,"Mech Command") + "|" +
                padCenteredStringTo(28,"Controller Error Sum") + "|"
        );
        System.out.println("--------------------------------------------------------------------------------------------------------------------------------------");
    }


    private void printStatus() {
        System.out.println(
                padCenteredStringTo(8, String.format("%5.2f",simTime)) + "|" +
                padCenteredStringTo(10,String.valueOf(arrived)) + "|" +
                padCenteredStringTo(30, mecanumNavigation.getCurrentPosition().toString()) + "|" +
//                padCenteredStringTo(25,drive.robotFramePower.toStringPure()) + "|" +
                padCenteredStringTo(25,drive.movementCommand.toString()) + "|" +
                padCenteredStringTo(28,drive.getErrorSum()) + "|"
        );
    }




}
