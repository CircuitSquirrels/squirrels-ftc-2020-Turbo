import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;
import org.firstinspires.ftc.teamcode.Utilities.PositionController;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.ScheduledExecutorService;

import FakeHardware.FakeRobotHardware;
import TestUtilities.SimFormat;

import static TestUtilities.SimFormat.padStringTo;

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
    public void run() {
        double stepTime = 0.02;
        double maxTime = 10.0;
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
        System.out.println(padStringTo(8, "SimTime") +
                padStringTo(13,"|   Arrived") +
                padStringTo(32,"|      Current Position") +
                padStringTo(17,"|  Robot Frame Power"));
        System.out.println("--------------------------------------------------------------------------------------------------------------------------------------");
    }


    private void printStatus() {
//        System.out.print("SimTime: ");
        System.out.print( padStringTo(12, String.format("%5.2f",simTime) ));
//        System.out.print("Arrived: ");
        System.out.print( padStringTo(12,String.valueOf(arrived)));
//        System.out.print("Position:    ");
//        System.out.print(drive.);
        System.out.print(padStringTo(30, mecanumNavigation.getCurrentPosition().toString()));
        System.out.println(drive.robotFramePower.toStringPure());
    }




}
