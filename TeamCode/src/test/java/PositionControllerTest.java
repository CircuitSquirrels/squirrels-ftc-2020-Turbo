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
        double maxTime = 20.0;
        Navigation2D targetPosition = new Navigation2D(10, 0, Math.toRadians(90));

        printHeading();
        while (simTime <= maxTime) {
            //Move
            arrived = drive.driveTo(targetPosition, 1);

            // Display
            if(SimFormat.isDisplayInterval(2,simTime,stepTime)) {
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
                padStringTo(17,"|                 "));
        System.out.println("--------------------------------------------------------------------------------------------------------------------------------------");
    }


    private void printStatus() {
//        System.out.print("SimTime: ");
        System.out.print( padStringTo(12, String.format("%5.2f",simTime) ));
//        System.out.print("Arrived: ");
        System.out.print( padStringTo(12,String.valueOf(arrived)));
//        System.out.print("Position:    ");
        System.out.println(mecanumNavigation.getCurrentPosition());
    }




}
