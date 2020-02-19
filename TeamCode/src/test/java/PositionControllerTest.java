import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.PositionController;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import FakeHardware.FakeRobotHardware;

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

        scheduler = Executors.newScheduledThreadPool(1);
        scheduler.scheduleAtFixedRate(new Thread(() -> {
            System.out.println("Arrived: " + arrived
                    + "\nX: " + mecanumNavigation.getCurrentPosition().x
                    + "\nY: " + mecanumNavigation.getCurrentPosition().y
                    + "\nTheta: " + mecanumNavigation.getCurrentPosition().theta
                    + "\nOuput: " );
        }), 2, 2, TimeUnit.SECONDS);
    }

    @Test
    public void run() {
        double stepTime = 0.02;
        // Keep thread running
        while (true) {
            MecanumNavigation.Navigation2D targetPosition = new MecanumNavigation.Navigation2D(10, 0, Math.toRadians(90));
            arrived = drive.driveTo(targetPosition, 1);
            opMode.updateAndIntegrateFakeOpMode(simTime);
            mecanumNavigation.update();
            simTime += stepTime;
        }
    }
}
