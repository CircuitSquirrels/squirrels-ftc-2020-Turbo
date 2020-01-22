// jUnit
import org.junit.Before;
import org.junit.Test;

// Truth
import static com.google.common.truth.Truth.assertThat;

// Mockito
//import org.mockito.Mock;
//import static org.mockito.Mockito.*;
//import org.mockito.Mockito;
//import org.mockito.MockitoAnnotations;

// Teamcode
import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;


import java.text.DecimalFormat;
import FakeHardware.FakeRobotHardware;

public class AutoDriveTest {

    DecimalFormat df;

    private void assertMecanumWheelEqual(Mecanum.Wheels wheels1, Mecanum.Wheels wheels2) {
        assertThat(wheels1.backLeft).isEqualTo(wheels2.backLeft);
        assertThat(wheels1.backRight).isEqualTo(wheels2.backRight);
        assertThat(wheels1.frontLeft).isEqualTo(wheels2.frontLeft);
        assertThat(wheels1.frontRight).isEqualTo(wheels2.frontRight);
    }

    private void printWheels(Mecanum.Wheels wheels) {
        System.out.println( wheels.backLeft + " , " + wheels.backRight + " , " + wheels.frontLeft + " , " + wheels.frontRight);
    }

    private void printCommand(Mecanum.Command command) {
        System.out.println(command.vx + ", " + command.vy + ", " + command.av);
    }



    double simTime = 0.0;

    FakeRobotHardware opMode = new FakeRobotHardware();
    MecanumNavigation mecanumNavigation;
    AutoDrive autoDrive;

    @Before
    public void setupTest() {
        //        MockitoAnnotations.initMocks(this);
        opMode.initializeFakeOpMode();
        df = new DecimalFormat("0.00");

        mecanumNavigation = new MecanumNavigation(opMode, Constants.getDriveTrainMecanum());
        mecanumNavigation.initialize(new MecanumNavigation.Navigation2D(0,0,0));
        autoDrive = new AutoDrive(opMode,mecanumNavigation);

        simTime = 0.0;
    }

    @Test
    public void initializeAutoDrive() {
        assertThat(autoDrive).isNotNull();
        // Test Motion command for errors
        autoDrive.rotateThenDriveToPosition(new Navigation2D(0,0,0), 1.0);
    }

    @Test
    public void simulateAutoDrive() {
        double simulationEndTime = 6.5;
        double simulationStepTime = 0.02;
        double driveSpeed = 0.5;

        Navigation2D targetPosition = new Navigation2D(10,0,Math.toRadians(90));

        while (simTime <= simulationEndTime) {

            boolean arrived = autoDrive.driveToPositionTranslateOnly(targetPosition, driveSpeed);

            if(isDisplayInterval(5, simTime,simulationStepTime)) {
                System.out.print("SimTime:  " + df.format(simTime) + "   Arrived:  " + arrived + "    ");
                System.out.println(mecanumNavigation.currentPosition.toString());
//                System.out.println();
            }

            mecanumNavigation.update();
            opMode.updateAndIntegrateFakeOpMode(simTime);
            simTime += simulationStepTime;
        }

    }

    private boolean isDisplayInterval(int displaysPerSecond, double simTime, double simulationTimeStep) {
        int simStep = (int) Math.floor( simTime/simulationTimeStep );
        int stepsPerSecond = (int) Math.floor(1/simulationTimeStep);
        int stepsPerDisplay = stepsPerSecond / displaysPerSecond;
        return (simStep % stepsPerDisplay) == 0;
    }

}
