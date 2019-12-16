// jUnit

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;
import org.firstinspires.ftc.teamcode.Utilities.RobotStateContext;
import org.junit.Before;
import org.junit.Test;

import java.text.DecimalFormat;

import FakeHardware.FakeAutoOpmode;
import FakeHardware.FakeIMUUtilities;

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

    FakeAutoOpmode opMode = new FakeAutoOpmode();
    MecanumNavigation mecanumNavigation;
    AutoDrive autoDrive;
    RobotStateContext robotStateContext;
    FakeIMUUtilities imuUtilities;
    ElapsedTime elapsedTime;

    @Before
    public void setupTest() {
        //        MockitoAnnotations.initMocks(this);
        opMode.initializeFakeOpMode();
        df = new DecimalFormat("0.00");
        df_prec = new DecimalFormat(("0.00000000"));

        mecanumNavigation = new MecanumNavigation(opMode, Constants.getDriveTrainMecanum());
        mecanumNavigation.initialize(new Navigation2D(0,0,0));
        autoDrive = new AutoDrive(opMode,mecanumNavigation);
        opMode.setAutoDrive(autoDrive);
        opMode.setMecanumNavigation(mecanumNavigation);
        imuUtilities = new FakeIMUUtilities(opMode,"IMU_1");
        opMode.setIMUUtilities(imuUtilities);
        opMode.controller1 = new Controller(opMode.gamepad1);
        elapsedTime = new ElapsedTime();

        // Interactive Init settings
        opMode.setDriveSpeed(1.0);
        opMode.setDropStones(true);
        opMode.setParkInner(false);
        opMode.setSimpleAuto(false);


        robotStateContext = new RobotStateContext(opMode,Color.Ftc.RED, RobotHardware.StartPosition.FIELD_LOADING);
        robotStateContext.init();

        simTime = 0.0;
    }

    @Test
    public void createAutoDrive() {
        assertThat(autoDrive).isNotNull();
        // Test Motion command for errors
        autoDrive.rotateThenDriveToPosition(new Navigation2D(0,0,0), 1.0);
    }

    SimulationTime simulationTime = new SimulationTime();
    @Test
    public void simulateAutoDrive() {
        simulationTime.setTime(0);

        boolean realtime = true;
        boolean notStopped = true;
        double previousTime = 0;
        double deltaTime = 0;
        double simulationEndTime = 50;
        double simulationStepTime = 0.02;
        double driveSpeed = 0.5;
        boolean isNewState = true;
        String previousState = "";

        while (simulationTime.time() <= simulationEndTime && notStopped) {
            robotStateContext.update();


            isNewState = !previousState.equals(robotStateContext.getCurrentState());
            previousState = robotStateContext.getCurrentState();
            if (realtime) {
                simulationStepTime = deltaTime;
            }
            if(isNewState || isDisplayInterval(2, simTime,simulationStepTime)) {
                System.out.print("SimTime:  " + df.format(simTime) + "    " + "dt: " + df_prec.format(deltaTime) + "   ");
                System.out.print(robotStateContext.getCurrentState());
                System.out.println(mecanumNavigation.currentPosition.toString());
                notStopped = !(robotStateContext.getCurrentState().startsWith("Stop_State") || robotStateContext.getCurrentState().startsWith("A_Manual"));
//                System.out.println();
            }

            mecanumNavigation.update();
            opMode.updateAndIntegrateFakeOpMode(simTime);


            if(realtime) {
                previousTime = simulationTime.time();
                simulationTime.setTime(elapsedTime.time());
                deltaTime = simulationTime.time() - previousTime;

            } else {
                simulationTime.incrementTime(simulationStepTime);
            }
            simTime = simulationTime.time();
        }

    }

    private boolean isDisplayInterval(int displaysPerSecond, double simTime, double simulationTimeStep) {
        int simStep = (int) Math.floor( simTime/simulationTimeStep );
        int stepsPerSecond = (int) Math.floor(1/simulationTimeStep);
        int stepsPerDisplay = stepsPerSecond / displaysPerSecond;
        return (simStep % stepsPerDisplay) == 0;
    }


    class SimulationTime {
        private double time = 0.0;

        public void setTime(double time) {
            this.time = time;
        }

        public void incrementTime(double timeStep) {
            this.time += timeStep;
        }

        public double time() {
            return time;
        }
    }
}
