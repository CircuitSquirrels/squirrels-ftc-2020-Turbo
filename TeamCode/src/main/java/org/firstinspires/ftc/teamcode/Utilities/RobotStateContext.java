package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.Utilities.Executive.StateMachine.StateType.DRIVE;

public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    AutoOpmode opMode;
    Executive.StateMachine stateMachine;
    Color.Ftc teamColor;
    RobotHardware.StartPosition startPosition;
    Waypoints waypoints;
    double driveSpeed = 0.8;


    public RobotStateContext(AutoOpmode opMode, Color.Ftc teamColor, RobotHardware.StartPosition startPosition) {
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        stateMachine = new Executive.StateMachine(opMode);
        waypoints = new Waypoints(teamColor, startPosition, true);
    }

    public void init() {
        stateMachine.changeState(DRIVE, new Start_State());
//        stateMachine.changeState(Executive.StateMachine.StateType.ARM, new ArmLevelState());
        stateMachine.init();
    }

    public void update() {
        stateMachine.update();
    }

    public String getCurrentState() {
        return stateMachine.getCurrentState();
    }

    /**
     * Define Concrete State Classes
     */



    class Start_State extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            if(stateTimer.seconds() > 1) {
                opMode.mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
                opMode.imuUtilities.updateNow();
                nextState(DRIVE, new Drive_Right());
            }
        }
    }

    class Drive_Right extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            if(stateTimer.seconds() > 1) {
                arrived = opMode.autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(3, 3, 0), driveSpeed);
                opMode.imuUtilities.updateNow();
                if(arrived) {
                    nextState(DRIVE, new Stop_State());
                }
            }
        }
    }


    class Stop_State extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            opMode.stopAllMotors();
        }
    }




}
