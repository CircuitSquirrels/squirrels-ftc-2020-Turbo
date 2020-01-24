package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;

import static org.firstinspires.ftc.teamcode.RobotHardware.StartPosition.*;
import static org.firstinspires.ftc.teamcode.RobotHardware.ClawPositions.*;
import static org.firstinspires.ftc.teamcode.Utilities.Executive.StateMachine.StateType.*;
import static org.firstinspires.ftc.teamcode.Utilities.Waypoints.LocationBuild.FOUNDATION;
import static org.firstinspires.ftc.teamcode.Utilities.Waypoints.LocationLoading.*;

public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    // Required class variables
    private AutoOpmode opMode;
    private Executive.StateMachine<AutoOpmode> stateMachine;
    private Color.Ftc teamColor;
    private RobotHardware.StartPosition startPosition;
    private Waypoints waypoints;

    // Interactive Init booleans
    private double driveSpeed;
    private boolean simple;
    private boolean parkInner;
    private boolean dropStones;
    private boolean foundation;
    private boolean conservativeRoute;

    private boolean manualEnd = true;

    private final int liftRaised = 1500;
    private final int liftLowered = 0;
    private final double courseTolerance = 0.5;
    private final double liftSpeed = 1.0;
    private final double wallStoneSpeed = 0.5;
    private final double foundationDragSpeed = 0.5;

    // Delays
    private final double start_Delay = 0.25;
    private final double scan_Delay = 0.5;
    private final double placeFoundation_Delay = 0.25;

    private Controller controller1;


    public RobotStateContext(AutoOpmode opMode, Color.Ftc teamColor, AutoOpmode.StartPosition startPosition) {
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        this.stateMachine = new Executive.StateMachine(opMode);
        this.waypoints = new Waypoints(teamColor);
        stateMachine.update();

        controller1 = opMode.controller1;
    }

    public void init() {
        stateMachine.changeState(DRIVE, new Start_State());

        stateMachine.init();

        this.driveSpeed = opMode.DriveSpeed.get();
        this.simple = opMode.SimpleAuto.get();
        this.parkInner = opMode.ParkInner.get();
        this.dropStones = opMode.DropStones.get();
        this.foundation = opMode.Foundation.get();
        this.conservativeRoute = opMode.ConservativeRoute.get();
    }



    public void update() {
        stateMachine.update();
        opMode.updateMecanumHeadingFromGyroNow();
    }

    public String getCurrentState() {
        return stateMachine.getCurrentStates();
    }
    
    

    /**
     * Define Concrete State Classes
     */

    // Fork between Build and Load side starting positions.
    class Start_State extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            switch (startPosition) {
                case FIELD_LOADING:
                    setupInitialPosition(waypoints.loading.get(Waypoints.LocationLoading.INITIAL_POSITION));
                    nextState(DRIVE, new Start_Loading_Side());
                    break;
                case FIELD_BUILD:
                    setupInitialPosition(waypoints.building.get(Waypoints.LocationBuild.INITIAL_POSITION));
                    nextState(DRIVE, new Start_Building_Side());
                    break;
                default:
                   throw new IndexOutOfBoundsException("Field Position must be either "+FIELD_LOADING.name()+" or " + FIELD_BUILD.name());
            }
        }

        private void setupInitialPosition(Navigation2D initialPosition) {
            opMode.mecanumNavigation.setCurrentPosition(initialPosition);
            opMode.imuUtilities.updateNow();
            opMode.imuUtilities.setCompensatedHeading(radiansToDegrees(initialPosition.theta));
        }
    }
    /**
     * Loading Drive State
     * The initial start state
     */
    class Start_Loading_Side extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            if(stateTimer.seconds() > start_Delay && simple)
                nextState(DRIVE, new Simple_Start(), opMode.shouldContinue());
            else if(stateTimer.time() > start_Delay)
                nextState(DRIVE, new Scan_Position(), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * The state for scanning the first 3 stones to identify the skystone, uses EasyOpenCV. Does not move.
     */
    class Scan_Position extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            nextArmState(OPEN, liftRaised, false);
        }

        @Override
        public void update() {
            super.update();
            if(!isArmArrived()) return;

            if (opMode.skystoneDetector == null)
                waypoints.setSkystoneDetectionPosition(0);
            else
                waypoints.setSkystoneDetectionPosition(opMode.skystoneDetector.getSkystoneIndex());

            // Check if the robot has had enough time to scan
            if (stateTimer.seconds() > scan_Delay)
                nextState(DRIVE, new Align_Skystone(0), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * The state for aligning in-front of the detected skystone
     */
    class Align_Skystone extends Executive.StateBase<AutoOpmode> {

        Align_Skystone(int iteration) {
            super(iteration);
        }

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            nextArmState(OPEN, liftLowered, false);
        }

        @Override
        public void update() {
            super.update();
            switch (getIteration()) {
                case 0:
                    arrived = driveTo(waypoints.loading.get(ALIGNMENT_POSITION_A),
                            getDriveScale(stateTimer.seconds()) * driveSpeed, courseTolerance);
                    break;
                case 1:
                    if(waypoints.getSkystoneDetectionPosition() == 2)  // Waypoint offset adjusted to match WallStone Alignment state.
                        arrived = driveTo(waypoints.loading.get(ALIGNMENT_POSITION_B).addAndReturn(15, 0, 0),
                                getDriveScale(stateTimer.seconds()) * driveSpeed);
                    else
                        arrived = driveTo(waypoints.loading.get(ALIGNMENT_POSITION_B),
                                getDriveScale(stateTimer.seconds()) * driveSpeed, courseTolerance);
                    break;
                case 2:
                    arrived = driveTo(waypoints.loading.get(ALIGN_EXTRA_STONE_A),
                            getDriveScale(stateTimer.seconds()) * driveSpeed, courseTolerance);
                    break;
                case 3:
                    arrived = driveTo(waypoints.loading.get(ALIGN_EXTRA_STONE_B),
                            getDriveScale(stateTimer.seconds()) * driveSpeed, courseTolerance);
                    break;
                default:
                    throw new IndexOutOfBoundsException("Skystone Alignment Index out of bounds.");
            }
            if (!arrived) return;

            if(waypoints.getSkystoneDetectionPosition() == 2 && getIteration() == 1)
                nextState(DRIVE, new WallStone_Alignment(), opMode.shouldContinue());
            else
                nextState(DRIVE, new Grab_Skystone(getIteration()), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * The state for grabbing the detected skystone and/or regular stones.
     */
    class Grab_Skystone extends Executive.StateBase<AutoOpmode> {

        Grab_Skystone(int iteration) {
            super(iteration);
        }


        @Override
        public void update() {
            super.update();
            switch (getIteration()) {
                case 0:
                    arrived = driveTo(waypoints.loading.get(GRAB_SKYSTONE_A),
                            getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                case 1:
                    arrived = driveTo(waypoints.loading.get(GRAB_SKYSTONE_B),
                            getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                case 2:
                    arrived = driveTo(waypoints.loading.get(GRAB_EXTRA_STONE_A),
                            getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                case 3:
                    arrived = driveTo(waypoints.loading.get(GRAB_EXTRA_STONE_B),
                            getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                default:
                    throw new IndexOutOfBoundsException("Skystone Grab Index was out of bounds.");
            }
            if(!arrived) return;

            if(!armStateSet)
                nextArmState(CLOSED, liftLowered, true);

            if(isArmArrived())
                nextState(DRIVE, new Backup_Skystone(getIteration()), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * The state for backing up from the detected skystone to avoid knocking other stones over
     */
    class Backup_Skystone extends Executive.StateBase<AutoOpmode> {

        Backup_Skystone(int iteration) {
            super(iteration);
        }

        @Override
        public void update() {
            super.update();
            if (!armStateSet)
                nextArmState(CLOSED, liftRaised, false);

            if (!isArmArrived()) return;
            switch (getIteration()) {
                case 0:
                    arrived = driveTo(waypoints.loading.get(ALIGNMENT_POSITION_A), getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                case 1:
                    arrived = driveTo(waypoints.loading.get(ALIGNMENT_POSITION_B), getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                case 2:
                    arrived = driveTo(waypoints.loading.get(ALIGN_EXTRA_STONE_A), getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                case 3:
                    arrived = driveTo(waypoints.loading.get(ALIGN_EXTRA_STONE_B), getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                default:
                    throw new IndexOutOfBoundsException("Skystone Backup Index was out of bounds.");
            }
            if (!arrived) return;

            if (dropStones)
                nextState(DRIVE, new Build_Zone(getIteration()), opMode.shouldContinue());
            else
                nextState(DRIVE, new Align_Foundation(getIteration()), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * The state for driving to the build zone
     */
    class Build_Zone extends Executive.StateBase<AutoOpmode> {

        Build_Zone(int iteration) {
            super(iteration);
        }

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            nextArmState(CLOSED, 800, false);
        }

        @Override
        public void update() {
            super.update();
            arrived = driveTo(waypoints.loading.get(BUILD_ZONE),
                    getDriveScale(stateTimer.seconds()) * driveSpeed, courseTolerance);

            if(!arrived) return;

            nextState(DRIVE, new Drop_Skystone(getIteration()), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * The state for dropping the skystone and then driving to the next stone or parking
     */
    class Drop_Skystone extends Executive.StateBase<AutoOpmode> {

        Drop_Skystone(int iteration) {
            super(iteration);
        }

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            nextArmState(OPEN, liftLowered, false);
        }

        @Override
        public void update() {
            super.update();

            switch (getIteration()) {
                case 0: case 1: case 2:
                    nextState(DRIVE, new Align_Skystone(getIteration() + 1), opMode.shouldContinue());
                    break;
//                    if (!conservativeRoute) {
//                        nextState(DRIVE, new Align_Skystone(getIteration() + 1), opMode.shouldContinue());
//                        break;
//                    }
                case 3:
                    if(parkInner)
                        nextState(DRIVE, new Align_Inner(), opMode.shouldContinue());
                    else
                        nextState(DRIVE, new Align_Outer(), opMode.shouldContinue());
                    break;
                default:
                    throw new IndexOutOfBoundsException("Drop Skystone Index was out of bounds.");
            }
        }
    }
    /**
     * Loading Drive State
     * Aligns with the foundation for placing
     */
    class Align_Foundation extends Executive.StateBase<AutoOpmode> {

        Align_Foundation(int iteration) {
            super(iteration);
        }

        @Override
        public void update() {
            super.update();
            //Todo Add offset class vars
            switch (getIteration()) {
                case 0:
                    arrived = driveTo(waypoints.loading.get(FOUNDATION_ALIGNMENT).addAndReturn(12, 0, 0),
                            getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                case 1:
                    arrived = driveTo(waypoints.loading.get(FOUNDATION_ALIGNMENT).addAndReturn(-12, 0, 0),
                            getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                case 2:
                    arrived = driveTo(waypoints.loading.get(FOUNDATION_ALIGNMENT),
                            getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                default:
                    throw new IndexOutOfBoundsException("Align Foundation index out of bounds");
            }

            if(!arrived) return;

           nextState(DRIVE, new Place_Foundation(getIteration()), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * Aligns with the wallstone for easy grabbing
     */
    class WallStone_Alignment extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            if(teamColor.equals(Color.Ftc.BLUE))
                arrived = driveTo(waypoints.loading.get(ALIGNMENT_POSITION_B).addAndReturn(15, 0, degreesToRadians(-30)),
                        getDriveScale(stateTimer.seconds()) * driveSpeed);
            else
                arrived = driveTo(waypoints.loading.get(ALIGNMENT_POSITION_B).addAndReturn(15, 0, degreesToRadians(30)),
                        getDriveScale(stateTimer.seconds()) * driveSpeed);

            if(!arrived) return;

            nextState(DRIVE, new Grab_WallStone(), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * Grabs the wallstone
     */
    class Grab_WallStone extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            if(teamColor.equals(Color.Ftc.BLUE))
                arrived = driveTo(waypoints.loading.get(GRAB_SKYSTONE_B).addAndReturn(7, -5, degreesToRadians(-30)),
                        getDriveScale(stateTimer.seconds()) * wallStoneSpeed);
            else
                arrived = driveTo(waypoints.loading.get(GRAB_SKYSTONE_B).addAndReturn(3, 3, degreesToRadians(30)),
                        getDriveScale(stateTimer.seconds()) * wallStoneSpeed);

            if(!arrived) return;

            if(!armStateSet)
                nextArmState(CLOSED, liftLowered, true);

            if(!isArmArrived()) return;

            nextState(DRIVE, new Backup_WallStone(), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * Backup from wallstone to avoid knocking stones over
     */
    class Backup_WallStone extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            nextArmState(CLOSED, liftRaised, false);
        }

        @Override
        public void update() {
            super.update();
            if(!isArmArrived()) return;

            arrived = driveTo(waypoints.loading.get(ALIGNMENT_POSITION_B).addAndReturn(3, 0, 0),
                    getDriveScale(stateTimer.seconds()) * driveSpeed);

            if (!arrived) return;

            if (dropStones)
                nextState(DRIVE, new Build_Zone(1), opMode.shouldContinue());
            else
                nextState(DRIVE, new Align_Foundation(1), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * Places stone on foundation
     */
    class Place_Foundation extends Executive.StateBase<AutoOpmode> {

        Place_Foundation(int iteration) {
            super(iteration);
        }

        @Override
        public void update() {
            super.update();
            switch (getIteration()) {
                case 0:
                    arrived = driveTo(waypoints.loading.get(FOUNDATION_DROP_OFF).addAndReturn(12, 0, 0),
                            getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                case 1:
                    arrived = driveTo(waypoints.loading.get(FOUNDATION_DROP_OFF).addAndReturn(-12, 0, 0),
                            getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                case 2:
                    arrived = driveTo(waypoints.loading.get(FOUNDATION_DROP_OFF),
                            getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                default: throw new IndexOutOfBoundsException("Place Foundation index out of bounds");
            }
            if(!arrived) return;

            if(!armStateSet) {
                nextState(ARM, new Place_On_Foundation(1));
                stateTimer.reset();
            }

            if(!isArmArrived() || !(stateTimer.seconds() > placeFoundation_Delay)) return;

            if(foundation && (getIteration() == 2) || (getIteration() == 1 && conservativeRoute))
                nextState(DRIVE, new Drag_Foundation(), opMode.shouldContinue());
            else
                nextState(DRIVE, new Backup_Foundation(getIteration()), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * Backups up from foundation to avoid hitting it
     */
    class Backup_Foundation extends Executive.StateBase<AutoOpmode> {

        Backup_Foundation(int iteration) {
            super(iteration);
        }

        @Override
        public void update() {
            super.update();
            switch (getIteration()) {
                case 0:
                    arrived = driveTo(waypoints.loading.get(FOUNDATION_ALIGNMENT).addAndReturn(12, 0, 0),
                            getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                case 1:
                    if(!conservativeRoute)
                        arrived = driveTo(waypoints.loading.get(FOUNDATION_ALIGNMENT).addAndReturn(-12, 0, 0),
                                getDriveScale(stateTimer.seconds()) * driveSpeed);

                case 2:
                    arrived = driveTo(waypoints.loading.get(FOUNDATION_ALIGNMENT),
                            getDriveScale(stateTimer.seconds()) * driveSpeed);
                    break;
                default: throw new IndexOutOfBoundsException("Place Foundation index out of bounds");
            }
            if(!arrived) return;

            nextArmState(VERTICAL, null, false);
            switch (getIteration()) {
                case 0:
                    nextState(DRIVE, new Align_Skystone(1), opMode.shouldContinue());
                    break;
                case 1:
                    if(!conservativeRoute) {
                        // After 2nd (index 1) stone is placed, only go for the 3rd if conservativeRoute is false.
                        nextState(DRIVE, new Align_Skystone(2), opMode.shouldContinue());
                        break;
                    }
                case 2:
                    if(parkInner)
                        nextState(DRIVE, new Align_Inner(), opMode.shouldContinue());
                    else
                        nextState(DRIVE, new Align_Outer(), opMode.shouldContinue());
                    break;
                default:
                    throw new IndexOutOfBoundsException("Backup Foundation index out of bounds.");
            }
        }
    }
    /**
     * Loading Drive State
     * Drags foundation to build site
     */
    class Drag_Foundation extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            nextArmState(OPEN, liftLowered, true);
        }

        @Override
        public void update() {
            super.update();
            //Todo Check if delay is necessary, if not remove it!
            if(!isArmArrived() && stateTimer.seconds() > 0.6) return;

            arrived = driveTo(waypoints.loading.get(DRAG_FOUNDATION_OUTSIDE_WALL),
                    getDriveScale(stateTimer.seconds()) * foundationDragSpeed);
            if(!arrived) return;

            nextArmState(OPEN, liftRaised, true);
            // Reset robot's current position to compensate for slippage from dragging foundation.
            opMode.mecanumNavigation.setCurrentPosition(waypoints.loading.get(DRAG_FOUNDATION_INSIDE_WALL));
            nextState(DRIVE, new Foundation_End(), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * Gets rid of the arm states
     */
    class Foundation_End extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            nextArmState(VERTICAL, liftRaised, false);
        }
    }
    /**
     * Loading Drive State
     * Strafes away from the foundation for parking
     */
    class Strafe_From_Foundation extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            nextArmState(VERTICAL, liftRaised, false);
        }

        @Override
        public void update() {
            super.update();
            arrived = driveTo(waypoints.loading.get(STRAFE_AWAY_FROM_FOUNDATION),
                    getDriveScale(stateTimer.seconds()) * driveSpeed);
            if(!arrived) return;

            if(parkInner)
                nextState(DRIVE, new Park_Inner(), opMode.shouldContinue());
            else
                nextState(DRIVE, new Park_Outer(), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * Aligns outer parking area
     */
    class Align_Outer extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            arrived = driveTo(waypoints.loading.get(BRIDGE_ALIGNMENT_OUTER),
                    getDriveScale(stateTimer.seconds()) * driveSpeed);
            if(!arrived) return;

            nextState(DRIVE, new Park_Outer(), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * Parks on outer side of skybridge
     */
    class Park_Outer extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            nextArmState(VERTICAL, liftLowered, false);
        }

        @Override
        public void update() {
            super.update();
            arrived = driveTo(waypoints.loading.get(PARK_OUTER),
                    getDriveScale(stateTimer.seconds()) * driveSpeed);
            if(!arrived) return;

            if(manualEnd)
                nextState(DRIVE, new A_Manual());
            else
                nextState(DRIVE, new Stop_State(), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * Aligns at the inner side of the sky bridge
     */
    class Align_Inner extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            arrived = driveTo(waypoints.loading.get(BRIDGE_ALIGNMENT_INNER),
                    getDriveScale(stateTimer.seconds()) * driveSpeed);
            if(!arrived) return;

            nextState(DRIVE, new Park_Inner(), opMode.shouldContinue());
        }
    }
    /**
     * Loading Drive State
     * Parks on inner side of skybridge
     */
    class Park_Inner extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            nextArmState(VERTICAL, liftLowered, false);
        }

        @Override
        public void update() {
            super.update();
            arrived = driveTo(waypoints.loading.get(PARK_INNER),
                    getDriveScale(stateTimer.seconds()) * driveSpeed);
            if(!arrived) return;

            if(manualEnd)
                nextState(DRIVE, new A_Manual());
            else
                nextState(DRIVE, new Stop_State(), opMode.shouldContinue());
        }
    }


    class Arm_Control extends Executive.StateBase<AutoOpmode> {
        RobotHardware.ClawPositions positions;
        Integer liftPos;
        boolean wait;

        Arm_Control(RobotHardware.ClawPositions clawPositions, Integer liftTicks, boolean waitForArm) {
            this.positions = clawPositions;
            this.liftPos = liftTicks;
            this.wait = waitForArm;
        }

        @Override
        public void update() {
            super.update();
            arrived = armControl(positions, liftPos, wait);
        }

        @Override
        public String getAuxData() {
//            return " " + positions.toString() + " " + String.valueOf(liftPos) + " wait: " + String.valueOf(wait);
            return " " + positions.toString() + " " + String.valueOf(liftPos) + " " + String.valueOf(wait);
        }
    }

    private boolean armControl(RobotHardware.ClawPositions clawPositions, Integer liftTicks, boolean waitForArm) {
        boolean arrived;
        liftTicks = liftTicks == null ? opMode.getEncoderValue(RobotHardware.MotorName.LIFT_WINCH) : liftTicks;

        if(waitForArm) {
            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, liftTicks, liftSpeed);
            if(!arrived) return false;
            opMode.commandClaw(clawPositions);
        } else {
            opMode.commandClaw(clawPositions);
            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, liftTicks, liftSpeed);
        }
        return arrived;
    }

    public void nextArmState(RobotHardware.ClawPositions clawPositions, Integer liftTicks, boolean waitForArm) {
        stateMachine.getStateReference(DRIVE).nextState(ARM,  new Arm_Control(clawPositions, liftTicks, waitForArm));
    }

    class Place_On_Foundation extends Executive.StateBase<AutoOpmode> {
        int foundationLevel;

        Place_On_Foundation(int Foundation_Level) {
            this.foundationLevel = Foundation_Level;
        }

        @Override
        public void update() {
            super.update();
            arrived = armControl(OPEN, opMode.liftArmTicksForLevelFoundationKnob(foundationLevel, true, true), true);
        }
    }

    /**
     * Simple Autonomous states
     */

    class Simple_Start extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            if(opMode.shouldContinue()) {
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Simple_Align());
            }
        }
    }

    class Simple_Align extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            if(parkInner) arrived = driveTo(waypoints.loading.get(SIMPLE_ALIGNMENT_INNER), getDriveScale(stateTimer.seconds()) * driveSpeed);
            else arrived = true;
            if(arrived) {
                if(opMode.shouldContinue()) {
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Simple_Park());
                }
            }
        }
    }

    class Simple_Park extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            if(parkInner) arrived = driveTo(waypoints.loading.get(PARK_INNER), getDriveScale(stateTimer.seconds()) * driveSpeed);
            else arrived = driveTo(waypoints.loading.get(PARK_OUTER), getDriveScale(stateTimer.seconds()) * driveSpeed);
            if(arrived) {
                if(opMode.shouldContinue()) {
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Stop_State());
                }
            }
        }
    }

    class A_Manual extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.updateMecanumHeadingFromGyroNow();
            opMode.telemetry.clear();
            stateMachine.removeStateType(ARM);
        }

        @Override
        public void update() {
            super.update();

            opMode.setDriveForSimpleMecanum(controller1.left_stick_x * 0.2, controller1.left_stick_y * 0.2,
                    controller1.right_stick_x * 0.2, controller1.right_stick_y * 0.2);

            opMode.setPower(RobotHardware.MotorName.LIFT_WINCH, controller1.right_stick_y);

            if(controller1.rightBumper())
                opMode.commandClaw(OPEN);
            else if(controller1.leftBumper())
                opMode.commandClaw(CLOSED);
            
            for (RobotHardware.MotorName m : RobotHardware.MotorName.values()) {
                opMode.telemetry.addData(m.name() + ": ", opMode.getEncoderValue(m));
            }
        }
    }

    class Stop_State extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            opMode.stopAllMotors();
            stateMachine.removeStateType(ARM);
        }
    }


    class Start_Building_Side extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            if(simple) {
                stateMachine.changeState(DRIVE, new Simple_Park());
            } else {

            }
        }
    }

    class Align_With_Foundation extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            nextArmState(VERTICAL, liftRaised, false);
        }

        @Override
        public void update() {
            super.update();

        }
    }

    public double getDriveScale(double seconds) {
        double driveScale;
        double speedDivider = 0.75; // No idea what a good name is
        driveScale = seconds / speedDivider;
        driveScale = driveScale < 1.0 ? driveScale : 1.0;
        return driveScale;
    }


    private double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    private double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }
}
