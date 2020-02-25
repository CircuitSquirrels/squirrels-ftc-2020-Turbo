package org.firstinspires.ftc.teamcode.Utilities;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * Provides Executive robot Control through class based state machine.
 */
public class Executive {

    /**
     * Interface to be used as the interface for the class which hosts the
     * state machine object as well as the concrete implementations of the required states.
     */
    public interface RobotStateMachineContextInterface {
        void init();
        void update();
        String getCurrentState();
    }

    /**
     * Robot state machine, supporting multiple named simultaneous states, expanded by adding to an enum.
     */
    static public class StateMachine <T_opmode extends RobotHardware> {

        private Map<StateType, StateBase> stateMap = new HashMap<>();
        T_opmode opMode;

        /**
         * State Machine will support one additional concurrent state for each possible StateType.
         * Note that StateMachine.remove(StateType.ARM) would remove an existing ARM state, for
         * example.
         */
        public enum StateType {
            DRIVE,
            ARM,
            LIFT,
        }

        public StateMachine(T_opmode opMode) {
            this.opMode = opMode;
        }

        public void changeState(boolean isActive, StateType stateType, StateBase<T_opmode> state) {
            if (!isActive) return;
            stateMap.put(stateType, state);
            state.init(this);
        }

        public void changeState(StateType stateType, StateBase<T_opmode> state) {
            changeState(true, stateType, state);
        }

        public void removeStateType(StateType stateType) {
            stateMap.remove(stateType);
        }


        public void clearDeletedStates() {
            for (StateType stateType : StateType.values()) {
                if(stateMap.get(stateType) != null) {
                    if (stateMap.get(stateType).isDeleteRequested()) {
                        stateMap.remove(stateType);
                    }
                }
            }
        }

        /**
         * Allows a StateBase to access a reference to another running state, primarily for
         * reading the 'arrived' property.
         * @param stateType
         * @return
         */
        public StateBase getStateReference(StateType stateType) {
            return stateMap.get(stateType);
        }

        // Format state to only contain the class name.
        public String getCurrentStates(StateType stateType) {
            StateBase state = stateMap.get(stateType);
            String stateString;
            try {
                String stateName = state.getClass().getSimpleName();
                if (state.getIteration()  == -1) {
                    stateString = stateName;
                } else {
                    stateString = stateName + ":" + state.getIteration();
                }
                return stateString + state.getAuxData();
            } catch (Exception e){
                return "";
            }
        }

        public String getCurrentStates() {
            String stateString = "";
            Set<StateType> stateTypeSet = stateMap.keySet();
            StateType[] stateTypeKeyArray = stateTypeSet.toArray(new StateType[stateTypeSet.size()]);
            for (StateType type : stateTypeKeyArray) {
                String stateElement = getCurrentStates(type);
                stateString += stateElement + "  ";
            }
            return stateString;
        }

        public void init() {
            Set<StateType> stateTypeSet = stateMap.keySet();
            StateType[] stateTypeKeyArray = stateTypeSet.toArray(new StateType[stateTypeSet.size()]);
            for (StateType type : stateTypeKeyArray) {
                StateBase state = stateMap.get(type);
                if(state != null) {
                    state.init(this);
                }
            }
        }

        /**
         * Check if state is initialized, and throw exception if not.
         * Then, if nextStates has values, StateMachine.add() them and clear new states.
         * Then, if state.isDeleteRequested(), StateMachine.delete(state)
         * otherwise, run state.update()
         */
        public void update() {
            Set<StateType> stateTypeSet = stateMap.keySet();
            StateType[] stateTypeKeyArray = stateTypeSet.toArray(new StateType[stateTypeSet.size()]);
            for (StateType type : stateTypeKeyArray) {
                StateBase state = stateMap.get(type);
                if(state != null) {
                    if (!state.isInitialized()) {
                        throw new RuntimeException("ERROR: state called in StateMachine update() was never initialized");
                    }
                    // Delete or update state
                    if (!state.isDeleteRequested()) {
                        state.update();
                    }
                } else {
                    Log.w("Error", "Statemap null key");
                }
            }
            clearDeletedStates();
        }


        public void reset() {
            Set<StateType> stateTypeSet = stateMap.keySet();
            StateType[] stateTypeKeyArray = stateTypeSet.toArray(new StateType[stateTypeSet.size()]);
            for (StateType type : stateTypeKeyArray) {
                StateBase state = stateMap.get(type);
                state.reset();
            }
        }


    }


    /**
     * State base class.
     */
    // The class is abstract, but the internal methods are not abstract so that they can be optionally implemented
    static public abstract class StateBase <T_opmode extends RobotHardware> {

        StateMachine<T_opmode> stateMachine; // Reference to outer state machine, for modifying states.
        T_opmode opMode;
        ElapsedTime stateTimer; // Time how long state has been active
        ElapsedTime statePeriod; // Time how long since state has been executed.
        double lastStatePeriod = -1;
        boolean arrived = false;
        boolean armStateSet = false;

        private boolean initialized = false;
        private boolean deleteRequested = false;


        public StateBase() {
            this.iteration = -1;
            // Defining default constructor
            // However, we NEED the state machine reference.
            // Handled by allowing init to take a stateMachine argument.
        }

        StateBase(int iteration) {
            this.iteration = iteration;
        }


        public void init(StateMachine<T_opmode> stateMachine) {
            this.stateMachine = stateMachine;
            this.opMode = stateMachine.opMode;
            stateTimer = opMode.getNewElapsedTime();
            statePeriod = opMode.getNewElapsedTime();
            initialized = true;
            stateTimer.reset();
            statePeriod.reset();
        }

        public void update() {
            lastStatePeriod = statePeriod.seconds();
            statePeriod.reset();
        }

        protected void nextState(StateMachine.StateType stateType, StateBase state, boolean isActive) {
            if(stateType.equals(StateMachine.StateType.ARM))
                armStateSet = true;

            stateMachine.changeState(isActive, stateType, state);
            stateMachine.stateMap.get(stateType).init(stateMachine);
        }

        protected void nextState(StateMachine.StateType stateType, StateBase state) {
            nextState(stateType, state, true);
        }

        final private int iteration;
        public int getIteration() {
            return iteration;
        }

        public boolean driveTo(MecanumNavigation.Navigation2D waypoint, double driveSpeed) {
            return opMode.autoDrive.driveToPositionTranslateOnly(waypoint, driveSpeed);
//            return opMode.positionController.driveTo(waypoint, driveSpeed);  // New positioning system
        }

        public boolean driveTo(MecanumNavigation.Navigation2D waypoint, double driveSpeed, double minRate, double tolerance) {
            return opMode.autoDrive.driveToPositionTranslateOnly(waypoint, driveSpeed, minRate, tolerance);
        }

        public boolean driveTo(MecanumNavigation.Navigation2D waypoint, double driveSpeed, double tolerance) {
            return opMode.autoDrive.driveToPositionTranslateOnly(waypoint, driveSpeed, 0.05, tolerance);
        }

        public boolean isArmArrived() {
            StateBase armState = stateMachine.getStateReference(StateMachine.StateType.ARM);
            return armState.arrived;
        }

        public String getAuxData() {
            return "";
        }

        public void reset() {
            initialized = false;
        }

        boolean isInitialized() {
            return initialized;
        }

        public void requestDelete() {
            deleteRequested = true;
        }

        public boolean isDeleteRequested() {
            return deleteRequested;
        }
    }
}
