package org.firstinspires.ftc.teamcode.Utilities;

import java.util.ArrayList;

/**
 * Created by HomeStephen on 12/15/17.
 */

public class StateMachine {

    private ArrayList<StateBase> states = new ArrayList<>();

    public void add(StateBase state) {
        states.add(state);
    }

    public void add(ArrayList<StateBase> newStates) {
        for (StateBase state : newStates) {
            add(state);
        }
    }

    public void delete(StateBase state) {
        for (int i = states.size()-1; i >= 0; ++i) {
            if (states.get(i) == state) {
                states.remove(i);
            }
        }
    }

    public void clearDeletedStates() {
        for (int i = states.size() -1; i >= 0; ++i) {
            if (states.get(i).isDeleteRequested()) {
                states.remove(i);
            }
        }
    }

    public void init() {
        for (StateBase state : states) {
            state.init();
        }
    }

    /**
     * Check if state is initialized, and throw exception if not.
     * Then, if nextStates has values, StateMachine.add() them and clear new states.
     * Then, if state.isDeleteRequested(), StateMachine.delete(state)
     * otherwise, run state.update()
     */
    public void update()
    {
        for (StateBase state : states) {
            if (!state.isInitialized()) {
                throw new RuntimeException("ERROR: state called in StateMachine update() was never initialized") ;
            }
            // Load new states
            if (state.getNewStates().size() > 0) {
                add(state.getNewStates());
                state.clearNewStates();
            }
            // Delete or update state
            if (state.isDeleteRequested() == false) {
                state.update();
            }
        }
        clearDeletedStates();
    }

    public void reset() {
        for (StateBase state : states) {
            state.reset();
        }
    }

}
