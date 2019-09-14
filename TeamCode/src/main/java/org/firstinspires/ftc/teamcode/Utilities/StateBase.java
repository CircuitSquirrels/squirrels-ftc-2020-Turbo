package org.firstinspires.ftc.teamcode.Utilities;

import java.util.ArrayList;

/**
 * Created by HomeStephen on 12/15/17.
 */

// The class is abstract, but the internal methods are not abstract so that they can be optionally implemented
public abstract class StateBase {

    private boolean initialized = false;
    private boolean deleteRequested = false;
    private ArrayList<StateBase> newStates;

    public void init() {
        initialized = true;
    }

    public void update() {

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

    public ArrayList<StateBase> getNewStates() {
        return newStates;
    }

    /**
     * Clear nextStates array
     */
    public void clearNewStates() {
        while(newStates.size() >= 0) {
            newStates.remove(0);
        }
    }

}
