package org.firstinspires.ftc.teamcode.Utilities;

public class AutoStateMachine {

    StateMachine stateMachine;

    public void StateMachine() {
        this.init();
    }

    public void init() {
        stateMachine = new StateMachine();
        StateBase initialState = new StateDescendLander();
        initialState.init(); // Not required in this case.
        stateMachine.add(initialState);
        stateMachine.init();
    }

    public void update() {
        stateMachine.update();
    }


    private class StateDescendLander extends StateBase {


    }

}
