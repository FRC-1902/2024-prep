package frc.robot.states;

import frc.robot.statemachine.State;
import frc.robot.statemachine.RobotStateManager;

public class TestState implements State{
    private String name;
    private String parent;
    
    public TestState(String name, String parent){
        this.name = name;
        this.parent = parent;
    }
    
    @Override
    public String getName() {
        return name;
    }

    @Override
    public String getParent() {
        return parent;
    }

    @Override
    public void enter() {}

    @Override
    public void leave() {}

    @Override
    public void periodic(RobotStateManager rs) {}
}
