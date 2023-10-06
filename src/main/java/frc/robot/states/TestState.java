package frc.robot.states;

import frc.lib.statemachine.RobotStateManager;
import frc.lib.statemachine.State;

public class TestState implements State{
    private String name;
    private String parent;
    private RobotStateManager rs;
    
    public TestState(String name, String parent, RobotStateManager rs){
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
    public void periodic() {}
}
