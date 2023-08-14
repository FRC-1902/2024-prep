package frc.robot.states;

import frc.robot.statemachine.State;
import frc.robot.subsystems.Swerve;
import frc.robot.autos.exampleAuto;
import frc.robot.statemachine.RobotStateManager;

public class AutoState implements State{
    private String name;
    private String parent;
    
    public AutoState(String name, String parent){
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
    public void enter() {
        new exampleAuto(Swerve.getInstance());
    }

    @Override
    public void leave() {}

    @Override
    public void periodic(RobotStateManager rs) {}
}
