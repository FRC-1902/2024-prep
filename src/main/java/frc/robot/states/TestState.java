package frc.robot.states;

import frc.robot.statemachine.State;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.statemachine.RobotStateManager;

public class TestState implements State{
    private String name;
    private String parent;
    private SwerveModule swerveModule1;
    private SwerveModuleState forwardStationaryState;
    
    public TestState(String name, String parent){
        this.name = name;
        this.parent = parent;
        swerveModule1 = new SwerveModule(1, Constants.Swerve.Mod1.constants);
        forwardStationaryState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
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
    public void periodic(RobotStateManager rs) {
        DataLogManager.log(String.format("Angle: %.3f", swerveModule1.getCanCoder().getDegrees()));
        // swerveModule1.setDesiredState(forwardStationaryState, false);
    }
}
