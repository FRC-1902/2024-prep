package frc.robot.states;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.lib.statemachine.RobotStateManager;
import frc.lib.statemachine.State;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TestState implements State{
    private String name;
    private String parent;
    private Swerve swerve;
    
    public TestState(String name, String parent){
        this.name = name;
        this.parent = parent;
        swerve = Swerve.getInstance();
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
        swerve.drive(
            new Translation2d(0.2, 0).times(Constants.Swerve.maxSpeed), 
            0, 
            false, 
            true
        );

        SwerveModulePosition[] positions = swerve.getModulePositions();
        int i = 0;
        for(SwerveModulePosition pos: positions){
            DataLogManager.log(String.format("%d: %.3f", i, pos.angle.getDegrees()));
            i ++;
        }
    }
}
