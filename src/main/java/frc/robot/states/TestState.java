package frc.robot.states;

import frc.robot.statemachine.State;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.lib.sensors.IMU;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.statemachine.RobotStateManager;

public class TestState implements State{
    private String name;
    private String parent;
    private Swerve swerve;
    private IMU imu;
    
    public TestState(String name, String parent){
        this.name = name;
        this.parent = parent;
        swerve = Swerve.getInstance();
        imu = IMU.getInstance();
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
        
    }
}
