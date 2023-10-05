package frc.robot.states;

import frc.lib.statemachine.State;
import frc.lib.statemachine.Controllers.Axis;
import frc.lib.statemachine.Controllers.Button;
import frc.lib.statemachine.Controllers.ControllerName;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.sensors.IMU;
import frc.lib.statemachine.Controllers;
import frc.lib.statemachine.RobotStateManager;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TeleOpState implements State{
    private String name;
    private String parent;
    private final Swerve s_Swerve;
    private Controllers controllers;
    
    public TeleOpState(String name, String parent){
        this.name = name;
        this.parent = parent;
        controllers = Controllers.getInstance();
        s_Swerve = Swerve.getInstance();
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
        double translationVal = MathUtil.applyDeadband(-controllers.get(ControllerName.DRIVE, Axis.LY), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(-controllers.get(ControllerName.DRIVE, Axis.LX), Constants.STICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(-controllers.get(ControllerName.DRIVE, Axis.RX), Constants.STICK_DEADBAND);
        boolean isFieldRelative = !controllers.get(ControllerName.DRIVE, Button.LB);

        //cube controls for better handling, and scale down for softer pre-season movement
        translationVal = Math.pow(translationVal, 3.0);
        translationVal *= 0.5;
        strafeVal = Math.pow(strafeVal, 3.0);
        strafeVal *= 0.5;
        rotationVal = Math.pow(rotationVal, 3.0);
        rotationVal *= 0.8;

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), 
            rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY, 
            isFieldRelative, 
            true
        );

        if(controllers.getPressed(ControllerName.DRIVE, Button.Y)) {
            s_Swerve.zeroGyro();
            System.out.println(IMU.getInstance().getHeading());
        }
    }
}
