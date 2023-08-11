package frc.robot.states;

import frc.robot.statemachine.State;
import frc.robot.statemachine.Controllers.Axis;
import frc.robot.statemachine.Controllers.Button;
import frc.robot.statemachine.Controllers.ControllerName;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.statemachine.Controllers;
import frc.robot.statemachine.Event;
import frc.robot.statemachine.RobotStateManager;

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
        double translationVal = MathUtil.applyDeadband(-controllers.get(ControllerName.DRIVE, Axis.LY), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(-controllers.get(ControllerName.DRIVE, Axis.LX), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(-controllers.get(ControllerName.DRIVE, Axis.RX), Constants.stickDeadband);
        boolean isFieldRelative = !controllers.get(ControllerName.DRIVE, Button.LB);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            isFieldRelative, 
            true
        );
    }

    @Override
    public boolean handleEvent(Event event, RobotStateManager rs){
        if(event.button == Button.Y) {
            s_Swerve.zeroGyro(); //TODO: fix me
            return true;
        }
        return false;
    }
}
