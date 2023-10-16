package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.Waypoints;
import frc.robot.commands.PurePursuitCommand;

public class AutoSelector {
    private static final String CSV_FILEPATH = RobotBase.isReal() ? "/home/lvuser/deploy/pathplanner/generatedCSV/" : 
        System.getProperty("user.dir") + "/src/main/deploy/pathplanner/generatedCSV/";

    private SendableChooser<Command> autoChooser;
    
    public AutoSelector() {
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Do Nothing", new SequentialCommandGroup());
        autoChooser.addOption("TestAuto1Path", getTestAuto());
        SmartDashboard.putData("Auto Choices", autoChooser);
    }

    /**
     * @return The selected auto from smart dashboard
     */
    public Command getSelectedCommand() {
        return autoChooser.getSelected();
    }

    // Auto definitions

    private SequentialCommandGroup getTestAuto() {
        return new SequentialCommandGroup(
            new PurePursuitCommand(new Waypoints(CSV_FILEPATH + "TestPath1.csv"))
        );
    }
}