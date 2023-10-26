package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.Waypoints;
import frc.robot.commands.PurePursuitCommand;

public class AutoSelector {
    private static final String CSV_FILEPATH = RobotBase.isReal() ? "/home/lvuser/deploy/pathplanner/generatedCSV/" : 
        System.getProperty("user.dir") + "/src/main/deploy/pathplanner/generatedCSV/";

    private LoggedDashboardChooser<Command> autoChooser;
    
    public AutoSelector() {
        autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
        autoChooser.addDefaultOption("Do Nothing", new SequentialCommandGroup());
        autoChooser.addOption("TestAuto1Path", getTestAuto());
        SmartDashboard.putData("Auto Choices", autoChooser.getSendableChooser());
    }

    /**
     * @return The selected auto from smart dashboard
     */
    public Command getSelectedCommand() {
        return autoChooser.get();
    }

    // Auto definitions

    private SequentialCommandGroup getTestAuto() {
        return new SequentialCommandGroup(
            new PurePursuitCommand(new Waypoints(CSV_FILEPATH + "TestPath1.csv"))
        );
    }
}