package frc.robot.modes;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.PurePursuitFollower;
import frc.lib.util.OperationMode;
import frc.lib.util.Waypoints;

public class AutoMode implements OperationMode{
    private PurePursuitFollower pathing;
    private static final String CSV_FILEPATH = RobotBase.isReal() ? "/home/lvuser/deploy/pathplanner/generatedCSV/" : 
        System.getProperty("user.dir") + "/src/main/deploy/pathplanner/generatedCSV/";

    public AutoMode(){
        pathing = new PurePursuitFollower();
    }

    @Override
    public void enter() {
        pathing.queueWaypoint(
            new Waypoints(CSV_FILEPATH + "TestPath1.csv")
        );
    }

    @Override
    public void exit() {}

    @Override
    public void periodic() {
        if (!pathing.isFinished()) {
            pathing.periodic();
        } else {
            pathing.start();
        }
    }
}
