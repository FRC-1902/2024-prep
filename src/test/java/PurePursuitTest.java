import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.Waypoints;


public class PurePursuitTest {
    private static final String FILEPATH = System.getProperty("user.dir") + "/src/test/pathplanner/generatedCSV/";
    static final double DELTA = 1e-2;
    private Waypoints waypoints;

    @BeforeEach
    void setup() {
        waypoints = new Waypoints(FILEPATH + "UnitTest.csv");
    }

    @AfterEach
    void shutdown() {}

    @Test
    void facePointTest() {
        //irrelevent for this test, just needed for pose construction
        Rotation2d rot = new Rotation2d(1, 0); 

        double pos0   = waypoints.facePoint(new Pose2d(0.0, 0.0, rot), new Pose2d(1.0, 0.0, rot)).getDegrees();
        double pos45  = waypoints.facePoint(new Pose2d(1.8, 1.8, rot), new Pose2d(2.0, 2.0, rot)).getDegrees();
        double pos90  = waypoints.facePoint(new Pose2d(0.0, 1.0, rot), new Pose2d(0.0, 1.5, rot)).getDegrees();
        double pos135 = waypoints.facePoint(new Pose2d(-5.0, -5.0, rot), new Pose2d(-10, 0, rot)).getDegrees();
        double neg45  = waypoints.facePoint(new Pose2d(0.0, 0.0, rot), new Pose2d(2.0, -2.0, rot)).getDegrees();
        double neg140 = waypoints.facePoint(new Pose2d(1.0, 1.0, rot), new Pose2d(1.0 - 0.7660, 1.0 - 0.6428, rot)).getDegrees();
        
        assertEquals(0.0, pos0, DELTA);
        assertEquals(45.0, pos45, DELTA);
        assertEquals(90.0, pos90, DELTA);
        assertEquals(135.0, pos135, DELTA);
        assertEquals(-45.0, neg45, DELTA);
        assertEquals(-140.0, neg140, DELTA);
    }

    @Test
    void distanceBetweenPosesTest() {
        //TODO: write me
    }

    @Test
    void findClosestVelocityTest() {
        //TODO: write me
    }

    @Test
    void findLookaheadTest() {
        //TODO: write me
    }
}
