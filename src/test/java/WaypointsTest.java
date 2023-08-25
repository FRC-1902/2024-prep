import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.Waypoints;


public class WaypointsTest {
    private static final String FILEPATH = System.getProperty("user.dir") + "/src/test/java/pathplanner/generatedCSV/";
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
        double pos0   = waypoints.facePoint(new Translation2d(0.0, 0.0), new Translation2d(1.0, 0.0)).getDegrees();
        double pos45  = waypoints.facePoint(new Translation2d(1.8, 1.8), new Translation2d(2.0, 2.0)).getDegrees();
        double pos90  = waypoints.facePoint(new Translation2d(0.0, 1.0), new Translation2d(0.0, 1.5)).getDegrees();
        double pos135 = waypoints.facePoint(new Translation2d(-5.0, -5.0), new Translation2d(-10, 0)).getDegrees();
        double neg45  = waypoints.facePoint(new Translation2d(0.0, 0.0), new Translation2d(2.0, -2.0)).getDegrees();
        double neg140 = waypoints.facePoint(new Translation2d(1.0, 1.0), new Translation2d(1.0 - 0.7660, 1.0 - 0.6428)).getDegrees();
        
        assertEquals(0.0, pos0, DELTA);
        assertEquals(45.0, pos45, DELTA);
        assertEquals(90.0, pos90, DELTA);
        assertEquals(135.0, pos135, DELTA);
        assertEquals(-45.0, neg45, DELTA);
        assertEquals(-140.0, neg140, DELTA);
    }

    @Test
    void findClosestVelocityTest() {
        //TODO: write me
    }

    @Test 
    void findClosestPose() {
        Rotation2d rot = Rotation2d.fromDegrees(0.0);
        
        Pose2d a = waypoints.findClosestPose(new Pose2d(0.0, 0.0, rot));
        Pose2d b = waypoints.findClosestPose(new Pose2d(-10.0, -1.0, rot));
        Pose2d c = waypoints.findClosestPose(new Pose2d(0.2, 0.2, rot));
        Pose2d d = waypoints.findClosestPose(new Pose2d(3.0, 0.6, rot));
        Pose2d e = waypoints.findClosestPose(new Pose2d(0.05, 0.15, rot));

        assertEquals(0.0, a.getX(), DELTA);
        assertEquals(0.0, a.getY(), DELTA);
        assertEquals(0.0, b.getX(), DELTA);
        assertEquals(0.0, b.getY(), DELTA);
        assertEquals(0.2, c.getX(), DELTA);
        assertEquals(0.2, c.getY(), DELTA);
        assertEquals(0.2, d.getX(), DELTA);
        assertEquals(0.2, d.getY(), DELTA);
        assertEquals(0.1, e.getX(), DELTA);
        assertEquals(0.1, e.getY(), DELTA);
    }

    @Test
    void findLookaheadTest() {
        //TODO: write me
        
    }
}
