package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Utility class for simulating Limelight data in the WPILib simulator.
 * 
 * This allows you to test Limelight-dependent code without actual hardware:
 * - Mock AprilTag detection
 * - Simulate realistic pose data
 * - Test alignment logic with synthetic data
 * 
 * Usage in simulation:
 *   LimelightSimulation.init();  // Call this once during robotInit()
 *   LimelightSimulation.setSimulatedRobotPose(tagId, x, y, angle);
 */
public class LimelightSimulation {
    private static final String TABLE_NAME = "limelight";
    private static NetworkTable limelightTable;
    private static boolean isInitialized = false;

    /**
     * Initializes the Limelight simulation.
     * Must be called once during robot initialization.
     */
    public static void init() {
        if (isInitialized) {
            return;
        }

        try {
            limelightTable = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
            isInitialized = true;
            System.out.println("LimelightSimulation initialized for testing");
        } catch (Exception e) {
            System.err.println("Failed to initialize LimelightSimulation: " + e.getMessage());
        }
    }

    /**
     * Sets a simulated fiducial (AprilTag) pose relative to the robot.
     * 
     * @param tagId The AprilTag ID (e.g., 1-16)
     * @param xMeters Distance forward/backward relative to robot (positive = forward)
     * @param yMeters Distance left/right relative to robot (positive = left)
     * @param zMeters Vertical distance (usually ~0 for on-table tags)
     * @param yawDegrees Rotation of the tag relative to robot (degrees)
     */
    public static void setSimulatedFiducialPose(int tagId, double xMeters, double yMeters, 
                                                 double zMeters, double yawDegrees) {
        if (!isInitialized) {
            System.err.println("LimelightSimulation not initialized. Call init() first.");
            return;
        }

        try {
            // Convert degrees to radians
            double yawRadians = Math.toRadians(yawDegrees);

            // Create a JSON-like structure that mimics Limelight's output
            // Format: [fID, robotPose_TargetSpace_x, robotPose_TargetSpace_y, 
            //          robotPose_TargetSpace_z, robotPose_TargetSpace_roll, 
            //          robotPose_TargetSpace_pitch, robotPose_TargetSpace_yaw, ...]
            double[] fiducialData = new double[]{
                tagId,              // fiducial ID
                xMeters,            // x relative to tag
                yMeters,            // y relative to tag
                zMeters,            // z relative to tag
                0,                  // roll (usually 0)
                0,                  // pitch (usually 0)
                yawRadians          // yaw in radians
            };

            // Write to NetworkTables with key format matching actual Limelight
            String entryKey = "Fiducial_" + tagId;
            limelightTable.getDoubleArrayTopic(entryKey).publish().set(fiducialData);

            // Also set the basic "has target" flag
            limelightTable.getBooleanTopic("tv").publish().set(true);

        } catch (Exception e) {
            System.err.println("Error setting simulated fiducial: " + e.getMessage());
        }
    }

    /**
     * Sets a simulated robot pose relative to an AprilTag using a simple distance and angle.
     * This is the easiest way to simulate alignment scenarios.
     * 
     * Example: Robot is 1.5 meters in front of tag ID 2, facing it directly
     *   setSimulatedRobotPose(2, 1.5, 0, 0);
     * 
     * @param tagId The AprilTag ID
     * @param distanceMeters How far the robot is from the tag (positive = away)
     * @param horizontalOffsetMeters How far left/right (positive = left)
     * @param angleToTagDegrees What angle the robot is at relative to the tag (degrees)
     */
    public static void setSimulatedRobotPose(int tagId, double distanceMeters, 
                                              double horizontalOffsetMeters, double angleToTagDegrees) {
        // Convert polar coordinates (distance, angle) to Cartesian (x, y)
        double angleRad = Math.toRadians(angleToTagDegrees);
        double xMeters = distanceMeters * Math.cos(angleRad);
        double yMeters = distanceMeters * Math.sin(angleRad) + horizontalOffsetMeters;

        setSimulatedFiducialPose(tagId, xMeters, yMeters, 0, angleToTagDegrees);
    }

    /**
     * Simulates multiple AprilTags visible at once.
     * Useful for testing tag-selection logic.
     * 
     * @param tagPoses Array of [tagId, distance, horizontalOffset, angle] for each tag
     */
    public static void setSimulatedMultipleTags(double[][] tagPoses) {
        for (double[] pose : tagPoses) {
            int tagId = (int) pose[0];
            double distance = pose[1];
            double offset = pose[2];
            double angle = pose[3];
            setSimulatedRobotPose(tagId, distance, offset, angle);
        }
    }

    /**
     * Clears all simulated fiducials (simulates losing target).
     */
    public static void clearSimulation() {
        if (!isInitialized) {
            return;
        }

        try {
            limelightTable.getBooleanTopic("tv").publish().set(false);
            System.out.println("Cleared Limelight simulation data");
        } catch (Exception e) {
            System.err.println("Error clearing simulation: " + e.getMessage());
        }
    }

    /**
     * Simulates a robot driving forward while maintaining alignment.
     * This smoothly changes the simulated pose over time.
     * 
     * @param tagId The AprilTag to track
     * @param startDistance Starting distance (meters)
     * @param targetDistance Target distance (meters)
     * @param speedMetersPerSecond Simulation speed
     * @param elapsedSeconds How much time has passed
     */
    public static void simulateDrivingToTag(int tagId, double startDistance, double targetDistance,
                                             double speedMetersPerSecond, double elapsedSeconds) {
        double currentDistance = startDistance - (speedMetersPerSecond * elapsedSeconds);
        currentDistance = Math.max(currentDistance, targetDistance); // Don't go past target

        setSimulatedRobotPose(tagId, currentDistance, 0, 0);
    }

    /**
     * Gets the current simulated robot pose for a tag.
     * Useful for reading back what you set.
     * 
     * @param tagId The tag ID to query
     * @return Pose2d of the robot relative to the tag, or null if not set
     */
    public static Pose2d getSimulatedPose(int tagId) {
        if (!isInitialized) {
            return null;
        }

        try {
            String entryKey = "Fiducial_" + tagId;
            double[] data = limelightTable.getDoubleArrayTopic(entryKey).getEntry(new double[0]).get();
            
            if (data.length >= 7) {
                double x = data[1];
                double y = data[2];
                double yaw = data[6];
                return new Pose2d(x, y, new Rotation2d(yaw));
            }
        } catch (Exception e) {
            System.err.println("Error reading simulated pose: " + e.getMessage());
        }

        return null;
    }

    /**
     * Prints current simulated Limelight state to console.
     * Useful for debugging.
     */
    public static void debugPrintSimulatedState() {
        if (!isInitialized) {
            System.err.println("Simulation not initialized");
            return;
        }

        System.out.println("\n=== Simulated Limelight State ===");
        try {
            boolean hasTarget = limelightTable.getBooleanTopic("tv").getEntry(false).get();
            System.out.println("Has Target: " + hasTarget);
            
            // You could add more debug output here
        } catch (Exception e) {
            System.err.println("Error printing state: " + e.getMessage());
        }
    }
}
