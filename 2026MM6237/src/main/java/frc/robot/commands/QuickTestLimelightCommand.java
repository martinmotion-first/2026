package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightSimulation;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Quick start test command for testing Limelight alignment logic.
 * 
 * This command demonstrates both simulation and real hardware testing:
 * - In simulator: Uses fake Limelight data to test alignment calculation
 * - On robot: Reads real Limelight data
 * 
 * SmartDashboard will show simulated values if running in sim,
 * real values if on actual robot.
 * 
 * Usage:
 *   In RobotContainer.configureBindings():
 *   driver.x().onTrue(new QuickTestLimelightCommand(limelight));
 */
public class QuickTestLimelightCommand extends Command {
    private final LimelightSubsystem limelight;
    private AlignToAprilTagCommand alignmentCommand;
    private int executionCount = 0;

    // Test parameters
    private static final int TARGET_TAG_ID = 2;
    private static final double DESIRED_DISTANCE = 1.5;  // meters
    private static final double DESIRED_ANGLE = 0;       // radians (0 = facing tag)

    public QuickTestLimelightCommand(LimelightSubsystem limelight) {
        this.limelight = limelight;
        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        executionCount = 0;

        System.out.println("\n========== QUICK TEST LIMELIGHT ==========");
        System.out.println("Target Tag ID: " + TARGET_TAG_ID);
        System.out.println("Desired Distance: " + DESIRED_DISTANCE + " m");
        System.out.println("Desired Angle: " + DESIRED_ANGLE + " rad");

        // If running in simulator, set up fake data
        if (Utils.isSimulation()) {
            System.out.println("\n>>> Running in SIMULATOR mode <<<");
            System.out.println("Using simulated Limelight data");
            LimelightSimulation.init();

            // Simulate 4 different scenarios
            System.out.println("\nInitial scenario: Robot is too far away");
            LimelightSimulation.setSimulatedRobotPose(TARGET_TAG_ID, 2.5, 0, 0);
        } else {
            System.out.println("\n>>> Running on REAL ROBOT <<<");
            System.out.println("Aim Limelight at AprilTag " + TARGET_TAG_ID);
        }

        // Create alignment command
        alignmentCommand = new AlignToAprilTagCommand(
            limelight,
            TARGET_TAG_ID,
            DESIRED_DISTANCE,
            DESIRED_ANGLE,
            0.05,  // distance tolerance: 5cm
            0.1    // angle tolerance: ~0.1 radians (6 degrees)
        );

        System.out.println("\nAlignment command created");
        System.out.println("Gains: Distance=0.5, Angle=0.3 (defaults)");
        System.out.println("=========================================\n");
    }

    @Override
    public void execute() {
        // Update alignment command
        alignmentCommand.execute();

        executionCount++;

        // Every 10 cycles (~200ms at 50Hz), print status
        if (executionCount % 10 == 0) {
            printStatus();
        }

        // In simulator, simulate robot motion toward tag
        if (Utils.isSimulation() && executionCount % 50 == 0) {
            simulateMotion();
        }

        // Update dashboard
        updateDashboard();
    }

    /**
     * Prints current status to console.
     */
    private void printStatus() {
        boolean tagVisible = limelight.isTagVisible(TARGET_TAG_ID);
        System.out.println("[" + executionCount + "] " +
            "Tag Visible: " + tagVisible +
            " | Aligned: " + alignmentCommand.isAligned());

        if (tagVisible) {
            double distance = limelight.getHorizontalDistanceToTag(TARGET_TAG_ID);
            double angle = limelight.getAngleToTag(TARGET_TAG_ID);
            System.out.println("     Distance: " + String.format("%.2f m", distance) +
                " | Angle: " + String.format("%.1f°", Math.toDegrees(angle)));
        }
    }

    /**
     * Simulates robot movement in simulator.
     * Each call moves robot 0.1m closer to target.
     */
    private void simulateMotion() {
        // Get current simulated position
        var currentPose = LimelightSimulation.getSimulatedPose(TARGET_TAG_ID);
        if (currentPose != null) {
            double currentDistance = Math.hypot(currentPose.getX(), currentPose.getY());
            double newDistance = Math.max(currentDistance - 0.1, DESIRED_DISTANCE - 0.05);
            
            System.out.println("   [SIMULATION] Moving closer: " + 
                String.format("%.2f m", currentDistance) + 
                " -> " + String.format("%.2f m", newDistance));
            
            LimelightSimulation.setSimulatedRobotPose(TARGET_TAG_ID, newDistance, 0, 0);
        }
    }

    /**
     * Updates SmartDashboard with current values.
     */
    private void updateDashboard() {
        // Core status
        SmartDashboard.putBoolean("QuickTest/Tag Visible", 
            limelight.isTagVisible(TARGET_TAG_ID));
        SmartDashboard.putBoolean("QuickTest/Is Aligned", 
            alignmentCommand.isAligned());
        SmartDashboard.putNumber("QuickTest/Execution Count", executionCount);

        if (limelight.isTagVisible(TARGET_TAG_ID)) {
            // Measurements
            SmartDashboard.putNumber("QuickTest/Distance (m)", 
                limelight.getHorizontalDistanceToTag(TARGET_TAG_ID));
            SmartDashboard.putNumber("QuickTest/Angle (deg)", 
                Math.toDegrees(limelight.getAngleToTag(TARGET_TAG_ID)));

            // Errors
            var error = limelight.calculateAlignmentError(
                TARGET_TAG_ID, DESIRED_DISTANCE, DESIRED_ANGLE);
            if (error != null) {
                SmartDashboard.putNumber("QuickTest/Distance Error (m)", error.distanceError);
                SmartDashboard.putNumber("QuickTest/Angle Error (rad)", error.angleError);
            }

            // Calculated speeds
            var speeds = alignmentCommand.getChassisSpeedsForAlignment();
            SmartDashboard.putNumber("QuickTest/Forward Speed", speeds.vxMetersPerSecond);
            SmartDashboard.putNumber("QuickTest/Rotation Speed (rad/s)", speeds.omegaRadiansPerSecond);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("QuickTest command interrupted");
        } else {
            System.out.println("QuickTest command finished");
        }
        System.out.println("Total execution cycles: " + executionCount);
    }

    @Override
    public boolean isFinished() {
        // Run for ~10 seconds in simulator, or until aligned on real robot
        if (Utils.isSimulation()) {
            return executionCount > 500;  // 10 seconds at 50Hz
        } else {
            return alignmentCommand.isAligned() && executionCount > 50;
        }
    }
}
