package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Test command to read and log Limelight data.
 * 
 * This command is designed to help verify that:
 * 1. The Limelight is connected and returning data
 * 2. AprilTags are being detected correctly
 * 3. Pose data is in the expected range
 * 4. There are no data parsing errors
 * 
 * Run this command while aiming your Limelight at AprilTags to verify everything works.
 * Monitor SmartDashboard for outputs.
 */
public class TestLimelightCommand extends Command {
    private final LimelightSubsystem limelight;
    private int logCounter = 0;
    private static final int LOG_FREQUENCY = 10; // Log every 10 cycles (200ms at 50Hz)

    /**
     * Creates a TestLimelightCommand.
     * @param limelightSubsystem The Limelight subsystem to test
     */
    public TestLimelightCommand(LimelightSubsystem limelightSubsystem) {
        this.limelight = limelightSubsystem;
        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        System.out.println("=== TestLimelight Command Started ===");
        System.out.println("Aim your Limelight at AprilTags and watch the output");
    }

    @Override
    public void execute() {
        logCounter++;

        // Always update dashboard (no throttling)
        updateDashboard();

        // Log to console less frequently to avoid spam
        if (logCounter >= LOG_FREQUENCY) {
            logCounter = 0;
            logToConsole();
        }
    }

    /**
     * Updates SmartDashboard with current Limelight data.
     * This runs every cycle for real-time monitoring.
     */
    private void updateDashboard() {
        // Connection and target status
        SmartDashboard.putBoolean("Limelight/Has Target", limelight.hasValidTarget());
        SmartDashboard.putNumber("Limelight/Fiducials Detected", limelight.getDetectedFiducialCount());

        // Raw targeting data
        SmartDashboard.putNumber("Limelight/TX", limelight.getTX());
        SmartDashboard.putNumber("Limelight/TY", limelight.getTY());
        SmartDashboard.putNumber("Limelight/TA", limelight.getTA());

        // Pipeline info
        SmartDashboard.putNumber("Limelight/Pipeline", limelight.getCurrentPipeline());
        SmartDashboard.putNumber("Limelight/Latency Pipeline (ms)", limelight.getPipelineLatency());
        SmartDashboard.putNumber("Limelight/Latency Capture (ms)", limelight.getCaptureLatency());

        // Per-tag distance and angle data (for first visible tag)
        LimelightTarget_Fiducial[] fiducials = limelight.getDetectedFiducials();
        if (fiducials.length > 0) {
            LimelightTarget_Fiducial firstTag = fiducials[0];
            int tagId = (int) firstTag.fiducialID;
            
            SmartDashboard.putNumber("Limelight/First Tag ID", tagId);
            SmartDashboard.putNumber("Limelight/Distance to First Tag (m)", limelight.getDistanceToTag(tagId));
            SmartDashboard.putNumber("Limelight/Horizontal Distance (m)", limelight.getHorizontalDistanceToTag(tagId));
            SmartDashboard.putNumber("Limelight/Angle to First Tag (deg)", 
                Math.toDegrees(limelight.getAngleToTag(tagId)));

            // Pose data
            var pose = limelight.getRobotPoseRelativeToTag(tagId);
            if (pose != null) {
                SmartDashboard.putNumber("Limelight/Pose X (m)", pose.getX());
                SmartDashboard.putNumber("Limelight/Pose Y (m)", pose.getY());
                SmartDashboard.putNumber("Limelight/Pose Z (m)", pose.getZ());
                SmartDashboard.putNumber("Limelight/Pose Yaw (deg)", 
                    Math.toDegrees(pose.getRotation().getZ()));
            }
        }
    }

    /**
     * Logs detailed information to the console for debugging.
     * This runs less frequently to avoid console spam.
     */
    private void logToConsole() {
        System.out.println("\n--- Limelight Status (t=" + System.currentTimeMillis() + ") ---");
        System.out.println("Has Valid Target: " + limelight.hasValidTarget());
        System.out.println("Fiducials Detected: " + limelight.getDetectedFiducialCount());

        if (!limelight.hasValidTarget()) {
            System.out.println(">> No targets detected. Aim at AprilTags.");
            return;
        }

        // Get all detected fiducials
        LimelightTarget_Fiducial[] fiducials = limelight.getDetectedFiducials();
        System.out.println("Detected " + fiducials.length + " AprilTag(s):");

        for (LimelightTarget_Fiducial fiducial : fiducials) {
            int tagId = (int) fiducial.fiducialID;
            System.out.println("  [Tag ID: " + tagId + "]");

            // Distance and angle
            double distance = limelight.getHorizontalDistanceToTag(tagId);
            double angle = limelight.getAngleToTag(tagId);
            System.out.println("    Distance: " + String.format("%.3f m", distance));
            System.out.println("    Angle: " + String.format("%.1f°", Math.toDegrees(angle)));

            // Raw targeting
            System.out.println("    TX: " + String.format("%.1f°", fiducial.tx));
            System.out.println("    TY: " + String.format("%.1f°", fiducial.ty));
            System.out.println("    TA: " + String.format("%.1f%%", fiducial.ta));

            // Pose data
            var pose = fiducial.getRobotPose_TargetSpace();
            if (pose != null) {
                System.out.println("    Robot Pose (relative to tag):");
                System.out.println("      Position: " + String.format("(%.3f, %.3f, %.3f) m",
                    pose.getX(), pose.getY(), pose.getZ()));
                System.out.println("      Rotation (Yaw): " + String.format("%.1f°",
                    Math.toDegrees(pose.getRotation().getZ())));
            }
        }

        System.out.println("  Pipeline Latency: " + 
            String.format("%.1f ms", limelight.getPipelineLatency()));
        System.out.println("  Capture Latency: " + 
            String.format("%.1f ms", limelight.getCaptureLatency()));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("=== TestLimelight Command Ended ===");
    }

    @Override
    public boolean isFinished() {
        return false; // Command runs indefinitely until explicitly cancelled
    }
}
