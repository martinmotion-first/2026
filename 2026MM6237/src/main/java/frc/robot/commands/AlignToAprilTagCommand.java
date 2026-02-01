package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Command to align the robot to a specific AprilTag at a desired distance and angle.
 * 
 * This command:
 * 1. Reads the robot's current pose relative to the target AprilTag
 * 2. Calculates distance and angle errors
 * 3. Uses PID-like control to compute chassis speeds needed to correct those errors
 * 4. Provides the chassis speeds to the caller via getChassisSpeedsForAlignment()
 * 
 * The actual drivetrain movement is handled by the caller - this command just calculates
 * what speeds are needed. This makes it compatible with any drivetrain implementation.
 * 
 * Usage example in a command that drives:
 *   AlignToAprilTagCommand alignCommand = new AlignToAprilTagCommand(limelight, 2, 1.5, 0);
 *   ChassisSpeeds speeds = alignCommand.getChassisSpeedsForAlignment();
 *   drivetrain.drive(speeds);
 */
public class AlignToAprilTagCommand extends Command {
    private final LimelightSubsystem limelight;
    private final int targetTagId;
    private final double desiredDistance;      // meters
    private final double desiredAngle;          // radians
    private final double distanceTolerance;     // meters
    private final double angleTolerance;        // radians

    // PID-like gains (tunable)
    private double kPDistance = 0.5;      // Speed per meter of distance error
    private double kPAngle = 0.3;         // Angular speed per radian of angle error

    private ChassisSpeeds lastCalculatedSpeeds = new ChassisSpeeds(0, 0, 0);

    /**
     * Creates an alignment command with default tolerances.
     * 
     * @param limelight The Limelight subsystem
     * @param targetTagId The ID of the AprilTag to align to
     * @param desiredDistance Desired distance from the tag (meters)
     * @param desiredAngle Desired angle relative to the tag (radians, 0 = tag directly ahead)
     */
    public AlignToAprilTagCommand(LimelightSubsystem limelight, int targetTagId, 
                                   double desiredDistance, double desiredAngle) {
        this(limelight, targetTagId, desiredDistance, desiredAngle, 0.05, 0.05);
    }

    /**
     * Creates an alignment command with specified tolerances.
     * 
     * @param limelight The Limelight subsystem
     * @param targetTagId The ID of the AprilTag to align to
     * @param desiredDistance Desired distance from the tag (meters)
     * @param desiredAngle Desired angle relative to the tag (radians)
     * @param distanceTolerance Acceptable distance error to be considered "aligned" (meters)
     * @param angleTolerance Acceptable angle error to be considered "aligned" (radians)
     */
    public AlignToAprilTagCommand(LimelightSubsystem limelight, int targetTagId, 
                                   double desiredDistance, double desiredAngle,
                                   double distanceTolerance, double angleTolerance) {
        this.limelight = limelight;
        this.targetTagId = targetTagId;
        this.desiredDistance = desiredDistance;
        this.desiredAngle = desiredAngle;
        this.distanceTolerance = distanceTolerance;
        this.angleTolerance = angleTolerance;
        
        addRequirements(limelight);
    }

    /**
     * Sets the PID-like proportional gains for distance control.
     * @param gain Speed per meter of error (default 0.5)
     */
    public void setDistanceGain(double gain) {
        this.kPDistance = gain;
    }

    /**
     * Sets the PID-like proportional gains for angle control.
     * @param gain Angular speed per radian of error (default 0.3)
     */
    public void setAngleGain(double gain) {
        this.kPAngle = gain;
    }

    @Override
    public void initialize() {
        System.out.println("AlignToAprilTag: Starting alignment to tag " + targetTagId);
        System.out.println("  Desired distance: " + String.format("%.2f m", desiredDistance));
        System.out.println("  Desired angle: " + String.format("%.1f°", Math.toDegrees(desiredAngle)));
    }

    @Override
    public void execute() {
        // Check if the target tag is visible
        if (!limelight.isTagVisible(targetTagId)) {
            lastCalculatedSpeeds = new ChassisSpeeds(0, 0, 0);
            SmartDashboard.putBoolean("Alignment/Tag Visible", false);
            return;
        }

        SmartDashboard.putBoolean("Alignment/Tag Visible", true);

        // Get current pose relative to tag
        double currentDistance = limelight.getHorizontalDistanceToTag(targetTagId);
        double currentAngle = limelight.getAngleToTag(targetTagId);

        // Calculate errors
        double distanceError = currentDistance - desiredDistance;  // positive = too close
        double angleError = normalizeAngle(currentAngle - desiredAngle);

        // Update dashboard with error values
        SmartDashboard.putNumber("Alignment/Distance Error (m)", distanceError);
        SmartDashboard.putNumber("Alignment/Angle Error (deg)", Math.toDegrees(angleError));
        SmartDashboard.putNumber("Alignment/Current Distance (m)", currentDistance);
        SmartDashboard.putNumber("Alignment/Current Angle (deg)", Math.toDegrees(currentAngle));

        // Calculate chassis speeds using proportional control
        // Distance error: negative means too far, so we want positive (forward) speed
        double forwardSpeed = -kPDistance * distanceError;  // Negative sign: closer = negative error
        
        // Angle error: positive means we're rotated counter-clockwise from desired
        double rotationSpeed = -kPAngle * angleError;

        // Clamp speeds to reasonable values
        forwardSpeed = clamp(forwardSpeed, -1.0, 1.0);
        rotationSpeed = clamp(rotationSpeed, -1.0, 1.0);

        lastCalculatedSpeeds = new ChassisSpeeds(forwardSpeed, 0, rotationSpeed);

        SmartDashboard.putNumber("Alignment/Forward Speed", forwardSpeed);
        SmartDashboard.putNumber("Alignment/Rotation Speed", rotationSpeed);
        SmartDashboard.putBoolean("Alignment/Is Aligned", isAligned());
    }

    /**
     * Gets the calculated chassis speeds needed to align to the target.
     * These speeds should be applied to your drivetrain by the caller.
     * 
     * @return ChassisSpeeds object with forward and rotation speeds
     */
    public ChassisSpeeds getChassisSpeedsForAlignment() {
        return lastCalculatedSpeeds;
    }

    /**
     * Checks if the robot is currently aligned within tolerance.
     * @return true if both distance and angle errors are within tolerance
     */
    public boolean isAligned() {
        if (!limelight.isTagVisible(targetTagId)) {
            return false;
        }

        LimelightSubsystem.AlignmentError error = limelight.calculateAlignmentError(
            targetTagId, desiredDistance, desiredAngle);
        
        if (error == null) {
            return false;
        }

        return Math.abs(error.distanceError) <= distanceTolerance &&
               Math.abs(error.angleError) <= angleTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("AlignToAprilTag: Command interrupted");
        } else {
            System.out.println("AlignToAprilTag: Alignment complete");
        }
    }

    @Override
    public boolean isFinished() {
        return isAligned();
    }

    // ==================== HELPER METHODS ====================

    /**
     * Clamps a value between min and max.
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Normalizes an angle to the range [-π, π].
     * This helps with angle wraparound issues.
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}
