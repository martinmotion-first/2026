package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
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
    private double kPDistance = 2.0;      // Speed per meter of distance error (2.0 for 2m error = 1.0 m/s max)
    private double kPAngle = 0.8;         // Angular speed per radian of angle error

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
        // Initialization complete - rely on SmartDashboard for diagnostics
    }

    @Override
    public void execute() {
        // Check if the target tag is visible
        if (!limelight.isTagVisible(targetTagId)) {
            lastCalculatedSpeeds = new ChassisSpeeds(0, 0, 0);
            SmartDashboard.putBoolean("Limelight/Align/Tag Visible", false);
            return;
        }

        SmartDashboard.putBoolean("Limelight/Align/Tag Visible", true);

        // Get current pose relative to tag
        double currentDistance = limelight.getHorizontalDistanceToTag(targetTagId);
        double currentAngle = limelight.getAngleToTag(targetTagId);
        
        // Debug: Check the actual 3D pose to understand distance calculation
        Pose3d pose = limelight.getRobotPoseRelativeToTag(targetTagId);
        double poseX = (pose != null) ? pose.getX() : 0;
        double poseY = (pose != null) ? pose.getY() : 0;
        double poseZ = (pose != null) ? pose.getZ() : 0;
        SmartDashboard.putNumber("Limelight/Debug/Pose X", poseX);
        SmartDashboard.putNumber("Limelight/Debug/Pose Y", poseY);
        SmartDashboard.putNumber("Limelight/Debug/Pose Z", poseZ);
        SmartDashboard.putNumber("Limelight/Debug/Raw Distance", currentDistance);

        // Calculate errors
        // Positive error = too far (need to move forward), Negative error = too close (need to move backward)
        double distanceError = desiredDistance - currentDistance;
        double angleError = normalizeAngle(currentAngle - desiredAngle);

        // Update dashboard with error values
        SmartDashboard.putNumber("Limelight/Align/Desired Distance (m)", desiredDistance);
        SmartDashboard.putNumber("Limelight/Align/Current Distance (m)", currentDistance);
        SmartDashboard.putNumber("Limelight/Align/Distance Error (m)", distanceError);
        SmartDashboard.putNumber("Limelight/Align/Angle Error (deg)", Math.toDegrees(angleError));
        SmartDashboard.putNumber("Limelight/Align/Current Angle (deg)", Math.toDegrees(currentAngle));

        // Calculate chassis speeds using proportional control
        // Strategy: Move forward/backward proportionally while also rotating
        
        // Rotation: Proportional to angle error
        double rotationSpeed = -kPAngle * angleError;
        
        // Forward motion: Positive distance error means too far (move forward), negative means too close (move backward)
        // forwardSpeed = kPDistance * distanceError will give us this behavior:
        //   - If distanceError > 0 (too far): forwardSpeed > 0 (move forward)
        //   - If distanceError < 0 (too close): forwardSpeed < 0 (move backward)
        double forwardSpeed = kPDistance * distanceError;
        
        // Scale down forward/backward motion if angle error is large (angle > 45 degrees)
        // This prevents moving significantly when pointed wrong direction
        if (Math.abs(angleError) > 0.785) {  // ~45 degrees
            forwardSpeed *= 0.3;  // 30% of normal speed when angle is way off
        }

        // Clamp speeds to reasonable values (-1.0 to 1.0)
        forwardSpeed = clamp(forwardSpeed, -1.0, 1.0);
        rotationSpeed = clamp(rotationSpeed, -1.0, 1.0);

        lastCalculatedSpeeds = new ChassisSpeeds(forwardSpeed, 0, rotationSpeed);

        SmartDashboard.putNumber("Limelight/Align/Forward Speed", forwardSpeed);
        SmartDashboard.putNumber("Limelight/Align/Rotation Speed", rotationSpeed);
        SmartDashboard.putBoolean("Limelight/Align/Is Aligned", isAligned());
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
        // Command ended - rely on SmartDashboard for diagnostics
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
