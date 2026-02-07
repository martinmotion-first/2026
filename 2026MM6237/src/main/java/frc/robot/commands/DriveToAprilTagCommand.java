package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Command that drives the robot to align with an AprilTag.
 * 
 * This command:
 * 1. Uses AlignToAprilTagCommand to calculate alignment speeds
 * 2. Applies those speeds to the drivetrain
 * 3. Stops when the robot is aligned
 * 
 * Unlike AlignToAprilTagCommand which just calculates speeds,
 * this command actually drives the robot toward the tag.
 */
public class DriveToAprilTagCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final LimelightSubsystem limelight;
    private final AlignToAprilTagCommand alignmentCommand;
    private final int targetTagId;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * Creates a command that drives the robot to align with an AprilTag.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param limelight The Limelight subsystem
     * @param targetTagId The ID of the AprilTag to align to
     * @param desiredDistance Desired distance from the tag (meters)
     * @param desiredAngle Desired angle relative to the tag (radians)
     */
    public DriveToAprilTagCommand(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight, 
                                   int targetTagId, double desiredDistance, double desiredAngle) {
        this(drivetrain, limelight, targetTagId, desiredDistance, desiredAngle, 0.05, 0.05);
    }

    /**
     * Creates a command that drives the robot to align with an AprilTag.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param limelight The Limelight subsystem
     * @param targetTagId The ID of the AprilTag to align to
     * @param desiredDistance Desired distance from the tag (meters)
     * @param desiredAngle Desired angle relative to the tag (radians)
     * @param distanceTolerance Acceptable distance error (meters)
     * @param angleTolerance Acceptable angle error (radians)
     */
    public DriveToAprilTagCommand(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight,
                                   int targetTagId, double desiredDistance, double desiredAngle,
                                   double distanceTolerance, double angleTolerance) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.targetTagId = targetTagId;
        this.alignmentCommand = new AlignToAprilTagCommand(limelight, targetTagId, 
                                                           desiredDistance, desiredAngle,
                                                           distanceTolerance, angleTolerance);
        
        // Require both the drivetrain and limelight so nothing else drives while aligning
        addRequirements(drivetrain, limelight);
    }

    /**
     * Set the distance gain for PID-like control.
     */
    public void setDistanceGain(double gain) {
        alignmentCommand.setDistanceGain(gain);
    }

    /**
     * Set the angle gain for PID-like control.
     */
    public void setAngleGain(double gain) {
        alignmentCommand.setAngleGain(gain);
    }

    @Override
    public void initialize() {
        alignmentCommand.initialize();
    }

    @Override
    public void execute() {
        // Step 1: Update the alignment calculation
        alignmentCommand.execute();
        
        // Step 2: Get the calculated speeds
        ChassisSpeeds alignmentSpeeds = alignmentCommand.getChassisSpeedsForAlignment();
        
        // Step 3: Apply them to the drivetrain using robot-centric request
        drivetrain.setControl(driveRequest
            .withVelocityX(alignmentSpeeds.vxMetersPerSecond)
            .withVelocityY(alignmentSpeeds.vyMetersPerSecond)
            .withRotationalRate(alignmentSpeeds.omegaRadiansPerSecond));
        
        // Step 4: Update SmartDashboard with drivetrain command status
        SmartDashboard.putNumber("Limelight/Drive/VelocityX", alignmentSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Limelight/Drive/VelocityY", alignmentSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Limelight/Drive/OmegaRadPerSec", alignmentSpeeds.omegaRadiansPerSecond);
        SmartDashboard.putBoolean("Limelight/Drive/IsAligned", alignmentCommand.isAligned());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all drivetrain motion when command ends
        drivetrain.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        // Stop when aligned OR when target is no longer visible
        return !limelight.isTagVisible(targetTagId) || alignmentCommand.isAligned();
    }
}
