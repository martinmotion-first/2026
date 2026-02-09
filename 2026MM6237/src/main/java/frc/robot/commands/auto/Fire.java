package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LimelightSubsystem6237;

/**
 * Autonomous command to fire the note at the hub.
 * 
 * Runs the feeder motor for a set amount of time to feed the note through the shooter.
 * The feeder speed is determined by distance from the hub.
 * After firing, stops both the feeder and shooter rollers.
 */
public class Fire extends Command {
    private final Feeder feeder;
    private final Shooter shooter;
    private final LimelightSubsystem6237 limelight;
    
    private final Timer fireTimer = new Timer();
    private static final double FIRE_DURATION_SECONDS = 0.5; // Duration to run feeder (tunable constant)
    private static final int HUB_TAG_ID = 4; // AprilTag ID for the hub (adjust as needed)
    private static final double DEFAULT_FEEDER_PERCENT_OUTPUT = 0.8; // Default feeder speed

    public Fire(Feeder feeder, Shooter shooter, LimelightSubsystem6237 limelight) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.limelight = limelight;
        addRequirements(feeder, shooter);
    }

    @Override
    public void initialize() {
        fireTimer.reset();
        fireTimer.start();
        
        // Determine feeder speed based on distance
        double feederPercentOutput = DEFAULT_FEEDER_PERCENT_OUTPUT;
        
        if (limelight.hasValidTarget()) {
            double distanceToHub = limelight.getDistanceToTag(HUB_TAG_ID);
            if (distanceToHub > 0) {
                // Calculate feeder speed based on distance
                feederPercentOutput = calculateFeederSpeed(distanceToHub);
            }
        }
        
        // Start feeder at calculated speed
        feeder.setPercentOutput(feederPercentOutput);
    }

    @Override
    public void execute() {
        // Feeder is running, shooter is already spinning from PrepareToFire
    }

    @Override
    public boolean isFinished() {
        // Command finishes after firing duration has elapsed
        return fireTimer.hasElapsed(FIRE_DURATION_SECONDS);
    }

    @Override
    public void end(boolean interrupted) {
        fireTimer.stop();
        // Stop feeder
        feeder.setPercentOutput(0.0);
        // Stop shooter rollers
        shooter.stop();
    }

    /**
     * Calculates the required feeder speed based on distance from the hub.
     * This is a placeholder implementation that can be tuned based on testing.
     * 
     * @param distanceMeters Distance to the hub in meters
     * @return Feeder percent output (0.0 to 1.0)
     */
    private double calculateFeederSpeed(double distanceMeters) {
        // Placeholder linear relationship: adjust these values based on tuning
        double minDistance = 1.0;      // Minimum distance in meters
        double maxDistance = 8.0;      // Maximum distance in meters
        double minPercentOutput = 0.6; // Feeder speed at minimum distance
        double maxPercentOutput = 1.0; // Feeder speed at maximum distance
        
        if (distanceMeters <= minDistance) {
            return minPercentOutput;
        } else if (distanceMeters >= maxDistance) {
            return maxPercentOutput;
        } else {
            // Linear interpolation between min and max speeds
            double fraction = (distanceMeters - minDistance) / (maxDistance - minDistance);
            return minPercentOutput + (maxPercentOutput - minPercentOutput) * fraction;
        }
    }
}
