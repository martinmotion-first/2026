package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class LimelightSubsystem implements Subsystem {
    private final String limelightName;

    public LimelightSubsystem() {
        this("");
    }

    public LimelightSubsystem(String name) {
        this.limelightName = name;
    }

    @Override
    public void periodic() {
        updateDashboard();
    }

    private void updateDashboard() {
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
        
        // "Connected" means the Limelight is responding (has valid results)
        SmartDashboard.putBoolean("Limelight/Connected", results.valid);
        
        // "Has Target" means we detected at least one AprilTag fiducial
        boolean hasTarget = results.targets_Fiducials != null && results.targets_Fiducials.length > 0;
        SmartDashboard.putBoolean("Limelight/Has Target", hasTarget);
    }

    public boolean hasValidTarget() {
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
        return results.valid;
    }

    public LimelightTarget_Fiducial[] getDetectedFiducials() {
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
        if (results == null || results.targets_Fiducials == null) {
            return new LimelightTarget_Fiducial[0];
        }
        return results.targets_Fiducials;
    }

    public int getDetectedFiducialCount() {
        return getDetectedFiducials().length;
    }

    public boolean isTagVisible(int tagId) {
        for (LimelightTarget_Fiducial fiducial : getDetectedFiducials()) {
            if ((int)fiducial.fiducialID == tagId) {
                return true;
            }
        }
        return false;
    }

    public Pose3d getRobotPoseRelativeToTag(int tagId) {
        if (!isTagVisible(tagId)) {
            return null;
        }
        for (LimelightTarget_Fiducial fiducial : getDetectedFiducials()) {
            if ((int)fiducial.fiducialID == tagId) {
                return fiducial.getRobotPose_TargetSpace();
            }
        }
        return null;
    }

    public Pose2d getRobotPose2dRelativeToTag(int tagId) {
        if (!isTagVisible(tagId)) {
            return null;
        }
        for (LimelightTarget_Fiducial fiducial : getDetectedFiducials()) {
            if ((int)fiducial.fiducialID == tagId) {
                return fiducial.getRobotPose_TargetSpace2D();
            }
        }
        return null;
    }

    public double getDistanceToTag(int tagId) {
        Pose3d pose = getRobotPoseRelativeToTag(tagId);
        if (pose == null) {
            return -1;
        }
        return Math.sqrt(pose.getX() * pose.getX() + pose.getY() * pose.getY() + pose.getZ() * pose.getZ());
    }

    public double getHorizontalDistanceToTag(int tagId) {
        Pose3d pose = getRobotPoseRelativeToTag(tagId);
        if (pose == null) {
            return -1;
        }
        // Z is the forward/backward distance from the tag (negative = away from tag)
        // Take absolute value to get actual distance
        return Math.abs(pose.getZ());
    }

    public double getAngleToTag(int tagId) {
        // Return the horizontal offset angle (tx) in radians
        // tx is the angle in degrees from the crosshair to the target
        // Positive = tag to the right, Negative = tag to the left, 0 = centered
        for (LimelightTarget_Fiducial fiducial : getDetectedFiducials()) {
            if ((int)fiducial.fiducialID == tagId) {
                return Math.toRadians(fiducial.tx);
            }
        }
        return 0;
    }

    public AlignmentError calculateAlignmentError(int tagId, double desiredDistance, double desiredAngle) {
        double actualDistance = getHorizontalDistanceToTag(tagId);
        double actualAngle = getAngleToTag(tagId);
        if (actualDistance < 0) {
            return null;
        }
        return new AlignmentError(
            actualDistance - desiredDistance,
            actualAngle - desiredAngle
        );
    }

    public boolean isAligned(int tagId, double desiredDistance, double distanceTolerance,
                            double desiredAngle, double angleTolerance) {
        AlignmentError error = calculateAlignmentError(tagId, desiredDistance, desiredAngle);
        if (error == null) {
            return false;
        }
        return Math.abs(error.distanceError) <= distanceTolerance &&
               Math.abs(error.angleError) <= angleTolerance;
    }

    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }

    public double getTA() {
        return LimelightHelpers.getTA(limelightName);
    }

    public double getCurrentPipeline() {
        return LimelightHelpers.getCurrentPipelineIndex(limelightName);
    }

    public double getPipelineLatency() {
        return LimelightHelpers.getLatency_Pipeline(limelightName);
    }

    public double getCaptureLatency() {
        return LimelightHelpers.getLatency_Capture(limelightName);
    }

    public static class AlignmentError {
        public final double distanceError;
        public final double angleError;

        public AlignmentError(double distanceError, double angleError) {
            this.distanceError = distanceError;
            this.angleError = angleError;
        }

        @Override
        public String toString() {
            return String.format("AlignmentError{distance=%.3fm, angle=%.3frad}", distanceError, angleError);
        }
    }
}