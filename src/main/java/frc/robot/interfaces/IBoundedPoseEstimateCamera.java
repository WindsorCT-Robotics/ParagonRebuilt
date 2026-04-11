package frc.robot.interfaces;

import frc.robot.generated.LimelightHelpers.PoseEstimate;

public interface IBoundedPoseEstimateCamera extends IPoseEstimateCamera {
    public boolean withinBounds(PoseEstimate poseEstimate);

    public boolean measurementValid(PoseEstimate poseEstimate);
}