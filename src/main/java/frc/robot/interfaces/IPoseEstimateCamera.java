package frc.robot.interfaces;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import frc.robot.generated.LimelightHelpers.PoseEstimate;
import frc.robot.hardware.base_sensors.LimelightVisionBase.StandardVisionDeviations;

public interface IPoseEstimateCamera {
    public void updateRobotOrientation();

    public PoseEstimate getPoseEstimate();

    public AprilTag[] getDetectedTags(PoseEstimate poseEstimate);

    /**
     * The lower the standard deviation, the more the vision trusts the measured
     * position.
     * The higher the standard deviation, the less the vision trusts the measured
     * position.
     * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html
     * 
     * @return
     */
    public Optional<StandardVisionDeviations> getStandardDeviations(PoseEstimate poseEstimate);
}