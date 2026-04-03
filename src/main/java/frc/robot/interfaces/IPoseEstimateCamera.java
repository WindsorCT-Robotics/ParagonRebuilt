package frc.robot.interfaces;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.units.measure.Distance;
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
    public StandardVisionDeviations getStandardDeviations(Distance threshold, double standardDeviationScalar);
}