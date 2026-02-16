package frc.robot.interfaces;

import java.util.Optional;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.LimelightHelpers.PoseEstimate;

public interface IPerceptionCamera extends ICamera {
    PoseEstimate estimateRobotPosition(Angle yaw);

    boolean isEnabled();

    boolean hasDetectedValidTarget();

    Optional<Distance> distanceToTarget();
}
