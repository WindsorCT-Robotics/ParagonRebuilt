package frc.robot.hardware.sensors;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.hardware.base_sensors.LimelightVisionBase;

public class LauncherVision extends LimelightVisionBase {
    public LauncherVision(
            String name,
            Pose3d cameraPose,
            Supplier<Angle> yaw,
            Supplier<AngularVelocity> yawRate,
            Supplier<Angle> pitch,
            Supplier<AngularVelocity> pitchRate,
            Supplier<Angle> roll,
            Supplier<AngularVelocity> rollRate,
            Distance standardDeviationThreshold,
            double standardDeviationScalar,
            String networkTableDirectory) {
        super(
                name,
                cameraPose,
                yaw,
                yawRate,
                pitch,
                pitchRate,
                roll,
                rollRate,
                standardDeviationThreshold,
                standardDeviationScalar,
                networkTableDirectory);
    }
}