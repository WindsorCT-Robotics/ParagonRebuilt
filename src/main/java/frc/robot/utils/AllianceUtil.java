package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceUtil {
    public static Optional<Angle> angleOffset(Angle angle) {
        return DriverStation.getAlliance().map(alliance -> {
            if (alliance == Alliance.Red) {
                return AngleUtil.invert(angle);
            }

            return angle;
        });
    }

    public static Optional<Pose2d> normalizePose2d(Pose2d pose2d) {
        return DriverStation.getAlliance().map(alliance -> {
            if (alliance == Alliance.Red) {
                return new Pose2d(pose2d.getMeasureX(), pose2d.getMeasureY(), new Rotation2d(AngleUtil.invert(Degrees.of(pose2d.getRotation().getDegrees()))));
            }

            return pose2d;
        });
    }
}