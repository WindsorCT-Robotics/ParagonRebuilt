package frc.robot.hardware.base_sensors;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.LimelightHelpers;
import frc.robot.generated.LimelightHelpers.PoseEstimate;
import frc.robot.generated.LimelightHelpers.RawFiducial;
import frc.robot.interfaces.IBoundedPoseEstimateCamera;
import frc.robot.utils.FieldUtil;

public class LimelightVisionBase implements IBoundedPoseEstimateCamera {
    private final String name;
    private final Supplier<Angle> yaw;
    private final Supplier<AngularVelocity> yawRate;
    private final Supplier<Angle> pitch;
    private final Supplier<AngularVelocity> pitchRate;
    private final Supplier<Angle> roll;
    private final Supplier<AngularVelocity> rollRate;

    private static final Angle GYRO_DEVIATION = Radians.of(Integer.MAX_VALUE); // Uses gyro and not vision estimate to
                                                                               // determine thea.

    public LimelightVisionBase(
            String name,
            Pose3d cameraPose,
            Supplier<Angle> yaw,
            Supplier<AngularVelocity> yawRate,
            Supplier<Angle> pitch,
            Supplier<AngularVelocity> pitchRate,
            Supplier<Angle> roll,
            Supplier<AngularVelocity> rollRate) {
        this.name = name;
        this.yaw = yaw;
        this.yawRate = yawRate;
        this.pitch = pitch;
        this.pitchRate = pitchRate;
        this.roll = roll;
        this.rollRate = rollRate;

        LimelightHelpers.setCameraPose_RobotSpace(
                name,
                cameraPose.getMeasureX().in(Meters),
                cameraPose.getMeasureY().in(Meters),
                cameraPose.getMeasureZ().in(Meters),
                cameraPose.getRotation().getMeasureX().in(Degrees),
                cameraPose.getRotation().getMeasureY().in(Degrees),
                cameraPose.getRotation().getMeasureZ().in(Degrees));
    }

    @Override
    public boolean measurementValid(PoseEstimate poseEstimate) {
        return poseEstimate.tagCount > 0 && withinBounds(poseEstimate);
    }

    @Override
    public boolean withinBounds(PoseEstimate poseEstimate) {
        return FieldUtil.withinBounds(poseEstimate.pose);
    }

    @Override
    public AprilTag[] getDetectedTags(PoseEstimate poseEstimate) {
        RawFiducial[] fiducials = poseEstimate.rawFiducials;
        AprilTag[] aprilTags = new AprilTag[fiducials.length];
        for (int i = 0; i < aprilTags.length; i++) {
            int tagId = fiducials[i].id;
            Optional<Pose3d> position = FieldUtil.getTagPosition(tagId);
            if (position.isPresent()) {
                aprilTags[i] = new AprilTag(tagId, position.get());
            }
        }

        return aprilTags;
    }

    @Override
    public PoseEstimate getPoseEstimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }

    @Override
    public void updateRobotOrientation() {
        LimelightHelpers.SetRobotOrientation(
                name,
                yaw.get().in(Degrees),
                yawRate.get().in(DegreesPerSecond),
                pitch.get().in(Degrees),
                pitchRate.get().in(DegreesPerSecond),
                roll.get().in(Degrees),
                rollRate.get().in(DegreesPerSecond));
    }

    private Distance distanceToClosestTag() {
        return Meters.of(
                LimelightHelpers.getTargetPose3d_CameraSpace(name).getTranslation().getDistance(Translation3d.kZero));
    }

    private Distance getVisionUncertainty(Distance tag, Distance threshold, double standardDeviationScalar) {
        if (tag.lte(threshold)) { // A measurement less than or equal to 1 meter will be added.
            return Meters.of(0.05);
        } else {
            return tag.div(standardDeviationScalar);
        }
    }

    public record StandardVisionDeviations(Distance deviationX, Distance deviationY, Angle deviationRotation,
            Distance tagDistance) {
    }

    @Override
    public StandardVisionDeviations getStandardDeviations(Distance threshold, double standardDeviationScalar) {
        Distance tagDistance = distanceToClosestTag();
        Distance visionDeviationX = getVisionUncertainty(tagDistance, threshold, standardDeviationScalar);
        Distance visionDeviationY = getVisionUncertainty(tagDistance, threshold, standardDeviationScalar);
        return new StandardVisionDeviations(visionDeviationX, visionDeviationY, GYRO_DEVIATION, tagDistance);
    }
}
