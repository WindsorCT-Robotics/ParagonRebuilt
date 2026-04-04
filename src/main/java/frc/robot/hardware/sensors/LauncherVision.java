package frc.robot.hardware.sensors;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.LimelightHelpers.PoseEstimate;
import frc.robot.hardware.base_sensors.LimelightVisionBase;

public class LauncherVision extends LimelightVisionBase {
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard/Subsystems/Drive");
    private final StructArrayPublisher<Pose2d> perceptedTags = table.getStructArrayTopic("Launcher Vision Tags",
            Pose2d.struct).publish();

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
            double standardDeviationScalar) {
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
                standardDeviationScalar);
    }

    public void updateNetworkTables(PoseEstimate poseEstimate) {
        AprilTag[] aprilTags = getDetectedTags(poseEstimate);
        Pose2d[] tags = new Pose2d[aprilTags.length];

        for (int i = 0; i < tags.length; i++) {
            tags[i] = aprilTags[i].pose.toPose2d();
        }

        perceptedTags.set(tags);
    }
}