package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.RectanglePoseArea;

public class FieldUtil {
    private static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltAndymark);
    private static final RectanglePoseArea AREA = new RectanglePoseArea(
            new Translation2d(),
            new Translation2d(getFieldLength(), getFieldWidth()));

    /**
     * X-axis
     * 
     * @return
     */
    public static Distance getFieldLength() {
        return Meters.of(APRIL_TAG_FIELD_LAYOUT.getFieldLength());
    }

    /**
     * Y-axis
     * 
     * @return
     */
    public static Distance getFieldWidth() {
        return Meters.of(APRIL_TAG_FIELD_LAYOUT.getFieldWidth());
    }

    public static Optional<Pose3d> getTagPosition(int id) {
        return APRIL_TAG_FIELD_LAYOUT.getTagPose(id);
    }

    public static boolean withinBounds(Pose2d position) {
        return AREA.isPoseWithinArea(position);
    }
}