package frc.robot.utils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class LaunchCalculator {
    private final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap kickerMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    private final Supplier<Pose2d> robotPosition;
    private final Pose2d hubPosition;

    public LaunchCalculator(Supplier<Pose2d> robotPosition) {
        this.robotPosition = robotPosition;
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        Pose3d blueHubCloseRight = layout.getTagPose(26).get();
        Pose3d blueHubFarLeft = layout.getTagPose(26).get();
        Pose3d redHubCloseRight = layout.getTagPose(10).get();
        Pose3d redHubFarLeft = layout.getTagPose(4).get();

        // TODO: Is getMeasureY() the length of the field?
        Distance x;
        Distance y;
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            x = blueHubFarLeft.getMeasureX().plus(blueHubCloseRight.getMeasureX()).div(2);
            y = blueHubFarLeft.getMeasureY().plus(blueHubCloseRight.getMeasureY()).div(2);
        } else {
            x = redHubFarLeft.getMeasureX().plus(blueHubCloseRight.getMeasureX()).div(2);
            y = redHubCloseRight.getMeasureY().plus(blueHubCloseRight.getMeasureY()).div(2);
        }

        hubPosition = new Pose2d(x, y, new Rotation2d());

        putShooterMap();
        putKickerMap();
        putTimeOfFlightMap();
    }

    private Distance distanceToHub() {
        Pose2d currentPosition = robotPosition.get();
        Distance a = currentPosition.getMeasureX().minus(hubPosition.getMeasureX());
        Distance b = currentPosition.getMeasureY().minus(hubPosition.getMeasureY());
        return Meters.of(
                Math.sqrt(
                        Math.pow(a.in(Meters), 2) + Math.pow(b.in(Meters), 2)));
    }

    public AngularVelocity getShooterVelocity() {
        return RPM.of(shooterMap.get(distanceToHub().in(Meters)));
    }

    public AngularVelocity getKickerVelocity() {
        return RPM.of(kickerMap.get(distanceToHub().in(Meters)));
    }

    public Time getTimeOfFlight() {
        return Seconds.of(timeOfFlightMap.get(distanceToHub().in(Meters)));
    }

    private void putShooterMap() {
        shooterMap.put(Inches.of(45).in(Meters), 120.0);
        shooterMap.put(Inches.of(115).in(Meters), 360.0);
    }

    private void putKickerMap() {
        kickerMap.put(Inches.of(45).in(Meters), 180.0);
        kickerMap.put(Inches.of(115).in(Meters), 420.0);
    }

    private void putTimeOfFlightMap() {
    }
}