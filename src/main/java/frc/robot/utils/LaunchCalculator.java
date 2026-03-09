package frc.robot.utils;

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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LaunchCalculator implements Sendable {
    private final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap kickerMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    private final Supplier<Pose2d> robotPosition;

    public LaunchCalculator(Supplier<Pose2d> robotPosition) {
        this.robotPosition = robotPosition;

        // TODO: Double check these tag numbers
        // TODO: Check for the existence of the tags before calling .get().

        putTestedShooterMap();
        putTestedKickerMap();
        putTestedTimeOfFlightMap();
    }

    private Pose2d getHubPosition(Alliance alliance) {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        Pose3d blueHubYCenter = layout.getTagPose(26).get();
        Pose3d blueHubXCenter = layout.getTagPose(21).get();
        Pose3d redHubYCenter = layout.getTagPose(10).get();
        Pose3d redHubXCenter = layout.getTagPose(5).get();

        Distance xHub;
        Distance yHub;
        if (alliance.equals(Alliance.Blue)) {
            xHub = blueHubXCenter.getMeasureX();
            yHub = blueHubYCenter.getMeasureY();
        } else {
            xHub = redHubXCenter.getMeasureX();
            yHub = redHubYCenter.getMeasureY();
        }

        return new Pose2d(xHub, yHub, new Rotation2d());
    }

    private Distance distanceToHub() {
        Pose2d currentPosition = robotPosition.get();

        Alliance alliance = DriverStation.getAlliance().get();
        Pose2d hubPosition = getHubPosition(alliance);
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

    private void putTestedShooterMap() {
        shooterMap.put(2.0, 2175.0);
        shooterMap.put(3.0, 2400.00);
    }

    private void putTestedKickerMap() {
        kickerMap.put(2.0, 2175.00);
        kickerMap.put(3.0, 2400.00);
    }

    private void putTestedTimeOfFlightMap() {
        timeOfFlightMap.put(2.0, 1.0);
        timeOfFlightMap.put(3.0, 1.5);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Predicted Shooter Velocity (RPM)", () -> getShooterVelocity().in(RPM), null);
        builder.addDoubleProperty("Predicted Kicker Velocity (RPM)", () -> getKickerVelocity().in(RPM), null);
        builder.addDoubleProperty("Distance To Hub (M)", () -> distanceToHub().in(Meters), null);
    }
}