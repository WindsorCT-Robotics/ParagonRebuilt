package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class LaunchCalculator implements Sendable {
    private final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap kickerMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    private final Supplier<Pose2d> robotPosition;
    private final Supplier<Translation2d> hubPosition;

    public LaunchCalculator(Supplier<Pose2d> robotPosition, Supplier<Translation2d> hubPosition) {
        this.robotPosition = robotPosition;
        this.hubPosition = hubPosition;

        putTestedShooterMap();
        putTestedKickerMap();
        putTestedTimeOfFlightMap();
    }

    private Distance distanceToHub() {
        Pose2d currentPosition = robotPosition.get();
        Translation2d hubPose = hubPosition.get();
        Distance a = currentPosition.getMeasureX().minus(hubPose.getMeasureX());
        Distance b = currentPosition.getMeasureY().minus(hubPose.getMeasureY());
        return Meters.of(
                Math.sqrt(
                        Math.pow(a.in(Meters), 2) + Math.pow(b.in(Meters), 2)));
    }

    public AngularVelocity getShooterVelocityToHub() {
        return RPM.of(shooterMap.get(distanceToHub().in(Meters)));
    }

    public AngularVelocity getShooterVelocityToDistance(Distance distance) {
        return RPM.of(shooterMap.get(distance.in(Meters)));
    }

    public AngularVelocity getKickerVelocityToHub() {
        return RPM.of(kickerMap.get(distanceToHub().in(Meters)));
    }

    public AngularVelocity getKickerVelocityToDistance(Distance distance) {
        return RPM.of(kickerMap.get(distance.in(Meters)));
    }

    public Time getTimeOfFlightToHub() {
        return Seconds.of(timeOfFlightMap.get(distanceToHub().in(Meters)));
    }

    private void putTestedShooterMap() {
        shooterMap.put(1.5, 1925.0);
        shooterMap.put(2.0, 2050.0);
        shooterMap.put(2.5, 2175.0);
        shooterMap.put(3.0, 2400.0);
        shooterMap.put(3.5, 2450.0);
        shooterMap.put(4.0, 2600.0);
        shooterMap.put(4.5, 2700.0);
        shooterMap.put(5.0, 3000.0);
    }

    private void putTestedKickerMap() {
        kickerMap.put(1.5, 1925.0);
        kickerMap.put(2.0, 2050.0);
        kickerMap.put(2.5, 2175.0);
        kickerMap.put(3.0, 2400.0);
        kickerMap.put(3.5, 2450.0);
        kickerMap.put(4.0, 2600.0);
        kickerMap.put(4.5, 2700.0);
        kickerMap.put(5.0, 3000.0);
    }

    private void putTestedTimeOfFlightMap() {
        timeOfFlightMap.put(2.0, 1.0);
        timeOfFlightMap.put(2.0, 1.0);
        timeOfFlightMap.put(2.0, 1.0);
        timeOfFlightMap.put(2.0, 1.0);
        timeOfFlightMap.put(2.0, 1.0);
        timeOfFlightMap.put(2.0, 1.0);
        timeOfFlightMap.put(2.0, 1.0);
        timeOfFlightMap.put(3.0, 1.5);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Predicted Shooter Velocity (RPM)", () -> getShooterVelocityToHub().in(RPM), null);
        builder.addDoubleProperty("Predicted Kicker Velocity (RPM)", () -> getKickerVelocityToHub().in(RPM), null);
        builder.addDoubleProperty("Distance To Hub (M)", () -> distanceToHub().in(Meters), null);
    }
}