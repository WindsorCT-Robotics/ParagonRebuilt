package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class LaunchCalculator {
    public final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();
    public final InterpolatingDoubleTreeMap kickerMap = new InterpolatingDoubleTreeMap();
    public final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    public LaunchCalculator() {
        putShooterMap();
        putKickerMap();
        putTimeOfFlightMap();
    }

    public AngularVelocity getShooterVelocity(Distance distance) {
        return RPM.of(shooterMap.get(distance.in(Meters)));
    }

    public AngularVelocity getKickerVelocity(Distance distance) {
        return RPM.of(kickerMap.get(distance.in(Meters)));
    }

    public Time getTimeOfFlight(Distance distance) {
        return Seconds.of(timeOfFlightMap.get(distance.in(Meters)));
    }

    private void putShooterMap() {
        shooterMap.put(null, null);
    }

    private void putKickerMap() {
        kickerMap.put(null, null);
    }

    private void putTimeOfFlightMap() {
        timeOfFlightMap.put(null, null);
    }
}