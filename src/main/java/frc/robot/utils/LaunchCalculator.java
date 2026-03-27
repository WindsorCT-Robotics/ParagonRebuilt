package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class LaunchCalculator implements Sendable {
    private final InterpolatingDoubleTreeMap launcherMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap kickerMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    private AngularVelocity lastLauncherVelocity = RPM.zero();
    private AngularVelocity lastKickerVelocity = RPM.zero();
    private Distance lastLauncherDistanceTarget = Meters.zero();
    private Distance lastKickerDistanceTarget = Meters.zero();

    public LaunchCalculator() {
        putTestedLauncherMap();
        putTestedKickerMap();
        putTestedTimeOfFlightMap();
    }

    public AngularVelocity getLauncherVelocityToDistance(Distance distance) {
        lastLauncherVelocity = RPM.of(launcherMap.get(distance.in(Meters)));
        return lastLauncherVelocity;
    }

    public AngularVelocity getKickerVelocityToDistance(Distance distance) {
        lastKickerVelocity = RPM.of(kickerMap.get(distance.in(Meters)));
        return lastKickerVelocity;
    }

    private void putTestedLauncherMap() {
        launcherMap.put(1.5, 2000.0);
        launcherMap.put(2.0, 2125.0);
        launcherMap.put(2.5, 2225.0);
        launcherMap.put(3.0, 2325.0);
        launcherMap.put(3.5, 2450.0);
        launcherMap.put(4.0, 2590.0);
        launcherMap.put(4.5, 2690.0);
        launcherMap.put(5.0, 2850.0);
    }

    private void putTestedKickerMap() {
        kickerMap.put(1.5, 2000.0);
        kickerMap.put(2.0, 2125.0);
        kickerMap.put(2.5, 2225.0);
        kickerMap.put(3.0, 2325.0);
        kickerMap.put(3.5, 2450.0);
        kickerMap.put(4.0, 2590.0);
        kickerMap.put(4.5, 2690.0);
        kickerMap.put(5.0, 2850.0);
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

    private AngularVelocity getLastLauncherVelocity() {
        return lastLauncherVelocity;
    }

    private Distance getLastLauncherDistanceTarget() {
        return lastLauncherDistanceTarget;
    }

    private AngularVelocity getLastKickerVelocity() {
        return lastKickerVelocity;
    }

    private Distance getLastKickerDistanceTarget() {
        return lastKickerDistanceTarget;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Predicted Launcher Velocity (RPM)", () -> getLastLauncherVelocity().in(RPM), null);
        builder.addDoubleProperty("Target Launcher Distance (Meters)", () -> getLastLauncherDistanceTarget().in(Meters),
                null);
        builder.addDoubleProperty("Predicted Kicker Velocity (RPM)", () -> getLastKickerVelocity().in(RPM), null);
        builder.addDoubleProperty("Target Kicker Distance (Meters)", () -> getLastKickerDistanceTarget().in(Meters),
                null);
    }
}