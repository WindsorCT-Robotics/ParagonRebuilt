package frc.robot;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public record LUTEntry(Distance distance, AngularVelocity angularVelocity, Time timeOfFlight) {
}
