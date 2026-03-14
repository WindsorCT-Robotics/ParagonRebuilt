package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.utils.LaunchCalculator;

public class LaunchFuelToTargetDistance extends ParallelCommandGroup {
        public LaunchFuelToTargetDistance(
                        LaunchCalculator launchCalculator,
                        Supplier<Distance> launchTo,
                        AngularVelocity velocityThreshold,
                        Shooter shooter,
                        Kicker kicker,
                        Spindexer spindexer) {
                addCommands(
                                shooter.launchFuel(
                                                () -> launchCalculator.getShooterVelocityToDistance(launchTo.get())),
                                kicker.kickFuel(
                                                () -> launchCalculator.getKickerVelocityToDistance(launchTo.get())),
                                spindexer.indexFuelAtFlyWheelVelocity(
                                                () -> RPM.of(1250),
                                                () -> shooter.getLaunchVelocity(),
                                                velocityThreshold));
        }
}
