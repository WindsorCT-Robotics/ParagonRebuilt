package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.utils.LaunchCalculator;

public class LaunchFuelToHubDistance extends ParallelCommandGroup {
        public LaunchFuelToHubDistance(
                        LaunchCalculator launchCalculator,
                        AngularVelocity velocityThreshold,
                        Supplier<Boolean> isAligned,
                        Shooter shooter,
                        Kicker kicker,
                        Spindexer spindexer) {
                addCommands(
                                shooter.launchFuel(
                                                () -> launchCalculator.getShooterVelocityToHub()),
                                kicker.kickFuel(
                                                () -> launchCalculator.getKickerVelocityToHub()),
                                spindexer.indexFuelAtFlyWheelVelocityToHub(
                                                () -> RPM.of(1250),
                                                () -> shooter.getLaunchVelocity(),
                                                velocityThreshold,
                                                isAligned));
        }

}
