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
                                                () -> launchCalculator.getShooterVelocityToHub().minus(RPM.of(20))),
                                kicker.kickFuel(
                                                () -> launchCalculator.getKickerVelocityToHub().minus(RPM.of(20))),
                                spindexer.indexFuelAtFlyWheelVelocityToHub(
                                                () -> RPM.of(1250),
                                                () -> shooter.getLaunchVelocity(),
                                                velocityThreshold,
                                                isAligned));
        }

}
