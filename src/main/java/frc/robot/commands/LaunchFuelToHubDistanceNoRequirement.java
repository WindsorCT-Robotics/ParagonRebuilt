package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.utils.LaunchCalculator;

public class LaunchFuelToHubDistanceNoRequirement extends ParallelCommandGroup {
        private static final AngularVelocity VELOCITY_THRESHOLD = RPM.of(40);

        public LaunchFuelToHubDistanceNoRequirement(
                        Drive drive,
                        Shooter shooter,
                        Kicker kicker,
                        Spindexer spindexer,
                        LaunchCalculator launchCalculator,
                        Supplier<Dimensionless> velocityAdjustment,
                        Trigger unstuckFuel) {
                Trigger isAligned = new Trigger(() -> true);
                Trigger onAllianceSide = new Trigger(() -> true);

                addCommands(
                                shooter.launchFuelAdjustableToHub(
                                                () -> launchCalculator.getShooterVelocityToHub(),
                                                velocityAdjustment,
                                                RPM.of(100),
                                                onAllianceSide),
                                kicker.kickFuelToHub(
                                                () -> launchCalculator.getKickerVelocityToHub(),
                                                onAllianceSide),
                                spindexer.indexFuelAtFlyWheelVelocityToHub(
                                                () -> RPM.of(1250),
                                                () -> shooter.getLaunchVelocity(),
                                                VELOCITY_THRESHOLD,
                                                isAligned,
                                                unstuckFuel,
                                                onAllianceSide));
        }

}
