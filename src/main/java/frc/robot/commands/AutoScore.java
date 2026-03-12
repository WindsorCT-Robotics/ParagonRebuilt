package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.utils.LaunchCalculator;

public class AutoScore extends ParallelCommandGroup {
        public AutoScore(
                        Drive drive,
                        Supplier<Dimensionless> x,
                        Supplier<Dimensionless> y,
                        Shooter shooter,
                        Kicker kicker,
                        Spindexer spindexer,
                        LaunchCalculator launchCalculator,
                        Supplier<Dimensionless> velocityAdjustment,
                        Trigger unstuckFuel
        ) {
                addCommands(
                                new LaunchFuelToHubDistance(
                                                drive,
                                                shooter,
                                                kicker,
                                                spindexer,
                                                launchCalculator,
                                                velocityAdjustment,
                                                unstuckFuel)
                                                .alongWith(drive.angleToHub(x, y)));
        }
}