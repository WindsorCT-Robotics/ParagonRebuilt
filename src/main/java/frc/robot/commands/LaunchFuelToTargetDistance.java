package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.utils.LaunchCalculator;

public class LaunchFuelToTargetDistance extends ParallelCommandGroup {
        public LaunchFuelToTargetDistance(
                        AngularVelocity velocityThreshold,
                        Supplier<Pose2d> robotPosition,
                        Shooter shooter,
                        Kicker kicker,
                        Spindexer spindexer) {
                LaunchCalculator launchCalculator = new LaunchCalculator(robotPosition);
                addCommands(
                                new ParallelCommandGroup(
                                                shooter.shootFuel(() -> launchCalculator
                                                                .getShooterVelocity()),
                                                kicker.kickStartFuel(() -> launchCalculator
                                                                .getKickerVelocity()),
                                                spindexer.indexFuelAtFlyWheelVelocity(() -> RPM.of(240),
                                                                () -> shooter.getShootTargetVelocity(),
                                                                velocityThreshold)));
        }

}
