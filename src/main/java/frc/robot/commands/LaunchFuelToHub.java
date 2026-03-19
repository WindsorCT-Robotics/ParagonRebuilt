package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.utils.LaunchCalculator;

public class LaunchFuelToHub extends ParallelCommandGroup {
        private static final AngularVelocity VELOCITY_THRESHOLD = RPM.of(40);
        private static final AngularVelocity MAX_VELOCITY_ADJUSTMENT = RPM.of(100);

        /**
         * Has SAFEGUARDS to prevent shooting and launch fuel at requested distance.
         * 
         * @param shooter
         * @param kicker
         * @param spindexer
         * @param unstuckFuel
         * @param isAligned
         * @param onAllianceSide
         * @param launchCalculator
         * @param velocityAdjustment
         */
        public LaunchFuelToHub(
                        Shooter shooter,
                        Kicker kicker,
                        Spindexer spindexer,
                        LaunchCalculator launchCalculator,
                        Trigger manualUnstuckFuel,
                        Trigger overrideNearLauncherAtTargetRPM,
                        Trigger nearLauncherTargetRPM,
                        Trigger isAligned,
                        Trigger onAllianceSide,
                        Supplier<AngularVelocity> indexTargetVelocity,
                        Supplier<Dimensionless> velocityAdjustment) {
                addCommands(
                                shooter.launchFuelAdjustableToHub(
                                                () -> launchCalculator.getShooterVelocityToHub(),
                                                velocityAdjustment,
                                                MAX_VELOCITY_ADJUSTMENT,
                                                onAllianceSide),
                                kicker.kickFuelToHub(
                                                () -> launchCalculator.getKickerVelocityToHub(),
                                                onAllianceSide),
                                spindexer.indexFuelAtFlyWheelVelocityToHub(
                                                indexTargetVelocity,
                                                nearLauncherTargetRPM,
                                                overrideNearLauncherAtTargetRPM,
                                                isAligned,
                                                manualUnstuckFuel,
                                                onAllianceSide));
        }

        /**
         * Has NO SAFEGUARDS to prevent shooting and will launch fuel at requested
         * distance.
         * 
         * @param shooter
         * @param kicker
         * @param spindexer
         * @param launchCalculator
         * @param velocityAdjustment
         */
        public LaunchFuelToHub(
                        Shooter shooter,
                        Kicker kicker,
                        Spindexer spindexer,
                        LaunchCalculator launchCalculator,
                        Trigger nearLauncherTargetRPM,
                        Trigger overrideNearLauncherAtTargetRPM,
                        Trigger manualUnstuckFuel,
                        Supplier<AngularVelocity> indexTargetVelocity,
                        Supplier<Dimensionless> velocityAdjustment) {
                Trigger onAllianceSide = new Trigger(() -> true);
                Trigger isAligned = new Trigger(() -> true);
                addCommands(
                                shooter.launchFuelAdjustableToHub(
                                                () -> launchCalculator.getShooterVelocityToHub(),
                                                velocityAdjustment,
                                                MAX_VELOCITY_ADJUSTMENT,
                                                onAllianceSide),
                                kicker.kickFuelToHub(
                                                () -> launchCalculator.getKickerVelocityToHub(),
                                                onAllianceSide),
                                spindexer.indexFuelAtFlyWheelVelocityToHub(
                                                indexTargetVelocity,
                                                nearLauncherTargetRPM,
                                                overrideNearLauncherAtTargetRPM,
                                                isAligned,
                                                manualUnstuckFuel,
                                                onAllianceSide));
        }

        /**
         * Has NO SAFEGUARDS and was meant for autonomous.
         * 
         * @param shooter
         * @param kicker
         * @param spindexer
         * @param launchCalculator
         * @param velocityAdjustment
         */
        public LaunchFuelToHub(
                        Shooter shooter,
                        Kicker kicker,
                        Spindexer spindexer,
                        LaunchCalculator launchCalculator,
                        Trigger nearLauncherTargetRPM,
                        Trigger overrideNearLauncherAtTargetRPM,
                        Supplier<AngularVelocity> indexTargetVelocity,
                        Supplier<Dimensionless> velocityAdjustment) {
                Trigger onAllianceSide = new Trigger(() -> true);
                Trigger unstuckFuel = new Trigger(() -> false);
                Trigger isAligned = new Trigger(() -> true);

                addCommands(
                                shooter.launchFuelAdjustableToHub(
                                                () -> launchCalculator.getShooterVelocityToHub(),
                                                velocityAdjustment,
                                                MAX_VELOCITY_ADJUSTMENT,
                                                onAllianceSide),
                                kicker.kickFuelToHub(
                                                () -> launchCalculator.getKickerVelocityToHub(),
                                                onAllianceSide),
                                spindexer.indexFuelAtFlyWheelVelocityToHub(
                                                indexTargetVelocity,
                                                nearLauncherTargetRPM,
                                                overrideNearLauncherAtTargetRPM,
                                                isAligned,
                                                unstuckFuel,
                                                onAllianceSide));
        }
}
