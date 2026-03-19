package frc.robot.commands;

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

public class AutoScore extends ParallelCommandGroup {

        /**
         * Scores into the hub with SAFETYGUARDS
         * 
         * @param drive
         * @param x
         * @param y
         * @param unstuckFuel
         * @param isAligned
         * @param onAllianceSide
         * @param shooter
         * @param kicker
         * @param spindexer
         * @param launchCalculator
         * @param velocityAdjustment
         */
        public AutoScore(
                        Drive drive,
                        Supplier<Dimensionless> x,
                        Supplier<Dimensionless> y,
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
                                new LaunchFuelToHub(
                                                shooter,
                                                kicker,
                                                spindexer,
                                                launchCalculator,
                                                manualUnstuckFuel,
                                                overrideNearLauncherAtTargetRPM,
                                                nearLauncherTargetRPM,
                                                isAligned,
                                                onAllianceSide,
                                                indexTargetVelocity,
                                                velocityAdjustment)
                                                .alongWith(drive.angleToHub(x, y)));
        }

        /**
         * Attermps to score into the hub REGUARDLESS
         * 
         * @param drive
         * @param x
         * @param y
         * @param shooter
         * @param kicker
         * @param spindexer
         * @param launchCalculator
         * @param velocityAdjustment
         */
        public AutoScore(
                        Drive drive,
                        Supplier<Dimensionless> x,
                        Supplier<Dimensionless> y,
                        Shooter shooter,
                        Kicker kicker,
                        Spindexer spindexer,
                        LaunchCalculator launchCalculator,
                        Trigger manualUnstuckFuel,
                        Trigger overrideNearLauncherAtTargetRPM,
                        Trigger nearLauncherTargetRPM,
                        Supplier<AngularVelocity> indexTargetVelocity,
                        Supplier<Dimensionless> velocityAdjustment) {
                addCommands(
                                new LaunchFuelToHub(
                                                shooter,
                                                kicker,
                                                spindexer,
                                                launchCalculator,
                                                nearLauncherTargetRPM,
                                                overrideNearLauncherAtTargetRPM,
                                                manualUnstuckFuel,
                                                indexTargetVelocity,
                                                velocityAdjustment)
                                                .alongWith(drive.angleToHub(x, y)));
        }
}