package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.drive.Drive;
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
                        Trigger unstuckFuel,
                        Trigger isAligned,
                        Trigger onAllianceSide,
                        Shooter shooter,
                        Kicker kicker,
                        Spindexer spindexer,
                        LaunchCalculator launchCalculator,
                        Supplier<AngularVelocity> indexTargetVelocity,
                        Supplier<Dimensionless> velocityAdjustment) {
                addCommands(
                                new LaunchFuelToHub(
                                                shooter,
                                                kicker,
                                                spindexer,
                                                unstuckFuel,
                                                isAligned,
                                                onAllianceSide,
                                                launchCalculator,
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
                        Supplier<AngularVelocity> indexTargetVelocity,
                        Supplier<Dimensionless> velocityAdjustment) {
                addCommands(
                                new LaunchFuelToHub(
                                                shooter,
                                                kicker,
                                                spindexer,
                                                launchCalculator,
                                                indexTargetVelocity,
                                                velocityAdjustment)
                                                .alongWith(drive.angleToHub(x, y)));
        }
}