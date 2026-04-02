package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.generated.launch_calculator.ShotCalculator.LaunchParameters;
import frc.robot.subsystems.BayDoor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Spindexer;

public class AutoScore extends ParallelCommandGroup {
    public AutoScore(
            Drive drive,
            Launcher launcher,
            Kicker kicker,
            Spindexer spindexer,
            BayDoor bayDoor,
            Intake intake,
            Supplier<Optional<LaunchParameters>> parameters,
            Supplier<Dimensionless> x,
            Supplier<Dimensionless> y) {
        
        Supplier<AngularVelocity> launchVelocity = () -> {
            if (parameters.get().isEmpty()) {
                return RPM.zero();
            }

            LaunchParameters launch = parameters.get().get();

            SmartDashboard.putBoolean("Launch Is Valid", launch.isValid());
            SmartDashboard.putNumber("Cool", launch.confidence());
            if (launch.isValid() && launch.confidence() > 0.5) {
                return RPM.of(launch.rpm());
            }

            return RPM.zero();
        };

        Supplier<Optional<Angle>> targetAngle = () -> {
            if (parameters.get().isEmpty()) {
                return Optional.empty();
            }

            SmartDashboard.putNumber("Drive Angle", parameters.get().get().driveAngle().getMeasure().in(Degrees));
            // SmartDashboard.putNumber("", parameters.get().get());
            return Optional.of(parameters.get().get().driveAngle().getMeasure());
        };

        addCommands(
            drive.aimTo(x, y, targetAngle)
            .alongWith(launcher.launchFuel(launchVelocity)
            .alongWith(kicker.kickFuel(launchVelocity))
            .alongWith(spindexer.indexFuel()))
        );
    }
}