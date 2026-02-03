package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;

public class Drive { // TODO: Extend generated swervedrive after generated swervedrive code is
                     // created.

    public Drive() {

    }

    private enum RelativeReference {
        ROBOT_RELATIVE,
        FIELD_RELATIVE
    }

    public Command move(
            Supplier<LinearVelocity> velocityX,
            Supplier<LinearVelocity> velocityY,
            Supplier<AngularVelocity> rotateRate,
            RelativeReference reference) {
        return run(() -> {
            switch (reference) {
                case ROBOT_RELATIVE:
                    return robotRelativeSwerveRequest(velocityX, velocityY, rotateRate);
                case FIELD_RELATIVE:
                    return fieldRelativeSwerveRequest(velocityX, velocityY, rotateRate);
                default:
                    throw new IllegalArgumentException(
                            "Unable to determine the SwerveRequest return. Illegal RelativeReference: " + reference);
            }
        });
    }

    private

    public Command moveWithPercentages(
            Supplier<Dimensionless> x,
            Supplier<Dimensionless> y,
            Supplier<Dimensionless> rotateRate) {

    }
}
