package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;

public class Drive { // TODO: Extend generated swervedrive after generated swervedrive code is
                     // created.

    private static final LinearVelocity MAX_LINEAR_VELOCITY = MetersPerSecond.of(1); // TODO: Edit value to actual max
                                                                                     // speed.
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(1); // TODO: Edit value to actual
                                                                                        // max

    public Drive() {

    }

    private enum RelativeReference {
        ROBOT_RELATIVE,
        FIELD_RELATIVE
    }

    private Command robotCentricMove(
            Supplier<LinearVelocity> x,
            Supplier<LinearVelocity> y,
            Supplier<AngularVelocity> rotateRate) {
        return run(() -> setControl(
                new RobotCentric()
                        .withVelocityX(x.get())
                        .withVelocityY(y.get())
                        .withRotationalRate(rotateRate.get())));
    }

    private Command fieldCentricMove(
            Supplier<LinearVelocity> x,
            Supplier<LinearVelocity> y,
            Supplier<AngularVelocity> rotateRate) {
        return run(() -> setControl(
                new FieldCentric()
                        .withVelocityX(x.get())
                        .withVelocityY(y.get())
                        .withRotationalRate(rotateRate.get())));
    }

    public Command move(
            Supplier<LinearVelocity> x,
            Supplier<LinearVelocity> y,
            Supplier<AngularVelocity> rotateRate,
            RelativeReference reference) {
        switch (reference) {
            case ROBOT_RELATIVE:
                return robotCentricMove(x, y, rotateRate);
            case FIELD_RELATIVE:
                return fieldCentricMove(x, y, rotateRate);
            default:
                throw new IllegalArgumentException(
                        "Unable to determine the SwerveRequest return. Illegal RelativeReference: " + reference);
        }
    }

    private LinearVelocity percentageToLinearVelocity(LinearVelocity velocity, Supplier<Dimensionless> percent) {
        return velocity.times(percent.get());
    }

    private AngularVelocity percentToAngularVelocity(AngularVelocity velocity, Supplier<Dimensionless> percent) {
        return velocity.times(percent.get());
    }

    public Command moveWithPercentages(
            Supplier<Dimensionless> x,
            Supplier<Dimensionless> y,
            Supplier<Dimensionless> rotateRate,
            RelativeReference reference) {
        return move(
                () -> percentageToLinearVelocity(MAX_LINEAR_VELOCITY, x),
                () -> percentageToLinearVelocity(MAX_LINEAR_VELOCITY, y),
                () -> percentToAngularVelocity(MAX_ANGULAR_VELOCITY, rotateRate),
                reference);
    }

    private Command turnToTargetAngle(
            Supplier<Angle> targetAngle,
            Supplier<LinearVelocity> x,
            Supplier<LinearVelocity> y,
            Supplier<AngularVelocity> rotateRate) {

    }
}
