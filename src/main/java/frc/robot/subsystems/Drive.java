package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.GeneratedDrive;

public class Drive extends GeneratedDrive {
    // TODO: Max velocities should be properly tested.
    private static final LinearVelocity MAX_LINEAR_VELOCITY = MetersPerSecond.of(1);
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(1);

    public Drive(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
    }

    public Drive(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
    }

    public Drive(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
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
}