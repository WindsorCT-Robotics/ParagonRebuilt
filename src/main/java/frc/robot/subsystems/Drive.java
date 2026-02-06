package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.GeneratedDrive;
import frc.robot.result.Failure;
import frc.robot.result.Result;
import frc.robot.result.Success;

public class Drive extends GeneratedDrive {
    // TODO: Max velocities should be properly tested.
    private static final LinearVelocity MAX_LINEAR_VELOCITY = MetersPerSecond.of(1);
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(1);
    private static final PIDConstants DEFAULT_TARGET_DIRECTION_PID = new PIDConstants(7, 0, 0);
    private static final Angle ALLIANCE_BLUE_SIDE = Degrees.of(0.0);
    private static final Angle ALLIANCE_RED_SIDE = Degrees.of(180.0);

    private final RobotConfig robotConfiguration;
    private final ChassisSpeeds currentSpeeds;
    private final SwerveModuleState[] currentStates;
    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    public sealed interface CommandError permits AllianceUnknown {
    }

    public record AllianceUnknown() implements CommandError {
    }

    public Drive(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) throws IOException, ParseException {

        super(drivetrainConstants, modules);

        robotConfiguration = RobotConfig.fromGUISettings();

        currentSpeeds = getState().Speeds;
        currentStates = getState().ModuleStates;
        setpointGenerator = new SwerveSetpointGenerator(robotConfiguration, MAX_ANGULAR_VELOCITY);
        previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates,
                DriveFeedforwards.zeros(getModules().length));
    }

    private enum RelativeReference {
        ROBOT_RELATIVE,
        FIELD_RELATIVE
    }

    private Command robotCentricMove(
            Supplier<LinearVelocity> x,
            Supplier<LinearVelocity> y,
            Supplier<AngularVelocity> rotateRate) {
        return applyRequest(() -> new RobotCentric()
                .withVelocityX(x.get())
                .withVelocityY(y.get())
                .withRotationalRate(rotateRate.get()));
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

    /**
     * Moves with the ability to control rotation with with a target angle.
     * 
     * @param x
     * @param y
     * @param targetAngle
     */
    private void moveWithLockedAngle(
            LinearVelocity x,
            LinearVelocity y,
            Angle targetAngle) {
        setControl(
                new FieldCentricFacingAngle()
                        .withVelocityX(x)
                        .withVelocityY(y)
                        .withHeadingPID(DEFAULT_TARGET_DIRECTION_PID.kP, DEFAULT_TARGET_DIRECTION_PID.kI,
                                DEFAULT_TARGET_DIRECTION_PID.kD)
                        .withTargetDirection(new Rotation2d(targetAngle.in(Degrees))));
    }

    public Result<Command, CommandError> angleToOutpost(
            Supplier<LinearVelocity> x,
            Supplier<LinearVelocity> y) {

        if (DriverStation.getAlliance().isEmpty()) {
            return new Failure<>(new AllianceUnknown());
        }

        return new Success<>(
                run(() -> {
                    Alliance alliance = DriverStation.getAlliance().orElseThrow();
                    Angle targetAngle;

                    if (alliance.equals(Alliance.Blue)) {
                        targetAngle = ALLIANCE_BLUE_SIDE;
                    } else {
                        targetAngle = ALLIANCE_RED_SIDE;
                    }

                    moveWithLockedAngle(x.get(), y.get(), targetAngle);
                }));
    }

    private void setModuleStates(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < moduleStates.length; i++) {
            ModuleRequest moduleRequest = new ModuleRequest().withState(moduleStates[i]);
            getModule(i).apply(moduleRequest);
        }
    }

    private void driveTorqueBased(ChassisSpeeds speeds) {
        previousSetpoint = setpointGenerator.generateSetpoint(
                previousSetpoint,
                speeds,
                TimedRobot.kDefaultPeriod);
        setModuleStates(previousSetpoint.moduleStates());
    }

    public Command moveTorqueBased() {

    }
}
