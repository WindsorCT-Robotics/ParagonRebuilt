package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.HubUtil;
import frc.robot.generated.launch_calculator.ShotCalculator;
import frc.robot.generated.launch_calculator.ShotCalculator.LaunchParameters;
import frc.robot.subsystems.BayDoor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Drive.RelativeReference;
import frc.robot.utils.AngleUtil;
import frc.robot.utils.ControllerUtil;

public class RobotContainer implements Sendable {
        private static final Dimensionless DRIVER_CONTROLLER_DEADBAND = Percent.of(5);
        private static final LinearVelocity MAX_SPEED_LAUNCH = MetersPerSecond.of(2); // Max speed of the robot when
                                                                                      // aiming
        private static final double MOVE_CURVE = 2.0;
        private static final double TURN_CURVE = 2.0;

        private final Drive drive;
        private final Spindexer spindexer;
        private final BayDoor bayDoor;
        private final Intake intake;
        private final Kicker kicker;
        private final Launcher launcher;

        private final CommandXboxController driver = new CommandXboxController(0);
        private final CommandXboxController operator = new CommandXboxController(1);

        private final Supplier<Dimensionless> moveX = () -> ControllerUtil
                        .getAxisWithDeadBandAndCurve(driver.getLeftX(), DRIVER_CONTROLLER_DEADBAND, MOVE_CURVE);
        private final Supplier<Dimensionless> moveY = () -> ControllerUtil
                        .getAxisWithDeadBandAndCurve(driver.getLeftY(), DRIVER_CONTROLLER_DEADBAND, MOVE_CURVE);
        private final Supplier<Dimensionless> turnX = () -> ControllerUtil
                        .getAxisWithDeadBandAndCurve(driver.getRightX(), DRIVER_CONTROLLER_DEADBAND, TURN_CURVE);

        private final ShotCalculator hubCalculator;
        private final ShotCalculator snowBlowCalculator;

        private Optional<ShotCalculator.LaunchParameters> hubLaunchParameters = Optional.empty();
        private Optional<ShotCalculator.LaunchParameters> snowBlowLaunchParameters = Optional.empty();

        private final SendableChooser<Command> autoChooser;

        private RelativeReference relativeReference;

        private final Bindings bindings;

        public RobotContainer() {
                try {
                        drive = new Drive(Drive.class.getSimpleName());
                } catch (IOException | ParseException e) {
                        throw new IllegalStateException("PathPlanner Configuration failed to load.", e);
                }

                intake = new Intake(Intake.class.getSimpleName());
                bayDoor = new BayDoor(BayDoor.class.getSimpleName());
                spindexer = new Spindexer(Spindexer.class.getSimpleName());
                launcher = new Launcher(Launcher.class.getSimpleName());
                kicker = new Kicker(Kicker.class.getSimpleName());

                relativeReference = RelativeReference.FIELD_CENTRIC;

                hubCalculator = new ShotCalculator();
                hubCalculator.loadLUTEntry(1.5, 2000, 1);
                hubCalculator.loadLUTEntry(2.0, 2125, 1.05);
                hubCalculator.loadLUTEntry(2.5, 2225, 1.1);
                hubCalculator.loadLUTEntry(3, 2325, 1.15);
                hubCalculator.loadLUTEntry(3.5, 2450, 1.2);
                hubCalculator.loadLUTEntry(4, 2590, 1.25);
                hubCalculator.loadLUTEntry(4.5, 2690, 1.3);
                hubCalculator.loadLUTEntry(5, 2850, 1.35);

                snowBlowCalculator = new ShotCalculator();
                snowBlowCalculator.loadLUTEntry(1.5, 2000, 1);
                snowBlowCalculator.loadLUTEntry(2.0, 2125, 1);
                snowBlowCalculator.loadLUTEntry(2.5, 2225, 1);
                snowBlowCalculator.loadLUTEntry(3, 2325, 1);
                snowBlowCalculator.loadLUTEntry(3.5, 2450, 1);
                snowBlowCalculator.loadLUTEntry(4, 2590, 1);
                snowBlowCalculator.loadLUTEntry(4.5, 2690, 1);
                snowBlowCalculator.loadLUTEntry(5, 2850, 1);

                bindings = new Bindings(
                                drive,
                                launcher,
                                kicker,
                                spindexer,
                                bayDoor,
                                intake,
                                driver,
                                operator,
                                () -> hubLaunchParameters,
                                () -> snowBlowLaunchParameters);

                autoChooser = AutoBuilder.buildAutoChooser();
                initPathPlannerCommands();
                initSmartdashBoard();

                bindCommands();
        }

        // region non-bindings
        private void initSmartdashBoard() {
                SmartDashboard.putData(drive);
                SmartDashboard.putData(intake);
                SmartDashboard.putData(bayDoor);
                SmartDashboard.putData(spindexer);
                SmartDashboard.putData(launcher);
                SmartDashboard.putData(kicker);
                SmartDashboard.putData(CommandScheduler.getInstance());

                SmartDashboard.putData("Robot Container", this);
                SmartDashboard.putData("Robot Container/Controllers/Driver", driver.getHID());
                SmartDashboard.putData("Robot Container/Controllers/Operator", operator.getHID());
                SmartDashboard.putData("Robot Container/Autonomous", autoChooser);
                SmartDashboard.putData("Robot Container/Hub Calculator", hubCalculator);
                SmartDashboard.putData("Robot Container/SnowBlow Calculator", snowBlowCalculator);
                SmartDashboard.putData("Hub", HubUtil.getInstance());
        }

        @Override
        public void initSendable(SendableBuilder builder) {

        }

        private LaunchParameters getLaunchParametersFor(ShotCalculator calculator, Translation2d target) {
                SwerveDriveState driveState = drive.getState();
                Pigeon2 gyro = drive.getPigeon2();

                Pose2d drivePosition = drive.getRawPosition();
                Angle pitch = AngleUtil.wrap(gyro.getPitch().getValue());
                Angle yaw = AngleUtil.wrap(gyro.getYaw().getValue());
                Angle roll = AngleUtil.wrap(gyro.getRoll().getValue());

                ChassisSpeeds robotSpeeds = driveState.Speeds;
                ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds,
                                new Rotation2d(yaw));

                ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
                                drivePosition,
                                fieldSpeeds,
                                robotSpeeds,
                                target,
                                0.9,
                                pitch.in(Degrees),
                                roll.in(Degrees));

                return calculator.calculate(inputs);
        }

        public void updateLaunchParameters() {
                hubLaunchParameters = DriverStation.getAlliance()
                                .map(alliance -> getLaunchParametersFor(hubCalculator, drive.getHubTarget(alliance)));
                snowBlowLaunchParameters = DriverStation.getAlliance()
                                .map(alliance -> getLaunchParametersFor(snowBlowCalculator,
                                                drive.getSnowblowTarget(alliance)));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        private void initPathPlannerCommands() {
                NamedCommands.registerCommand("score",
                                drive.aimToWithFF(
                                                () -> Percent.zero(),
                                                () -> Percent.zero(),
                                                () -> angleToHub(),
                                                () -> angleToHubWithFF(),
                                                MetersPerSecond.zero())
                                                .alongWith(launcher.launchFuel(() -> launchVelocityToHub()))
                                                .alongWith(kicker.kickFuel(() -> launchVelocityToHub())
                                                                .alongWith(spindexer.indexFuel())
                                                                .alongWith(bayDoor.agitateFuel())));

                NamedCommands.registerCommand("scorenoaim",
                launcher.launchFuel(() -> launchVelocityToHub())
                                                .alongWith(kicker.kickFuel(() -> launchVelocityToHub())
                                                                .alongWith(spindexer.indexFuel())
                                                                .alongWith(bayDoor.agitateFuel())));
                NamedCommands.registerCommand("baydooropen", bayDoor.open());
                NamedCommands.registerCommand("baydoorclose", bayDoor.close());
                NamedCommands.registerCommand("intakefuel", intake.intakeFuel());
        }

        private void switchRelativeReference() {
                if (relativeReference == RelativeReference.FIELD_CENTRIC) {
                        relativeReference = RelativeReference.ROBOT_CENTRIC;
                } else {
                        relativeReference = RelativeReference.FIELD_CENTRIC;
                }
        }

        private RelativeReference getRelativeReference() {
                return relativeReference;
        }

        private Command angleToRedAlliance() {
                return drive.angleToRedAlliance(moveX, moveY);
        }

        private AngularVelocity launchVelocityToHub() {
                return hubLaunchParameters.map(parameters -> RPM.of(parameters.rpm())).orElse(RPM.zero());
        }

        private Optional<Angle> angleToHub() {
                Optional<Rotation2d> angle = hubLaunchParameters.map(parameters -> parameters.driveAngle());
                if (angle.isPresent()) {
                }
                return Optional.of(Degrees.of(angle.get().getDegrees()));
        }

        private Optional<AngularVelocity> angleToHubWithFF() {
                return hubLaunchParameters
                                .map(parameters -> RadiansPerSecond.of(parameters.driveAngularVelocityRadPerSec()));
        }

        private AngularVelocity launchVelocityToSnowBlow() {
                return snowBlowLaunchParameters.map(parameters -> RPM.of(parameters.rpm())).orElse(RPM.zero());
        }

        private Optional<Angle> angleToSnowBlow() {
                return snowBlowLaunchParameters.map(parameters -> parameters.driveAngle().getMeasure());
        }

        private Optional<AngularVelocity> angleToSnowBlowWithFF() {
                return snowBlowLaunchParameters
                                .map(parameters -> RadiansPerSecond.of(parameters.driveAngularVelocityRadPerSec()));
        }
        // endregion

        private void bindCommands() {
                drive.setDefaultCommand(
                                drive.moveWithPercentages(
                                                moveX,
                                                moveY,
                                                turnX,
                                                this::getRelativeReference)
                                                .withName("Drive With Percentages"));

                intake.setDefaultCommand(intake.stopIntake().withName("Stop Intake"));

                bindings.cmd_switchRelativeReference.onTrue(new InstantCommand(() -> switchRelativeReference()));

                bindings.cmd_prepareFuel.whileTrue(launcher.prepareFuel().withName("Launcher Prepare Fuel"));
                bindings.cmd_prepareFuel.whileTrue(kicker.prepareFuel().withName("Kicker Prepare Fuel"));
                bindings.cmd_prepareFuel.whileTrue(spindexer.prepareFuel().withName("Spindexer Prepare Fuel"));

                bindAutoScore();
                bindSnowBlow();
                bindPartialManualScore();
                bindManualScore();

                bindings.cmd_autoUnjam.whileTrue(
                                bayDoor.open()
                                                .alongWith(spindexer.agitateFuel())
                                                .withName("Auto Unjam"));

                bindings.cmd_manualUnjam.whileTrue(
                                bayDoor.open()
                                                .alongWith(spindexer.agitateFuel())
                                                .withName("Manual Unjam"));

                bindings.cmd_autoIntake.whileTrue(
                                bayDoor.open()
                                                .alongWith(intake.intakeFuel())
                                                .withName("Auto Intake"));
                bindings.cmd_autoShuttle.whileTrue(
                                bayDoor.open()
                                                .alongWith(intake.shuttleFuel())
                                                .withName("Auto Shuttle"));

                bindings.t_openBayDoor.onTrue(bayDoor.open()
                                .withName("Open Bay Door"));
                bindings.t_closeBayDoor.onTrue(bayDoor.close()
                                .withName("Close Bay Door"));

                bindings.t_faceRedAlliance.whileTrue(angleToRedAlliance()
                                .withName("Face Red Alliance"));

                bindings.t_incrementLauncherOffset
                                .onTrue(new InstantCommand(() -> hubCalculator.adjustOffset(RPM.of(25).in(RPM))));

                bindings.t_decrementLauncherOffset
                                .onTrue(new InstantCommand(() -> hubCalculator.adjustOffset(RPM.of(-25).in(RPM))));

                bindings.t_resetGyro.onTrue(drive.resetGyroCommand());
        }

        private void bindAutoScore() {
                bindings.cmd_autoScore_launchFuel
                                .whileTrue(launcher.launchFuel(() -> launchVelocityToHub())
                                                .withName("Launch Fuel To Hub"));

                bindings.cmd_autoScore_launchFuel
                                .whileTrue(kicker.kickFuel(() -> launchVelocityToHub())
                                                .withName("Kick Fuel To Hub"));

                bindings.cmd_autoScore_launchFuel
                                .whileTrue(bayDoor.agitateFuel()
                                                .withName("Agitate Bay Door Fuel To Hub"));

                bindings.cmd_autoScore_launchFuel
                                .whileTrue(intake.agitateFuel()
                                                .withName("Agitate Intake Fuel To Hub"));

                bindings.cmd_autoScore_indexFuel
                                .whileTrue(spindexer.indexFuel()
                                                .withName("Index Fuel To Hub"));

                bindings.t_autoScore
                                .whileTrue(
                                                drive.aimToWithFF(
                                                                moveX,
                                                                moveY,
                                                                () -> angleToHub(),
                                                                () -> angleToHubWithFF(),
                                                                MAX_SPEED_LAUNCH)
                                                                .withName("Auto Aim To Hub"));
        }

        private void bindSnowBlow() {
                bindings.t_autoSnowBlow.whileTrue(
                                launcher.launchFuel(() -> launchVelocityToSnowBlow())
                                                .withName("Launch Fuel To SnowBlow"));

                bindings.t_autoSnowBlow.whileTrue(
                                kicker.kickFuel(() -> launchVelocityToSnowBlow())
                                                .withName("Kick Fuel To SnowBlow"));

                bindings.t_autoSnowBlow.whileTrue(
                                spindexer.indexFuel()
                                                .withName("Index Fuel To SnowBlow"));

                bindings.t_autoSnowBlow.whileTrue(
                                drive.aimToWithFF(
                                                moveX,
                                                moveY,
                                                () -> angleToSnowBlow(),
                                                () -> angleToSnowBlowWithFF(),
                                                MAX_SPEED_LAUNCH)
                                                .withName("Auto Aim To SnowBlow"));

                bindings.t_autoSnowBlow.whileTrue(
                                bayDoor.open()
                                                .withName("Open Bay Door To SnowBlow"));

                bindings.t_autoSnowBlow.whileTrue(
                                intake.intakeFuel()
                                                .withName("Intake Fuel To SnowBlow"));
        }

        private void bindPartialManualScore() {
                bindings.t_partialManualScore.whileTrue(launcher.smartDashboardLaunchFuel());
                bindings.t_partialManualScore.whileTrue(kicker.smartDashboardKickFuel());
                bindings.t_partialManualScore.whileTrue(spindexer.indexFuel());
                bindings.t_partialManualScore.whileTrue(drive.aimTo(moveX, moveY, () -> angleToHub()));
        }

        private void bindManualScore() {
                bindings.t_manualScore.whileTrue(launcher.smartDashboardLaunchFuel());
                bindings.t_manualScore.whileTrue(kicker.smartDashboardKickFuel());
                bindings.t_manualScore.whileTrue(spindexer.indexFuel());
        }
}