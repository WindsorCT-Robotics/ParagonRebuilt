
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.RPM;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.hardware.CanId;
import frc.robot.hardware.DigitalInputOutput;
import frc.robot.subsystems.BayDoor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Drive.RelativeReference;
import frc.robot.utils.LaunchCalculator;

public class RobotContainer implements Sendable {
  // private static final LinearVelocity MAX_SPEED =
  // TunerConstants.kSpeedAt12Volts;
  // private final Telemetry logger;

  private final Drive drive;
  private final Intake intake;
  private final BayDoor bayDoor;
  private final Spindexer spindexer;
  private final Launcher launcher;
  private final Kicker kicker;

  private final LaunchCalculator launchCalculator;

  private static final CanId INTAKE_ROLLER_MOTOR_CAN_ID = new CanId((byte) 16);

  private static final CanId BAYDOOR_MOTOR_LEFT_CAN_ID = new CanId((byte) 14);
  private static final CanId BAYDOOR_MOTOR_RIGHT_CAN_ID = new CanId((byte) 15);
  private static final DigitalInputOutput LEFT_BAYDOOR_DIO = new DigitalInputOutput((byte) 0);
  private static final DigitalInputOutput RIGHT_BAYDOOR_DIO = new DigitalInputOutput((byte) 1);

  private static final CanId SPINDEXER_MOTOR_CAN_ID = new CanId((byte) 13);
  private static final CanId FUEL_SENSOR_CAN_ID = new CanId((byte) 21);

  private static final CanId KICKER_MOTOR_CAN_ID = new CanId((byte) 17);

  private static final CanId LAUNCHER_MOTOR_LEFT_CAN_ID = new CanId((byte) 18);
  private static final CanId LAUNCHER_MOTOR_RIGHT_CAN_ID = new CanId((byte) 19);

  private static final Distance LAUNCHER_CLOSE_DISTANCE = Meters.of(1.5);
  private static final Distance LAUNCHER_TRENCH_DISTANCE = Inches.of(158.84).minus(Inches.of(49.84).div(2));

  private static final Dimensionless REDUCE_SPEED = Percent.of(50);
  private static final double MOVE_ROBOT_CURVE = 2.0;
  private static final double TURN_ROBOT_CURVE = 2.0;
  private static final Dimensionless DEADBAND = Percent.of(5);
  private final CommandXboxController driver;
  private final CommandXboxController operator;

  private final Supplier<Dimensionless> driverLeftAxisX;
  private final Supplier<Dimensionless> driverLeftAxisY;
  private final Supplier<Dimensionless> driverRightAxisX;
  private final Supplier<Dimensionless> driverRightTrigger;
  private final Supplier<Dimensionless> driverLeftTrigger;

  private final Trigger autoScoreTrigger;
  private final Trigger autoScoreNoCalculationTrigger;
  private final Trigger agitateHighFuelTrigger;
  private final Trigger snowblowTrigger;
  private final Trigger autoUnjamTrigger;
  private final Trigger manualUnjamTrigger;
  private final Trigger incrementLauncherOffset;
  private final Trigger decrementLauncherOffset;
  private final Trigger faceRedAlliance;

  private final Trigger manualCloseScoreTrigger;
  private final Trigger manualTrenchScoreTrigger;

  private final Trigger bayDoorHomeTrigger;
  private final Trigger bayDoorOpenTrigger;
  private final Trigger bayDoorCloseTrigger;

  private RelativeReference relativeReference;

  private final SendableChooser<Command> autonomousChooser;
  private static final String DEFAULT_AUTO = "";

  public RobotContainer() {
    try {
      drive = new Drive(
          Drive.class.getSimpleName(),
          "limelight",
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);
    } catch (IOException | ParseException e) {
      throw new IllegalStateException("PathPlanner Configuration failed to load.", e);
    }

    intake = new Intake(
        Intake.class.getSimpleName(),
        INTAKE_ROLLER_MOTOR_CAN_ID);

    bayDoor = new BayDoor(
        BayDoor.class.getSimpleName(),
        BAYDOOR_MOTOR_LEFT_CAN_ID,
        BAYDOOR_MOTOR_RIGHT_CAN_ID,
        LEFT_BAYDOOR_DIO,
        RIGHT_BAYDOOR_DIO);

    spindexer = new Spindexer(
        Spindexer.class.getSimpleName(),
        SPINDEXER_MOTOR_CAN_ID,
        FUEL_SENSOR_CAN_ID);

    launcher = new Launcher(
        Launcher.class.getSimpleName(),
        LAUNCHER_MOTOR_LEFT_CAN_ID,
        LAUNCHER_MOTOR_RIGHT_CAN_ID);

    kicker = new Kicker(
        Kicker.class.getSimpleName(),
        KICKER_MOTOR_CAN_ID);

    launchCalculator = new LaunchCalculator();

    driver = new CommandXboxController(0);
    operator = new CommandXboxController(1);

    registerPathplannerCommands();
    autonomousChooser = AutoBuilder.buildAutoChooser(DEFAULT_AUTO);

    // logger = new Telemetry(MAX_SPEED.in(MetersPerSecond));
    // drive.registerTelemetry(logger::telemeterize);

    driverLeftAxisX = () -> Value.of(driver.getLeftX());
    driverLeftAxisY = () -> Value.of(driver.getLeftY());
    driverRightAxisX = () -> Value.of(driver.getRightX());
    driverRightTrigger = () -> Value.of(driver.getRightTriggerAxis());
    driverLeftTrigger = () -> Value.of(driver.getLeftTriggerAxis());

    snowblowTrigger = new Trigger(() -> driverRightTrigger.get().gt(Percent.of(20)));
    autoScoreTrigger = driver.rightBumper();
    autoScoreNoCalculationTrigger = operator.povLeft();
    manualCloseScoreTrigger = operator.b();
    manualTrenchScoreTrigger = operator.y();
    autoUnjamTrigger = spindexer.autoUnjamTrigger;
    manualUnjamTrigger = operator.a();
    incrementLauncherOffset = operator.rightBumper();
    decrementLauncherOffset = operator.leftBumper();
    faceRedAlliance = new Trigger(driver.leftStick());
    agitateHighFuelTrigger = new Trigger(() -> driverLeftTrigger.get().gt(Percent.of(20)));

    bayDoorHomeTrigger = operator.leftStick().and(operator.rightStick());
    bayDoorOpenTrigger = operator.povUp();
    bayDoorCloseTrigger = operator.povDown();

    relativeReference = RelativeReference.FIELD_CENTRIC;

    initSmartDashboard();
    configureControllerBindings();
  }

  private void initSmartDashboard() {
    SmartDashboard.putData(drive);
    SmartDashboard.putData(intake);
    SmartDashboard.putData(bayDoor);
    SmartDashboard.putData(spindexer);
    SmartDashboard.putData(launcher);
    SmartDashboard.putData(kicker);
    SmartDashboard.putData("Robot Container/Launch Calculations", launchCalculator);
    SmartDashboard.putData("Robot Container/Controllers/Driver", driver.getHID());
    SmartDashboard.putData("Robot Container/Controllers/Operator", operator.getHID());
    SmartDashboard.putData("Robot Container/Autonomous", autonomousChooser);
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData("Robot Container", this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("Relative Reference", () -> getRelativeReference().toString(), null);
    builder.addBooleanProperty("autoScoreTrigger", autoScoreTrigger, null);
    builder.addBooleanProperty("autoScoreNoCalculationTrigger", autoScoreNoCalculationTrigger, null);
    builder.addBooleanProperty("incrementLauncherOffset", incrementLauncherOffset, null);
    builder.addBooleanProperty("decrementLauncherOffset", decrementLauncherOffset, null);
    builder.addBooleanProperty("faceRedAlliance", faceRedAlliance, null);
    builder.addBooleanProperty("snowblowTrigger", snowblowTrigger, null);
    builder.addBooleanProperty("autoUnjamTrigger", autoUnjamTrigger, null);
  }

  private Dimensionless curveAxis(Dimensionless percent, double exponent) {
    return Value.of(
        Math.abs(Math.pow(
            percent.in(Value), exponent - 1)) * percent.unaryMinus().in(Value));
  }

  private Dimensionless getAxisWithDeadBandAndCurve(Dimensionless value, Dimensionless deadband, double curve) {
    return curveAxis(Value.of(MathUtil.applyDeadband(
        value.in(Value),
        deadband.in(Value))),
        curve);
  }

  private RelativeReference getRelativeReference() {
    return relativeReference;
  }

  private void configureControllerBindings() {
    bindDriver();
    bindOperator();
    bindAutoScore();
    bindSnowblow();

    drive.setDefaultCommand(drive.moveWithPercentages(
        () -> getAxisWithDeadBandAndCurve(driverLeftAxisX.get(), DEADBAND, MOVE_ROBOT_CURVE),
        () -> getAxisWithDeadBandAndCurve(driverLeftAxisY.get(), DEADBAND, MOVE_ROBOT_CURVE),
        () -> getAxisWithDeadBandAndCurve(driverRightAxisX.get(), DEADBAND, TURN_ROBOT_CURVE),
        this::getRelativeReference).withName("Drive With Percentages"));

    bayDoor.setDefaultCommand(bayDoor.ensuredHome().withName("Home Baydoor"));

    intake.setDefaultCommand(intake.stopIntake().withName("Stop Intake"));
    drive.onAllianceSide.and(() -> DriverStation.isTeleop()).and(autoScoreTrigger.negate())
        .and(snowblowTrigger.negate())
        .whileTrue(launcher.prepareLaunch().alongWith(kicker.prepareFuel()).withName("Prepare Fuel Launch"));
  }

  private void bindDriver() {
    driver.leftBumper().onTrue(Commands.runOnce(() -> {
      if (getRelativeReference() == RelativeReference.ROBOT_CENTRIC) {
        relativeReference = RelativeReference.FIELD_CENTRIC;
      } else {
        relativeReference = RelativeReference.ROBOT_CENTRIC;
      }
    }).withName("Switching Relative Drive"));

    // Opens Baydoor and Intakes Fuel
    driver.x().toggleOnTrue(
        bayDoor.open()
            .alongWith(intake.intakeFuel())
            .until(driver.b().or(driver.y()))
            .withName("Open Bay Door & Intake Fuel"));

    // Opens Baydoor and Shuttles Fuel
    driver.b().toggleOnTrue(
        bayDoor.open()
            .alongWith(intake.shuttleFuel())
            .until(driver.x().or(driver.y()))
            .withName("Open Bay Door & Shuttle Fuel"));

    driver.y().onTrue(bayDoor.close().withName("Close Baydoor"));

    // This command should run automatically when baydoor is no longer targeting the
    // open and close setpoints.
    // driver.onTrue(bayDoor.middle());

    driver.povDown().onTrue(drive.resetGyroCommand().withName("Reset Gyro"));
    faceRedAlliance.whileTrue(drive.angleToRedAlliance(
        () -> getAxisWithDeadBandAndCurve(driverLeftAxisX.get(), DEADBAND, MOVE_ROBOT_CURVE),
        () -> getAxisWithDeadBandAndCurve(driverLeftAxisY.get(), DEADBAND, MOVE_ROBOT_CURVE))
        .withName("Angle To Red Alliance"));
  }

  private void bindOperator() {
    // Manual Launch Fuel With Smartdashboard values.
    autoScoreNoCalculationTrigger.whileTrue(
        launcher.smartDashboardLaunchFuel()
            .alongWith(kicker.smartDashboardKickFuel())
            .alongWith(spindexer.smartDashboardIndexFuel())
            .alongWith(drive.angleToHub(
                () -> getAxisWithDeadBandAndCurve(driverLeftAxisX.get(), DEADBAND, MOVE_ROBOT_CURVE),
                () -> getAxisWithDeadBandAndCurve(driverLeftAxisY.get(), DEADBAND, MOVE_ROBOT_CURVE)))
            .withName("Score With Fixed Values"));

    // Manual Score Close
    manualCloseScoreTrigger.whileTrue(
        launcher.launchFuel(() -> launchCalculator.getLauncherVelocityToDistance(LAUNCHER_CLOSE_DISTANCE))
            .alongWith(kicker.kickFuel(() -> launchCalculator.getKickerVelocityToDistance(LAUNCHER_CLOSE_DISTANCE)))
            .alongWith(spindexer.indexFuel()).withName("Manual Score Close"));

    // Manual Score Trench
    manualTrenchScoreTrigger.whileTrue(
        launcher.launchFuel(() -> launchCalculator.getLauncherVelocityToDistance(LAUNCHER_TRENCH_DISTANCE))
            .alongWith(kicker.kickFuel(() -> launchCalculator.getKickerVelocityToDistance(LAUNCHER_TRENCH_DISTANCE)))
            .alongWith(spindexer.indexFuel()).withName("Manual Score Trench"));

    // Homes baydoor
    bayDoorHomeTrigger.onTrue(bayDoor.home().withName("Home Baydoor"));

    bayDoorOpenTrigger.onTrue(bayDoor.open().withName("Manual Open Bay Door"));
    bayDoorCloseTrigger.onTrue(bayDoor.close().withName("Manual Close Bay Door"));

    incrementLauncherOffset.onTrue(launcher.incrementLauncherOffset().withName("Increase Launcher Offset"));
    decrementLauncherOffset.onTrue(launcher.decrementLauncherOffset().withName("Decrease Launcher Offset"));

    operator.start().whileTrue(bayDoor.agitateLowFuel().withName("Manual Baydoor Fuel Agitation"));

    manualUnjamTrigger.and(drive.onAllianceSide.negate()).and(autoScoreTrigger.negate()).whileTrue(new RepeatCommand(
        spindexer.manualAgitateFuel().alongWith(bayDoor.agitateLowFuel()).alongWith(intake.agitateFuel()))
        .withName("Manual Unjam"));

    manualUnjamTrigger.and(drive.onAllianceSide).and(autoScoreTrigger.negate()).whileTrue(new RepeatCommand(
        spindexer.manualAgitateFuel().alongWith(bayDoor.agitateLowFuel()).alongWith(intake.agitateFuel()))
        .withName("Manual Unjam"));

    manualUnjamTrigger.and(drive.onAllianceSide).and(autoScoreTrigger).whileTrue(new RepeatCommand(
        spindexer.manualAgitateFuel().alongWith(bayDoor.agitateLowFuel()).alongWith(intake.agitateFuel()))
        .alongWith(launchHubDistance()).withName("Manual Unjam & Launch Velocity"));
  }

  private void bindAutoScore() {
    // If not aligned to hub then angle and prepare the fuel for launch.
    autoScoreTrigger
        .and(drive.onAllianceSide)
        .and(drive.launcherAlignedToHub.negate())
        .and(autoUnjamTrigger.negate())
        .and(manualUnjamTrigger.negate())
        .whileTrue(new RepeatCommand(angleToHub().alongWith(launchHubDistance())).withName("Auto Score: Not Aligned"));

    // If aligned to hub then attempt to stay angled and agitate fuel, anticipating
    // LOW fuel.
    autoScoreTrigger
        .and(agitateHighFuelTrigger.negate())
        .and(drive.onAllianceSide)
        .and(drive.launcherAlignedToHub)
        .and(autoUnjamTrigger.negate())
        .and(manualUnjamTrigger.negate())
        .whileTrue(new RepeatCommand(angleToHub().alongWith(launchHubDistance()).alongWith(indexLowFuel()))
            .withName("Auto Score: Aligned & Index Fast"));

    // If aligned to hub then attempt to stay angled and agitate fuel, anticipating
    // HIGH fuel.
    autoScoreTrigger
        .and(agitateHighFuelTrigger)
        .and(drive.onAllianceSide)
        .and(drive.launcherAlignedToHub)
        .and(autoUnjamTrigger.negate())
        .and(manualUnjamTrigger.negate())
        .whileTrue(new RepeatCommand(angleToHub().alongWith(launchHubDistance()).alongWith(indexLowFuel()))
            .withName("Auto Score: Aligned & Intake Fuel Wait"));

    autoScoreTrigger
        .and(autoUnjamTrigger)
        .and(manualUnjamTrigger.negate())
        .whileTrue(
            new RepeatCommand(
                angleToHub()
                    .alongWith(launchHubDistance())
                    .alongWith(spindexer.agitateFuel()))
                .withName("Auto Score: Auto Unjam"));
  }

  private void bindSnowblow() {
    snowblowTrigger.whileTrue(
        new RepeatCommand(angleToSnowblow().alongWith(launchSnowblowDistance())).withName("Snowblow: Not Aligned"));

    snowblowTrigger.and(drive.launcherAlignedToSnowblow)
        .whileTrue(new RepeatCommand(
            angleToSnowblow()
                .alongWith(launchSnowblowDistance())
                .alongWith(spindexer.indexFuel())
                .alongWith(intake.intakeFuel())
                .alongWith(bayDoor.open()))
            .withName("Snowblow: Aligned"));
  }

  private Command launchHubDistance() {
    return launcher.launchFuel(() -> {
      Optional<Distance> hubDistance = drive.getDistanceToHub();
      if (hubDistance.isEmpty())
        return RPM.zero();
      return launchCalculator.getLauncherVelocityToDistance(hubDistance.get());
    }).alongWith(kicker.kickFuel(() -> {
      Optional<Distance> hubDistance = drive.getDistanceToHub();
      if (hubDistance.isEmpty())
        return RPM.zero();
      return launchCalculator.getKickerVelocityToDistance(hubDistance.get());
    }));
  }

  private Command launchSnowblowDistance() {
    return launcher.launchFuel(() -> {
      Optional<Distance> snowblowDistance = drive.getDistanceToSnowblow();
      if (snowblowDistance.isEmpty())
        return RPM.zero();
      return launchCalculator.getLauncherVelocityToDistance(snowblowDistance.get());
    }).alongWith(kicker.kickFuel(() -> {
      Optional<Distance> snowblowDistance = drive.getDistanceToHub();
      if (snowblowDistance.isEmpty())
        return RPM.zero();
      return launchCalculator.getKickerVelocityToDistance(snowblowDistance.get());
    }));
  }

  private Command indexLowFuel() {
    return spindexer.indexFuel().alongWith(bayDoor.agitateLowFuel()).alongWith(intake.agitateFuel());
  }

  private Command indexHighFuel() {
    return spindexer.indexFuel().alongWith(bayDoor.agitateHighFuel()).alongWith(intake.agitateFuel());
  }

  private Command angleToHub() {
    return drive.angleToHub(
        () -> getAxisWithDeadBandAndCurve(driverLeftAxisX.get(), DEADBAND,
            MOVE_ROBOT_CURVE).times(REDUCE_SPEED),
        () -> getAxisWithDeadBandAndCurve(driverLeftAxisY.get(), DEADBAND,
            MOVE_ROBOT_CURVE).times(REDUCE_SPEED));
  }

  private Command angleToSnowblow() {
    return drive.angleToSnowblow(
        () -> getAxisWithDeadBandAndCurve(driverLeftAxisX.get(), DEADBAND,
            MOVE_ROBOT_CURVE).times(REDUCE_SPEED),
        () -> getAxisWithDeadBandAndCurve(driverLeftAxisY.get(), DEADBAND,
            MOVE_ROBOT_CURVE).times(REDUCE_SPEED));
  }

  private void registerPathplannerCommands() {
    NamedCommands.registerCommand("score", angleToHub().alongWith(launchHubDistance()).alongWith(indexHighFuel()));
    NamedCommands.registerCommand("baydooropen", bayDoor.open());
    NamedCommands.registerCommand("baydoorclose", bayDoor.close());
    NamedCommands.registerCommand("baydoorhome", bayDoor.ensuredHome());
    NamedCommands.registerCommand("intakefuel", intake.intakeFuel());
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }

  // region SysId
  @SuppressWarnings("unused")
  private void bindDriveSystemDynamics() {
    /*
     * Note that each routine should be run exactly once in a single log.
     */
    driver.back().and(driver.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
  }

  @SuppressWarnings("unused")
  private void bindBayDoorSystemDynamics() {
    driver.back().and(driver.y()).whileTrue(bayDoor.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(bayDoor.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(bayDoor.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(bayDoor.sysIdQuasistatic(Direction.kReverse));
  }

  @SuppressWarnings("unused")
  private void bindIntakeSystemDynamics() {
    driver.back().and(driver.y()).whileTrue(intake.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(intake.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(intake.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(intake.sysIdQuasistatic(Direction.kReverse));
  }

  @SuppressWarnings("unused")
  private void bindSpindexerSystemDynamics() {
    driver.back().and(driver.y()).whileTrue(spindexer.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(spindexer.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(spindexer.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(spindexer.sysIdQuasistatic(Direction.kReverse));
  }

  @SuppressWarnings("unused")
  private void bindlauncherSystemDynamics() {
    driver.back().and(driver.y()).whileTrue(launcher.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(launcher.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(launcher.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(launcher.sysIdQuasistatic(Direction.kReverse));
  }

  @SuppressWarnings("unused")
  private void bindKickerSystemDynamics() {
    driver.back().and(driver.y()).whileTrue(kicker.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(kicker.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(kicker.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(kicker.sysIdQuasistatic(Direction.kReverse));
  }
  // endregion

}