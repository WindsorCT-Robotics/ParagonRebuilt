
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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

  private final Trigger autoScoreTrigger;
  private final Trigger autoScoreNoCalculationTrigger;
  private final Trigger snowblowTrigger;
  private final Trigger unjamFuel;

  private RelativeReference relativeReference;

  private final SendableChooser<Command> autonomousChooser;
  private static final String DEFAULT_AUTO = ""; // TODO: Once formed autos pick an auto to default to.

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

    snowblowTrigger = new Trigger(() -> driverRightTrigger.get().gt(Percent.of(20)));
    autoScoreTrigger = driver.rightBumper();
    autoScoreNoCalculationTrigger = operator.povDown();
    unjamFuel = operator.a().or(spindexer.unjamFuel);

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
        this::getRelativeReference));

    bayDoor.hasBayDoorHomed.negate().whileTrue(bayDoor.home());

    intake.setDefaultCommand(intake.stopIntake());
  }

  private void bindDriver() {
    driver.leftBumper().onTrue(Commands.runOnce(() -> {
      if (getRelativeReference() == RelativeReference.ROBOT_CENTRIC) {
        relativeReference = RelativeReference.FIELD_CENTRIC;
      } else {
        relativeReference = RelativeReference.ROBOT_CENTRIC;
      }
    }));

    drive.onAllianceSide.and(autoScoreTrigger.negate())
        .whileTrue(launcher.prepareLaunch().alongWith(kicker.prepareFuel()));

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

    driver.a().onTrue(bayDoor.open());

    driver.y().onTrue(bayDoor.close());

    // This command should run automatically when baydoor is no longer targeting the
    // open and close setpoints.
    // driver.onTrue(bayDoor.middle());

    driver.povDown().onTrue(drive.resetGyroCommand());
  }

  private void bindOperator() {
    // Manual Launch Fuel With Smartdashboard values.
    autoScoreNoCalculationTrigger.whileTrue(
        launcher.smartDashboardLaunchFuel()
            .alongWith(kicker.smartDashboardKickFuel())
            .alongWith(spindexer.smartDashboardIndexFuel())
            .alongWith(drive.angleToHub(
                () -> getAxisWithDeadBandAndCurve(driverLeftAxisX.get(), DEADBAND, MOVE_ROBOT_CURVE),
                () -> getAxisWithDeadBandAndCurve(driverLeftAxisY.get(), DEADBAND, MOVE_ROBOT_CURVE))));

    // Homes baydoor
    operator.x().onTrue(bayDoor.home());
  }

  private void bindAutoScore() {
    autoScoreTrigger.and(drive.onAllianceSide).and(unjamFuel.negate())
        .whileTrue(
            angleToHub()
                .alongWith(launchHubDistance())
                .alongWith(indexFuel().onlyWhile(drive.launcherAlignedToHub)));

    autoScoreTrigger.and(unjamFuel).whileTrue(
        angleToHub()
            .alongWith(launchHubDistance())
            .alongWith(spindexer.agitateFuel()));
  }

  private void bindSnowblow() {
    snowblowTrigger.whileTrue(angleToSnowblow().alongWith(launchSnowblowDistance())
        .alongWith(
            spindexer.indexFuel()
                .onlyWhile(drive.launcherAlignedToSnowblow.and(drive.launcherObstructedByHub.negate()))));
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

  private Command indexFuel() {
    return spindexer.indexFuel().alongWith(bayDoor.agitateFuel());
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
    NamedCommands.registerCommand("shoothubdistance", launchHubDistance().alongWith(indexFuel()));
    // TODO: Use spindexer sensor to determine if 20 fuel has past.
    // TODO: Compare the average amount of fuel in hopper vs the flow of balls to
    // determine good fuel value.
    NamedCommands.registerCommand("shoothubdistancetil20", launchHubDistance().alongWith(indexFuel()));
    NamedCommands.registerCommand("baydooropen", bayDoor.open());
    NamedCommands.registerCommand("baydoormiddle", bayDoor.middle());
    NamedCommands.registerCommand("baydoorclose", bayDoor.close());
    NamedCommands.registerCommand("baydoorhome", bayDoor.home());
    NamedCommands.registerCommand("intakefuel", intake.intakeFuel());
    NamedCommands.registerCommand("shuttlefuel", intake.shuttleFuel());
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