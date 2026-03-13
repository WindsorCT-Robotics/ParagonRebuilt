
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.RPM;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoScore;
import frc.robot.commands.LaunchFuelToHubDistance;
import frc.robot.commands.LaunchFuelToTargetDistance;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.hardware.CanId;
import frc.robot.hardware.DigitalInputOutput;
import frc.robot.subsystems.BayDoor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Drive.RelativeReference;
import frc.robot.utils.LaunchCalculator;

public class RobotContainer implements Sendable {
  private static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
  private final Telemetry logger;

  private final Drive drive;
  private final Intake intake;
  private final BayDoor bayDoor;
  private final Spindexer spindexer;
  private final Shooter shooter;
  private final Kicker kicker;

  private static final CanId INTAKE_ROLLER_MOTOR_CAN_ID = new CanId((byte) 16);
  private static final CanId BAYDOOR_MOTOR_LEFT_CAN_ID = new CanId((byte) 14);
  private static final CanId BAYDOOR_MOTOR_RIGHT_CAN_ID = new CanId((byte) 15);
  private static final DigitalInputOutput LEFT_BAYDOOR_DIO = new DigitalInputOutput((byte) 0);
  private static final DigitalInputOutput RIGHT_BAYDOOR_DIO = new DigitalInputOutput((byte) 1);

  private static final CanId SPINDEXER_MOTOR_CAN_ID = new CanId((byte) 13);

  private static final CanId KICKER_MOTOR_CAN_ID = new CanId((byte) 17);

  private static final CanId SHOOTER_MOTOR_LEFT_CAN_ID = new CanId((byte) 18);
  private static final CanId SHOOTER_MOTOR_RIGHT_CAN_ID = new CanId((byte) 19);

  private final CommandXboxController driver;
  private final CommandXboxController operator;
  private static final Dimensionless REDUCE_SPEED = Percent.of(65);
  private static final double MOVE_ROBOT_CURVE = 2.0;
  private static final double TURN_ROBOT_CURVE = 2.0;

  private static final Dimensionless DEADBAND = Percent.of(5);
  private RelativeReference relativeReference;

  private final LaunchCalculator launchCalculator;

  private final SendableChooser<Command> autonomousChooser;
  private static final String DEFAULT_AUTO = ""; // TODO: Once formed autos pick an auto to default to.
  private final Supplier<Dimensionless> driverLeftAxisX;
  private final Supplier<Dimensionless> driverLeftAxisY;
  private final Supplier<Dimensionless> driverRightAxisX;
  private final Supplier<Dimensionless> driverRightTrigger;
  private final Trigger driverRightTriggered;

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

    intake = new Intake(Intake.class.getSimpleName(), INTAKE_ROLLER_MOTOR_CAN_ID);
    bayDoor = new BayDoor(BayDoor.class.getSimpleName(), BAYDOOR_MOTOR_LEFT_CAN_ID, BAYDOOR_MOTOR_RIGHT_CAN_ID,
        LEFT_BAYDOOR_DIO,
        RIGHT_BAYDOOR_DIO);
    spindexer = new Spindexer(Spindexer.class.getSimpleName(), SPINDEXER_MOTOR_CAN_ID);
    shooter = new Shooter(Shooter.class.getSimpleName(), SHOOTER_MOTOR_LEFT_CAN_ID, SHOOTER_MOTOR_RIGHT_CAN_ID);
    kicker = new Kicker(Kicker.class.getSimpleName(), KICKER_MOTOR_CAN_ID);

    launchCalculator = new LaunchCalculator(() -> drive.getState().Pose, () -> drive.getHubPosition(DriverStation.getAlliance().orElse(Alliance.Blue)));

    relativeReference = RelativeReference.FIELD_CENTRIC;

    driver = new CommandXboxController(0);
    operator = new CommandXboxController(1);

    registerPathplannerCommands();
    autonomousChooser = AutoBuilder.buildAutoChooser(DEFAULT_AUTO);
    SmartDashboard.putString("Relative Reference", getRelativeReference().toString());

    logger = new Telemetry(MAX_SPEED.in(MetersPerSecond));
    drive.registerTelemetry(logger::telemeterize);

    driverLeftAxisX = () -> Value.of(driver.getLeftX());
    driverLeftAxisY = () -> Value.of(driver.getLeftY());
    driverRightAxisX = () -> Value.of(driver.getRightX());
    driverRightTrigger = () -> Value.of(driver.getRightTriggerAxis());
    driverRightTriggered = new Trigger(() -> driverRightTrigger.get().gt(Percent.of(20)));
    initSmartDashboard();
    configureControllerBindings();
  }

  private void initSmartDashboard() {
    SmartDashboard.putData(drive);
    SmartDashboard.putData(intake);
    SmartDashboard.putData(bayDoor);
    SmartDashboard.putData(spindexer);
    SmartDashboard.putData(shooter);
    SmartDashboard.putData(kicker);
    SmartDashboard.putData("Launch Calculations", launchCalculator);
    SmartDashboard.putData("Controllers/Driver", driver.getHID());
    SmartDashboard.putData("Controllers/Operator", operator.getHID());
    SmartDashboard.putData("Autonomous", autonomousChooser);
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData("Robot Container", this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {

  }

  private Dimensionless getOperatorTriggerAdjustment() {
    return Value.of(-operator.getLeftTriggerAxis() + operator.getRightTriggerAxis());
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

    drive.setDefaultCommand(drive.moveWithPercentages(
        () -> getAxisWithDeadBandAndCurve(driverLeftAxisX.get(), DEADBAND, MOVE_ROBOT_CURVE),
        () -> getAxisWithDeadBandAndCurve(driverLeftAxisY.get(), DEADBAND, MOVE_ROBOT_CURVE),
        () -> getAxisWithDeadBandAndCurve(driverRightAxisX.get(), DEADBAND, TURN_ROBOT_CURVE),
        this::getRelativeReference));

    bayDoor.setDefaultCommand(bayDoor.home());
    intake.setDefaultCommand(intake.stopIntake());

    shooter.setDefaultCommand(shooter.prepareLaunch(() -> drive.getState().Pose).withName("Launcher Prepare Launch"));
    kicker.setDefaultCommand(
        kicker.prepareFuel(() -> drive.getState().Pose, drive.onAllianceSide()).withName("Kicker Prepare Launch"));
    spindexer.setDefaultCommand(spindexer.prepareFuel().withName("Spindexer Prepare Launch"));
  }

  private void bindDriver() {
    driver.leftBumper().onTrue(Commands.runOnce(() -> {
      if (getRelativeReference() == RelativeReference.ROBOT_CENTRIC) {
        relativeReference = RelativeReference.FIELD_CENTRIC;
      } else {
        relativeReference = RelativeReference.ROBOT_CENTRIC;
      }

      SmartDashboard.putString("Relative Reference", getRelativeReference().toString());
    }));

    driver.rightBumper().whileTrue(drive.moveWithPercentages(
        () -> getAxisWithDeadBandAndCurve(driverLeftAxisX.get(), DEADBAND, MOVE_ROBOT_CURVE).times(REDUCE_SPEED),
        () -> getAxisWithDeadBandAndCurve(driverLeftAxisY.get(), DEADBAND, MOVE_ROBOT_CURVE).times(REDUCE_SPEED),
        () -> getAxisWithDeadBandAndCurve(driverRightAxisX.get(), DEADBAND, TURN_ROBOT_CURVE).times(REDUCE_SPEED),
        this::getRelativeReference));

    // Angles launcher
    // If within the RPM range then spindexer indexes the fuel
    // If within angle range it launches fuel
    // Calculates shooter and kicker speed based off of distance
    // If speed of shooter and kicker was off then operator can adjust with right
    // trigger
    driver.rightStick().toggleOnTrue(new AutoScore(
        drive,
        () -> getAxisWithDeadBandAndCurve(driverLeftAxisX.get(), DEADBAND, MOVE_ROBOT_CURVE),
        () -> getAxisWithDeadBandAndCurve(driverLeftAxisY.get(), DEADBAND, MOVE_ROBOT_CURVE),
        shooter,
        kicker,
        spindexer,
        launchCalculator,
        () -> getOperatorTriggerAdjustment(),
        operator.start()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("LaunchFuelToHub"));

    // region toggle outpost angle
    driver.start().toggleOnTrue(
        drive.angleToOutpost(
            () -> getAxisWithDeadBandAndCurve(driverLeftAxisX.get(), DEADBAND, MOVE_ROBOT_CURVE),
            () -> getAxisWithDeadBandAndCurve(driverLeftAxisY.get(), DEADBAND, MOVE_ROBOT_CURVE)));
    // endregion

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

    driver.back()
        .whileTrue(new LaunchFuelToHubDistance(
            drive,
            shooter,
            kicker,
            spindexer,
            launchCalculator,
            () -> getOperatorTriggerAdjustment(),
            operator.start()));

    // If the Right Axis was greater than 20% then will launch fuel based on percent
    // to meters
    driverRightTriggered.whileTrue(new LaunchFuelToTargetDistance(launchCalculator,
        Meters.of(driver.getRightTriggerAxis() * 5), RPM.of(100), shooter, kicker, spindexer));

    driver.povDown().onTrue(drive.resetGyro());
  }

  private void bindOperator() {
    // Manual Launch Fuel With Smartdashboard values.
    operator.povDown().whileTrue(
        shooter.smartDashboardLaunchFuel()
            .alongWith(kicker.smartDashboardKickFuel())
            .alongWith(spindexer.smartDashboardIndexFuel())
            .alongWith(drive.angleToHub(
                () -> getAxisWithDeadBandAndCurve(driverLeftAxisX.get(), DEADBAND, MOVE_ROBOT_CURVE),
                () -> getAxisWithDeadBandAndCurve(driverLeftAxisY.get(), DEADBAND, MOVE_ROBOT_CURVE))));

    // Spins the spindexer backwards.
    operator.back().whileTrue(
        spindexer.smartDashboardShuttleFuel()
            .unless(operator.leftBumper())
            .until(operator.leftBumper()));

    // Homes baydoor
    operator.x().onTrue(bayDoor.home());
  }

  private void registerPathplannerCommands() {
    NamedCommands.registerCommand("shoothubdistance",
        new LaunchFuelToHubDistance(drive,
            shooter,
            kicker,
            spindexer,
            launchCalculator,
            () -> Percent.zero(),
            new Trigger(() -> false)));
    NamedCommands.registerCommand("baydooropen", bayDoor.open());
    NamedCommands.registerCommand("baydoorclose", bayDoor.close());
    NamedCommands.registerCommand("baydoorhome", bayDoor.home());
    NamedCommands.registerCommand("intakefuel", intake.intakeFuel());
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
  private void bindShooterSystemDynamics() {
    driver.back().and(driver.y()).whileTrue(shooter.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(shooter.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
  }

  @SuppressWarnings("unused")
  private void bindKickerSystemDynamics() {
    driver.back().and(driver.y()).whileTrue(kicker.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(kicker.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(kicker.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(kicker.sysIdQuasistatic(Direction.kReverse));
  }
  // endregion

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}