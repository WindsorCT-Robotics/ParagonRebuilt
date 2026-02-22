
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
  private static final DigitalInputOutput INTAKE_LEFT_BAYDOOR_DIO = new DigitalInputOutput((byte) 1);
  private static final DigitalInputOutput INTAKE_RIGHT_BAYDOOR_DIO = new DigitalInputOutput((byte) 0);

  private static final CanId SPINDEXER_MOTOR_CAN_ID = new CanId((byte) 13);

  private static final CanId SHOOTER_MOTOR_LEFT_CAN_ID = new CanId((byte) 18);
  private static final CanId SHOOTER_MOTOR_RIGHT_CAN_ID = new CanId((byte) 19);

  private static final CanId KICKER_MOTOR_CAN_ID = new CanId((byte) 17);

  private final CommandXboxController driver;
  private final CommandXboxController operator;
  private Dimensionless maxDriverLeftJoyStickSpeedX = Percent.of(100);
  private Dimensionless maxDriverLeftJoyStickSpeedY = Percent.of(100);
  private Dimensionless maxDriverRightJoyStickSpeedX = Percent.of(100);
  private static final Dimensionless REDUCE_SPEED = Percent.of(75);
  private static final Dimensionless TRIGGER_THRESHOLD = Percent.of(10);
  private static final double MOVE_ROBOT_CURVE = 3.0;
  private static final double TURN_ROBOT_CURVE = 2.0;
  private final Supplier<Dimensionless> driverLeftAxisX;
  private final Supplier<Dimensionless> driverLeftAxisY;
  private final Supplier<Dimensionless> driverRightAxisX;

  private RelativeReference relativeReference;

  private final SendableChooser<Command> autonomousChooser;
  private static final String DEFAULT_AUTO = ""; // TODO: Once formed autos pick an auto to default to.

  public RobotContainer() {
    try {
      drive = new Drive(
          Drive.class.getSimpleName(),
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
        INTAKE_RIGHT_BAYDOOR_DIO,
        INTAKE_LEFT_BAYDOOR_DIO);
    spindexer = new Spindexer(Spindexer.class.getSimpleName(), SPINDEXER_MOTOR_CAN_ID);
    shooter = new Shooter(Shooter.class.getSimpleName(), SHOOTER_MOTOR_LEFT_CAN_ID, SHOOTER_MOTOR_RIGHT_CAN_ID);
    kicker = new Kicker(Kicker.class.getSimpleName(), KICKER_MOTOR_CAN_ID);

    SmartDashboard.putData(intake);
    SmartDashboard.putData(bayDoor);
    SmartDashboard.putData(spindexer);
    SmartDashboard.putData(shooter);
    SmartDashboard.putData(kicker);

    // Home Motors
    CommandScheduler.getInstance().schedule(bayDoor.home().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    relativeReference = RelativeReference.FIELD_CENTRIC;

    driver = new CommandXboxController(0);
    operator = new CommandXboxController(1);

    driverLeftAxisX = () -> Percent.of(driver.getLeftX())
        .times(maxDriverLeftJoyStickSpeedX);
    driverLeftAxisY = () -> Percent.of(driver.getLeftY())
        .times(maxDriverLeftJoyStickSpeedY);
    driverRightAxisX = () -> Percent.of(driver.getRightX());

    autonomousChooser = AutoBuilder.buildAutoChooser(DEFAULT_AUTO);
    SmartDashboard.putData("Autonomous", autonomousChooser);
    SmartDashboard.putString("Relative Reference", getRelativeReference().toString());
    configureControllerBindings();

    logger = new Telemetry(MAX_SPEED.in(MetersPerSecond));
    drive.registerTelemetry(logger::telemeterize);
  }

  @Override
  public void initSendable(SendableBuilder builder) {

  }

  private Supplier<Dimensionless> curveAxis(Supplier<Dimensionless> percent, double exponent) {
    return () -> Percent
        .of(Math.abs(Math.pow(percent.get().in(Percent), exponent - 1)) * percent.get().times(-1).in(Percent) * 100);
  }

  private RelativeReference getRelativeReference() {
    return relativeReference;
  }

  private Dimensionless getBoundsOfAxis(Dimensionless percent, Dimensionless bounds) {
    return percent.times(bounds);
  }

  private void configureControllerBindings() {
    bindDrive();
    bindBayDoor();
    bindIntake();
    bindSpindexer();
    bindKicker();
    bindShooter();

    driver.x().toggleOnTrue(bayDoor.openBayDoor().alongWith(intake.intakeFuel()));
    driver.b().toggleOnTrue(bayDoor.openBayDoor().alongWith(intake.shuttleFuel()));

    driver.leftBumper()
        .whileTrue(spindexer.indexFuel().alongWith(kicker.kickStartFuel()).alongWith(shooter.shootFuel()));
  }

  private void bindDrive() {
    drive.setDefaultCommand(drive.moveWithPercentages(
        curveAxis(() -> getBoundsOfAxis(driverLeftAxisX.get(), maxDriverLeftJoyStickSpeedX), MOVE_ROBOT_CURVE),
        curveAxis(() -> getBoundsOfAxis(driverLeftAxisY.get(), maxDriverLeftJoyStickSpeedY), MOVE_ROBOT_CURVE),
        curveAxis(() -> getBoundsOfAxis(driverRightAxisX.get(), maxDriverRightJoyStickSpeedX), TURN_ROBOT_CURVE),
        this::getRelativeReference));

    driver.b().onTrue(Commands.runOnce(() -> {
      if (getRelativeReference() == RelativeReference.ROBOT_CENTRIC) {
        relativeReference = RelativeReference.FIELD_CENTRIC;
      } else {
        relativeReference = RelativeReference.ROBOT_CENTRIC;
      }

      SmartDashboard.putString("Relative Reference", getRelativeReference().toString());
    }));

    driver.rightBumper().whileTrue(Commands.runEnd(() -> {
      maxDriverLeftJoyStickSpeedX = REDUCE_SPEED;
      maxDriverLeftJoyStickSpeedY = REDUCE_SPEED;
    }, () -> {
      maxDriverLeftJoyStickSpeedX = Percent.of(100);
      maxDriverLeftJoyStickSpeedY = Percent.of(100);
    }));

    driver.rightTrigger(TRIGGER_THRESHOLD.in(Percent)).whileTrue(Commands.runEnd(
        () -> {
          Dimensionless limit = Percent.of(100).minus(Percent.of(driver.getRightTriggerAxis()));
          maxDriverLeftJoyStickSpeedX = limit;
          maxDriverLeftJoyStickSpeedY = limit;
        }, () -> {
          maxDriverLeftJoyStickSpeedX = Percent.of(100);
          maxDriverLeftJoyStickSpeedY = Percent.of(100);
        }));

    driver.y().toggleOnTrue(
        drive.angleToOutpost(
            curveAxis(driverLeftAxisX, MOVE_ROBOT_CURVE),
            curveAxis(driverLeftAxisY, MOVE_ROBOT_CURVE)));

    driver.povDown().onTrue(drive.resetGyro());
  }

  private void bindBayDoor() {
    operator.y().whileTrue(bayDoor.openBayDoor());
  }

  private void bindIntake() {
    intake.setDefaultCommand(intake.stopIntake());
  }

  private void bindSpindexer() {

  }

  private void bindKicker() {

  }

  private void bindShooter() {

  }

  private void bindPIDTuning() {
    /*
     * Note that each routine should be run exactly once in a single log.
     */
    driver.back().and(driver.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}