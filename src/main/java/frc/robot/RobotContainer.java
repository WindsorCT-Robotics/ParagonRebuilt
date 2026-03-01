
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
  public final BayDoor bayDoor;
  private final Spindexer spindexer;
  private final Shooter shooter;
  private final Kicker kicker;

  private static final CanId INTAKE_ROLLER_MOTOR_CAN_ID = new CanId((byte) 16);
  private static final CanId BAYDOOR_MOTOR_LEFT_CAN_ID = new CanId((byte) 14);
  private static final CanId BAYDOOR_MOTOR_RIGHT_CAN_ID = new CanId((byte) 15);
  private static final DigitalInputOutput INTAKE_LEFT_BAYDOOR_DIO = new DigitalInputOutput((byte) 0);
  private static final DigitalInputOutput INTAKE_RIGHT_BAYDOOR_DIO = new DigitalInputOutput((byte) 1);

  private static final CanId SPINDEXER_MOTOR_CAN_ID = new CanId((byte) 13);

  private static final CanId SHOOTER_MOTOR_LEFT_CAN_ID = new CanId((byte) 18);
  private static final CanId SHOOTER_MOTOR_RIGHT_CAN_ID = new CanId((byte) 19);

  private static final CanId KICKER_MOTOR_CAN_ID = new CanId((byte) 17);

  private final CommandXboxController driver;
  private final CommandXboxController operator;
  private static final Dimensionless REDUCE_SPEED = Percent.of(75);
  private static final double MOVE_ROBOT_CURVE = 3.0;
  private static final double TURN_ROBOT_CURVE = 2.0;

  private static final Dimensionless DEADBAND = Percent.of(5);
  private RelativeReference relativeReference;

  private final SendableChooser<Command> autonomousChooser;
  private static final String DEFAULT_AUTO = ""; // TODO: Once formed autos pick an auto to default to.
  private final Supplier<Dimensionless> driverLeftAxisX;
  private final Supplier<Dimensionless> driverLeftAxisY;
  private final Supplier<Dimensionless> driverRightAxisX;

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

    relativeReference = RelativeReference.FIELD_CENTRIC;

    driver = new CommandXboxController(0);
    operator = new CommandXboxController(1);

    autonomousChooser = AutoBuilder.buildAutoChooser(DEFAULT_AUTO);
    SmartDashboard.putString("Relative Reference", getRelativeReference().toString());
    configureControllerBindings();

    logger = new Telemetry(MAX_SPEED.in(MetersPerSecond));
    // drive.registerTelemetry(logger::telemeterize); // TODO: Remove logger?

    driverLeftAxisX = () -> Value.of(driver.getLeftX());
    driverLeftAxisY = () -> Value.of(driver.getLeftY());
    driverRightAxisX = () -> Value.of(driver.getRightX());

    initSmartDashboard();
  }

  private void initSmartDashboard() {
    SmartDashboard.putData(intake);
    SmartDashboard.putData(bayDoor);
    SmartDashboard.putData(spindexer);
    SmartDashboard.putData(shooter);
    SmartDashboard.putData(kicker);
    SmartDashboard.putData("Controllers/Driver", driver.getHID());
    SmartDashboard.putData("Controllers/Operator", operator.getHID());
    SmartDashboard.putData("Autonomous", autonomousChooser);
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData("Robot Container", this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Curved Percent", () -> curveAxis(driverLeftAxisX, MOVE_ROBOT_CURVE).get().in(Percent),
        null);
    builder.addDoubleProperty("Percent", () -> driverLeftAxisX.get().in(Percent), null);
  }

  private Supplier<Dimensionless> curveAxis(Supplier<Dimensionless> percent, double exponent) {
    return () -> {
      double p = percent.get().in(Value);

      return Value
          .of(
              Math.abs(
                  Math.pow(p, exponent - 1)) * -p);
    };
  }

  private RelativeReference getRelativeReference() {
    return relativeReference;
  }

  private void configureControllerBindings() {
    // bindDrive();
    // bindBayDoor();
    // bindIntake();
    bindSpindexer();
    bindKicker();
    bindShooter();

    // driver.x().toggleOnTrue(bayDoor.open().alongWith(intake.intakeFuel()).until(driver.b()).withName("Open
    // Bay Door & Intake Fuel"));
    // driver.b().toggleOnTrue(bayDoor.open().alongWith(intake.shuttleFuel()).until(driver.x()).withName("Open
    // Bay Door & Shuttle Fuel"));

    driver.leftBumper()
        .whileTrue(spindexer.indexFuel().alongWith(kicker.kickStartFuel()).alongWith(shooter.shootFuel()));
  }

  private void bindDrive() {
    drive.setDefaultCommand(drive.moveWithPercentages(
        curveAxis(
            () -> Value.of(
                MathUtil.applyDeadband(
                    driverLeftAxisX.get().in(Value),
                    DEADBAND.in(Value))),
            MOVE_ROBOT_CURVE),
        curveAxis(
            () -> Value.of(
                MathUtil.applyDeadband(
                    driverLeftAxisY.get().in(Value),
                    DEADBAND.in(Value))),
            MOVE_ROBOT_CURVE),
        curveAxis(
            () -> Value.of(
                MathUtil.applyDeadband(
                    driverRightAxisX.get().in(Value),
                    DEADBAND.in(Value))),
            TURN_ROBOT_CURVE),
        this::getRelativeReference));

    driver.b().onTrue(Commands.runOnce(() -> {
      if (getRelativeReference() == RelativeReference.ROBOT_CENTRIC) {
        relativeReference = RelativeReference.FIELD_CENTRIC;
      } else {
        relativeReference = RelativeReference.ROBOT_CENTRIC;
      }

      SmartDashboard.putString("Relative Reference", getRelativeReference().toString());
    }));

    driver.rightBumper().whileTrue(drive.moveWithPercentages(
        curveAxis(
            () -> Percent.of(
                MathUtil.applyDeadband(
                    driverLeftAxisX.get().in(Percent),
                    DEADBAND.in(Percent)))
                .times(REDUCE_SPEED),
            MOVE_ROBOT_CURVE),
        curveAxis(
            () -> Percent.of(
                MathUtil.applyDeadband(
                    driverLeftAxisY.get().in(Percent),
                    DEADBAND.in(Percent)))
                .times(REDUCE_SPEED),
            MOVE_ROBOT_CURVE),
        curveAxis(
            () -> Percent.of(
                MathUtil.applyDeadband(
                    driverRightAxisX.get().in(Percent),
                    DEADBAND.in(Percent))),
            TURN_ROBOT_CURVE),
        this::getRelativeReference));

    // TODO: Make it so that when true the robot can go in only 4 cardinal
    // directions.
    // driver.rightTrigger(TRIGGER_THRESHOLD.in(Percent)).whileTrue(Commands.runEnd(
    // () -> {
    // Dimensionless limit =
    // Percent.of(100).minus(Percent.of(driver.getRightTriggerAxis()));
    // maxDriverLeftJoyStickSpeedX = limit;
    // maxDriverLeftJoyStickSpeedY = limit;
    // }, () -> {
    // maxDriverLeftJoyStickSpeedX = Percent.of(100);
    // maxDriverLeftJoyStickSpeedY = Percent.of(100);
    // }));

    driver.y().toggleOnTrue(
        drive.angleToOutpost(
            curveAxis(driverLeftAxisX, MOVE_ROBOT_CURVE),
            curveAxis(driverLeftAxisY, MOVE_ROBOT_CURVE)));
    //
    driver.povDown().onTrue(drive.resetGyro());
  }

  private void bindBayDoor() {
    bayDoor.setDefaultCommand(bayDoor.home());
    operator.y().onTrue(bayDoor.open());
    operator.a().onTrue(bayDoor.close());
    operator.x().onTrue(bayDoor.home());
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

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}