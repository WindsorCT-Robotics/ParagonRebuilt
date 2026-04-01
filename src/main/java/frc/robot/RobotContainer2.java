package frc.robot;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.BayDoor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Drive.RelativeReference;
import frc.robot.utils.ControllerUtil;

public class RobotContainer2 implements Sendable {
    private static final Dimensionless DRIVER_CONTROLLER_DEADBAND = Percent.of(5);
    private static final double MOVE_CURVE = 2.0;
    private static final double TURN_CURVE = 2.0;

    private final Trigger t_autoScore;
    private final Trigger t_manualScore;
    private final Trigger t_partialManualScore;
    private final Trigger t_delayAgitation;
    private final Trigger t_autoSnowBlow;
    private final Trigger t_unjam;
    private final Trigger t_incrementLauncherOffset;
    private final Trigger t_decrementLauncherOffset;
    private final Trigger t_faceRedAlliance;
    private final Trigger t_autoIntake;
    private final Trigger t_autoShuttle;
    private final Trigger t_homeBayDoor;
    private final Trigger t_openBayDoor;
    private final Trigger t_closeBayDoor;

    private final Drive drive;
    private final Spindexer spindexer;
    private final BayDoor bayDoor;
    private final Intake intake;
    private final Kicker kicker;
    private final Launcher launcher;

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final SendableChooser<Command> autoChooser;

    private RelativeReference relativeReference;

    public RobotContainer2() {
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

        t_autoScore = driver.rightBumper();
        t_manualScore = operator.b();
        t_partialManualScore = operator.povLeft();
        t_delayAgitation = driver.a();
        t_autoSnowBlow = driver.rightTrigger(Percent.of(0.2).in(Value));
        t_unjam = operator.a();
        t_incrementLauncherOffset = operator.rightBumper();
        t_decrementLauncherOffset = operator.leftBumper();
        t_faceRedAlliance = driver.leftStick();
        t_autoIntake = driver.x();
        t_autoShuttle = driver.b();
        t_homeBayDoor = operator.leftStick().and(operator.rightStick());
        t_openBayDoor = operator.povDown();
        t_closeBayDoor = operator.povUp();

        autoChooser = new SendableChooser<>();
        initPathPlannerCommands();
        initSmartdashBoard();

        bindCommands();
    }

    private void initSmartdashBoard() {
        SmartDashboard.putData(drive);
        SmartDashboard.putData(intake);
        SmartDashboard.putData(bayDoor);
        SmartDashboard.putData(spindexer);
        SmartDashboard.putData(launcher);
        SmartDashboard.putData(kicker);
        SmartDashboard.putData("Robot Container/Controllers/Driver", driver.getHID());
        SmartDashboard.putData("Robot Container/Controllers/Operator", operator.getHID());
        SmartDashboard.putData("Robot Container/Autonomous", autoChooser);
        SmartDashboard.putData("Robot Container", this);
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void initPathPlannerCommands() {
        // NamedCommands.registerCommand("score",
        // angleToHub().alongWith(launchHubDistance()).alongWith(indexHighFuel()));
        NamedCommands.registerCommand("baydooropen", bayDoor.open());
        NamedCommands.registerCommand("baydoorclose", bayDoor.close());
        NamedCommands.registerCommand("baydoorhome", bayDoor.ensuredHome());
        NamedCommands.registerCommand("intakefuel", intake.intakeFuel());
    }

    private RelativeReference getRelativeReference() {
        return relativeReference;
    }

    private void bindCommands() {
    }
}