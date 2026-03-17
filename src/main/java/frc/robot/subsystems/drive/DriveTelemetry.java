package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class DriveTelemetry {
    private final NetworkTableInstance instance = NetworkTableInstance.getDefault();

    private final NetworkTable driveStateTable = instance.getTable("SmartDashboard/Subsystems/Drive");

    private final StructPublisher<Pose2d> drivePose = driveStateTable
            .getStructTopic("Pose", Pose2d.struct).publish();

    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable
            .getStructTopic("Speeds", ChassisSpeeds.struct).publish();

    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable
            .getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();

    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable
            .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();

    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable
            .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();

    public DriveTelemetry() {

    }

    public void telemeterize(SwerveDriveState state) {

    }
}