package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    private final LauncherIO io;
    private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();
    private final Alert leftMotorAlert = new Alert("Left Launcher Motor Disconnected!", AlertType.kError);
    private final Alert rightMotorAlert = new Alert("Right Launcher Motor Disconnected!", AlertType.kError);

    public Launcher(LauncherIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("", inputs);
        leftMotorAlert.set(!inputs.leftMotorConnected);
        rightMotorAlert.set(!inputs.rightMotorConnected);
    }

    public void setVelocity(AngularVelocity velocity) {
        io.setVelocity(velocity);
    }

    public void stop() {
        io.stop();
    }
}
