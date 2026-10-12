package frc.robot.subsystems.baydoor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PowerLogger;
import frc.robot.PowerLogger.Device;

public class Baydoor extends SubsystemBase {
    private final BaydoorIO io;
    private final BaydoorIOInputsAutoLogged inputs = new BaydoorIOInputsAutoLogged();
    private final Alert leftMotorAlert = new Alert("Left Baydoor Motor Disconnected!", AlertType.kError);
    private final Alert rightMotorAlert = new Alert("Right Baydoor Motor Disconnected!", AlertType.kError);


    public Baydoor(BaydoorIO io) {
        this.io = io;
        PowerLogger.addSubsystem(
            "Baydoor", 
            new Device("Left Baydoor Motor", () -> inputs.leftSupplyCurrentAmps), 
            new Device("Right Baydoor Motor", () -> inputs.rightSupplyCurrentAmps)
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("~MyInputs/Baydoor/", inputs);
        leftMotorAlert.set(!inputs.leftMotorConnected);
        rightMotorAlert.set(!inputs.rightMotorConnected);
    }

    public void setPosition(Angle angle) {
        io.setPosition(angle);
    }

    public void setDutyCycle(Dimensionless percent) {
        io.setDutyCycle(percent);
    }

    public void stop() {
        io.stop();
    }
}