package frc.robot.interfaces;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface ISystemDynamics {
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction);

    public Command sysIdDynamic(SysIdRoutine.Direction direction);

    public Command overrideMotorVoltage(Voltage v);
}
