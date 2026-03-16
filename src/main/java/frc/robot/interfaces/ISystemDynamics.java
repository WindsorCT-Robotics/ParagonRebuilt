package frc.robot.interfaces;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface ISystemDynamics<Motor> {
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction);

    public Command sysIdDynamic(SysIdRoutine.Direction direction);

    public void log(SysIdRoutineLog log, Motor motor, String name);
}
