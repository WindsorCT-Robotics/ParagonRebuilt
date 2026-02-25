package frc.robot.subsystems;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.hardware.basic_implementations.intake_motors.BayDoorMotorBasic;
import frc.robot.interfaces.ISystemDynamics;

public class BayDoorKraken extends SubsystemBase implements ISystemDynamics<BayDoorMotorBasic> {

    public BayDoorKraken() {

    }

    @Override
    public void log(SysIdRoutineLog log, BayDoorMotorBasic motor, String name) {
        // TODO Auto-generated method stub

    }

    @Override
    public Command sysIdDynamic(Direction direction) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Command sysIdQuasistatic(Direction direction) {
        // TODO Auto-generated method stub
        return null;
    }

}
