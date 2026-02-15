package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.intakeMotors.IntakeBayDoorDualMotors;

public class Intake extends SubsystemBase {

    public Intake(
            String name,
            IntakeBayDoorDualMotors dualMotors) {
        SendableRegistry.add(this, name);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}
