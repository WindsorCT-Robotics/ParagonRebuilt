package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SendableTrigger extends Trigger implements Sendable {
    private final BooleanSupplier condition;
    private final String name;

    public SendableTrigger(EventLoop loop, BooleanSupplier condition, String name) {
        super(loop, condition);
        this.condition = condition;
        this.name = name;
    }

    public SendableTrigger(BooleanSupplier condition, String name) {
        super(condition);
        this.condition = condition;
        this.name = name;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty(name, condition, null);
    }
}