package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ToggleableTrigger {
    @SuppressWarnings("unused")
    private final Trigger toggler;
    private final Trigger toggle;
    private boolean toggled = false;

    public ToggleableTrigger(BooleanSupplier toggler) {
        this.toggler = new Trigger(toggler).onTrue(new InstantCommand(() -> toggle()));
        toggle = new Trigger(() -> toggled);
    }

    private void toggle() {
        toggled = !toggled;
    }

    public boolean getToggle() {
        return toggled;
    }

    public Trigger getTrigger() {
        return toggle;
    }
}