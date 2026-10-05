package frc.robot.robot_container;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

public class Bindings {
  /** XboxCL (Xbox Controller Layout) */
  public enum XboxCL {
    // Face buttons
    A,
    B,
    X,
    Y,

    LB,
    RB,

    // Analog triggers, treated as buttons past a deflection threshold
    LT,
    RT,

    // Stick clicks
    LS,
    RS,

    // Center buttons
    BACK,
    START,

    // D-pad
    D_UP,
    D_DOWN,
    D_RIGHT,
    D_LEFT
  }

  public enum TriggerBehavior {
    ON_TRUE,
    WHILE_TRUE,
    TOGGLE_ON_TRUE
  }

  private record QueueBinding(Command command, TriggerBehavior tb, Set<XboxCL> buttons) {
    // Forces the buttons to be immutable.
    QueueBinding {
      buttons = Set.copyOf(buttons);
    }
  }

  private record BindingLog(BooleanSupplier bindingSupplier, String name) {}

  private final String name;
  private final XboxController controller;
  private final ArrayList<BindingLog> layout = new ArrayList<>();
  private final ArrayList<QueueBinding> queuedBindings = new ArrayList<>();
  private static final double TRIGGER_THRESHOLD = 0.5; // Percent
  private static final double DEBOUNCE = 0.15; // Seconds

  private boolean build = false;

  public Bindings(String name, XboxController controller) {
    this.name = name;
    this.controller = controller;
  }

  public void bind(Command command, TriggerBehavior tb, XboxCL button, XboxCL... buttons) {
    if (build) throw new IllegalStateException("Cannot bind after build()!");

    Set<XboxCL> buttonSet = new HashSet<>();
    buttonSet.add(button);
    for (XboxCL b : buttons) {
      if (!buttonSet.add(b)) throw new IllegalStateException("Cannot add duplicate buttons.");
    }

    for (QueueBinding queuedBinding : queuedBindings) {
      boolean sameSize = queuedBinding.buttons().size() == buttonSet.size();
      boolean sameSet = queuedBinding.buttons().containsAll(buttonSet);
      if (sameSize && sameSet)
        throw new IllegalStateException("Button combination already exists.");
    }

    queuedBindings.add(new QueueBinding(command, tb, buttonSet));
  }

  public void build() {
    if (build) {
      return;
    }
    build = true;

    for (QueueBinding binding : queuedBindings) {
      Trigger trigger = new Trigger(buttonsToBooleanSupplier(binding.buttons()));
      boolean debounce = false;
      // Check for overlap. If overlap, add debounce accordingly.
      for (QueueBinding otherBinding : queuedBindings) {
        int bindingSize = binding.buttons().size();
        int otherBindingSize = otherBinding.buttons().size();

        boolean isOtherBigger = otherBindingSize > bindingSize;
        boolean doesOtherContainBinding = otherBinding.buttons().containsAll(binding.buttons());

        /*
         * This if-statement checks:
         * 1. Is the combo size of other larger than binding.
         * If so, then binding must be the smaller of the two,
         * thus binding should have debounce to prevent
         * binding from accidental triggering.
         * 2. Is the combo of binding in the combo of other.
         * If so, then binding may be triggered by accident,
         * thus, binding should have debounce.
         */
        if (isOtherBigger && doesOtherContainBinding) {
          // The buttons that must not be pressed.
          // It creates a hashset to make the set mutable.
          Set<XboxCL> condition = new HashSet<>(otherBinding.buttons());
          condition.removeAll(binding.buttons());

          Trigger conditionTrigger = new Trigger(buttonsToBooleanSupplier(condition)).negate();
          trigger = trigger.and(conditionTrigger);
          debounce = true;
        }
      }

      if (debounce) trigger = trigger.debounce(DEBOUNCE);

      Command command = binding.command();

      switch (binding.tb) {
        case ON_TRUE:
          trigger.onTrue(command);
          break;
        case WHILE_TRUE:
          trigger.whileTrue(command);
          break;
        case TOGGLE_ON_TRUE:
          trigger.toggleOnTrue(command);
          break;
      }

      String buttonOrder =
          binding.buttons().stream().map(Enum::name).sorted().collect(Collectors.joining("+"));
      String name =
          "~MyOutputs/Controllers/"
              + this.name
              + "/"
              + buttonOrder
              + " | "
              + binding.command().getName();

      layout.add(new BindingLog(trigger, name));
    }
  }

  public void periodic() {
    layout.forEach(
        (log) -> {
          Logger.recordOutput(log.name(), log.bindingSupplier());
        });
  }

  private BooleanSupplier buttonsToBooleanSupplier(Set<XboxCL> buttons) {
    BooleanSupplier buttonsSupplier = null;
    for (XboxCL button : buttons) {
      BooleanSupplier next = buttonToBooleanSupplier(button);
      if (buttonsSupplier == null) {
        buttonsSupplier = next;
      } else {
        final BooleanSupplier prev = buttonsSupplier;
        buttonsSupplier = () -> (prev.getAsBoolean() && next.getAsBoolean());
      }
    }

    return buttonsSupplier;
  }

  private BooleanSupplier buttonToBooleanSupplier(XboxCL button) {
    BooleanSupplier buttonSupplier;
    switch (button) {
      case A:
        buttonSupplier = () -> controller.getAButton();
        break;
      case B:
        buttonSupplier = () -> controller.getBButton();
        break;
      case X:
        buttonSupplier = () -> controller.getXButton();
        break;
      case Y:
        buttonSupplier = () -> controller.getYButton();
        break;
      case LB:
        buttonSupplier = () -> controller.getLeftBumperButton();
        break;
      case RB:
        buttonSupplier = () -> controller.getRightBumperButton();
        break;
      case LT:
        buttonSupplier = () -> controller.getLeftTriggerAxis() >= TRIGGER_THRESHOLD;
        break;
      case RT:
        buttonSupplier = () -> controller.getRightTriggerAxis() >= TRIGGER_THRESHOLD;
        break;
      case LS:
        buttonSupplier = () -> controller.getLeftStickButton();
        break;
      case RS:
        buttonSupplier = () -> controller.getRightStickButton();
        break;
      case BACK:
        buttonSupplier = () -> controller.getBackButton();
        break;
      case START:
        buttonSupplier = () -> controller.getStartButton();
        break;
      case D_UP:
        buttonSupplier = () -> controller.getPOV() == 0;
        break;
      case D_DOWN:
        buttonSupplier = () -> controller.getPOV() == 180;
        break;
      case D_RIGHT:
        buttonSupplier = () -> controller.getPOV() == 90;
        break;
      case D_LEFT:
        buttonSupplier = () -> controller.getPOV() == 270;
        break;
      default:
        throw new IllegalStateException("Unknown XboxCL Enum: " + button.toString());
    }

    return buttonSupplier;
  }
}
