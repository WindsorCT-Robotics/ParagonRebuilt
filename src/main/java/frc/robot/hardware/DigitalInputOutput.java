package frc.robot.hardware;

public record DigitalInputOutput(byte Id) {
    public DigitalInputOutput {
        if (Byte.toUnsignedInt(Id) < 0 || Byte.toUnsignedInt(Id) > 10) {
            throw new IllegalArgumentException(
                    String.format("Valid DigitalInputOutput are byte values between 1-10. Provided value: %d", Id));
        }
    }
}