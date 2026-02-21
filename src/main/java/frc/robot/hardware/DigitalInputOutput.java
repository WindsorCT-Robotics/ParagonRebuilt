package frc.robot.hardware;

public record DigitalInputOutput(byte Id) {
    private static final byte MIN_PORT_ID = 0;
    private static final byte MAX_PORT_ID = 9;

    public DigitalInputOutput {
        if (Byte.toUnsignedInt(Id) < MIN_PORT_ID || Byte.toUnsignedInt(Id) > MAX_PORT_ID) {
            throw new IllegalArgumentException(
                    String.format("Valid DigitalInputOutput port IDs are in the range [%d, %d]. Provided value: %d", MIN_PORT_ID, MAX_PORT_ID, Id));
        }
    }
}