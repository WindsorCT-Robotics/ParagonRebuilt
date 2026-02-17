package frc.robot.hardware;

public record CanId(byte Id) {
    public CanId {
        if (Byte.toUnsignedInt(Id) < 0 || Byte.toUnsignedInt(Id) > 0x3F) {
            throw new IllegalArgumentException(
                    String.format("Valid CAN IDs are 6-bit unsigned integers. Provided value: %d", Id));
        }
    }
}