package frc.robot;

public record CanID(byte ID) {
  public CanID {
    if (Byte.toUnsignedInt(ID) < 0 || Byte.toUnsignedInt(ID) > 0x3F) {
      throw new IllegalArgumentException(
          String.format("Valid CAN IDs are 6-bit unsigned integers. Provided value: %d", ID));
    }
  }
}
