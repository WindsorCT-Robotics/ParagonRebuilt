package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.CanID;

public class LedCandleIO implements LedIO {
  private final CANdle led;

  public LedCandleIO(CanID can) {
    led = new CANdle(can.ID());
  }

  @Override
  public void setColor(Color color, int startIndex, int endIndex) {
    led.setControl(new SolidColor(startIndex, endIndex).withColor(new RGBWColor(color)));
  }

  @Override
  public void setPattern(LedPattern pattern, Color color, int startIndex, int endIndex) {
    switch (pattern) {
      case TWINKLE:
        led.setControl(new TwinkleAnimation(startIndex, endIndex).withColor(new RGBWColor(color)));
        break;
      case STROBE:
        led.setControl(new StrobeAnimation(startIndex, endIndex).withColor(new RGBWColor(color)));
        break;
      case SINGLE_FADE:
        led.setControl(
            new SingleFadeAnimation(startIndex, endIndex).withColor(new RGBWColor(color)));
        break;
      case RGB_FADE:
        led.setControl(new RgbFadeAnimation(startIndex, endIndex));
        break;
      case LARSON:
        led.setControl(new LarsonAnimation(startIndex, endIndex).withColor(new RGBWColor(color)));
        break;
      case FIRE:
        led.setControl(new FireAnimation(startIndex, endIndex));
        break;
      case COLOR_FLOW:
        led.setControl(
            new ColorFlowAnimation(startIndex, endIndex).withColor(new RGBWColor(color)));
        break;
    }
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.connected = led.isConnected();
  }
}
