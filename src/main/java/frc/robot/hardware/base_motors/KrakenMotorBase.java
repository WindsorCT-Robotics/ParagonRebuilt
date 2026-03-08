package frc.robot.hardware.base_motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.hardware.CanId;

public abstract class KrakenMotorBase extends TalonFXMotorBase {
    // https://docs.wcproducts.com/welcome/electronics/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance#trapezoidal-commutation
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RPM.of(6000);
    // https://www.chiefdelphi.com/t/announcing-kraken-x60-powered-by-talon-fx/442236/1212?page=59
    public static final Current DEFAULT_CURRENT = Amps.of(80);

    protected KrakenMotorBase(
            String name,
            CanId canId) {
        super(
                name,
                canId);
    }
}
