package frc.robot.hardware.base_motors;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.hardware.CanId;

public abstract class KrakenMotorX60Base extends TalonFXMotorBase {
    // https://docs.wcproducts.com/welcome/electronics/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance#trapezoidal-commutation
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RPM.of(6000);

    protected KrakenMotorX60Base(
            String name,
            CanId canId,
            TalonFXConfiguration configuration) {
        super(
                name,
                canId,
                configuration);
    }
}
