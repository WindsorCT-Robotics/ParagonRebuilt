package frc.robot.hardware.base_motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.hardware.CanId;

public abstract class KrakenMotorBase extends SparkMaxMotorBase {
    // https://docs.wcproducts.com/welcome/electronics/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance#trapezoidal-commutation
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RPM.of(6000);
    // https://www.chiefdelphi.com/t/announcing-kraken-x60-powered-by-talon-fx/442236/1212?page=59
    protected static final Current DEFAULT_CURRENT = Amps.of(80);
    private static final MotorType MOTOR_TYPE = MotorType.kBrushless;

    protected KrakenMotorBase(
            String name,
            CanId canId,
            SparkBaseConfig motorConfiguration,
            ResetMode resetMode,
            PersistMode persistMode) {
        super(
                name,
                canId,
                MOTOR_TYPE,
                MAX_ANGULAR_VELOCITY,
                motorConfiguration,
                resetMode,
                persistMode);
    }
}
