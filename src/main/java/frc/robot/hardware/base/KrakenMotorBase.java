package frc.robot.hardware.base;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.hardware.CanId;

public abstract class KrakenMotorBase extends SparkMaxMotorBase {
    // https://docs.wcproducts.com/welcome/electronics/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance#trapezoidal-commutation
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RPM.of(6000);
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
