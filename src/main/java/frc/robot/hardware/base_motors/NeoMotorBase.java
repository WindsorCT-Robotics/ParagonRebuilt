package frc.robot.hardware.base_motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Consumer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.CanId;

public abstract class NeoMotorBase extends SparkMaxMotorBase {
    // https://docs.revrobotics.com/brushless/neo/v1.1#motor-specifications
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RPM.of(5676);
    // https://docs.revrobotics.com/brushless/spark-max/gs/make-it-spin#limiting-current
    protected static final Current DEFAULT_CURRENT = Amps.of(60);

    /**
     * @param canId
     * @param configuration
     * @param resetMode     Don't reset safe parameters if intending to update
     *                      configuration after initalization.
     * @param persistMode
     */
    protected NeoMotorBase(
            CanId canId,
            SparkBaseConfig configuration,
            ResetMode resetMode,
            PersistMode persistMode,
            Consumer<Dimensionless> dutyCycleSetter,
            Consumer<Voltage> voltageSetter) {
        super(
                canId,
                configuration,
                resetMode,
                persistMode,
                dutyCycleSetter,
                voltageSetter);
    }
}