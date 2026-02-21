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

public abstract class NeoMotorBase extends SparkMaxMotorBase {
    // https://docs.revrobotics.com/brushless/neo/v1.1#motor-specifications
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RPM.of(5676);
    // https://docs.revrobotics.com/brushless/spark-max/gs/make-it-spin#limiting-current
    protected static final Current DEFAULT_CURRENT = Amps.of(60);
    private static final MotorType MOTOR_TYPE = MotorType.kBrushless;

    protected NeoMotorBase(
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