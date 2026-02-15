package frc.robot.hardware.baseMotors;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.hardware.CanId;

public abstract class NeoMotorBase extends SparkMaxMotorBase {
    // https://docs.revrobotics.com/brushless/neo/v1.1#motor-specifications
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RPM.of(5676);
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