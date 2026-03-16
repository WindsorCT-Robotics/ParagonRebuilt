package frc.robot.hardware.motors;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorBase;

public class ShooterMotor extends KrakenMotorBase {
        public ShooterMotor(
                        String name,
                        CanId canId,
                        TalonFXConfiguration configuration) {
                super(
                                name,
                                canId,
                                configuration);
        }

        /**
         * This must be the target velocity because we only set target velocity and not
         * target position.
         * 
         * @return Angular Velocity
         */
        public AngularVelocity getTargetVelocity() {
                return RPM.of(motor.getClosedLoopReference().getValueAsDouble());
        }
}