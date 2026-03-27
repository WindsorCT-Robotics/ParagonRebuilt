package frc.robot.hardware.motors;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorBase;

public class LauncherMotor extends KrakenMotorBase {
        public AngularVelocity targetVelocity = RPM.zero();

        public LauncherMotor(
                        String name,
                        CanId canId,
                        TalonFXConfiguration configuration) {
                super(
                                name,
                                canId,
                                configuration);
        }

        @Override
        public void initSendable(SendableBuilder builder) {
                super.initSendable(builder);
                builder.addDoubleProperty("Target Velocity (RPM)", () -> getTargetVelocity().in(RPM), null);
        }

        /**
         * This must be the target velocity because we only set target velocity and not
         * target position.
         * 
         * @return Angular Velocity
         */
        public AngularVelocity getTargetVelocity() {
                return targetVelocity;
        }

        @Override
        public void setPointVelocity(AngularVelocity velocity) {
                super.setPointVelocity(velocity);
                targetVelocity = velocity;
        }
}