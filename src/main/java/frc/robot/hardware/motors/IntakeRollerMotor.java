package frc.robot.hardware.motors;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.hardware.CanId;
import frc.robot.hardware.IntakeMotorState;
import frc.robot.hardware.base_motors.KrakenMotorX60Base;

public class IntakeRollerMotor extends KrakenMotorX60Base {
        private AngularVelocity targetVelocity = RPM.of(0);
        private IntakeMotorState intakeMotorState = IntakeMotorState.IDLE;
        private Color intakeMotorStateIdle = new Color(255, 255, 255);
        private Color intakeMotorStateIntaking = new Color(67, 217, 155);
        private Color intakeMotorStateShuttling = new Color(217, 72, 67);
        private Color intakeMotorStateColor = new Color(255, 255, 255);

        public IntakeRollerMotor(
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
                builder.addStringProperty("Intake Motor State", () -> getIntakeMotorState().toString(), null);
                builder.addStringProperty("Intake Motor State Color", () -> getIntakeStateColor(), null);
                builder.addDoubleProperty("Target Velocity (RPM)", () -> getTargetVelocity().in(RPM), null);
        }

        @Override
        public void setPointVelocity(AngularVelocity angularVelocity) {
                targetVelocity = angularVelocity;
                super.setPointVelocity(angularVelocity);
        }

        private void setIntakeStateColor(Color color) {
                intakeMotorStateColor = color;
        }

        private String getIntakeStateColor() {
                return intakeMotorStateColor.toHexString();
        }

        private IntakeMotorState getIntakeMotorState() {
                return intakeMotorState;
        }

        public void setState(IntakeMotorState motorState) {
                intakeMotorState = motorState;

                switch (getIntakeMotorState()) {
                        case IDLE:
                                setIntakeStateColor(intakeMotorStateIdle);
                                break;
                        case INTAKING:
                                setIntakeStateColor(intakeMotorStateIntaking);
                                break;
                        case SHUTTLING:
                                setIntakeStateColor(intakeMotorStateShuttling);
                                break;
                }
        }

        private AngularVelocity getTargetVelocity() {
                return targetVelocity;
        }
}