package frc.robot.hardware.tuners;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.interfaces.IMaxMotionTuner;

public class SparkMaxTuner implements IMaxMotionTuner, Sendable {
    private final SparkMax motor;
    private final ResetMode resetMode;
    private final PersistMode persistMode;

    // TODO: Figure `double` conversions to a measurement.
    // TODO: Ensure that it sets kV with Rotations Per Second.
    /**
     * 
     * @param motor
     * @param resetMode   Use parameter ResetMode.kNoResetSafeParameters if your
     *                    want to update config and not reset.
     * @param persistMode
     */
    public SparkMaxTuner(SparkMax motor, ResetMode resetMode, PersistMode persistMode) {
        this.motor = motor;
        this.resetMode = resetMode;
        this.persistMode = persistMode;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Proportional Gain", this::getP, this::updateP);
        builder.addDoubleProperty("Integral Gain", this::getI, this::updateI);
        builder.addDoubleProperty("Derivative Gain", this::getD, this::updateD);
        builder.addDoubleProperty("Static Gain (V)", () -> getKS().in(Volts), this::updateKS);
        builder.addDoubleProperty("Velocity Gain (RPS)", () -> getKV().in(Volts), this::updateKS);
        builder.addDoubleProperty("Acceleration Gain (RPS^2)", () -> getKA().in(Volts), this::updateKA);
        builder.addDoubleProperty("Elevator Gravity Gain", () -> getKG().in(Volts), this::updateKG);
        builder.addDoubleProperty("Arm Gravity Gain", () -> getKCos().in(Volts), this::updateKCos);
        builder.addDoubleProperty("Ratio Constant of Arm Gravity Gain", this::getKCosRatio, this::updateKCosRatio);
        builder.addDoubleProperty("Allowed Profile Error (Rotations)", () -> getAllowedProfileError().in(Rotations),
                this::updateAllowedProfileError);
        builder.addDoubleProperty("Cruise Velocity (RPS)", () -> getCruiseVelocity().in(RotationsPerSecond),
                this::updateCruiseVelocity);
        builder.addDoubleProperty("Max Acceleration (RPS^2)",
                () -> getMaxAcceleration().in(RotationsPerSecondPerSecond), this::updateMaxAcceleration);
    }

    // region PID
    @Override
    public void updateP(double value) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.p(value);
        update(config);
    }

    @Override
    public double getP() {
        return getConfigAccessor().closedLoop.getP();
    }

    @Override
    public void updateI(double value) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.i(value);
        update(config);
    }

    @Override
    public double getI() {
        return getConfigAccessor().closedLoop.getI();
    }

    @Override
    public void updateD(double value) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.d(value);
        update(config);
    }

    @Override
    public double getD() {
        return getConfigAccessor().closedLoop.getD();
    }
    // endregion

    // region FeedForward
    @Override
    public void updateKS(double value) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.feedForward.kS(value);
        update(config);
    }

    @Override
    public Voltage getKS() {
        return Volts.of(getConfigAccessor().closedLoop.feedForward.getkS());
    }

    @Override
    public void updateKV(double value) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.feedForward.kV(value);
        update(config);
    }

    @Override
    public Voltage getKV() {
        return Volts.of(getConfigAccessor().closedLoop.feedForward.getkV());
    }

    @Override
    public void updateKA(double value) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.feedForward.kA(value);
        update(config);
    }

    @Override
    public Voltage getKA() {
        return Volts.of(getConfigAccessor().closedLoop.feedForward.getkA());
    }

    @Override
    public void updateKG(double value) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.feedForward.kCos(value);
        update(config);
    }

    @Override
    public Voltage getKG() {
        return Volts.of(getConfigAccessor().closedLoop.feedForward.getkG());
    }

    @Override
    public void updateKCos(double value) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.feedForward.kCos(value);
        update(config);
    }

    @Override
    public Voltage getKCos() {
        return Volts.of(getConfigAccessor().closedLoop.feedForward.getkCos());
    }

    @Override
    public void updateKCosRatio(double value) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.feedForward.kCosRatio(value);
        update(config);
    }

    @Override
    public double getKCosRatio() {
        return getConfigAccessor().closedLoop.feedForward.getkCosRatio();
    }
    // endregion

    // region MaxMotion
    @Override
    public void updateAllowedProfileError(double value) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.maxMotion.allowedProfileError(value);
        update(config);
    }

    @Override
    public Angle getAllowedProfileError() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void updateCruiseVelocity(double value) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.maxMotion.cruiseVelocity(value);
        update(config);
    }

    @Override
    public AngularVelocity getCruiseVelocity() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void updateMaxAcceleration(double value) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.maxMotion.maxAcceleration(value);
        update(config);
    }

    @Override
    public AngularAcceleration getMaxAcceleration() {
        // TODO Auto-generated method stub
        return null;
    }
    // endregion

    private SparkMaxConfigAccessor getConfigAccessor() {
        return motor.configAccessor;
    }

    private void update(SparkBaseConfig config) {
        motor.configure(config, resetMode, persistMode);
    }
}