package frc.robot.hardware.base_sensors;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Milliseconds;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.hardware.CanId;
import frc.robot.interfaces.ITimeOfFlightSensor;

public class TimeOfFlightSensorBase implements ITimeOfFlightSensor, Sendable {
    private final TimeOfFlight sensor;
    private final String name;
    private Distance threshold;

    public TimeOfFlightSensorBase(
            String name,
            CanId canId,
            Distance threshold,
            RangingMode mode,
            Time sampleTime) {
        sensor = new TimeOfFlight(canId.Id());
        this.threshold = threshold;
        this.name = name;
        setRangeMode(mode, sampleTime);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Sensor Distance (Millimeters)", () -> getDistance().in(Millimeters), null);
        builder.addBooleanProperty("pastThreshold", this::pastThreshold, null);
        builder.addBooleanProperty("belowThreshold", this::belowThreshold, null);
    }

    @Override
    public Distance getDistance() {
        return Millimeters.of(sensor.getRange());
    }

    @Override
    public boolean pastThreshold() {
        return getDistance().gt(threshold);
    }

    @Override
    public boolean belowThreshold() {
        return getDistance().lt(threshold);
    }

    @Override
    public boolean atThreshold(Distance leniency) {
        boolean atThreshold = getDistance().isNear(threshold, leniency);
        SmartDashboard.putBoolean("atThreshold", atThreshold);
        return atThreshold;
    }

    @Override
    public void setThreshold(Distance threshold) {
        this.threshold = threshold;
    }

    @Override
    public void setRangeMode(RangingMode mode, Time sampleTime) {
        sensor.setRangingMode(mode, sampleTime.in(Milliseconds));
    }

    public String getSmartDashboardName() {
        return name;
    }
}