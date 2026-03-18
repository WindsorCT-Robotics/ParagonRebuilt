package frc.robot.hardware.sensors;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Milliseconds;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.hardware.CanId;
import frc.robot.interfaces.ITimeOfFlightSensor;

public class TimeOfFlightSensor implements ITimeOfFlightSensor, Sendable {
    private final TimeOfFlight sensor;
    private final String name;
    private Distance threshold;

    public TimeOfFlightSensor(String name, CanId canId, Distance threshold) {
        sensor = new TimeOfFlight(canId.Id());
        this.threshold = threshold;
        this.name = name;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Sensor Distance (Inches)", () -> getDistance().in(Inches), null);
        builder.addBooleanProperty("atThreshold", this::atThreshold, null);
    }

    @Override
    public Distance getDistance() {
        return Millimeters.of(sensor.getRange());
    }

    @Override
    public boolean atThreshold() {
        return threshold.gte(getDistance());
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