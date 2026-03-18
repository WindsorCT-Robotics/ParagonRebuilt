package frc.robot.interfaces;

import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public interface ITimeOfFlightSensor {
    public Distance getDistance();
    public boolean atThreshold();
    public void setThreshold(Distance threshold);
    public void setRangeMode(RangingMode mode, Time sampleTime);
}
