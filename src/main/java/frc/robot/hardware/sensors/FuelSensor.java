package frc.robot.hardware.sensors;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;

import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.hardware.CanId;

public class FuelSensor extends TimeOfFlightSensorBase {
    private final BooleanSupplier isIndexingToScore;
    private final Timer elapsedIndexingToScore = new Timer();
    private final Timer elapsedSinceNofuel = new Timer();
    private boolean fuelDetected = false;
    private boolean wasIndexingToScore = false;
    private int fuelCount = 0;
    private int fuelTotalCount = 0;

    public FuelSensor(
            String name,
            CanId canId,
            Distance threshold,
            RangingMode mode,
            Time sampleTime,
            BooleanSupplier isIndexingToScore) {
        super(name, canId, threshold, mode, sampleTime);
        this.isIndexingToScore = isIndexingToScore;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addIntegerProperty("Fuel Count", this::getFuelCount, null);
        builder.addIntegerProperty("Total Fuel Count", this::getFuelTotalCount, null);
        builder.addDoubleProperty("Average Fuel Per Second", this::getFuelAveragePerSecond, null);
        builder.addBooleanProperty("Fuel Detected", this::isFuelDetected, null);
    }

    public boolean isFuelDetected() {
        return belowThreshold();
    }

    public void resetFuelCount() {
        fuelCount = 0;
    }

    public int getFuelCount() {
        return fuelCount;
    }

    public int getFuelTotalCount() {
        return fuelTotalCount;
    }

    public double getFuelAveragePerSecond() {
        if (elapsedIndexingToScore.get() == 0)
            return 0.0;

        return fuelCount / elapsedIndexingToScore.get();
    }

    public Time getElapsedSinceNofuel() {
        return Seconds.of(elapsedSinceNofuel.get());
    }

    private void updateBallPerIndexPeriod() {
        boolean indexingToScore = isIndexingToScore.getAsBoolean();

        if (wasIndexingToScore) {
            if (!indexingToScore) {
                resetFuelCount();
                elapsedIndexingToScore.stop();
                elapsedIndexingToScore.reset();
                return;
            }
        } else if (indexingToScore) {
            elapsedIndexingToScore.start();
        }

        boolean fuel = belowThreshold();

        if (fuelDetected) {
            if (!fuel) {
                elapsedSinceNofuel.start();
                fuelCount++;
                fuelTotalCount++;
            }
        } else if (fuel) {
            elapsedSinceNofuel.stop();
            elapsedSinceNofuel.reset();
        }

        fuelDetected = fuel;
    }

    public void update() {
        updateBallPerIndexPeriod();
    }
}