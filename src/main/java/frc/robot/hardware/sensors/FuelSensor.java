package frc.robot.hardware.sensors;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;

import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.hardware.CanId;
import frc.robot.hardware.base_sensors.TimeOfFlightSensorBase;

public class FuelSensor extends TimeOfFlightSensorBase {
    private final BooleanSupplier isIndexingToScore;
    private final Timer elapsedIndexingToScore = new Timer();
    private final Timer elapsedSinceNofuel = new Timer();
    private boolean wasFuelDetected = false;
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

    public void update() {
        boolean indexingToScore = isIndexingToScore.getAsBoolean();

        // If indexing to score into hub but wasn't indexing before.
        if (indexingToScore && !wasIndexingToScore) {
            wasIndexingToScore = true;
            elapsedIndexingToScore.start();
            /*
             * On start-up, the conditions:
             * currentlyFuelDetected && !wasFuelDetected
             * !currentlyFuelDetected && wasFuelDetected
             * are both false, thus to ensure the first
             * fuel is unjammed it must presume that
             * there was no fuel on start-up.
             */
            elapsedSinceNofuel.restart();
        }

        // If not indexing to score into hub but was indexing before.
        if (!indexingToScore && wasIndexingToScore) {
            resetFuelCount();
            elapsedIndexingToScore.stop();
            elapsedIndexingToScore.reset();
            wasIndexingToScore = false;
        }

        boolean currentlyFuelDetected = belowThreshold();

        // If the fuel was detected and previously wasn't.
        // Fuel is present.
        if (currentlyFuelDetected && !wasFuelDetected) {
            wasFuelDetected = true;
            elapsedSinceNofuel.stop();
            elapsedSinceNofuel.reset();
        }

        // If the fuel was previously detected and the fuel was no longer detected.
        // Fuel is no longer present, thus measures for how long until the next fuel.
        if (!currentlyFuelDetected && wasFuelDetected) {
            wasFuelDetected = false;
            elapsedSinceNofuel.start();
            fuelCount++;
            fuelTotalCount++;
        }
    }
}