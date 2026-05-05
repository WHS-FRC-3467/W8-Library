package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayDeque;
import java.util.function.BooleanSupplier;

/**
 * Counts the number of times a boolean supplier becomes true (rising edge) within a sliding time
 * window.
 */
public class Count {
    private final BooleanSupplier source;
    private final double timeWindow;

    private final ArrayDeque<Double> events = new ArrayDeque<>();

    /**
     * Constructs a Count detector.
     *
     * @param source The boolean supplier to count rising edges from
     * @param time The sliding window size in seconds
     */
    public Count(BooleanSupplier source, double time) {
        this.source = RisingEdge.of(source);
        this.timeWindow = time;
    }

    /**
     * Creates a Count detector from a boolean supplier.
     *
     * @param time The sliding window size in seconds
     * @param source The boolean supplier to count over
     * @return A new Count instance
     */
    public static Count over(double time, BooleanSupplier source) {
        return new Count(source, time);
    }

    /**
     * Should be called periodically. Automatically called by {@link Count#get()} and other
     * data-accessing methods. Only call this manually if you are not periodically accessing values
     * from this class
     */
    public void update() {
        double t = Timer.getTimestamp();

        // source is already wrapped in RisingEdge, so a true value is a single event pulse
        if (source.getAsBoolean()) {
            events.addLast(t);
        }

        // Remove events older than the time window
        double cutoff = t - timeWindow;
        while (!events.isEmpty() && events.peekFirst() < cutoff) {
            events.removeFirst();
        }
    }

    /**
     * @return The number of rising-edge events in the last timeWindow seconds
     */
    public int get() {
        update();
        return events.size();
    }

    /**
     * @return If the number of rising-edge events in the last timeWindow seconds is greater than
     *     {@code n}
     */
    public BooleanSupplier greaterThan(int n) {
        return () -> get() > n;
    }

    /**
     * @return If the number of rising-edge events in the last timeWindow seconds is less than
     *     {@code n}
     */
    public BooleanSupplier lessThan(int n) {
        return () -> get() < n;
    }

    /**
     * @return If the number of rising-edge events in the last timeWindow seconds is greater than or
     *     equal to {@code n}
     */
    public BooleanSupplier greaterThanEquals(int n) {
        return () -> get() >= n;
    }

    /**
     * @return If the number of rising-edge events in the last timeWindow seconds is less than or
     *     equal to {@code n}
     */
    public BooleanSupplier lessThanEquals(int n) {
        return () -> get() <= n;
    }

    /**
     * @return If the number of rising-edge events in the last timeWindow seconds is equal to {@code
     *     n}
     */
    public BooleanSupplier is(int n) {
        return () -> get() == n;
    }
}
