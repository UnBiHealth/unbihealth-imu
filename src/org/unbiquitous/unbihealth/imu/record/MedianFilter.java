package org.unbiquitous.unbihealth.imu.record;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.Comparator;

/**
 * This filter stores the last <code>k</code> submitted samples and, each time, returns the median
 * value amongst them. For better results, <code>k</code> should be odd.
 *
 * @param <T> The type of every sample in the curve.
 * @author Luciano Santos
 */
public final class MedianFilter<T> {
    private final T[] lastSamples;
    private int k;
    private Comparator<? super T> comparator;

    /**
     * Creates a new instance of the filter for given {@link Comparable} class, with
     * k = 3.
     *
     * @param clazz The type of the samples in the curve.
     * @param <T>   A comparable type.
     * @return A new median filter using default comparator for comparable type <code>T</code>.
     * @see #create(Class, int)
     */
    public static <T extends Comparable<T>> MedianFilter<T> create(Class<T> clazz) {
        return create(clazz, 3);
    }

    /**
     * Creates a new instance of the filter for given {@link Comparable} class.
     *
     * @param clazz The type of the samples in the curve.
     * @param k     The number of samples to consider.
     * @param <T>   A comparable type.
     * @return A new median filter using default comparator for comparable type <code>T</code>.
     * @throws IllegalArgumentException If k < 3.
     */
    public static <T extends Comparable<T>> MedianFilter<T> create(Class<T> clazz, int k) {
        Comparator<T> comparator = new Comparator<T>() {
            @Override
            public int compare(T o1, T o2) {
                return o1.compareTo(o2);
            }
        };
        return new MedianFilter<>(clazz, comparator, k);
    }

    /**
     * Creates a new instance of the filter for given class and a comparator, with
     * k = 3.
     *
     * @param comparator The comparator for the type of the samples in the curve.
     * @param <T>        Type of the samples in the curve.
     * @return A new median filter using the given comparator.
     * @throws NullPointerException If either <code>clazz</code> or <code>comparator</code> is null.
     * @see #create(Class, Comparator, int)
     */
    public static <T> MedianFilter<T> create(Class<T> clazz, Comparator<T> comparator) {
        return new MedianFilter<>(clazz, comparator, 3);
    }

    /**
     * Creates a new instance of the filter for given class and a comparator.
     *
     * @param comparator The comparator for the type of the samples in the curve.
     * @param k          The number of samples to consider.
     * @param <T>        Type of the samples in the curve.
     * @return A new median filter using the given comparator and k.
     * @throws NullPointerException     If either <code>clazz</code> or <code>comparator</code> is null.
     * @throws IllegalArgumentException If k < 3.
     */
    public static <T> MedianFilter<T> create(Class<T> clazz, Comparator<T> comparator, int k) {
        return new MedianFilter<>(clazz, comparator, k);
    }

    @SuppressWarnings("unchecked")
    private MedianFilter(Class<T> clazz, Comparator<T> comparator, int k) {
        if (comparator == null)
            throw new NullPointerException("comparator");
        if (k < 3)
            throw new IllegalArgumentException("k to small");

        this.lastSamples = (T[]) Array.newInstance(clazz, k);
        this.k = 0;
        this.comparator = comparator;
    }

    /**
     * Stores the given sample at the end of the sequence of last k samples, discarding the oldest
     * one if necessary, and returns the median of these samples.
     *
     * @param point The new sample.
     * @return The current median of the last k samples.
     */
    public T sample(T point) {
        // Adds sample to the local history.
        log(point);

        // Returns the median of the samples.
        return median();
    }

    private void log(T point) {
        // Did not fill the array yet...
        if (k < lastSamples.length)
            lastSamples[k++] = point;
        else {
            // Already filled, discards oldest sample.
            int i = 0;
            for (; i < k - 1; ++i)
                lastSamples[i] = lastSamples[i + 1];
            lastSamples[i] = point;
        }
    }

    private T median() {
        if (k < 3)
            return lastSamples[k - 1];
        T[] sorted = Arrays.copyOf(lastSamples, k);
        Arrays.sort(sorted, comparator);
        return sorted[k / 2];
    }
}
