package org.unbiquitous.unbihealth.imu.record;

import org.apache.commons.math3.complex.Quaternion;
import org.unbiquitous.unbihealth.imu.Sample;

import java.util.ArrayList;
import java.util.List;

import static org.unbiquitous.unbihealth.imu.util.QuaternionUtils.interpolate;

/**
 * Holds a sensor's recorded curve.
 *
 * @author Luciano Santos
 */
public class Recorder {
    private String id;
    private int step;
    private MedianFilter<?>[] filters;
    private boolean interpolate;
    private List<Sample> data;

    public Recorder(String id, int step, boolean interpolate) {
        this.id = id;
        this.step = step;
        this.interpolate = interpolate;

        filters = new MedianFilter<?>[4];
        for (int i = 0; i < filters.length; ++i)
            filters[i] = MedianFilter.create(Double.class);

        data = new ArrayList<>();
    }

    public String getId() {
        return id;
    }

    @SuppressWarnings("unchecked")
    public void add(long ts, Quaternion q) {
        double q0 = ((MedianFilter<Double>) filters[0]).sample(q.getQ0());
        double q1 = ((MedianFilter<Double>) filters[1]).sample(q.getQ1());
        double q2 = ((MedianFilter<Double>) filters[2]).sample(q.getQ2());
        double q3 = ((MedianFilter<Double>) filters[3]).sample(q.getQ3());

        // The last sample.
        Sample n = new Sample(ts, new Quaternion(q0, q1, q2, q3));

        if (data.isEmpty())
            // If it's the first sample, just stores.
            data.add(n);
        else {
            // Sets the value as the last sample.
            if (data.size() == 1)
                data.add(n);
            else {
                Sample n_1 = data.get(data.size() - 1);
                Sample n_2 = data.get(data.size() - 2);
                double dt = n_1.getTimestamp() - n_2.getTimestamp();
                if (dt >= step) {
                    if (!this.interpolate)
                        data.add(n);
                    else {
                        while (dt >= step) {
                            n_2 = new Sample(
                                    n_2.getTimestamp() + step,
                                    interpolate(n_2.getQuaternion(), n_1.getQuaternion(), step / dt)
                            );
                            data.add(data.size() - 1, n_2);
                            dt = n_1.getTimestamp() - n_2.getTimestamp();
                        }
                        data.set(data.size() - 1, n);
                    }
                } else
                    data.set(data.size() - 1, n);
            }
        }
    }

    public List<Sample> getData() {
        return data;
    }
}
