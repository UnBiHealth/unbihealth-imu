package org.unbiquitous.unbihealth.imu;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import org.apache.commons.math3.complex.Quaternion;
import org.unbiquitous.unbihealth.imu.util.QuaternionDeserializer;
import org.unbiquitous.unbihealth.imu.util.QuaternionSerializer;

public class Sample {
    @JsonProperty(required = true)
    private long timestamp;

    @JsonProperty(required = true)
    @JsonSerialize(using = QuaternionSerializer.class)
    @JsonDeserialize(using = QuaternionDeserializer.class)
    private Quaternion quaternion;

    public Sample() {
        this(-1, null);
    }

    public Sample(long timestamp, Quaternion quaternion) {
        this.timestamp = timestamp;
        this.quaternion = quaternion;
    }

    public long getTimestamp() {
        return timestamp;
    }

    public void setTimestamp(long timestamp) {
        this.timestamp = timestamp;
    }

    public Quaternion getQuaternion() {
        return quaternion;
    }

    public void setQuaternion(Quaternion quaternion) {
        this.quaternion = quaternion;
    }
}
