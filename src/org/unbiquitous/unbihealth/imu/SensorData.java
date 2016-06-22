package org.unbiquitous.unbihealth.imu;

import com.fasterxml.jackson.annotation.JsonProperty;

public class SensorData extends Sample {
    @JsonProperty(required = true)
    private String id;

    public String getId() {
        return id;
    }

    public void setId(String sensor) {
        this.id = sensor;
    }
}
