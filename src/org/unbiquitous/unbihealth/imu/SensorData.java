package org.unbiquitous.unbihealth.imu;

import org.apache.commons.math3.complex.Quaternion;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;

public class SensorData {
	@JsonProperty(required = true)
	private String id;

	@JsonProperty(required = true)
	private Long timestamp;

	@JsonProperty(required = true)
	@JsonSerialize(using = QuaternionSerializer.class)
	@JsonDeserialize(using = QuaternionDeserializer.class)
	private Quaternion quaternion;

	public String getId() {
		return id;
	}

	public void setId(String sensor) {
		this.id = sensor;
	}

	public Long getTimestamp() {
		return timestamp;
	}

	public void setTimestamp(Long timestamp) {
		this.timestamp = timestamp;
	}

	public Quaternion getQuaternion() {
		return quaternion;
	}

	public void setQuaternion(Quaternion quaternion) {
		this.quaternion = quaternion;
	}
}
