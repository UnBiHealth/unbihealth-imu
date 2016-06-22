package org.unbiquitous.unbihealth.imu.util;

import java.io.IOException;

import org.apache.commons.math3.complex.Quaternion;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonSerializer;
import com.fasterxml.jackson.databind.SerializerProvider;

/**
 * Serializes a {@link Quaternion}.
 * 
 * @author Luciano Santos
 */
public class QuaternionSerializer extends JsonSerializer<Quaternion> {
	@Override
	public void serialize(Quaternion q, JsonGenerator gen, SerializerProvider provider)
					throws IOException, JsonProcessingException {
		gen.writeStartObject();
		gen.writeObjectField("w", q.getQ0());
		gen.writeObjectField("x", q.getQ1());
		gen.writeObjectField("y", q.getQ2());
		gen.writeObjectField("z", q.getQ3());
		gen.writeEndObject();
	}
}
