package org.unbiquitous.unbihealth.imu;

import java.io.IOException;

import org.apache.commons.math3.complex.Quaternion;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.JsonDeserializer;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

/**
 * Deserializes a {@link Quaternion}.
 * 
 * @author Luciano Santos
 */
public class QuaternionDeserializer extends JsonDeserializer<Quaternion> {

	@Override
	public Quaternion deserialize(JsonParser parser, DeserializationContext context)
			throws IOException, JsonProcessingException {

		final String[] components = { "x", "y", "z", "w" };

		ObjectNode node = parser.readValueAsTree();
		double[] data = new double[4];
		for (int i = 0; i < data.length; ++i) {
			JsonNode value = node.get(components[i]);
			if ((value != null) && value.isFloatingPointNumber())
				data[i] = value.asDouble();
			else
				throw context.mappingException("Expected field component '" + components[i] + "'.");
		}

		return new Quaternion(data[3], data[0], data[1], data[2]);
	}
}
