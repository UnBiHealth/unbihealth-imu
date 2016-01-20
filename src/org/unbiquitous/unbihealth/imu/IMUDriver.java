package org.unbiquitous.unbihealth.imu;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;

import org.apache.commons.math3.complex.Quaternion;
import org.unbiquitous.uos.core.InitialProperties;
import org.unbiquitous.uos.core.UOSLogging;
import org.unbiquitous.uos.core.adaptabitilyEngine.Gateway;
import org.unbiquitous.uos.core.adaptabitilyEngine.NotifyException;
import org.unbiquitous.uos.core.applicationManager.CallContext;
import org.unbiquitous.uos.core.driverManager.UosDriver;
import org.unbiquitous.uos.core.driverManager.UosEventDriver;
import org.unbiquitous.uos.core.messageEngine.dataType.UpDevice;
import org.unbiquitous.uos.core.messageEngine.dataType.UpDriver;
import org.unbiquitous.uos.core.messageEngine.dataType.UpNetworkInterface;
import org.unbiquitous.uos.core.messageEngine.messages.Call;
import org.unbiquitous.uos.core.messageEngine.messages.Notify;
import org.unbiquitous.uos.core.messageEngine.messages.Response;
import org.unbiquitous.uos.core.network.model.NetworkDevice;

import com.fasterxml.jackson.databind.ObjectMapper;

public class IMUDriver implements UosEventDriver {
	public static final String DRIVER_NAME = "org.unbiquitous.ubihealth.IMUDriver";
	public static final String CHANGE_EVENT_NAME = "change";
	public static final String CHANGE_NEW_DATA_PARAM_NAME = "newData";
	public static final String SENSOR_ID_KEY = "imudriver.sensorid";
	public static final int DEFAULT_SENSOR_ID = 0;

	private static final UpDriver _driver = new UpDriver(DRIVER_NAME) {
		{
			addEvent(CHANGE_EVENT_NAME);
		}
	};
	private static Logger logger = UOSLogging.getLogger();
	private static ObjectMapper mapper = new ObjectMapper();

	private Gateway gateway;
	private String instanceId;
	private int sensorId;
	private ConcurrentHashMap<UpNetworkInterface, UpDevice> listeners = new ConcurrentHashMap<UpNetworkInterface, UpDevice>();

	/**
	 * External systems shall call this method to notify the smartspace of
	 * sensor data changes.
	 * 
	 * @param data
	 *            The new sensor data.
	 * 
	 * @throws IOException
	 * @throws NotifyException
	 */
	public void sensorChanged(Quaternion data) throws IOException, NotifyException {
		SensorData newSensorData = new SensorData();
		newSensorData.setId(sensorId);
		newSensorData.setQuaternion(data);
		newSensorData.setTimestamp(System.currentTimeMillis());
		Notify n = new Notify(CHANGE_EVENT_NAME, DRIVER_NAME, instanceId);
		n.addParameter(CHANGE_NEW_DATA_PARAM_NAME, mapper.writeValueAsString(newSensorData));
		doNotify(n);
	}

	private void doNotify(Notify n) throws NotifyException {
		logger.fine(DRIVER_NAME + ": notify -> " + n.toString());
		for (UpDevice device : listeners.values())
			gateway.notify(n, device);
	}

	public int getSensorId() {
		return sensorId;
	}

	public void setSensorId(int sensorId) {
		this.sensorId = sensorId;
	}

	@Override
	public UpDriver getDriver() {
		return _driver;
	}

	@Override
	public List<UpDriver> getParent() {
		return null;
	}

	/**
	 * User UOS init properties field {@link #SENSOR_ID_KEY} to set an integer
	 * value for the sensor. Default value is 0.
	 * 
	 * @see UosDriver#init(Gateway, InitialProperties, String)
	 */
	@Override
	public void init(Gateway gateway, InitialProperties props, String id) {
		logger.info(DRIVER_NAME + ": init instance [" + id + "].");
		this.gateway = gateway;
		this.instanceId = id;
		this.sensorId = props.getInt(SENSOR_ID_KEY, DEFAULT_SENSOR_ID);
	}

	@Override
	public void destroy() {
		listeners.clear();
		logger.info(DRIVER_NAME + ": destroy instance [" + instanceId + "]. Bye!");
	}

	@Override
	public synchronized void registerListener(Call call, Response response, CallContext context) {
		logger.info(DRIVER_NAME + ": registerListener.");
		UpNetworkInterface uni = getNetworkInterface(context);
		if (!listeners.containsKey(uni))
			listeners.put(uni, context.getCallerDevice());
	}

	@Override
	public void unregisterListener(Call call, Response response, CallContext context) {
		logger.info(DRIVER_NAME + ": unregisterListener.");
		listeners.remove(getNetworkInterface(context));
	}

	private static UpNetworkInterface getNetworkInterface(CallContext context) {
		NetworkDevice networkDevice = context.getCallerNetworkDevice();
		String host = networkDevice.getNetworkDeviceName().split(":")[1];
		return new UpNetworkInterface(networkDevice.getNetworkDeviceType(), host);
	}
}
