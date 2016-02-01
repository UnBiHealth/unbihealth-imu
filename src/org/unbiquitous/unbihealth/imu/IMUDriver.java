package org.unbiquitous.unbihealth.imu;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.apache.commons.math3.complex.Quaternion;
import org.unbiquitous.uos.core.InitialProperties;
import org.unbiquitous.uos.core.UOSLogging;
import org.unbiquitous.uos.core.adaptabitilyEngine.Gateway;
import org.unbiquitous.uos.core.adaptabitilyEngine.NotifyException;
import org.unbiquitous.uos.core.adaptabitilyEngine.SmartSpaceGateway;
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

import java.io.IOException;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;

public class IMUDriver implements UosEventDriver {
    public static final String DRIVER_NAME = "org.unbiquitous.ubihealth.IMUDriver";
    public static final String CHANGE_EVENT_NAME = "change";
    public static final String CHANGE_NEW_DATA_PARAM_NAME = "newData";
    public static final String SENSOR_ID_KEY = "imudriver.sensorid";
    public static final String DEFAULT_SENSOR_ID = "0";
    public static final String SENSITIVITY_KEY = "imudriver.sensitivity";
    public static final double DEFAULT_SENSITIVITY = 0.0;

    private static final UpDriver _driver = new UpDriver(DRIVER_NAME) {
        {
            addEvent(CHANGE_EVENT_NAME);
        }
    };
    private static Logger logger = UOSLogging.getLogger();
    private static ObjectMapper mapper = new ObjectMapper();

    private Gateway gateway;
    private String instanceId;
    private String sensorId;
    private ConcurrentHashMap<UpNetworkInterface, UpDevice> listeners = new ConcurrentHashMap<UpNetworkInterface, UpDevice>();
    private Quaternion lastData = Quaternion.ZERO;
    private double sensitivity;

    /**
     * External systems shall call this method to notify the smartspace of
     * sensor data changes and use system timestamp.
     *
     * @param values The new sensor data.
     * @throws IOException
     * @throws NotifyException
     */
    public void sensorChanged(Quaternion values) throws IOException, NotifyException {
        sensorChanged(values, null);
    }

    /**
     * External systems shall call this method to notify the smartspace of
     * sensor data changes.
     *
     * @param values    The new sensor data.
     * @param timestamp The change event timestamp, or null, to use system timestamp.
     * @throws IOException
     * @throws NotifyException
     */
    public void sensorChanged(Quaternion values, Long timestamp) throws IOException, NotifyException {
        if (maxOffset(values, lastData) >= sensitivity) {
            lastData = new Quaternion(values.getQ0(), values.getQ1(), values.getQ2(), values.getQ3());
            SensorData newSensorData = new SensorData();
            newSensorData.setId(sensorId);
            newSensorData.setQuaternion(values);
            newSensorData.setTimestamp(timestamp == null ? System.currentTimeMillis() : timestamp);
            Notify n = new Notify(CHANGE_EVENT_NAME, DRIVER_NAME, instanceId);
            n.addParameter(CHANGE_NEW_DATA_PARAM_NAME, newSensorData);
            doNotify(n);
        }
    }

    private static double maxOffset(Quaternion a, Quaternion b) {
        Quaternion aux = a.subtract(b);
        double max = Math.abs(aux.getQ0());
        max = Math.max(Math.abs(aux.getQ1()), max);
        max = Math.max(Math.abs(aux.getQ2()), max);
        max = Math.max(Math.abs(aux.getQ3()), max);
        return max;
    }

    private void doNotify(Notify n) throws NotifyException {
        logger.fine(DRIVER_NAME + ": notify -> " + n.toString());
        for (UpDevice device : listeners.values())
            gateway.notify(n, device);
    }

    public String getInstanceId() {
        return instanceId;
    }

    public String getSensorId() {
        return sensorId;
    }

    public void setSensorId(String sensorId) {
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
        this.gateway = gateway;
        this.instanceId = id;
        this.sensorId = props.getString(SENSOR_ID_KEY, DEFAULT_SENSOR_ID);
        this.sensitivity = props.getDouble(SENSITIVITY_KEY, DEFAULT_SENSITIVITY);
        logger.info(DRIVER_NAME + ": init instance [" + id + "] with sensor id [" + sensorId + "], sensitivity for changes >= " + sensitivity);
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
