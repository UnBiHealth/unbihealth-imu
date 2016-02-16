package org.unbiquitous.unbihealth.imu;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.apache.commons.lang3.StringUtils;
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
import org.unbiquitous.uos.core.messageEngine.dataType.UpService;
import org.unbiquitous.uos.core.messageEngine.messages.Call;
import org.unbiquitous.uos.core.messageEngine.messages.Notify;
import org.unbiquitous.uos.core.messageEngine.messages.Response;
import org.unbiquitous.uos.core.network.model.NetworkDevice;

import java.io.IOException;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Level;
import java.util.logging.Logger;

public class IMUDriver implements UosEventDriver {
    public static final String DRIVER_NAME = "org.unbiquitous.ubihealth.IMUDriver";
    public static final String CHANGE_EVENT_NAME = "change";
    public static final String CHANGE_NEW_DATA_PARAM_NAME = "newData";
    public static final String LIST_IDS_NAME = "listIds";
    public static final String IDS_PARAM_NAME = "ids";
    public static final String GET_SENSITIVITY_NAME = "getSensitivity";
    public static final String SENSITIVITY_PARAM_NAME = "sensitivity";
    public static final String DEFAULT_SENSOR_ID_KEY = "imudriver.defaultsensorid";
    public static final String DEFAULT_SENSOR_ID = "0";
    public static final String VALID_IDS_KEY = "imudriver.validids";
    public static final String SENSITIVITY_KEY = "imudriver.sensitivity";
    public static final double DEFAULT_SENSITIVITY = 0.0;

    private static final UpDriver _driver = new UpDriver(DRIVER_NAME) {
        {
            addService(LIST_IDS_NAME);
            addService(GET_SENSITIVITY_NAME);
            addEvent(CHANGE_EVENT_NAME)
                    .addParameter(CHANGE_NEW_DATA_PARAM_NAME, UpService.ParameterType.MANDATORY);
        }
    };
    private static Logger logger = UOSLogging.getLogger();
    private static ObjectMapper mapper = new ObjectMapper();

    private Gateway gateway;
    private String instanceId;
    private String defaultSensorId;
    private Set<String> validIds = new HashSet<>();
    private ConcurrentHashMap<UpNetworkInterface, UpDevice> listeners = new ConcurrentHashMap<UpNetworkInterface, UpDevice>();
    private Quaternion lastData = Quaternion.ZERO;
    private double sensitivity;

    /**
     * External systems shall call this method to notify the smartspace of
     * sensor data changes and use default sensor id and system's current timestamp.
     *
     * @param values The new sensor data.
     * @throws IOException
     * @throws NotifyException
     */
    public void sensorChanged(Quaternion values) throws IOException, NotifyException {
        sensorChanged(values, null, null);
    }

    /**
     * External systems shall call this method to notify the smartspace of
     * sensor data changes and use system's current timestamp.
     *
     * @param values   The new sensor data.
     * @param sensorId The sensor id.
     * @throws IOException
     * @throws NotifyException
     */
    public void sensorChanged(Quaternion values, String sensorId) throws IOException, NotifyException {
        sensorChanged(values, sensorId, null);
    }

    /**
     * External systems shall call this method to notify the smartspace of
     * sensor data changes and use default sensor id.
     *
     * @param values    The new sensor data.
     * @param timestamp The change event timestamp.
     * @throws IOException
     * @throws NotifyException
     */
    public void sensorChanged(Quaternion values, Long timestamp) throws IOException, NotifyException {
        sensorChanged(values, null, timestamp);
    }

    /**
     * External systems shall call this method to notify the smartspace of
     * sensor data changes.
     *
     * @param newValue  The new sensor data.
     * @param sensorId  The sensor id or null to use default.
     * @param timestamp The change event timestamp, or null, to use system timestamp.
     * @throws IOException
     * @throws NotifyException
     */
    public void sensorChanged(Quaternion newValue, String sensorId, Long timestamp) throws IOException {
        if (newValue == null)
            throw new NullPointerException("newValue");

        if (sensorId == null)
            sensorId = defaultSensorId;
        else if (!validIds.contains(sensorId))
            throw new IllegalArgumentException("Invalid sensor id.");

        if (maxOffset(newValue, lastData) >= sensitivity) {
            lastData = newValue;
            doNotify(sensorId, newValue, timestamp);
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

    private void doNotify(String sensorId, Quaternion newValue, Long timestamp) {
        SensorData newSensorData = new SensorData();
        newSensorData.setId(sensorId);
        newSensorData.setQuaternion(newValue);
        newSensorData.setTimestamp(timestamp == null ? System.currentTimeMillis() : timestamp);
        Notify n = new Notify(CHANGE_EVENT_NAME, DRIVER_NAME, instanceId);
        n.addParameter(CHANGE_NEW_DATA_PARAM_NAME, newSensorData);

        notify(n, null);
        for (UpDevice device : listeners.values())
            notify(n, device);
    }

    private void notify(Notify n, UpDevice device) {
        try {
            gateway.notify(n, device);
        } catch (NotifyException e) {
            logger.log(Level.SEVERE, "Failed to notify listener '" + device.getName() + "'.", e);
        }
    }

    /**
     * Given a {@link Notify}, verifies if it's a valid IMUDriver, with valid {@link SensorData}; if so,
     * extracts and returns the data.
     *
     * @param n The notify to be parsed.
     * @return The extracted sensor data.
     * @throws IllegalArgumentException If this is not a valid IMUDriver notify or any data is missing.
     * @throws IOException              If there's any JSON parsing/conversion error.
     */
    public static SensorData extractSensorData(Notify n) throws IOException {
        if (!(DRIVER_NAME.equals(n.getDriver()) && CHANGE_EVENT_NAME.equals(n.getEventKey())))
            throw new IllegalArgumentException("This is not an IMUDriver notify.");
        Object param = n.getParameter(CHANGE_NEW_DATA_PARAM_NAME);
        if (param == null)
            throw new IllegalArgumentException("Event data not present.");
        SensorData data = (param instanceof String) ?
                mapper.readValue((String) param, SensorData.class) : mapper.convertValue(param, SensorData.class);
        if (StringUtils.isEmpty(data.getId()))
            throw new IllegalArgumentException("sensor id must not be empty or null");
        if (data.getTimestamp() == null)
            throw new IllegalArgumentException("timestamp must not be null");
        if (data.getQuaternion() == null)
            throw new IllegalArgumentException("quaternion value must not be null");
        return data;
    }

    public String getInstanceId() {
        return instanceId;
    }

    public String getDefaultSensorId() {
        return defaultSensorId;
    }

    public void setDefaultSensorId(String defaultSensorId) {
        this.defaultSensorId = defaultSensorId;
    }

    public static UpDriver getDriverStatic() {
        return _driver;
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
     * User UOS init properties field {@link #DEFAULT_SENSOR_ID_KEY} to set an integer
     * value for the sensor. Default value is 0.
     *
     * @see UosDriver#init(Gateway, InitialProperties, String)
     */
    @Override
    public void init(Gateway gateway, InitialProperties props, String id) {
        this.gateway = gateway;
        this.instanceId = id;

        defaultSensorId = props.getString(DEFAULT_SENSOR_ID_KEY, DEFAULT_SENSOR_ID).trim();
        if (defaultSensorId.isEmpty())
            defaultSensorId = DEFAULT_SENSOR_ID;

        validIds.add(defaultSensorId);
        StringBuilder idlist = new StringBuilder(defaultSensorId);
        String[] ids = props.getString(VALID_IDS_KEY, "").split(",");
        for (String validId : ids) {
            validId = validId.trim();
            if (!(validId.isEmpty() || validIds.contains(validId))) {
                validIds.add(validId);
                idlist.append(",");
                idlist.append(validId);
            }
        }

        this.sensitivity = props.getDouble(SENSITIVITY_KEY, DEFAULT_SENSITIVITY);

        logger.info(DRIVER_NAME + "[" + id + "]: " + "default sensor id - " + defaultSensorId + ".");
        logger.info(DRIVER_NAME + "[" + id + "]: " + "valid ids - " + idlist + ".");
        logger.info(DRIVER_NAME + "[" + id + "]: " + "sensitivity to changes >= " + sensitivity + ".");
    }

    @Override
    public void destroy() {
        listeners.clear();
        logger.info(DRIVER_NAME + ": destroy instance [" + instanceId + "]. Bye!");
    }

    @Override
    public synchronized void registerListener(Call call, Response response, CallContext context) {
        logger.fine(DRIVER_NAME + ": registerListener.");
        UpNetworkInterface uni = getNetworkInterface(context);
        if (!listeners.containsKey(uni)) {
            UpDevice device = context.getCallerDevice();
            listeners.put(uni, device);
            logger.info(DRIVER_NAME + ": registered listener from device '" + device.getName() + "'.");
        }
    }

    @Override
    public synchronized void unregisterListener(Call call, Response response, CallContext context) {
        logger.fine(DRIVER_NAME + ": unregisterListener.");
        UpDevice device = listeners.remove(getNetworkInterface(context));
        if (device != null)
            logger.info(DRIVER_NAME + ": unregistered listener from device '" + device.getName() + "'.");
        else
            logger.info(DRIVER_NAME + ": there was no listener registered for device '" + device.getName() + "'.");
    }

    private static UpNetworkInterface getNetworkInterface(CallContext context) {
        NetworkDevice networkDevice = context.getCallerNetworkDevice();
        String host = networkDevice.getNetworkDeviceName().split(":")[1];
        return new UpNetworkInterface(networkDevice.getNetworkDeviceType(), host);
    }

    public void getSensitivity(Call call, Response response, CallContext context) {
        response.addParameter(SENSITIVITY_PARAM_NAME, sensitivity);
    }

    public void listIds(Call call, Response response, CallContext context) {
        response.addParameter(IDS_PARAM_NAME, validIds.toArray(new String[0]));
    }
}
