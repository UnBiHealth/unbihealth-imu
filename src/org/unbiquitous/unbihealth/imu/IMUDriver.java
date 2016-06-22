package org.unbiquitous.unbihealth.imu;

import com.fasterxml.jackson.databind.JavaType;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.apache.commons.lang3.StringUtils;
import org.apache.commons.math3.complex.Quaternion;
import org.apache.commons.math3.exception.ZeroException;
import org.unbiquitous.unbihealth.imu.record.Recorder;
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
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Level;
import java.util.logging.Logger;

public class IMUDriver implements UosEventDriver {
    public static final String DRIVER_NAME = "org.unbiquitous.ubihealth.IMUDriver";
    public static final String LIST_IDS_NAME = "listIds";
    public static final String IDS_PARAM_NAME = "ids";
    public static final String GET_SENSITIVITY_NAME = "getSensitivity";
    public static final String SENSITIVITY_PARAM_NAME = "sensitivity";
    public static final String TARE_NAME = "tare";
    public static final String CHANGE_EVENT_NAME = "change";
    public static final String CHANGE_NEW_DATA_PARAM_NAME = "newData";
    public static final String START_RECORD_NAME = "startRecording";
    public static final String STOP_RECORD_NAME = "stopRecording";
    public static final String SENSOR_ID_PARAM_NAME = "sensorId";
    public static final String STEP_TIME_PARAM_NAME = "stepTime";
    public static final String INTERPOLATE_PARAM_NAME = "interpolate";
    public static final int DEFAULT_STEP_TIME = 16;
    public static final String RECORD_ID_PARAM_NAME = "recordId";
    public static final String RECORD_DATA_PARAM_NAME = "recordData";
    public static final String DEFAULT_SENSOR_ID_KEY = "imudriver.defaultsensorid";
    public static final String DEFAULT_SENSOR_ID = "0";
    public static final String VALID_IDS_KEY = "imudriver.validids";
    public static final String SENSITIVITY_KEY = "imudriver.sensitivity";
    public static final double DEFAULT_SENSITIVITY = 0.0;
    public static final String MIN_UPDATE_INTERVAL_KEY = "imudriver.step";
    public static final int DEFAULT_MIN_UPDATE_INTERVAL = 10;

    private static final UpDriver _driver = new UpDriver(DRIVER_NAME) {
        {
            addService(LIST_IDS_NAME);
            addService(GET_SENSITIVITY_NAME);
            addService(TARE_NAME);
            addEvent(CHANGE_EVENT_NAME)
                    .addParameter(CHANGE_NEW_DATA_PARAM_NAME, UpService.ParameterType.MANDATORY);
            addService(START_RECORD_NAME)
                    .addParameter(SENSOR_ID_PARAM_NAME, UpService.ParameterType.MANDATORY)
                    .addParameter(STEP_TIME_PARAM_NAME, UpService.ParameterType.OPTIONAL);
            addService(STOP_RECORD_NAME)
                    .addParameter(SENSOR_ID_PARAM_NAME, UpService.ParameterType.MANDATORY)
                    .addParameter(RECORD_ID_PARAM_NAME, UpService.ParameterType.MANDATORY);
        }
    };
    private static final Logger logger = UOSLogging.getLogger();
    private static final ObjectMapper mapper = new ObjectMapper();
    private static final JavaType ID_LIST_TYPE = mapper.getTypeFactory().constructParametrizedType(List.class, List.class, String.class);
    private static final JavaType SAMPLE_LIST_TYPE = mapper.getTypeFactory().constructParametrizedType(List.class, List.class, Sample.class);

    private Gateway gateway;
    private String instanceId;
    private String defaultSensorId;
    private Set<String> validIds = new HashSet<>();
    private ConcurrentHashMap<UpNetworkInterface, UpDevice> listeners = new ConcurrentHashMap<UpNetworkInterface, UpDevice>();
    private Quaternion refData = Quaternion.ZERO;
    private Quaternion lastData = Quaternion.ZERO;
    private double sensitivity;
    private long minUpdateInterval, lastUpdate = 0;
    private Map<String, List<Sample>> lastRecordedData = new HashMap<>();
    private Map<String, Recorder> recorders = new HashMap<>();

    /**
     * External systems shall call this method to notify the smartspace of
     * sensor data changes and use default sensor id and system's current timestamp.
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
     * @param newData  The new sensor data.
     * @param sensorId The sensor id or null to use default.
     * @throws IOException
     * @throws NotifyException
     */
    public void sensorChanged(Quaternion newData, String sensorId) throws IOException {
        if (newData == null)
            throw new NullPointerException("newValue");

        if (sensorId == null)
            sensorId = defaultSensorId;
        else if (!validIds.contains(sensorId))
            throw new IllegalArgumentException("Invalid sensor id.");

        // Checks update frequency...
        long cur = System.currentTimeMillis();
        if ((cur - lastUpdate) < minUpdateInterval)
            return;
        lastUpdate = cur;

        Quaternion normData;
        try {
            // Corrects for the axis, based on the calibration.
            normData = new Quaternion(
                    newData.getQ0(),
                    newData.getQ1() - refData.getQ1(),
                    newData.getQ2() - refData.getQ2(),
                    newData.getQ3() - refData.getQ3()
            ).normalize();
        } catch (ZeroException e) {
            normData = Quaternion.IDENTITY;
        }

        // If it's currently recording this sensor, notifies the recorder.
        Recorder recorder = recorders.get(sensorId);
        if (recorder != null)
            recorder.add(cur, normData);

        // Verifies sensitivity and notifies listeners if necessary.
        if (maxOffset(newData, lastData) >= sensitivity) {
            lastData = newData;
            doNotify(sensorId, normData, cur);
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

    private void doNotify(String sensorId, Quaternion newValue, long timestamp) {
        SensorData newSensorData = new SensorData();
        newSensorData.setId(sensorId);
        newSensorData.setQuaternion(newValue);
        newSensorData.setTimestamp(timestamp);
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
     * Given a {@link Response}, verifies if it contains the param for the list of valid ids; if so, extracts and
     * returns the list.
     *
     * @param resp The response to be processed.
     * @return The extracted list of ids.
     * @throws IllegalArgumentException If any data is missing or in an invalid format.
     * @throws IOException              If there's any JSON parsing/conversion error.
     */
    public static List<String> extractIdList(Response resp) throws IOException {
        Object param = resp.getResponseData(IDS_PARAM_NAME);
        if (param == null)
            throw new IllegalArgumentException("Id list not present.");
        return param instanceof String ?
                mapper.readValue((String) param, ID_LIST_TYPE) :
                mapper.convertValue(param, ID_LIST_TYPE);
    }

    /**
     * Given a {@link Notify}, verifies if it's from IMUDriver and contains valid {@link SensorData}; if so,
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
                mapper.readValue((String) param, SensorData.class) :
                mapper.convertValue(param, SensorData.class);
        if (StringUtils.isBlank(data.getId()))
            throw new IllegalArgumentException("sensor id must not be empty or null");
        if (data.getQuaternion() == null)
            throw new IllegalArgumentException("quaternion value must not be null");
        return data;
    }

    /**
     * Given a {@link Response}, verifies if it contains the param for the sample list and tries to retrieve it.
     *
     * @param resp The response to be processed.
     * @return The extracted data as a list of samples.
     * @throws IllegalArgumentException If the data is not in a valid format.
     * @throws IOException              If there's any JSON parsing/conversion error.
     */
    public static List<Sample> extractRecordedData(Response resp) throws IOException {
        Object param = resp.getResponseData(RECORD_DATA_PARAM_NAME);
        if (param == null)
            throw new IllegalArgumentException("sample list not present");
        return param instanceof String ?
                extractRecordedData((String) param) :
                mapper.convertValue(param, SAMPLE_LIST_TYPE);
    }

    /**
     * Given a list of samples serialized as a JSON string, tries to load it.
     *
     * @param json The serialized data to be parsed.
     * @return The extracted data as a list of samples.
     * @throws IllegalArgumentException If the data is not in a valid format.
     * @throws IOException              If there's any JSON parsing/conversion error.
     */
    public static List<Sample> extractRecordedData(String json) throws IOException {
        return mapper.readValue(json, SAMPLE_LIST_TYPE);
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

        sensitivity = props.getDouble(SENSITIVITY_KEY, DEFAULT_SENSITIVITY);
        if (sensitivity < 0) {
            logger.warning(DRIVER_NAME + "[" + id + "]: " + "invalid sensitivity provided, using default.");
            sensitivity = DEFAULT_SENSITIVITY;
        }

        minUpdateInterval = props.getInt(MIN_UPDATE_INTERVAL_KEY, DEFAULT_MIN_UPDATE_INTERVAL);
        if (minUpdateInterval < 0) {
            logger.warning(DRIVER_NAME + "[" + id + "]: " + "invalid min update interval provided, using default.");
            minUpdateInterval = DEFAULT_MIN_UPDATE_INTERVAL;
        }

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

    public void tare(Call call, Response response, CallContext context) {
        refData = lastData;
    }

    public void listIds(Call call, Response response, CallContext context) {
        response.addParameter(IDS_PARAM_NAME, validIds.toArray(new String[0]));
    }

    private String extractSensorId(Call call) {
        Object param = call.getParameter(SENSOR_ID_PARAM_NAME);
        if (param == null)
            throw new IllegalArgumentException("no sensor id provided");
        String sensorId = param.toString();
        if (!validIds.contains(sensorId))
            throw new IllegalArgumentException("invalid or unknown sensor id");
        return sensorId;
    }

    private String extractRecordId(Call call) {
        Object param = call.getParameter(RECORD_ID_PARAM_NAME);
        if (param == null)
            throw new IllegalArgumentException("no record id provided");
        return param.toString();
    }

    public void startRecording(Call call, Response response, CallContext context) {
        String sensorId;
        int stepTime = DEFAULT_STEP_TIME;
        boolean interpolate = false;
        try {
            // Validates sensor id.
            sensorId = extractSensorId(call);
            if (recorders.containsKey(sensorId))
                throw new IllegalArgumentException("already recording this sensor id");

            // Validates step time.
            Object param = call.getParameter(STEP_TIME_PARAM_NAME);
            if (param != null) {
                try {
                    stepTime = mapper.convertValue(param, Integer.class);
                } catch (Exception e) {
                    throw new IllegalArgumentException("invalid time step");
                }
                if (stepTime <= 0)
                    throw new IllegalArgumentException("non-positive step time");
            }
        } catch (IllegalArgumentException e) {
            response.setError(e.getMessage());
            return;
        }

        // Prepares data structures and generates an id.
        String id = UUID.randomUUID().toString();
        synchronized (recorders) {
            recorders.put(sensorId, new Recorder(id, stepTime, interpolate));
        }
        response.addParameter(RECORD_ID_PARAM_NAME, id);
    }

    public void stopRecording(Call call, Response response, CallContext context) {
        String sensorId, recordId;
        Recorder recorder;
        try {
            recordId = extractRecordId(call);
            sensorId = extractSensorId(call);
            recorder = recorders.get(sensorId);
            if (recorder == null)
                throw new IllegalArgumentException("not currently recording this sensor id");
            if (!recorder.getId().equals(recordId))
                throw new IllegalArgumentException("invalid or unknown record id");
        } catch (IllegalArgumentException e) {
            response.setError(e.getMessage());
            return;
        }

        List<Sample> data = recorder.getData();
        synchronized (recorders) {
            lastRecordedData.put(sensorId, data);
            recorders.remove(sensorId);
        }
        response.addParameter(RECORD_DATA_PARAM_NAME, data);
    }
}
