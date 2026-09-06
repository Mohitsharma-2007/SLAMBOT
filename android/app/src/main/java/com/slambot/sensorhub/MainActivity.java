package com.slambot.sensorhub;

import android.Manifest;
import android.content.Context;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.media.Image;
import android.media.ImageReader;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Base64;
import android.util.Log;
import android.view.View;
import android.view.WindowInsets;
import android.view.WindowInsetsController;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TextView;
import androidx.annotation.NonNull;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.nio.ByteBuffer;
import java.util.Collections;
import java.util.concurrent.TimeUnit;

import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.Response;
import okhttp3.WebSocket;
import okhttp3.WebSocketListener;

public class MainActivity extends AppCompatActivity implements SensorEventListener, LocationListener {
    private static final String TAG = "SLAMBotHub";
    private static final int PERMISSION_REQUEST_CODE = 100;
    private static final String PREFS_NAME = "SLAMBotPrefs";
    private static final String KEY_BACKEND_IP = "backend_ip";
    
    // Server connection settings
    private String mBackendIp = "10.180.122.189";
    private String mBackendPort = "8000";

    // Android Sensors
    private SensorManager mSensorManager;
    private Sensor mAccelerometer;
    private Sensor mGyroscope;
    private Sensor mRotationVector;
    private LocationManager mLocationManager;

    private float[] mGravity = new float[3];
    private float[] mGeomagnetic = new float[3];
    private float[] mRotationMatrix = new float[9];
    private float[] mOrientationAngles = new float[3];

    // Camera variables
    private CameraDevice mCameraDevice;
    private ImageReader mImageReader;
    private HandlerThread mBackgroundThread;
    private Handler mBackgroundHandler;

    // WebSockets & Views
    private OkHttpClient mClient;
    private WebSocket mWebSocket;
    private EyesView mEyesView;
    private LiveMapView mLiveMapView;
    private TextView mTextConnection;
    private TextView mTextSensors;

    private Button mBtnRecenter;
    private Button mBtnFullscreen;
    private Button mBtnAutoExplore;
    private Button mBtnEstop;
    private Button mBtnConfigIp;
    private boolean mIsFullscreen = false;
    private boolean mIsDestroyed = false;

    private final Handler mMainHandler = new Handler();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Load saved server IP
        SharedPreferences prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        mBackendIp = prefs.getString(KEY_BACKEND_IP, "10.180.122.189");

        mEyesView = findViewById(R.id.eyesView);
        mLiveMapView = findViewById(R.id.liveMapView);
        mTextConnection = findViewById(R.id.textConnection);
        mTextSensors = findViewById(R.id.textSensors);

        mBtnRecenter = findViewById(R.id.btnRecenter);
        mBtnFullscreen = findViewById(R.id.btnFullscreen);
        mBtnAutoExplore = findViewById(R.id.btnAutoExplore);
        mBtnEstop = findViewById(R.id.btnEstop);
        mBtnConfigIp = findViewById(R.id.btnConfigIp);

        if (mBtnConfigIp != null) {
            mBtnConfigIp.setOnClickListener(v -> showIpConfigDialog());
        }
        if (mTextConnection != null) {
            mTextConnection.setOnClickListener(v -> showIpConfigDialog());
        }

        mBtnRecenter.setOnClickListener(v -> {
            if (mLiveMapView != null) mLiveMapView.centerOnRobot();
        });

        mBtnFullscreen.setOnClickListener(v -> toggleFullscreen());

        mBtnAutoExplore.setOnClickListener(v -> {
            if (mWebSocket != null) {
                try {
                    JSONObject cmd = new JSONObject();
                    cmd.put("type", "command");
                    cmd.put("action", "auto_explore");
                    mWebSocket.send(cmd.toString());
                    mTextConnection.setText("EXPLORING: AUTO FRONTIER");
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });

        mBtnEstop.setOnClickListener(v -> {
            if (mWebSocket != null) {
                try {
                    JSONObject cmd = new JSONObject();
                    cmd.put("type", "command");
                    cmd.put("action", "estop");
                    mWebSocket.send(cmd.toString());
                    mTextConnection.setText("EMERGENCY STOP TRIGGERED");
                    if (mEyesView != null) mEyesView.setRobotState(EyesView.STATE_COLLISION);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });

        // Initialize Sensors
        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        if (mSensorManager != null) {
            mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            mGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
            mRotationVector = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        }

        mLocationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);

        // Check Permissions
        checkAndRequestPermissions();

        // Connect WebSocket
        initWebSocket();

        // Start UDP Auto Discovery Listener
        startUdpDiscovery();
    }

    private void showIpConfigDialog() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle("⚙ Configure Desktop Server IP");

        LinearLayout layout = new LinearLayout(this);
        layout.setOrientation(LinearLayout.VERTICAL);
        layout.setPadding(50, 40, 50, 10);

        final EditText input = new EditText(this);
        input.setHint("e.g. 10.180.122.189 or 192.168.1.x");
        input.setText(mBackendIp);
        input.setTextColor(Color.WHITE);
        input.setHintTextColor(Color.GRAY);
        layout.addView(input);

        TextView tip = new TextView(this);
        tip.setText("Tip: Check the IP shown on your Desktop Command Center screen.");
        tip.setTextColor(Color.LTGRAY);
        tip.setTextSize(12);
        tip.setPadding(0, 20, 0, 0);
        layout.addView(tip);

        builder.setView(layout);

        builder.setPositiveButton("Connect", (dialog, which) -> {
            String newIp = input.getText().toString().trim();
            if (!newIp.isEmpty()) {
                mBackendIp = newIp;
                SharedPreferences.Editor editor = getSharedPreferences(PREFS_NAME, MODE_PRIVATE).edit();
                editor.putString(KEY_BACKEND_IP, mBackendIp);
                editor.apply();

                mTextConnection.setText("CONNECTING TO " + mBackendIp + "...");
                initWebSocket();
            }
        });
        builder.setNegativeButton("Cancel", (dialog, which) -> dialog.cancel());
        builder.show();
    }

    private void startUdpDiscovery() {
        new Thread(() -> {
            DatagramSocket socket = null;
            try {
                socket = new DatagramSocket(8888);
                socket.setBroadcast(true);
                byte[] buf = new byte[1024];

                while (!mIsDestroyed) {
                    DatagramPacket packet = new DatagramPacket(buf, buf.length);
                    socket.receive(packet);
                    String data = new String(packet.getData(), 0, packet.getLength());
                    if (data.contains("slambot_server")) {
                        String hostIp = packet.getAddress().getHostAddress();
                        if (hostIp != null && !hostIp.equals(mBackendIp)) {
                            Log.i(TAG, "Discovered SLAM Bot Desktop Server at: " + hostIp);
                            mMainHandler.post(() -> {
                                mBackendIp = hostIp;
                                SharedPreferences.Editor editor = getSharedPreferences(PREFS_NAME, MODE_PRIVATE).edit();
                                editor.putString(KEY_BACKEND_IP, mBackendIp);
                                editor.apply();
                                initWebSocket();
                            });
                        }
                    }
                }
            } catch (Exception e) {
                // Ignore UDP timeout
            } finally {
                if (socket != null && !socket.isClosed()) socket.close();
            }
        }).start();
    }

    private void toggleFullscreen() {
        mIsFullscreen = !mIsFullscreen;
        if (mIsFullscreen) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                WindowInsetsController controller = getWindow().getInsetsController();
                if (controller != null) {
                    controller.hide(WindowInsets.Type.statusBars() | WindowInsets.Type.navigationBars());
                    controller.setSystemBarsBehavior(WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE);
                }
            } else {
                getWindow().getDecorView().setSystemUiVisibility(
                        View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                                | View.SYSTEM_UI_FLAG_FULLSCREEN
                                | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                                | View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                                | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                                | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN);
            }
            mBtnFullscreen.setText("Exit Full");
        } else {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                WindowInsetsController controller = getWindow().getInsetsController();
                if (controller != null) {
                    controller.show(WindowInsets.Type.statusBars() | WindowInsets.Type.navigationBars());
                }
            } else {
                getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_VISIBLE);
            }
            mBtnFullscreen.setText("⛶ Fullscreen");
        }
    }

    private void checkAndRequestPermissions() {
        String[] permissions = {
                Manifest.permission.CAMERA,
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.RECORD_AUDIO
        };

        boolean allGranted = true;
        for (String perm : permissions) {
            if (ActivityCompat.checkSelfPermission(this, perm) != PackageManager.PERMISSION_GRANTED) {
                allGranted = false;
                break;
            }
        }

        if (!allGranted) {
            ActivityCompat.requestPermissions(this, permissions, PERMISSION_REQUEST_CODE);
        } else {
            startCameraAndSensors();
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == PERMISSION_REQUEST_CODE) {
            startCameraAndSensors();
        }
    }

    private void startCameraAndSensors() {
        if (mSensorManager != null) {
            if (mAccelerometer != null) mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_GAME);
            if (mGyroscope != null) mSensorManager.registerListener(this, mGyroscope, SensorManager.SENSOR_DELAY_GAME);
            if (mRotationVector != null) mSensorManager.registerListener(this, mRotationVector, SensorManager.SENSOR_DELAY_GAME);
        }

        if (mLocationManager != null && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
            mLocationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1000, 1.0f, this);
        }

        startBackgroundThread();
        openCamera();
    }

    private synchronized void initWebSocket() {
        if (mWebSocket != null) {
            try { mWebSocket.close(1000, "Reconnecting"); } catch (Exception e) {}
            mWebSocket = null;
        }

        if (mClient == null) {
            mClient = new OkHttpClient.Builder()
                    .readTimeout(0, TimeUnit.MILLISECONDS)
                    .connectTimeout(5, TimeUnit.SECONDS)
                    .build();
        }

        String wsUrl = "ws://" + mBackendIp + ":" + mBackendPort + "/ws/phone";
        mTextConnection.setText("CONNECTING: " + mBackendIp);
        Request request = new Request.Builder().url(wsUrl).build();

        mWebSocket = mClient.newWebSocket(request, new WebSocketListener() {
            @Override
            public void onOpen(WebSocket webSocket, Response response) {
                mMainHandler.post(() -> {
                    mTextConnection.setText("CONNECTED: " + mBackendIp);
                    if (mEyesView != null) mEyesView.setRobotState(EyesView.STATE_IDLE);
                });
            }

            @Override
            public void onMessage(WebSocket webSocket, String text) {
                try {
                    JSONObject obj = new JSONObject(text);
                    String type = obj.optString("type", "");

                    // 1. Robot Expression
                    if (type.equals("expression") || type.equals("robot_state")) {
                        String state = obj.optString("state", "idle").toLowerCase();
                        mMainHandler.post(() -> {
                            if (mEyesView != null) {
                                if (state.contains("move") || state.contains("nav")) mEyesView.setRobotState(EyesView.STATE_MOVING);
                                else if (state.contains("warn")) mEyesView.setRobotState(EyesView.STATE_WARNING);
                                else if (state.contains("err") || state.contains("collis") || state.contains("stop")) mEyesView.setRobotState(EyesView.STATE_COLLISION);
                                else mEyesView.setRobotState(EyesView.STATE_IDLE);
                            }
                        });
                    }
                    // 2. Real-Time SLAM Map Stream
                    else if (type.equals("map")) {
                        mMainHandler.post(() -> {
                            if (mLiveMapView != null) {
                                mLiveMapView.updateMap(obj);
                            }
                        });
                    }
                    // 3. Robot Pose $(X, Y, \theta)$
                    else if (type.equals("pose") || type.equals("odom")) {
                        float px = (float) obj.optDouble("x", 0.0);
                        float py = (float) obj.optDouble("y", 0.0);
                        float pth = (float) obj.optDouble("theta", 0.0);
                        mMainHandler.post(() -> {
                            if (mLiveMapView != null) {
                                mLiveMapView.updateRobotPose(px, py, pth);
                            }
                        });
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            @Override
            public void onFailure(WebSocket webSocket, Throwable t, Response response) {
                mMainHandler.post(() -> {
                    mTextConnection.setText("OFFLINE (" + mBackendIp + ")");
                    if (mEyesView != null) mEyesView.setRobotState(EyesView.STATE_WARNING);
                    mMainHandler.postDelayed(() -> {
                        if (!mIsDestroyed) initWebSocket();
                    }, 4000);
                });
            }
        });
    }

    // --- Camera Streaming --------------------------------------------------
    private void startBackgroundThread() {
        mBackgroundThread = new HandlerThread("CameraBackground");
        mBackgroundThread.start();
        mBackgroundHandler = new Handler(mBackgroundThread.getLooper());
    }

    private void openCamera() {
        CameraManager manager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
        try {
            String[] cameraList = manager.getCameraIdList();
            if (cameraList.length == 0) return;
            String cameraId = cameraList[0];
            
            mImageReader = ImageReader.newInstance(320, 240, ImageFormat.YUV_420_888, 2);
            mImageReader.setOnImageAvailableListener(reader -> {
                Image image = null;
                try {
                    image = reader.acquireLatestImage();
                    if (image != null && mWebSocket != null) {
                        byte[] jpegBytes = yuv420ToJpeg(image);
                        if (jpegBytes != null) {
                            String base64Img = Base64.encodeToString(jpegBytes, Base64.NO_WRAP);
                            JSONObject packet = new JSONObject();
                            packet.put("type", "camera");
                            packet.put("format", "jpeg");
                            packet.put("image_base64", base64Img);
                            mWebSocket.send(packet.toString());
                        }
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                } finally {
                    if (image != null) image.close();
                }
            }, mBackgroundHandler);

            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
                manager.openCamera(cameraId, new CameraDevice.StateCallback() {
                    @Override
                    public void onOpened(@NonNull CameraDevice camera) {
                        mCameraDevice = camera;
                        try {
                            CaptureRequest.Builder builder = camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                            builder.addTarget(mImageReader.getSurface());
                            camera.createCaptureSession(Collections.singletonList(mImageReader.getSurface()),
                                    new CameraCaptureSession.StateCallback() {
                                        @Override
                                        public void onConfigured(@NonNull CameraCaptureSession session) {
                                            try {
                                                session.setRepeatingRequest(builder.build(), null, mBackgroundHandler);
                                            } catch (CameraAccessException e) {
                                                e.printStackTrace();
                                            }
                                        }
                                        @Override
                                        public void onConfigureFailed(@NonNull CameraCaptureSession session) {}
                                    }, mBackgroundHandler);
                        } catch (CameraAccessException e) {
                            e.printStackTrace();
                        }
                    }

                    @Override
                    public void onDisconnected(@NonNull CameraDevice camera) {
                        camera.close();
                    }

                    @Override
                    public void onError(@NonNull CameraDevice camera, int error) {
                        camera.close();
                    }
                }, mBackgroundHandler);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private byte[] yuv420ToJpeg(Image image) {
        try {
            Image.Plane[] planes = image.getPlanes();
            ByteBuffer yBuffer = planes[0].getBuffer();
            ByteBuffer uBuffer = planes[1].getBuffer();
            ByteBuffer vBuffer = planes[2].getBuffer();

            int ySize = yBuffer.remaining();
            int uSize = uBuffer.remaining();
            int vSize = vBuffer.remaining();

            byte[] nv21 = new byte[ySize + uSize + vSize];
            yBuffer.get(nv21, 0, ySize);
            vBuffer.get(nv21, ySize, vSize);
            uBuffer.get(nv21, ySize + vSize, uSize);

            YuvImage yuvImage = new YuvImage(nv21, ImageFormat.NV21, image.getWidth(), image.getHeight(), null);
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            yuvImage.compressToJpeg(new Rect(0, 0, image.getWidth(), image.getHeight()), 50, out);
            return out.toByteArray();
        } catch (Exception e) {
            return null;
        }
    }

    // --- Sensors Stream ----------------------------------------------------
    @Override
    public void onSensorChanged(SensorEvent event) {
        if (mWebSocket == null) return;

        try {
            JSONObject json = new JSONObject();
            json.put("timestamp", System.currentTimeMillis());

            if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
                json.put("type", "accelerometer");
                json.put("ax", event.values[0]);
                json.put("ay", event.values[1]);
                json.put("az", event.values[2]);
                mGravity = event.values.clone();
            } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
                json.put("type", "gyroscope");
                json.put("gx", event.values[0]);
                json.put("gy", event.values[1]);
                json.put("gz", event.values[2]);
            } else if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
                SensorManager.getRotationMatrixFromVector(mRotationMatrix, event.values);
                SensorManager.getOrientation(mRotationMatrix, mOrientationAngles);
                json.put("type", "orientation");
                json.put("yaw", mOrientationAngles[0]);
                json.put("pitch", mOrientationAngles[1]);
                json.put("roll", mOrientationAngles[2]);
            }

            mWebSocket.send(json.toString());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

    // --- GPS Location Stream -----------------------------------------------
    @Override
    public void onLocationChanged(@NonNull Location location) {
        if (mWebSocket == null) return;
        try {
            JSONObject json = new JSONObject();
            json.put("type", "gps");
            json.put("lat", location.getLatitude());
            json.put("lon", location.getLongitude());
            json.put("alt", location.getAltitude());
            json.put("speed", location.getSpeed());
            mWebSocket.send(json.toString());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void onDestroy() {
        mIsDestroyed = true;
        super.onDestroy();
        if (mWebSocket != null) mWebSocket.close(1000, "Activity Closed");
        if (mSensorManager != null) mSensorManager.unregisterListener(this);
    }
}
