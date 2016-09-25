package org.opencv.samples.colorblobdetect;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import java.lang.Math;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.view.View.OnTouchListener;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.ToggleButton;

import android.location.Location;
import com.google.android.gms.common.ConnectionResult;
import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.common.api.GoogleApiClient.ConnectionCallbacks;
import com.google.android.gms.common.api.GoogleApiClient.OnConnectionFailedListener;
import com.google.android.gms.location.LocationListener;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationServices;

import ioio.lib.api.AnalogInput;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.PulseInput;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.android.IOIOActivity;

import java.text.DateFormat;
import java.util.Date;

public class ColorBlobDetectionActivity extends IOIOActivity implements OnTouchListener, CvCameraViewListener2, ConnectionCallbacks, OnConnectionFailedListener, LocationListener {
    private static final String  TAG              = "OCVSample::Activity";
    private TextView textView_;
    private TextView PanValue_;
    private TextView SizeError_;
    private SeekBar seekBar_;
    private ToggleButton toggleButton_;
    private TextView PIDerr;
    private TextView PIDSpeed;
    private ToggleButton toggleDrive_;

    static private int screenWidth;
    static private int screenHeight;
    static private int blobx; //current x value of the centroid (center of mass) of the largest tracked blob contour
    static private int bloby; //current y value of the centroid of the largest tracked blob
    static private double maxContour = 0;
    static private double minContour = 1000; //smallest contour area that we will pay attention to
    static private double targetContour = -1; //what is the size of the maximum contour just after it is selected by touch? - serves as the target size (distance to maintain from the object)
    private int startTime = 100; //countdown timer until it's OK to travel - reset each time screen is touched to allow 5 seconds for operator to get out of the way

    /**
     * Provides the entry point to Google Play services.
     */
    protected GoogleApiClient mGoogleApiClient;

    /**
     * Represents a geographical location.
     */
    protected Location mLastLocation;


    /**
     * The desired interval for location updates. Inexact. Updates may be more or less frequent.
     */
    public static final long UPDATE_INTERVAL_IN_MILLISECONDS = 2000;

    /**
     * The fastest rate for active location updates. Exact. Updates will never be more frequent
     * than this value.
     */
    public static final long FASTEST_UPDATE_INTERVAL_IN_MILLISECONDS =
            UPDATE_INTERVAL_IN_MILLISECONDS / 2;

    // Keys for storing activity state in the Bundle.
    protected final static String REQUESTING_LOCATION_UPDATES_KEY = "requesting-location-updates-key";
    protected final static String LOCATION_KEY = "location-key";
    protected final static String LAST_UPDATED_TIME_STRING_KEY = "last-updated-time-string-key";

    /**
     * Stores parameters for requests to the FusedLocationProviderApi.
     */
    protected LocationRequest mLocationRequest;

    /**
     * Represents a geographical location.
     */
    protected Location mCurrentLocation;

    /**
     * Tracks the status of the location updates request. Value changes when the user presses the
     * Start Updates and Stop Updates buttons.
     */
    protected Boolean mRequestingLocationUpdates;

    /**
     * Time when the location was updated represented as a String.
     */
    protected String mLastUpdateTime;

    protected ArrayList<Location> headingTarget;
    protected ArrayList<Location> proximityStop;

//    private CameraControl mOpenCvCameraView;
    private boolean              mIsColorSelected = false;
    private Mat                  mRgba;
    private Scalar               mBlobColorRgba;
    private Scalar               mBlobColorHsv;
    private ColorBlobDetector    mDetector;
    private Mat                  mSpectrum;
    private Size                 SPECTRUM_SIZE;
    private Scalar               CONTOUR_COLOR;

    private CameraBridgeViewBase mOpenCvCameraView;

    private BaseLoaderCallback  mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                    mOpenCvCameraView.setOnTouchListener(ColorBlobDetectionActivity.this);
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };




    public ColorBlobDetectionActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.color_blob_detection_surface_view);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
        mOpenCvCameraView.setCvCameraViewListener(this);

        //setContentView(R.layout.main);

        textView_ = (TextView) findViewById(R.id.TextView);
        PanValue_ = (TextView) findViewById(R.id.PanValue);
        SizeError_ = (TextView) findViewById(R.id.SizeError);
        seekBar_ = (SeekBar) findViewById(R.id.SeekBar);
        toggleButton_ = (ToggleButton) findViewById(R.id.ToggleButton);
        toggleDrive_ = (ToggleButton) findViewById(R.id.enableButton);
        PIDerr = (TextView) findViewById(R.id.e);
        PIDSpeed = (TextView) findViewById(R.id.s);

        mRequestingLocationUpdates = true;
        mLastUpdateTime = "";

        // Update values using data stored in the Bundle.
        updateValuesFromBundle(savedInstanceState);

        buildGoogleApiClient(); //start location services

        enableUi(false); //set back to false to only enable onscreen controls once IOIO connects

    }
    @Override
    protected void onStart() {
        super.onStart();
        mGoogleApiClient.connect();
    }

    @Override
    protected void onStop() {
        super.onStop();
        if (mGoogleApiClient.isConnected()) {
            mGoogleApiClient.disconnect();
        }
    }

    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
        // Stop location updates to save battery, but don't disconnect the GoogleApiClient object.
        if (mGoogleApiClient.isConnected()) {
            stopLocationUpdates();
        }
    }

    @Override
    public void onResume()
    {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_11, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
        // Within {@code onPause()}, we pause location updates, but leave the
        // connection to GoogleApiClient intact.  Here, we resume receiving
        // location updates if the user has requested them.

        if (mGoogleApiClient.isConnected() && mRequestingLocationUpdates) {
            startLocationUpdates();
        }
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mDetector = new ColorBlobDetector();
        mSpectrum = new Mat();
        mBlobColorRgba = new Scalar(255);
        mBlobColorHsv = new Scalar(255);
        SPECTRUM_SIZE = new Size(200, 64);
        CONTOUR_COLOR = new Scalar(0,255,0,255);
        screenHeight=height;
        screenWidth=width;
    }

    public void onCameraViewStopped() {
        mRgba.release();
    }

    public boolean onTouch(View v, MotionEvent event) {
        int cols = mRgba.cols();
        int rows = mRgba.rows();

        int xOffset = (mOpenCvCameraView.getWidth() - cols) / 2;
        int yOffset = (mOpenCvCameraView.getHeight() - rows) / 2;

        int x = (int)event.getX() - xOffset;
        int y = (int)event.getY() - yOffset;

        Log.i(TAG, "Touch image coordinates: (" + x + ", " + y + ")");

        if ((x < 0) || (y < 0) || (x > cols) || (y > rows)) return false;

        Rect touchedRect = new Rect();

        touchedRect.x = (x>4) ? x-4 : 0;
        touchedRect.y = (y>4) ? y-4 : 0;

        touchedRect.width = (x+4 < cols) ? x + 4 - touchedRect.x : cols - touchedRect.x;
        touchedRect.height = (y+4 < rows) ? y + 4 - touchedRect.y : rows - touchedRect.y;

        Mat touchedRegionRgba = mRgba.submat(touchedRect);

        Mat touchedRegionHsv = new Mat();
        Imgproc.cvtColor(touchedRegionRgba, touchedRegionHsv, Imgproc.COLOR_RGB2HSV_FULL);

        // Calculate average color of touched region
        mBlobColorHsv = Core.sumElems(touchedRegionHsv);
        int pointCount = touchedRect.width*touchedRect.height;
        for (int i = 0; i < mBlobColorHsv.val.length; i++)
            mBlobColorHsv.val[i] /= pointCount;

        mBlobColorRgba = converScalarHsv2Rgba(mBlobColorHsv);

        Log.i(TAG, "Touched rgba color: (" + mBlobColorRgba.val[0] + ", " + mBlobColorRgba.val[1] +
                ", " + mBlobColorRgba.val[2] + ", " + mBlobColorRgba.val[3] + ")");

        mDetector.setHsvColor(mBlobColorHsv);

        Imgproc.resize(mDetector.getSpectrum(), mSpectrum, SPECTRUM_SIZE);

        mIsColorSelected = true;
        targetContour = -1; //reset target contour so we get a fresh target size on each touch event

        touchedRegionRgba.release();
        touchedRegionHsv.release();

        return false; // don't need subsequent touch events
    }

    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();

        if (mIsColorSelected) {
            mDetector.process(mRgba);
            List<MatOfPoint> contours = mDetector.getContours();
            Log.e(TAG, "Contours count: " + contours.size());
            Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR, 3);

            //get the centroid (center of mass) and area for each contour

            List<Moments> mu = new ArrayList<Moments>(contours.size());
            maxContour=0;
            for (int i = 0; i < contours.size(); i++) {
                mu.add(i, Imgproc.moments(contours.get(i), false));
                Moments p = mu.get(i);
                int x = (int) (p.get_m10() / p.get_m00());
                int y = (int) (p.get_m01() / p.get_m00());
                //Core.circle(mRgba, new Point(x, y), 4, new Scalar(255,49,0,255));
                Core.circle(mRgba, new Point(x, y), 5, CONTOUR_COLOR, -1);
                double area = Imgproc.contourArea(contours.get(i));
                if (area > maxContour)
                {
                    maxContour=area;
                    blobx=x;
                    bloby=y;
                }
            }

            if (targetContour == -1 && maxContour > 0 )
            {
                targetContour = maxContour; //new target size, thus distance to object
            }




            Mat colorLabel = mRgba.submat(4, 68, 4, 68);
            colorLabel.setTo(mBlobColorRgba);

            Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
            mSpectrum.copyTo(spectrumLabel);
        }

        return mRgba;
    }

    /**
     * Callback that fires when the location changes.
     */
    @Override
    public void onLocationChanged(Location location) {

        mLastUpdateTime = DateFormat.getTimeInstance().format(new Date());
        Log.i(TAG, "Lat: " + location.getLatitude() + ", Lon: " + location.getLongitude() + ", Accuracy: " + location.getAccuracy() + ", Turn To:" + location.bearingTo(mCurrentLocation) + ", Dist: " + location.distanceTo(mCurrentLocation));
        mCurrentLocation = location;
        //updateUI();
        //Toast.makeText(this, getResources().getString(R.string.location_updated_message),
        //        Toast.LENGTH_SHORT).show();
    }


    /**
     * Updates fields based on data stored in the bundle.
     *
     * @param savedInstanceState The activity state saved in the Bundle.
     */
    private void updateValuesFromBundle(Bundle savedInstanceState) {
        Log.i(TAG, "Updating values from bundle");
        if (savedInstanceState != null) {
            // Update the value of mRequestingLocationUpdates from the Bundle, and make sure that
            // the Start Updates and Stop Updates buttons are correctly enabled or disabled.
            if (savedInstanceState.keySet().contains(REQUESTING_LOCATION_UPDATES_KEY)) {
                mRequestingLocationUpdates = savedInstanceState.getBoolean(
                        REQUESTING_LOCATION_UPDATES_KEY);
                //setButtonsEnabledState();
            }

            // Update the value of mCurrentLocation from the Bundle and update the UI to show the
            // correct latitude and longitude.
            if (savedInstanceState.keySet().contains(LOCATION_KEY)) {
                // Since LOCATION_KEY was found in the Bundle, we can be sure that mCurrentLocation
                // is not null.
                mCurrentLocation = savedInstanceState.getParcelable(LOCATION_KEY);
            }

            // Update the value of mLastUpdateTime from the Bundle and update the UI.
            if (savedInstanceState.keySet().contains(LAST_UPDATED_TIME_STRING_KEY)) {
                mLastUpdateTime = savedInstanceState.getString(LAST_UPDATED_TIME_STRING_KEY);
            }
            //updateUI();
        }
    }
    /**
     * Runs when a GoogleApiClient object successfully connects.
     */
    @Override
    public void onConnected(Bundle connectionHint) {
        Log.i(TAG, "Connected to GoogleApiClient");

        // If the initial location was never previously requested, we use
        // FusedLocationApi.getLastLocation() to get it. If it was previously requested, we store
        // its value in the Bundle and check for it in onCreate(). We
        // do not request it again unless the user specifically requests location updates by pressing
        // the Start Updates button.
        //
        // Because we cache the value of the initial location in the Bundle, it means that if the
        // user launches the activity,
        // moves to a new location, and then changes the device orientation, the original location
        // is displayed as the activity is re-created.
        if (mCurrentLocation == null) {
            mCurrentLocation = LocationServices.FusedLocationApi.getLastLocation(mGoogleApiClient);
            mLastUpdateTime = DateFormat.getTimeInstance().format(new Date());
            //updateUI();
        }

        // If the user presses the Start Updates button before GoogleApiClient connects, we set
        // mRequestingLocationUpdates to true (see startUpdatesButtonHandler()). Here, we check
        // the value of mRequestingLocationUpdates and if it is true, we start location updates.
        if (mRequestingLocationUpdates) {
            startLocationUpdates();
        }
    }

    @Override
    public void onConnectionFailed(ConnectionResult result) {
        // Refer to the javadoc for ConnectionResult to see what error codes might be returned in
        // onConnectionFailed.
        Log.i(TAG, "Connection failed: ConnectionResult.getErrorCode() = " + result.getErrorCode());
    }


    @Override
    public void onConnectionSuspended(int cause) {
        // The connection to Google Play services was lost for some reason. We call connect() to
        // attempt to re-establish the connection.
        Log.i(TAG, "Connection suspended");
        mGoogleApiClient.connect();
    }
    private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
    }


    //public class IOIOSimpleApp extends IOIOActivity {

        /*@Override
        public void onCreate(Bundle savedInstanceState) {
            super.onCreate(savedInstanceState);
            setContentView(R.layout.main);

            textView_ = (TextView) findViewById(R.id.TextView);
            seekBar_ = (SeekBar) findViewById(R.id.SeekBar);
            toggleButton_ = (ToggleButton) findViewById(R.id.ToggleButton);

            enableUi(false);
        }*/

        class Looper extends BaseIOIOLooper {
            private AnalogInput input_;
            private PwmOutput pwmCameraPan_;
            private PwmOutput pwmCameraTilt;
            private PwmOutput pwmSteer;
            private PwmOutput pwmSpeed;
            private PulseInput pulseSpeed; //pulse control coming from the radio
            private PulseInput pulseSteer; //pulse control coming from the radio
            private DigitalOutput led_;
            private int PanServoTarget = 1500;
            private int TiltServoTarget = 1500;
            private int SteerServoTarget = 1500;
            private int SpeedTarget = 1500;
            float error_size;
            float terrainBoost = 1;

            @Override
            public void setup() throws ConnectionLostException {
                led_ = ioio_.openDigitalOutput(IOIO.LED_PIN, true);
                input_ = ioio_.openAnalogInput(40);
                pwmSteer = ioio_.openPwmOutput(10, 100);
                pwmSpeed = ioio_.openPwmOutput(11,100);
                pwmCameraPan_ = ioio_.openPwmOutput(12, 100);
                pwmCameraTilt= ioio_.openPwmOutput(13, 100);
                pulseSpeed = ioio_.openPulseInput(7, PulseInput.PulseMode.POSITIVE);
                pulseSteer = ioio_.openPulseInput(6, PulseInput.PulseMode.POSITIVE);
                enableUi(true);
            }

            @Override
            public void loop() throws ConnectionLostException, InterruptedException {
                //setNumber(input_.read());
                setNumber(pulseSpeed.getDuration()*1000000, PanServoTarget, error_size);
                terrainBoost = 1 + ((500 - seekBar_.getProgress()) /500);


                if (maxContour >= minContour) //follow the largest contour that meets minimum size requirement
                {
                    double offset;
                    float error_x;
                    float error_y;


                    // convert to relative % offset from center.  So center = 0, mid left = -.25
                    // then scale by the appropriate multiplier for the desired responsiveness
                    // then update the current servo target with the desired change and clip to legal servo range

                    // horizontal
                    error_x = ((screenWidth/2)-blobx); //absolute error from center
                    error_x = error_x/screenWidth; //relative error as a percentage of screen dimensions so that the Kp is more likely to be similar across different systems
                    //SteerServoTarget = ServoSafe(SteerServoTarget + (int)(error_x * 100)); //amplify the steering response with a stronger Kp
                    //pwmSteer.setPulseWidth(SteerServoTarget);
                    PanServoTarget = SafeRange(PanServoTarget + (int)(error_x * -80), 600, 2400);
                    pwmCameraPan_.setPulseWidth(PanServoTarget);  //center for the pan winch servo is currently off a bit - also damp its reactivity a whole bunch since it is a 3.5 turn winch servo


                    ///pwmCameraPan_.setPulseWidth((int)(offset * 1000 * .15) + 1540);  //center for the pan winch servo is currently off a bit - also reduce its reactivity a whole bunch since it is a 3.5 turn winch server

                    //vertical
                    error_y = ((screenHeight/2)-bloby); //absolute offset from center
                    error_y = error_y/screenHeight; //relative error as a percentage of screen dimensions so that the Kp is more likely to be similar across different systems
                    //pwmSteer.setPulseWidth((int) (((((float) blobx * 3 / (float) screenWidth * 1000) + 1000)-1500)*-1)+1500); // steering needs to be inverted around 1500
                    //pwmCameraPan_.setPulseWidth((int) ((float) blobx / (float) screenWidth * 1000 / 6) + 1460); //range reduced for winch servo
                    TiltServoTarget = ServoSafe(TiltServoTarget + (int)(error_y * -80));
                    pwmCameraTilt.setPulseWidth(TiltServoTarget);

                    //if (pulseSpeed.getDuration()*1000000<1000) //we have a speed (deadman) signal from the receiver that allows us to move
                    if (toggleDrive_.isChecked())
                    {
                        error_size = (float) Math.sqrt(Math.abs(targetContour - maxContour)); //linearize the apparent size error
                        if (targetContour > maxContour) //the current maxContour looks small - so we need to proceed forward
                            {
                                pwmSpeed.setPulseWidth(1500-(int)(1.5 * terrainBoost * error_size));
                                pwmSteer.setPulseWidth(ServoGain(ServoReverse(PanServoTarget),5)); // steering servo slaves off of pan servo, but reverse polarity for steering servo when going forward
                            }
                        else
                        {
                            pwmSpeed.setPulseWidth(1500+(int)(terrainBoost * error_size)); //smaller scaling up because when backing up we don't want it to be too fast and the apparent size increases faster
                            pwmSteer.setPulseWidth((ServoGain(PanServoTarget,5))); // steering servo slaves off of pan servo
                        }
                    }

                    else {pwmSpeed.setPulseWidth(1500);}  //no valid deadman signal so stop bot
                }
                else {pwmSpeed.setPulseWidth(1500);}  //no valid contour so stop bot

                led_.write(!toggleButton_.isChecked());
                Thread.sleep(10);
            }

            @Override
            public void disconnected() {
                //enableUi(false);
            }
            public int ServoSafe(int requestedValue){
                //clip requested value to safe range for a servo PWM signal
                requestedValue = (requestedValue < 1000) ? 1000: requestedValue;
                requestedValue = (requestedValue > 2000) ? 2000: requestedValue;
                return requestedValue;
            }
            public int ServoReverse(int requestedValue){
                //flip input value around 1500
                return (requestedValue-1500) *-1 + 1500;
            }
            public int SafeRange(int requestedValue, int lowestVal, int highestVal){
                //clip requested value to safe range for a servo PWM signal
                requestedValue = (requestedValue < lowestVal) ? lowestVal: requestedValue;
                requestedValue = (requestedValue > highestVal) ? highestVal: requestedValue;
                return requestedValue;
            }
            public int ServoGain(int servoValue, float gain){
                servoValue=ServoSafe((int)((float)(servoValue-1500) * gain)+1500);
                return servoValue;
            }
        }

        @Override
        protected IOIOLooper createIOIOLooper() {
            return new Looper();
        }

        private void enableUi(final boolean enable) {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    seekBar_.setEnabled(enable);
                    toggleButton_.setEnabled(enable);
                    toggleDrive_.setEnabled(enable);
                }
            });
        }

        private void setNumber(float f, float pan, float val3) {
            final String str = String.format("%.4f", f);
            final String strpan = String.format("%.4f", pan);
            final String str3 = String.format("%.4f", val3);
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    textView_.setText(str);
                    PanValue_.setText(strpan);
                    SizeError_.setText(str3);
                }
            });
        }

    /**
     * Builds a GoogleApiClient. Uses the addApi() method to request the LocationServices API.
     */
    protected synchronized void buildGoogleApiClient() {
        mGoogleApiClient = new GoogleApiClient.Builder(this)
                .addConnectionCallbacks(this)
                .addOnConnectionFailedListener(this)
                .addApi(LocationServices.API)
                .build();
        createLocationRequest();
    }
    //}

    /**
     * Sets up the location request. Android has two location request settings:
     * {@code ACCESS_COARSE_LOCATION} and {@code ACCESS_FINE_LOCATION}. These settings control
     * the accuracy of the current location. This sample uses ACCESS_FINE_LOCATION, as defined in
     * the AndroidManifest.xml.
     * <p/>
     * When the ACCESS_FINE_LOCATION setting is specified, combined with a fast update
     * interval (5 seconds), the Fused Location Provider API returns location updates that are
     * accurate to within a few feet.
     * <p/>
     * These settings are appropriate for mapping applications that show real-time location
     * updates.
     */
    protected void createLocationRequest() {
        mLocationRequest = new LocationRequest();

        // Sets the desired interval for active location updates. This interval is
        // inexact. You may not receive updates at all if no location sources are available, or
        // you may receive them slower than requested. You may also receive updates faster than
        // requested if other applications are requesting location at a faster interval.
        mLocationRequest.setInterval(UPDATE_INTERVAL_IN_MILLISECONDS);

        // Sets the fastest rate for active location updates. This interval is exact, and your
        // application will never receive updates faster than this value.
        mLocationRequest.setFastestInterval(FASTEST_UPDATE_INTERVAL_IN_MILLISECONDS);

        mLocationRequest.setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY);
    }

    /**
     * Requests location updates from the FusedLocationApi.
     */
    protected void startLocationUpdates() {
        // The final argument to {@code requestLocationUpdates()} is a LocationListener
        // (http://developer.android.com/reference/com/google/android/gms/location/LocationListener.html).
        LocationServices.FusedLocationApi.requestLocationUpdates(
                mGoogleApiClient, mLocationRequest, this);
    }

    /**
     * Removes location updates from the FusedLocationApi.
     */
    protected void stopLocationUpdates() {
        // It is a good practice to remove location requests when the activity is in a paused or
        // stopped state. Doing so helps battery performance and is especially
        // recommended in applications that request frequent location updates.

        // The final argument to {@code requestLocationUpdates()} is a LocationListener
        // (http://developer.android.com/reference/com/google/android/gms/location/LocationListener.html).
        LocationServices.FusedLocationApi.removeLocationUpdates(mGoogleApiClient, this);
    }

    protected void loadWaypoints() {
        /*
        headingTarget.add(locationBuilder(33.15581988258402, -96.93552490211482)); //h1
        headingTarget.add(locationBuilder(33.15809532917204, -96.93634372002995));
        headingTarget.add(locationBuilder(33.15723412378042, -96.93806983589656));
        headingTarget.add(locationBuilder(33.15842562191988, -96.9377334777235));
        //headingTarget.add(locationBuilder(33.15764166666666, -96.93692777777778));
        proximityStop.add(locationBuilder(33.15701859891353, -96.93643813365026)); //p1
        proximityStop.add(locationBuilder(33.15733, -96.936367));
        proximityStop.add(locationBuilder(33.15736106242152, -96.93676045559523));
        proximityStop.add(locationBuilder(33.1575695280246, -96.93695108803759));
        //proximityStop.add(locationBuilder(33.15764166666666, -96.93692777777778));
        */
        //home waypoints
        proximityStop.add(locationBuilder( 32.869201, -96.840563)); //p1
        headingTarget.add(locationBuilder( 32.868563, -96.840618)); //h1
        proximityStop.add(locationBuilder(  32.869442, -96.840550)); //p2
        headingTarget.add(locationBuilder(  32.870373, -96.840496)); //h1

    }
    protected Location locationBuilder(double latitude, double longitude){
        Location l = new Location("BecauseISaidSo" );
        l.setLatitude(latitude);
        l.setLongitude(longitude);
        l.setAccuracy(10.0f);
    return l;
    }
}
