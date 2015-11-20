package org.opencv.samples.colorblobdetect;

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

import ioio.lib.api.AnalogInput;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.PulseInput;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.android.IOIOActivity;

public class ColorBlobDetectionActivity extends IOIOActivity implements OnTouchListener, CvCameraViewListener2 {
    private static final String  TAG              = "OCVSample::Activity";
    private TextView textView_;
    private TextView PanValue_;
    private TextView SizeError_;
    private SeekBar seekBar_;
    private ToggleButton toggleButton_;
    static private int screenWidth;
    static private int screenHeight;
    static private int blobx; //current x value of the centroid (center of mass) of the largest tracked blob contour
    static private int bloby; //current y value of the centroid of the largest tracked blob
    static private double maxContour = 0;
    static private double minContour = 1000; //smallest contour area that we will pay attention to
    static private double targetContour = -1; //what is the size of the maximum contour just after it is selected by touch? - serves as the target size (distance to maintain from the object)

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

        enableUi(false);

    }

    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
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

                    if (pulseSpeed.getDuration()*1000000<1000) //we have a speed (deadman) signal from the receiver that allows us to move
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
                enableUi(false);
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
    //}
}
