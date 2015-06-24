package org.opencv.samples.colorblobdetect;

import java.util.ArrayList;
import java.util.List;

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
import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.android.IOIOActivity;

public class ColorBlobDetectionActivity extends IOIOActivity implements OnTouchListener, CvCameraViewListener2 {
    private static final String  TAG              = "OCVSample::Activity";
    private TextView textView_;
    private SeekBar seekBar_;
    private ToggleButton toggleButton_;
    static private int screenWidth;
    static private int screenHeight;
    static private int blobx;
    static private int bloby;
    static private double maxContour = 0;
    static private double minContour = 1000;

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
            private DigitalOutput led_;

            @Override
            public void setup() throws ConnectionLostException {
                led_ = ioio_.openDigitalOutput(IOIO.LED_PIN, true);
                input_ = ioio_.openAnalogInput(40);
                pwmSteer = ioio_.openPwmOutput(10, 100);
                pwmSpeed = ioio_.openPwmOutput(11,100);
                pwmCameraPan_ = ioio_.openPwmOutput(12, 100);
                pwmCameraTilt= ioio_.openPwmOutput(13, 100);
                enableUi(true);
            }

            @Override
            public void loop() throws ConnectionLostException, InterruptedException {
                setNumber(input_.read());
                //pwmCameraTilt.setPulseWidth(500 + seekBar_.getProgress() * 2);
                if (maxContour >= minContour) //follow the largest countour that meets minimum size requirement
                {
                    double offset;
                    // convert to % offset from center.  So center = 50% + 0%, mid left = 50% + -25% = 25%
                    // then convert to servo range of 1000 to 2000 centered on 1500
                    // then scale by the appropriate multiplier for the desired responsiveness
                    // then clip to legal servo range
                    // horizontal
                    offset = -((screenWidth/2)-blobx); //absolute offset from center
                    offset = offset/screenWidth; //relative offset
                    pwmSteer.setPulseWidth((int)(offset * 1000) * -3 + 1500);
                    pwmCameraPan_.setPulseWidth((int)(offset * 1000 * .15) + 1540);  //center for the pan winch servo is currently off a bit - also reduce its reactivity a whole bunch since it is a 3.5 turn winch server
                    //vertical
                    offset = -((screenHeight/2)-bloby); //absolute offset from center
                    offset = offset/screenHeight; //relative offset
                    //pwmSteer.setPulseWidth((int) (((((float) blobx * 3 / (float) screenWidth * 1000) + 1000)-1500)*-1)+1500); // steering needs to be inverted around 1500
                    //pwmCameraPan_.setPulseWidth((int) ((float) blobx / (float) screenWidth * 1000 / 6) + 1460); //range reduced for winch servo
                    pwmCameraTilt.setPulseWidth((int)(offset * 1000 * 1) + 1500);
                }

                led_.write(!toggleButton_.isChecked());
                Thread.sleep(10);
            }

            @Override
            public void disconnected() {
                enableUi(false);
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

        private void setNumber(float f) {
            final String str = String.format("%.2f", f);
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    textView_.setText(str);
                }
            });
        }
    //}
}
