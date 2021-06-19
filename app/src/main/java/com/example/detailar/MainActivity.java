package com.example.detailar;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

import android.annotation.TargetApi;
import android.content.DialogInterface;
import android.content.pm.PackageManager;
import android.hardware.Camera;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.Collections;
import java.util.List;

import static android.Manifest.permission.CAMERA;

public class MainActivity extends AppCompatActivity implements PortraitCameraView.CvCameraViewListener2 {


    private final String _TAG = "MainActivity:";

    private Mat matInput;
    private Mat matResult;
    private PortraitCameraView mOpenCvCameraView; // 카메라 역할

    public native void FindBiliards(long matAddrInput, long matAddrResult); // native-lib에서 구현


    static {
        System.loadLibrary("opencv_java4");
        System.loadLibrary("native-lib");
    }



    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override

        public void onManagerConnected(int status) {
            String TAG = new StringBuilder(_TAG).append("onManagerConnected").toString();

            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                    break;
                default:
                    super.onManagerConnected(status);
                    break;
            }
        }
    };

    // onCreate함수는 가장 먼저 실행되는 함수이다. didMount
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN); // 상태바 안보이게 하기
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON,
                WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON); // 화면 켜진상태 유지

        setContentView(R.layout.activity_main);

        // 카메라 설정
        mOpenCvCameraView = (PortraitCameraView)findViewById(R.id.cameraView);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
        mOpenCvCameraView.setMPreviewFormat(PortraitCameraBridgeViewBase.RGBA); // RGBA : RGB CAMERA, GRAY : GRAY CAMERA
        mOpenCvCameraView.setCameraIndex(0); // front-camera(1),  back-camera(0)
        mOpenCvCameraView.setMaxFrameSize(1440, 1080);
    }

    // Activitiy가 run하고 있을 때 동기화 하는 역할
    @Override
    public void onPause()
    {
        String TAG = new StringBuilder(_TAG).append("onPause").toString();
        stopCamera(TAG);
        super.onPause();
    }

    // 정지되었다가 다시 시작
    @Override
    public void onResume()
    {
        super.onResume();

        if (!OpenCVLoader.initDebug()) {
            Log.d(_TAG, "onResume :: Internal OpenCV library not found.");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_2_0, this, mLoaderCallback);
        } else {
            Log.d(_TAG, "onResum :: OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    // componentWillUnMount
    public void onDestroy() {
        String TAG = new StringBuilder(_TAG).append("onDestroy").toString();
        stopCamera(TAG);

        super.onDestroy();
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        String TAG = new StringBuilder(_TAG).append("onCameraViewStarted").toString();

        Log.i(TAG, "OpenCV CameraView Stopped");
    }

    @Override
    public void onCameraViewStopped() {
        String TAG = new StringBuilder(_TAG).append("onCameraViewStoped").toString();

        Log.i(TAG, "OpenCV CameraView Stoped");
    }

    @Override
    public Mat onCameraFrame(PortraitCameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        matInput = inputFrame.rgba();
        if ( matResult == null )
            matResult = new Mat(matInput.rows(), matInput.cols(), matInput.type());

        Mat  resizeImage = new Mat();
        Size sz = new Size(1000, 562); // Scale up to 800x600
        Imgproc.resize(matInput, resizeImage, sz);

        FindBiliards(resizeImage.getNativeObjAddr(), resizeImage.getNativeObjAddr());

        /* Size 960 - 720, 720 - 480 */
        Size szAfter = new Size(960, 720); // Scale back down to 640x480 (original dim.)
        Imgproc.resize(resizeImage, matResult, szAfter);

        return matResult;
    }


    protected List<? extends PortraitCameraBridgeViewBase> getCameraViewList() {
        return Collections.singletonList(mOpenCvCameraView);
    }


    private void startCamera(String TAG) {
        if (!OpenCVLoader.initDebug()) {
            Log.i(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initiation");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.i(TAG, "OpenCV library found inside package. Using it");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    private void stopCamera(String TAG) {
        Log.i(TAG, "Disabling a camera view");

        if (mOpenCvCameraView != null) {
            mOpenCvCameraView.disableView();
        }
    }


    //여기서부턴 퍼미션 관련 메소드
    private static final int CAMERA_PERMISSION_REQUEST_CODE = 200;


    protected void onCameraPermissionGranted() {
        List<? extends PortraitCameraBridgeViewBase> cameraViews = getCameraViewList();
        if (cameraViews == null) {
            return;
        }
        for (PortraitCameraBridgeViewBase PortraitCameraBridgeViewBase: cameraViews) {
            if (PortraitCameraBridgeViewBase != null) {
                PortraitCameraBridgeViewBase.setCameraPermissionGranted();
            }
        }
    }

    @Override
    protected void onStart() {
        super.onStart();
        boolean havePermission = true;
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            if (checkSelfPermission(CAMERA) != PackageManager.PERMISSION_GRANTED) {
                requestPermissions(new String[]{CAMERA}, CAMERA_PERMISSION_REQUEST_CODE);
                havePermission = false;
            }
        }
        if (havePermission) {
            onCameraPermissionGranted();
        }
    }

    @Override
    @TargetApi(Build.VERSION_CODES.M)
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        if (requestCode == CAMERA_PERMISSION_REQUEST_CODE && grantResults.length > 0
                && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            onCameraPermissionGranted();
        }else{
            showDialogForPermission("앱을 실행하려면 퍼미션을 허가하셔야합니다.");
        }
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
    }


    @TargetApi(Build.VERSION_CODES.M)
    private void showDialogForPermission(String msg) {

        AlertDialog.Builder builder = new AlertDialog.Builder( MainActivity.this);
        builder.setTitle("알림");
        builder.setMessage(msg);
        builder.setCancelable(false);
        builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface dialog, int id){
                requestPermissions(new String[]{CAMERA}, CAMERA_PERMISSION_REQUEST_CODE);
            }
        });
        builder.setNegativeButton("아니오", new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface arg0, int arg1) {
                finish();
            }
        });
        builder.create().show();
    }
}