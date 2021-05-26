package com.example.detailar;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

import android.annotation.TargetApi;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.database.Cursor;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.media.ExifInterface;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.provider.MediaStore;
import android.util.Log;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.SeekBar;
import android.widget.TextView;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.io.IOException;
import java.util.Collections;
import java.util.List;

import static android.Manifest.permission.CAMERA;

public class MainActivity extends AppCompatActivity {


    static {
        System.loadLibrary("opencv_java4");
        System.loadLibrary("native-lib");
    }


    ImageView imageVIewInput;
    ImageView imageVIewOuput;
    private Mat img_input;
    private Mat img_output;

    private static final String TAG = "opencv";
    private final int GET_GALLERY_IMAGE = 200;

    boolean isReady = false;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        imageVIewInput = (ImageView)findViewById(R.id.imageViewInput);
        imageVIewOuput = (ImageView)findViewById(R.id.imageViewOutput);

        Button Button = (Button)findViewById(R.id.button);
        Button.setOnClickListener(new View.OnClickListener(){
            public void onClick(View v){
                if (isReady==false) return;

                if (img_output == null)
                    img_output = new Mat();
                imageprocessing(img_input.getNativeObjAddr(), img_output.getNativeObjAddr());

                Bitmap bitmapOutput = Bitmap.createBitmap(img_output.cols(), img_output.rows(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(img_output, bitmapOutput);
                imageVIewOuput.setImageBitmap(bitmapOutput);
            }
        });

        imageVIewInput.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                Intent intent = new Intent(Intent.ACTION_PICK);
                intent.setData(android.provider.MediaStore.Images.Media.EXTERNAL_CONTENT_URI);
                intent.setType("image/*");
                startActivityForResult(intent, GET_GALLERY_IMAGE);
            }
        });
        if (!hasPermissions(PERMISSIONS)) { //퍼미션 허가를 했었는지 여부를 확인
            requestNecessaryPermissions(PERMISSIONS);//퍼미션 허가안되어 있다면 사용자에게 요청
        }

    }

    @Override
    protected void onResume() {
        super.onResume();

        isReady = true;
    }

    public native void imageprocessing(long inputImage, long outputImage);

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {

        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == GET_GALLERY_IMAGE) {


            if (data.getData() != null) {
                Uri uri = data.getData();

                try {
                    String path = getRealPathFromURI(uri);
                    int orientation = getOrientationOfImage(path); // 런타임 퍼미션 필요
                    Bitmap temp = MediaStore.Images.Media.getBitmap(getContentResolver(), uri);
                    Bitmap bitmap = getRotatedBitmap(temp, orientation);
                    imageVIewInput.setImageBitmap(bitmap);

                    img_input = new Mat();
                    Bitmap bmp32 = bitmap.copy(Bitmap.Config.RGB_565, true);
                    Utils.bitmapToMat(bmp32, img_input);


                } catch (Exception e) {
                    e.printStackTrace();
                }

            }

        }
    }


    private String getRealPathFromURI(Uri contentUri) {

        String[] proj = {MediaStore.Images.Media.DATA};
        Cursor cursor = getContentResolver().query(contentUri, proj, null, null, null);
        cursor.moveToFirst();
        int column_index = cursor.getColumnIndexOrThrow(MediaStore.Images.Media.DATA);

        return cursor.getString(column_index);
    }

    public int getOrientationOfImage(String filepath) {
        ExifInterface exif = null;

        try {
            exif = new ExifInterface(filepath);
        } catch (IOException e) {
            Log.d("@@@", e.toString());
            return -1;
        }

        int orientation = exif.getAttributeInt(ExifInterface.TAG_ORIENTATION, -1);

        if (orientation != -1) {
            switch (orientation) {
                case ExifInterface.ORIENTATION_ROTATE_90:
                    return 90;

                case ExifInterface.ORIENTATION_ROTATE_180:
                    return 180;

                case ExifInterface.ORIENTATION_ROTATE_270:
                    return 270;
            }
        }

        return 0;
    }

    public Bitmap getRotatedBitmap(Bitmap bitmap, int degrees) throws Exception {
        if(bitmap == null) return null;
        if (degrees == 0) return bitmap;

        Matrix m = new Matrix();
        m.setRotate(degrees, (float) bitmap.getWidth() / 2, (float) bitmap.getHeight() / 2);

        return Bitmap.createBitmap(bitmap, 0, 0, bitmap.getWidth(), bitmap.getHeight(), m, true);
    }



    // 퍼미션 코드
    static final int PERMISSION_REQUEST_CODE = 1;
    String[] PERMISSIONS  = {"android.permission.WRITE_EXTERNAL_STORAGE"};

    private boolean hasPermissions(String[] permissions) {
        int ret = 0;
        //스트링 배열에 있는 퍼미션들의 허가 상태 여부 확인
        for (String perms : permissions){
            ret = checkCallingOrSelfPermission(perms);
            if (!(ret == PackageManager.PERMISSION_GRANTED)){
                //퍼미션 허가 안된 경우
                return false;
            }

        }
        //모든 퍼미션이 허가된 경우
        return true;
    }

    private void requestNecessaryPermissions(String[] permissions) {
        //마시멜로( API 23 )이상에서 런타임 퍼미션(Runtime Permission) 요청
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            requestPermissions(permissions, PERMISSION_REQUEST_CODE);
        }
    }



    @Override
    public void onRequestPermissionsResult(int permsRequestCode, @NonNull String[] permissions, @NonNull int[] grantResults){
        switch(permsRequestCode){

            case PERMISSION_REQUEST_CODE:
                if (grantResults.length > 0) {
                    boolean writeAccepted = grantResults[0] == PackageManager.PERMISSION_GRANTED;

                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {

                        if (!writeAccepted )
                        {
                            showDialogforPermission("앱을 실행하려면 퍼미션을 허가하셔야합니다.");
                            return;
                        }
                    }
                }
                break;
        }
    }

    private void showDialogforPermission(String msg) {

        final AlertDialog.Builder myDialog = new AlertDialog.Builder(  MainActivity.this);
        myDialog.setTitle("알림");
        myDialog.setMessage(msg);
        myDialog.setCancelable(false);
        myDialog.setPositiveButton("예", new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface arg0, int arg1) {
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                    requestPermissions(PERMISSIONS, PERMISSION_REQUEST_CODE);
                }

            }
        });
        myDialog.setNegativeButton("아니오", new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface arg0, int arg1) {
                finish();
            }
        });
        myDialog.show();
    }
}

/*
public class MainActivity extends AppCompatActivity implements PortraitCameraBridgeViewBase.CvCameraViewListener2 {


    private final String _TAG = "MainActivity:";

    private Mat matInput;
    private Mat matResult;
    private PortraitCameraBridgeViewBase mOpenCvCameraView; // 카메라 역할

    public native void ConvertRGBtoGray(long matAddrInput, long matAddrResult); // native-lib에서 구현


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
        mOpenCvCameraView = (PortraitCameraBridgeViewBase)findViewById(R.id.cameraView);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
        mOpenCvCameraView.setMPreviewFormat(PortraitCameraBridgeViewBase.RGBA); // RGBA : RGB CAMERA, GRAY : GRAY CAMERA
        mOpenCvCameraView.setCameraIndex(0); // front-camera(1),  back-camera(0)
        mOpenCvCameraView.setMaxFrameSize(400, 400);

        Button changeColorButton = (Button) findViewById(R.id.changeColorButton);
        changeColorButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                String TAG = new StringBuilder(_TAG).append("onClickChangeColorButton").toString();

                stopCamera(TAG);
                int cameraColor = mOpenCvCameraView.getMPreviewFormat();

                switch(cameraColor){
                    case PortraitCameraBridgeViewBase.RGBA:
                        mOpenCvCameraView.setMPreviewFormat(PortraitCameraBridgeViewBase.GRAY);
                        break;
                    case PortraitCameraBridgeViewBase.GRAY:
                        mOpenCvCameraView.setMPreviewFormat(PortraitCameraBridgeViewBase.RGBA);
                        break;
                    default:
                        mOpenCvCameraView.setMPreviewFormat(PortraitCameraBridgeViewBase.RGBA);
                        break;
                }

                startCamera(TAG);
            }
        });

        Button changeCameraButton = (Button) findViewById(R.id.changeCameraButton);
        changeCameraButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                String TAG = new StringBuilder(_TAG).append("onClickChangeCameraButton").toString();

                stopCamera(TAG);

                int cameraIndex = mOpenCvCameraView.getCameraIndex();
                switch (cameraIndex) {
                    case PortraitCameraBridgeViewBase.CAMERA_ID_FRONT:
                        mOpenCvCameraView.setCameraIndex(PortraitCameraBridgeViewBase.CAMERA_ID_BACK);
                        break;
                    case PortraitCameraBridgeViewBase.CAMERA_ID_BACK:
                        mOpenCvCameraView.setCameraIndex(PortraitCameraBridgeViewBase.CAMERA_ID_FRONT);
                        break;
                    default:
                        mOpenCvCameraView.setCameraIndex(PortraitCameraBridgeViewBase.CAMERA_ID_BACK);
                        break;
                }

                startCamera(TAG);
            }
        });
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

        int color = mOpenCvCameraView.getMPreviewFormat();
        switch(color){
            case PortraitCameraBridgeViewBase.RGBA:
                return matInput;
            case PortraitCameraBridgeViewBase.GRAY:
                ConvertRGBtoGray(matInput.getNativeObjAddr(), matResult.getNativeObjAddr());
        }

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
}*/