#include <jni.h>

#include <opencv2/opencv.hpp>
#include "Detection.cpp"

using namespace cv;


extern "C"
JNIEXPORT void JNICALL
    Java_com_example_detailar_MainActivity_ConvertRGBtoGray(JNIEnv *env,
                                                                            jobject thiz,
                                                                            jlong mat_addr_input,
                                                                            jlong mat_addr_result) {
    // TODO: implement ConvertRGBtoGray()
    Mat &matInput = *(Mat *)mat_addr_input;
    Mat &matResult = *(Mat *)mat_addr_result;
    Detection_Main(matInput, matResult);
}extern "C"

JNIEXPORT void JNICALL
Java_com_example_detailar_MainActivity_imageprocessing(JNIEnv *env, jobject thiz, jlong input_image,
                                                       jlong output_image) {
    // TODO: implement imageprocessing()
    Mat &img_input = *(Mat *) input_image;
    Mat &img_output = *(Mat *) output_image;

    //cvtColor( img_input, img_output, COLOR_RGB2GRAY);

    //blur( img_output, img_output, Size(5,5) );
    //Canny( img_output, img_output, 10, 10);
    Detection_Main(img_input, img_output);
}