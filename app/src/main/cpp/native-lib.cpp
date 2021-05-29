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
}