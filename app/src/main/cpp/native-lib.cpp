#include <jni.h>

#include <opencv2/opencv.hpp>
#include "Detail_AR_Main.cpp"
#include "add_ons.cpp"
#include "Detection.cpp"
#include "Detect_Table_Func.cpp"
#include "Detect_Ball_Func.cpp"
#include "Geo_Proc.cpp"

using namespace cv;


extern "C"
JNIEXPORT void JNICALL
    Java_com_example_detailar_MainActivity_FindBiliards(JNIEnv *env,
                                                                            jobject thiz,
                                                                            jlong mat_addr_input,
                                                                            jlong mat_addr_result) {
    // TODO: implement ConvertRGBtoGray()
    Mat &matInput = *(Mat *)mat_addr_input;
    Mat &matResult = *(Mat *)mat_addr_result;
    Detail_AR_Main(matInput, matResult);
}