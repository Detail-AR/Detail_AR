#include <jni.h>

#include <opencv2/opencv.hpp>
#include "Detail_AR_Main.cpp"
#include "add_ons.cpp"
#include "Detection.cpp"
#include "Detect_Table_Func.cpp"
#include "Detect_Ball_Func.cpp"
#include "Geo_Proc.cpp"
#include "Find_Cam_Pos.cpp"
#include "solution.cpp"

using namespace cv;


extern "C"
JNIEXPORT int JNICALL
    Java_com_example_detailar_MainActivity_FindBiliards(JNIEnv *env,
                                                                            jobject thiz,
                                                                            jlong mat_addr_input,
                                                                            jlong mat_addr_result,
                                                                            jint btn_index,
                                                                            jint target_color) {
    // TODO: implement ConvertRGBtoGray()
    Mat &matInput = *(Mat *)mat_addr_input;
    Mat &matResult = *(Mat *)mat_addr_result;
    int c_situation = -1;
    // btn_index : 0 기본 1 인식 2 색깔 3 경로 4 취
    Detail_AR_Main(matInput, matResult, btn_index, target_color, c_situation);

    if(c_situation == 1){
        return 1; // 1이면 모서리를 인식을 하라는 메세지 출력
    }else if(c_situation == 2){
        return 2; // 2이면 인식버튼을 눌르라는 메세지 출력
    }
    return -1; // 기본값(출력X
}