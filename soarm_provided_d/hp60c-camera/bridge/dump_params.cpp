// HP60C intrinsic dumper (STEP 0 helper, 검수팀 생성 — 기존 shm_bridge.cpp 미변경)
//
// shm_bridge 가 onAttached 시점(스트림 시작 직후)에 AS_SDK_GetCamParameter 를
// 한 번만 호출하는데, HP60C 펌웨어는 스트림이 안정화된 뒤에야 intrinsic 을
// 돌려주는 경우가 있어 /tmp/hp60c_params.txt 가 생성되지 않을 수 있다.
// 이 도구는 카메라를 열고 스트림을 켠 뒤 GetCamParameter 를 여러 번 재시도해서
// 성공하면 모든 파라미터를 출력하고 /tmp/hp60c_params.txt 형식으로 저장한다.
//
// 주의: SDK 는 카메라 핸들을 동시에 하나만 허용하므로, 실행 전 shm_bridge 를
//       반드시 종료해야 한다.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <dirent.h>

#include "as_camera_sdk_api.h"
#include "as_camera_sdk_def.h"

static AS_CAM_PTR g_camera = nullptr;
static std::string g_config_dir;

static void scanDir(const std::string& dir, std::vector<std::string>& out) {
    DIR* d = opendir(dir.c_str());
    if (!d) return;
    struct dirent* e;
    while ((e = readdir(d)) != nullptr) {
        std::string n = e->d_name;
        if (n == "." || n == "..") continue;
        if (n.size() > 5 && n.substr(n.size() - 5) == ".json")
            out.push_back(dir + "/" + n);
    }
    closedir(d);
}

static std::string findConfigFile(const std::string& dir, const std::string& key) {
    std::vector<std::string> files;
    scanDir(dir, files);
    for (auto& f : files)
        if (f.find(key) != std::string::npos) return f;
    return "";
}

static bool dumpParams(AS_CAM_PTR cam) {
    AS_CAM_Parameter_s p;
    for (int i = 0; i < 30; ++i) {
        memset(&p, 0, sizeof(p));
        int ret = AS_SDK_GetCamParameter(cam, &p);
        if (ret == 0 && (p.fxir != 0.0f || p.fxrgb != 0.0f)) {
            printf("[dump_params] GetCamParameter OK on attempt %d\n", i + 1);
            printf("IR : fx=%.3f fy=%.3f cx=%.3f cy=%.3f\n", p.fxir, p.fyir, p.cxir, p.cyir);
            printf("RGB: fx=%.3f fy=%.3f cx=%.3f cy=%.3f\n", p.fxrgb, p.fyrgb, p.cxrgb, p.cyrgb);
            printf("T  : %.6f %.6f %.6f\n", p.T1, p.T2, p.T3);
            if (FILE* fp = fopen("/tmp/hp60c_params.txt", "w")) {
                fprintf(fp, "fxir=%.6f\nfyir=%.6f\ncxir=%.6f\ncyir=%.6f\n", p.fxir, p.fyir, p.cxir, p.cyir);
                fprintf(fp, "fxrgb=%.6f\nfyrgb=%.6f\ncxrgb=%.6f\ncyrgb=%.6f\n", p.fxrgb, p.fyrgb, p.cxrgb, p.cyrgb);
                fprintf(fp, "T1=%.6f\nT2=%.6f\nT3=%.6f\n", p.T1, p.T2, p.T3);
                // 추가: 회전행렬과 전체 왜곡계수도 저장 (브릿지가 안 쓰는 항목, 캘리브에 유용)
                fprintf(fp, "R00=%.6f\nR01=%.6f\nR02=%.6f\n", p.R00, p.R01, p.R02);
                fprintf(fp, "R10=%.6f\nR11=%.6f\nR12=%.6f\n", p.R10, p.R11, p.R12);
                fprintf(fp, "R20=%.6f\nR21=%.6f\nR22=%.6f\n", p.R20, p.R21, p.R22);
                fprintf(fp, "K1ir=%.6f\nK2ir=%.6f\nK3ir=%.6f\nP1ir=%.6f\nP2ir=%.6f\n", p.K1ir, p.K2ir, p.K3ir, p.P1ir, p.P2ir);
                fprintf(fp, "K1rgb=%.6f\nK2rgb=%.6f\nK3rgb=%.6f\nP1rgb=%.6f\nP2rgb=%.6f\n", p.K1rgb, p.K2rgb, p.K3rgb, p.P1rgb, p.P2rgb);
                fclose(fp);
                printf("[dump_params] wrote /tmp/hp60c_params.txt\n");
            }
            return true;
        }
        printf("[dump_params] attempt %d: ret=%d fxir=%.3f (retrying)\n", i + 1, ret, p.fxir);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return false;
}

static void onAttached(AS_CAM_ATTR_S* attr, void*) {
    if (g_camera) return;
    AS_CAM_PTR cam;
    if (AS_SDK_CreateCamHandle(cam, attr) != 0) { fprintf(stderr, "CreateCamHandle failed\n"); return; }
    AS_SDK_CAM_MODEL_E model;
    AS_SDK_GetCameraModel(cam, model);
    printf("[dump_params] Camera attached, model type: %d\n", model);
    std::string cfg = findConfigFile(g_config_dir, "hp60c_");
    if (cfg.empty()) cfg = findConfigFile(g_config_dir, "hp60cn_");
    if (cfg.empty()) { fprintf(stderr, "no config\n"); return; }
    printf("[dump_params] Using config: %s\n", cfg.c_str());
    if (AS_SDK_OpenCamera(cam, cfg.c_str()) != 0) { fprintf(stderr, "OpenCamera failed\n"); return; }
    if (AS_SDK_StartStream(cam) != 0) { fprintf(stderr, "StartStream failed\n"); return; }
    printf("[dump_params] streaming, waiting then querying params...\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    dumpParams(cam);
    g_camera = cam;
}

static void onDetached(AS_CAM_ATTR_S*, void*) {}

int main(int argc, char* argv[]) {
    g_config_dir = (argc > 1) ? argv[1] : "../configurationfiles";
    printf("[dump_params] Config dir: %s\n", g_config_dir.c_str());
    if (AS_SDK_Init() != 0) { fprintf(stderr, "AS_SDK_Init failed\n"); return 1; }
    AS_LISTENER_CALLBACK_S listener;
    listener.onAttached  = onAttached;
    listener.onDetached  = onDetached;
    listener.privateData = nullptr;
    AS_SDK_StartListener(listener, AS_LISTENNER_TYPE_USB, true);
    // attach 콜백 + 재시도 시간 확보
    std::this_thread::sleep_for(std::chrono::seconds(20));
    if (g_camera) {
        AS_SDK_StopStream(g_camera);
        AS_SDK_CloseCamera(g_camera);
        AS_SDK_DestoryCamHandle(g_camera);
    }
    AS_SDK_StopListener();
    AS_SDK_Deinit();
    return 0;
}
