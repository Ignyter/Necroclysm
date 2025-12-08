#include <SDL3/SDL.h>

export module displayLoader;
export import globalVar;

import util;
import constVar;

namespace dispOption
{
    bool fullScreen = true;            // 풀스크린 여부
    bool useLetterbox = true;         // 풀스크린 시 레터박스 사용 여부 (창모드에서는 상관없음)
    bool fixScreenRatio = true;       // 창모드 시 비율 고정 여부 (전체화면에서는 상관없음)
    int resolutionPreset = 0;          // 비율 프리셋 (0~1) fixScreenRatio가 true일 때만 적용 (전체화면에서는 상관없음)
    double windowScale = 1.0;          // 창 크기 배율 (전체화면에서는 상관없음)
}

export void displayLoader()
{
    SDL_SetHint(SDL_HINT_RENDER_DRIVER, "opengl");
    SDL_DisplayID disp = SDL_GetPrimaryDisplay();
    const SDL_DisplayMode* mode = SDL_GetCurrentDisplayMode(disp);
    if (!mode) errorBox(L"디스플레이 정보를 읽을 수 없습니다.");

    int screenW = mode->w;
    int screenH = mode->h;
    int winW, winH;

    if (dispOption::fullScreen)
    {
        winW = screenW;
        winH = screenH;

        if (dispOption::useLetterbox)
        {
            cameraW = 1080;
            cameraH = 1080;
        }
        else
        {
            cameraH = 1080;
            cameraW = (int)(((float)screenW / (float)screenH) * 1080.0f);
        }
    }
    else  // 창모드
    {
        if (dispOption::fixScreenRatio)
        {
            switch (dispOption::resolutionPreset)
            {
            case 1:  // FHD (1920x1080)
                winW = (int)(1920 * dispOption::windowScale);
                winH = (int)(1080 * dispOption::windowScale);
                cameraW = 1920;
                cameraH = 1080;
                break;
            default: // 1:1 (1080x1080)
                winW = (int)(1080 * dispOption::windowScale);
                winH = (int)(1080 * dispOption::windowScale);
                cameraW = 1080;
                cameraH = 1080;
                break;
            }
        }
        else
        {
            // 자유 비율: 모니터 비율 유지, 스케일 적용
            cameraH = 1080;
            cameraW = (int)(((float)screenW / (float)screenH) * 1080.0f);
            winH = (int)(1080 * dispOption::windowScale);
            winW = (int)(((float)screenW / (float)screenH) * winH);
        }
    }

    if (!SDL_CreateWindowAndRenderer("Necroclysm", winW, winH,
        dispOption::fullScreen ? SDL_WINDOW_FULLSCREEN : 0,
        &window, &renderer))
        errorBox(L"창·렌더러 생성 실패");

    setPrimitiveRenderer(renderer);

    SDL_SetRenderLogicalPresentation(renderer, cameraW, cameraH,
        (dispOption::fullScreen && dispOption::useLetterbox)
        ? SDL_LOGICAL_PRESENTATION_LETTERBOX
        : SDL_LOGICAL_PRESENTATION_STRETCH);

    std::printf("Monitor: %dx%d | Window: %dx%d | Camera: %dx%d (%.2f:1) | %s %s\n",
        screenW, screenH, winW, winH, cameraW, cameraH,
        (float)cameraW / (float)cameraH,
        dispOption::fullScreen ? "Fullscreen" : "Windowed",
        (dispOption::fullScreen && dispOption::useLetterbox) ? "Letterbox" : "Stretch");

    //letterbox = { 0, 0, 782, 176 };
    //letterbox.x = (cameraW - letterbox.w) / 2;
    //letterbox.y = cameraH - letterbox.h + 6;

    //for (int i = 0; i < 7; ++i)
    //    barButton[i] = { cameraW / 2 - 300 + 88 * i, cameraH - 80, 72, 72 };

    //letterboxPopUpButton = { letterbox.x + letterbox.w - 39,
    //                         letterbox.y - 33,
    //                         29, 29 };
}