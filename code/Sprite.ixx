#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>

export module Sprite;

import std;
import util;

export class Sprite
{
private:
    SDL_Texture* texturePtr;
    int w;
    int h;
    bool notOwned = false;
public:
    Sprite(SDL_Renderer* renderer, std::string adr, int inputW, int inputH)
    {
        texturePtr = IMG_LoadTexture(renderer, adr.c_str());
        if (texturePtr == nullptr)
        {
            std::wstring errorMsg = L"이미지 로드 실패: " + std::wstring(adr.begin(), adr.end());
            errorBox(true, errorMsg.c_str());
            return;
        }
        w = inputW;
        h = inputH;
        SDL_SetTextureScaleMode(texturePtr, SDL_SCALEMODE_NEAREST);

        if(adr != "image/UI/icon48.png") SDL_SetTextureScaleMode(texturePtr, SDL_SCALEMODE_NEAREST);
        else SDL_SetTextureScaleMode(texturePtr, SDL_SCALEMODE_LINEAR);
    }
    Sprite(SDL_Renderer* renderer, SDL_Texture* inputTexture, int inputW, int inputH)
    {
        texturePtr = inputTexture;
        w = inputW;
        h = inputH;
        notOwned = true;
    }
    ~Sprite()
    {
        //prt(L"Sprite : 소멸자가 호출되었습니다..\n");
        if (notOwned == false) SDL_DestroyTexture(texturePtr);
    }
    SDL_Texture* getTexture() { return texturePtr; }
    int getW() { return w; }
    int getH() { return h; }
};