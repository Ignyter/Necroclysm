#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>

export module fontLoader;

import std;
import util;
import globalVar;
import constVar;
import drawText;

export void fontLoader()
{
    if (option::language == L"Korean")
    {
        for (int i = 8; i < MAX_FONT_SIZE; i++)
        {
            notoSansFont[i] = TTF_OpenFont("font/notoSans/NotoSansKR-Regular.ttf", i);
            notoSansBoldFont[i] = TTF_OpenFont("font/notoSans/NotoSansKR-Bold.ttf", i);
            pixelFont[i] = TTF_OpenFont("font/gulim/gulim-Regular.ttc", i);

            mainFont[i] = TTF_OpenFont("font/pretendard/Pretendard-Regular.ttf", i);
            mainFontMedium[i] = TTF_OpenFont("font/pretendard/Pretendard-Medium.ttf", i);
            mainFontBold[i] = TTF_OpenFont("font/pretendard/Pretendard-Bold.ttf", i);
            mainFontSemiBold[i] = TTF_OpenFont("font/pretendard/Pretendard-SemiBold.ttf", i);
            mainFontExtraBold[i] = TTF_OpenFont("font/pretendard/Pretendard-ExtraBold.ttf", i);
        }

        setFont(fontType::mainFont);
    }
    else if (option::language == L"English")
    {
        for (int i = 8; i < MAX_FONT_SIZE; i++)
        {
            notoSansFont[i] = TTF_OpenFont("font/notoSans/NotoSansKR-Regular.ttf", i);
            notoSansBoldFont[i] = TTF_OpenFont("font/notoSans/NotoSansKR-Bold.ttf", i);
            pixelFont[i] = TTF_OpenFont("font/gulim/gulim-Regular.ttc", i);

            mainFont[i] = TTF_OpenFont("font/pretendard/Pretendard-Regular.ttf", i);
            mainFontMedium[i] = TTF_OpenFont("font/pretendard/Pretendard-Medium.ttf", i);
            mainFontBold[i] = TTF_OpenFont("font/pretendard/Pretendard-Bold.ttf", i);
            mainFontSemiBold[i] = TTF_OpenFont("font/pretendard/Pretendard-SemiBold.ttf", i);
            mainFontExtraBold[i] = TTF_OpenFont("font/pretendard/Pretendard-ExtraBold.ttf", i);
        }

        setFont(fontType::mainFont);

    }
    else
    {

    }
}