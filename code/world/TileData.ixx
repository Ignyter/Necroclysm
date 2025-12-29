#include <SDL3/sdl.h>

export module TileData;

import std;
import util;
import ItemData;
import globalVar;
import constVar;

import Flame;
import Prop;
import ItemStack;
import Entity;
import gasData;
//__int8 : -128~127
//__int16 : -32768 ~32767
//__int32 : –2,147,483,648 ~2,147,483,647

export struct TileData //총용량 29바이트
{
    std::vector<gasData> gasVec;

    unsigned __int16 floor = 1;
    unsigned __int16 wall = 0;
    unsigned __int16 ceil = 0;

    __int16 temperature = 25;//섭씨온도
    unsigned __int8 moisture = 30; //수분(0~100)
    unsigned __int8 fertility = 30; //비옥도(0~100)
    unsigned __int16 plantCounter = 0; // 0 ~  65535 비옥도나 성장환경에 따라 가산되는 카운터(얼마나 잘 성장하는 중인지)
    unsigned __int16 plantDay = 0; //심은지 지난 날짜 0~65535


    //시야 관련
    bool blocker = false;
    fovFlag fov = fovFlag::black;

    //이동 관련
    bool walkable = true;

    __int16 wallHP = 100;
    __int16 wallFakeHP = 100;
    __int16 wallMaxHP = 100;
    int displayHPBarCount = 0; //양수 200으로 설정시 점점 떨어지다가 1이 되면 alpha를  대신 줄임. alpha마저 모두 줄면 0으로
    int alphaHPBar = 0;
    int alphaFakeHPBar = 0;

    std::unique_ptr<Entity> EntityPtr = nullptr;
    std::unique_ptr<ItemStack> ItemStackPtr = nullptr;
    std::unique_ptr<Prop> PropPtr = nullptr;
    std::unique_ptr<Flame> flamePtr = nullptr;
    void* VehiclePtr = nullptr;

    std::vector<SDL_Color> lightVec;

    unsigned __int16 randomVal = 0;

    bool hasSnow = false;

    TileData() { randomVal = randomRange(0, 65535); }
    void destoryWall() { wall = 0; }
    void setWall(int inputIndex) { wall = inputIndex; }
    void setFloor(int inputIndex) { floor = inputIndex; }
    void setCeil(int inputIndex) { ceil = inputIndex; }

    int checkGas(int inputCode)
    {
        for (int i = 0; i < gasVec.size(); i++) 
        {
            if (gasVec[i].gasCode == inputCode) return i;

            if (i == gasVec.size() - 1) return -1;
        }
        return -1;
    }
};

//식물 성장
//1. 비료 2. 수분 3. 온도
//addMap이나 setMap addTileBlueLight나 세분 함수로 전부 분류할 것
//레이드아머는 설치물로 하는게 좋지않아? 그러게
//설치물로 하면 레이드아머의 주소를 가리키는건 어떻게 하게?
//그건 다른 설치물들이 생기면 그 설치물의 정보를 나타내는 그런게 생길텐데 거기서 고려해야되지않음? 흠 맞는말이군