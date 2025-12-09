module;

#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <sol/sol.hpp>

export module globalVar;

import std;
import util;
import constVar;
import ItemData;
import EntityData;
import SkillData;
import Ani;
import Player;
import Drawable;

/////////////////////////////////////////////////////////////////
export namespace actSet
{
    std::vector<act> null = { act::status, act::equipment, act::profic, act::skill, act::runMode, act::craft, act::wait, act::test, act::quest,act::god, act::sleep };
    std::vector<act> lootPart = { act::pick, act::wield, act::equip, act::eat };
    std::vector<act> vehicle = { act::turnLeft, act::wait, act::turnRight, act::startEngine, act::shiftGear,act::brake, act::accel, act::headlight,act::test };
    std::vector<act> helicopter = { act::collectiveLever, act::wait, act::cyclicLever, act::startEngine, act::rpmLever, act::tailRotorPedalL, act::tailRotorPedalR };
    std::vector<act> train = { act::rpmLever, act::wait, act::brake, act::startEngine, act::shiftGear, act::blank,act::blank };

    std::vector<act> bionicActive = { act::skillActive, act::quickSlot };
    std::vector<act> bionicToggle = { act::skillToggle, act::quickSlot };

    std::vector<act> mutationActive = { act::skillActive, act::quickSlot };
    std::vector<act> mutationToggle = { act::skillToggle, act::quickSlot };
};
export namespace option
{
    std::wstring language = L"English";// 또는 "English"
    input inputMethod = input::mouse; //조작방식 설정
};

export namespace debug
{
    bool chunkLineDraw = false;
    bool noCraftMaterialNeed = true; //활성화할 경우 CRAFT에서 재료없이 조합 가능
    bool printCircuitLog = true; //활성화할 경우 회로 관련 로그들을 출력함
};

export std::vector<std::wstring> sysStr;

export namespace timer
{
    int timer600 = 0;
    int cursorHightlight = 0;
};

/////////////////////////////////////////////////////////////////
export SDL_Window* window;//게임의 메인 윈도우
export SDL_Renderer* renderer;//게임의 메인 렌더러
export SDL_Gamepad* controller; //메인컨트롤러
export sol::state lua; //루아스크립트

export turn turnCycle = turn::playerInput;//0:플레이어 입력_1:플레이어 애니메이션 재생_2:모든 엔티티 AI 작동(하나라도 false 반환시 3으로, 없으면 0으로)_3:엔티티 애니메이션 재생
export bool quit = false;// true일 경우 게임을 종료시킴
export bool stopLog = false; // 로그를 멈춘다. 시간이 지나도 사라지지 않음
export float timeGift = 0; // 유저의 행동에 의해 엔티티들에게 주어지는 시간

export std::vector<EntityData> entityDex; // Entity DB
export std::vector<ItemData> itemDex;// 아이템 DB
export std::vector<SkillData> skillDex; //스킬 DB

export std::vector<std::wstring> itemTooltip;
export std::vector<std::wstring> entityTooltip;

export std::map<std::wstring, void*, std::greater<std::wstring>> StickerList; // 화면에 고정되는 텍스처 객체에 대한 맵

/*******************************************************************************
* 카메라 관련 변수들
 *******************************************************************************/
export int cameraW = 304; // 카메라의 Width
export int cameraH = 244; // 카메라의 Height
export bool cameraFix = true; //카메라를 플레이어에 고정
export int cameraX = 0; // 카메라의 X 좌표(좌측상단)
export int cameraY = 0; // 카메라의 Y 좌표(좌측상단)
/*******************************************************************************/

//여기가 체크포인트
export float zoomScale = 3.0; // 줌 배율, 2.0부터 시작
export SDL_Event event; // SDL 이벤트
export act UIType = act::null; // 0 : 일반, 1 :루팅, 2: 인벤토리
export bool click = false; // 현재 플레이어가 화면을 누르고 있는지 나타냄
export SDL_Point clickTile = { 0,0 }; // 현재 플레이어가 터치하고 있는 타일(그리드 단위 절대좌표)
export Point3 lootTile = { 0,0,0 };
////////////////////////////////////////////////////////////////////////////////////////////////////////
export int detailScroll = 0; //상단 디테일의 스크롤
export SDL_Point clickDownPoint = { 0,0 }; //다운 이벤트를 실행한 좌표
export SDL_Point clickUpPoint = { 0,0 }; //업 이벤트를 실행한 좌표
export SDL_Point clickHoldPoint = { 0,0 }; //홀드 이벤트를 실행한 좌표
export bool deactClickUp = false; //true일 경우 클릭업 및 클릭라이트(업) 함수를 실행하지 않음
export bool itemListColorLock = false; //스크롤 행동시 마우스를 옮겼을 때 버튼들의 색변화 방지
export bool skillListColorLock = false; //스크롤 행동시 마우스를 옮겼을 때 버튼들의 색변화 방지
export std::vector<act> barAct = actSet::null; //하단에 표시되는 행동 리스트
export int dxClickStack = 0; //x 좌표의 이동값
export int dyClickStack = 0; //y 좌표의 이동값
export int dtClickStackStart = 0; //클릭 시간 측정 시작한 시간
export int dtClickStack = -1; //총 시간, 단 측정 중이지 않을 때에는 값이 -1
export __int64 clickStartTime = std::numeric_limits<__int64>::max(); //밀리초로 저장되는 클릭을 시작한 시간
export bool deactHold = false; //홀드이벤트를 비활성화, 예를 들어 카메라를 일정 값만큼 이동했을 경우 홀드 이벤트가 발생하지않음

export int selectMode = 0; // 선택모드 0일 경우 전체선택, 1이면 정밀선택
/////////////////////////////////////////////////////////////////
export bool exInput = false; // exInput(외부 텍스트 입력)가 작동 중인지의 여부
export std::wstring exInputText = L""; // 메시지 박스에 입력한 문자열
export int exInputCursor = 0; // 메시지 박스에서 현재 가리키는 커서
export bool exInputEditing = false; // 현재 입력이 수정 중인지, 예로 한글 완성 전에는 true임, 이 값을 바탕으로 그리는 위치도 변동됨
export int exInputIndex = -1; // -1은 미선택, 0부터 할당, 0은 아이템 선택 숫자 입력
////////////////////////////////////////////////////////////////
//HUD 관련 전역변수
export tabFlag tabType = tabFlag::attackNearby;
export SDL_Rect letterbox = { 0, 0, 0, 0 };
export SDL_Rect barButton[35] = { 0, 0, 0, 0 };
export SDL_Rect letterboxInButton[35] = { 0, 0, 0, 0 };
export SDL_Rect letterboxPopUpButton = { 0, 0, 0, 0 };
export int letterboxPopUpRelY = 0;
export SDL_Rect tab = { 0, 0, 0, 0 };
export SDL_Rect tabSmallBox = { 0, 0, 0, 0 };
export bool doPopUpSingleHUD = false;
export bool doPopDownHUD = false;

export int barActCursor = -1; //키보드 입력 시에 사용되는 레터박스 커서, 비활성은 -1

export SDL_Color mainLightColor = { 0xff,0xff,0xff };
export int mainLightBright = 20;
export int mainLightSight = 0;


/////////////////////코루틴 관련 변수///////////////////////////////
export Corouter* coFunc;
export std::wstring coAnswer = L"";
export bool coTurnSkip = false; //true일 경우 플레이어 턴에 도달시 coFunc에 할당된 코루틴 함수를 실행시킴, 즉 코루틴함수 도중에 턴을 스킵하는 기능을 할 수 있음
/////////////////////////////////////////////////////////////////

//export std::unique_ptr<ItemPocket> availableRecipe;//이게 인텔리센스 오류의 원인

export Vehicle* ctrlVeh = nullptr;
export std::map < dir16, std::unordered_map<Point2, Point2, Point2::Hash>> coordTransform;//좌표변환

export SDL_Rect quickSlotBtn[8];

export Player* PlayerPtr = nullptr;
export float hunger = PLAYER_MAX_CALORIE / 2;
export float thirst = PLAYER_MAX_HYDRATION / 2 + 1000;
export float fatigue = PLAYER_MAX_FATIGUE / 2;

export bool gestureInitialized = false;
export bool isPinchActive = false;
export float pinchAccumulator = 0.0f;
export Point2 pinchStartPos = { 0, 0 };
export int activeTouchCount = 0;
export Point2 touchStartGrid = { 0, 0 };

export namespace dur
{
    __int64 turnCycle = 0;
    __int64 stepEvent = 0;
    __int64 renderTile = 0;
    __int64 renderWeather = 0;
    __int64 renderSticker = 0;
    __int64 renderUI = 0;
    __int64 renderLog = 0;

    __int64 analysis = 0;
    __int64 tile = 0;
    __int64 corpse = 0;
    __int64 floorProp = 0;
    __int64 item = 0;
    __int64 entity = 0;
    __int64 damage = 0;
    __int64 bullet = 0;
    __int64 particle = 0;
    __int64 mulFog = 0;
    __int64 fog = 0;
    __int64 marker = 0;
    __int64 debug = 0;

    __int64 totalDelay = 0;
}

export int extraCameraLength = 0;

export godFlag playerGod = godFlag::none;
export int godPiety = 0;

//화면 내에 있지않아도 여기에 추가될 경우 반드시 그려짐
export std::vector<Drawable*> extraRenderVehList;
export std::vector<Drawable*> extraRenderEntityList;

export std::array<std::pair<quickSlotFlag, int>, 8> quickSlot = { std::pair(quickSlotFlag::NONE , -1), };

export SDL_Rect quickSlotRegion;
export SDL_Rect minimapRegion;

export int prevMouseX4Motion, prevMouseY4Motion = 0; //마우스모션에 대해 원래 마우스 클릭좌표, 기존 클릭좌표랑은 조금 다르니 유의할 것, 카메라 이동에 사용됨

export ThreadPool* threadPoolPtr;

export Point3 gamepadWhiteMarker = { std::numeric_limits<int>::max(), std::numeric_limits<int>::max(),std::numeric_limits<int>::max() }; //게임패드 방향을 가리키는 마커
export bool isPlayerMoving = false; //플레이어가 aStar를 따라서 움직이고 있는지.. 마우스의 whiteMarker 표시 여부를 바꿈
export std::vector<Point2> aStarTrail; //플레이어의 aStar로 생기는 궤적

export int dpadDelay = 0; //상하좌우키 연속입력 딜레이(꾹 누르는경우 여러번 입력되게..)
export int delayR2 = 0;

export Point2 contextMenuTargetGrid = { 0,0 }; //컨텍스트메뉴가 열렸을때 커서위치(컨메뉴가 존재하는지 확인하고 쓸 것)

export bool drawHUD = true;

export int currentUsingSkill = -1;

export std::unordered_set<Point2, Point2::Hash> rangeSet; //선택 범위를 나타내는 좌표들(스킬이나 건축 범위)

export SDL_Color rangeColor = { 0xff, 0xff, 0xff }; //선택 범위를 나타내는 색상
export bool rangeRay = false;

export std::unordered_set<Prop*> finalLoadSet; //전자회로 연산 전부 끝난 후 최종적으로 처리되는 부하들

export class Snowflake
{
public:
    int x = -50;
    int y = -50;
    int dstX = 0;
    int dstY = 0;
    unsigned __int8 alpha = 255;
    int size = 2;


    Snowflake(int inputDstX, int inputDstY, int inputSize)
    {
        dstX = inputDstX;
        dstY = inputDstY;
        x = inputDstX;
        y = inputDstY - 150;
        size = inputSize;
        alpha = randomRange(120, 255);
    };
};

export class Raindrop
{
public:
    int x = -50;
    int y = -50;
    int dstY = 0;
    int alpha = 255;
    double length = 20.0;
    double angle = 82.0 * (3.141592 / 180.0);
    double velocity = 20.0;

    Raindrop(int inputDstX, int inputDstY)
    {
        dstY = inputDstY;
        x = inputDstX;
        y = inputDstY - 150;
        alpha = randomRange(50, 200);
    };
};

export class Spatter
{
public:
    SDL_Color col = { 0xff,0xff,0xff };
    int lifetime = 30; //스텝
    int x = 0;
    int y = 0;
    double veloX = 1.0;
    double veloY = 1.0;
    double grad = 0.3;
};

export std::vector<std::unique_ptr<Snowflake>> snowflakes;
export std::vector<std::unique_ptr<Raindrop>> raindrops;
export std::vector<std::unique_ptr<Spatter>> spatters;

//트랜지스터의 연결 상태가 변경되었을 경우 다음 updateCircuitNetwork에서 이 컨테이너들로 BFS를 다시 돌림
//만약 두 컨테이너가 nullptr이면은 세이브 데이터가 없는 것임
export std::queue<Point3> saveFrontierQueue;
export std::unordered_set<Point3, Point3::Hash> saveVisitedSet;
export bool undoCircuitNetwork = false; //트랜지스터의 상태가 꺼져서 기존에 있었던 업데이트를 무효화시켜야 할 경우 true



/////////////////////////////////////////////전역함수////////////////////////////////////////////////////////////

export std::set<Ani*, bool(*)(Ani*, Ani*)> aniUSet(
    [](Ani* a, Ani* b) -> bool {
        if (a->getAniPriority() == b->getAniPriority())
            return a < b;
        else
            return a->getAniPriority() > b->getAniPriority();
    }
);
//AniUSet에 애니메이션을 추가한다. 단 턴을 넘기지는 않는다. 몬스터의 경우 모든 AI에서 실행 후 자동으로 턴이 넘어가므로...
export std::function<void(Ani*, aniFlag)> addAniUSet = [](Ani* tgtPtr, aniFlag inputType)
    {
        aniUSet.insert(tgtPtr);
        tgtPtr->setAniType(inputType);
    };

//AniUSet에 애니메이션을 추가한다. 플레이어의 입력턴을 강제로 종료하고 플레이어 애니메이션으로 넘어간다.
export std::function<void(Ani*, aniFlag)> addAniUSetPlayer = [](Ani* tgtPtr, aniFlag inputType)
    {
        addAniUSet(tgtPtr, inputType);
        turnCycle = turn::playerAnime;
    };

//AniUSet에 애니메이션을 추가한다. 몬스터의 연산턴을 강제로 종료하고 플레이어 애니메이션으로 넘어간다.
export std::function<void(Ani*, aniFlag)> addAniUSetMonster = [](Ani* tgtPtr, aniFlag inputType)
    {
        addAniUSet(tgtPtr, inputType);
        turnCycle = turn::monsterAnime;
    };


