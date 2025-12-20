module;

#include <SDL3/SDL.h>

export module constVar;

import std;

export constexpr int DARK_VISION_RADIUS = 13;

export constexpr double MAX_ZOOM = 5.0;
export constexpr double MIN_ZOOM = 2.0;

export constexpr int CHUNK_SIZE_X = 13; //청크의 x길이
export constexpr int CHUNK_SIZE_Y = 13; //청크의 y길이
export constexpr int CHUNK_SIZE_Z = 1; //청크의 z길이, 현재 1로 미사용됨
export constexpr int MAX_FONT_SIZE = 50;
export constexpr int EX_INPUT_TEXT_MAX = 30;
export constexpr int EQUIP_ITEM_MAX = 10;
export constexpr int LOOT_ITEM_MAX = 9;
export constexpr int INVENTORY_ITEM_MAX = 9;
export constexpr int DMG_FLAG_SIZE = 3;
export constexpr int TALENT_SIZE = 20;

// 13칸이 한 청크니까 걸칠 수 있는 정도로, 차량의 왼쪽끝(코어)이 청크의 오른쪽끝 그러면... 7청크*7청크 분석해야한다.
// ●|○○○○○○○○○○○○○|○○○○○○○○○○○○○|○○○○×××××××××|
export constexpr int MAX_VEHICLE_SIZE = 31;


export constexpr int CHUNK_LOADING_RANGE = 5;
export constexpr int MINIMAP_DIAMETER = 41; //미니맵의 지름 (홀수)
export constexpr int NAVIMAP_WIDTH = 99;//167;
export constexpr int NAVIMAP_HEIGHT = 58;//99;
export constexpr int SECTOR_SIZE = 400; // 절차적 맵 생성할 때 작용하는 범위
export constexpr int TOLERANCE_LSTICK = 10000; //LStick이 이 값을 넘어야 판정이 일어남
export constexpr int TOLERANCE_HOLD_DEL_XY = 20; //이 값 이상 움직일 경우 홀드 이벤트가 일어나지 않음

export constexpr int MARKER_LIMIT_DIST = 100; //플레이어로부터 100칸 이상 떨어지면 마커가 그려지지 않음

export constexpr int MAX_PROFIC_LEVEL = 27;

export constexpr int SKILL_GUI_MAX = 7;
export constexpr int QUICK_SLOT_MAX = 8;

export constexpr int CRAFT_MAX_ROW = 4;

export constexpr int MAX_ENC = 10; //최대 방해도
export constexpr int PART_MAX_HP = 100;

export constexpr int START_YEAR = 2099;
export constexpr int START_MONTH = 7;
export constexpr int START_DAY = 15;
export constexpr int START_HOUR = 12;
export constexpr int START_MINUTE = 0;

export constexpr std::array<int, 27> expTable =
{ 50, 100, 150, 200, 250, 300, 350, 400, 450,
550, 650, 750, 850, 950, 1050, 1150, 1250, 1350,
1500, 1650, 1800, 1950, 2100, 2250, 2400, 2550, 2750 };

export constexpr int CHAR_TEXTURE_WIDTH = 288;
export constexpr int CHAR_TEXTURE_HEIGHT = 384;


export constexpr int PLAYER_MAX_CALORIE = 17280;//10일 버팀
export constexpr int PLAYER_HUNGRY_CALORIE = 4320 * 3;
export constexpr int PLAYER_VERY_HUNGRY_CALORIE = 4320 * 2;
export constexpr int PLAYER_STARVE_CALORIE = 4320;

export constexpr float HUNGRY_SPPED = 1.2;


export constexpr int PLAYER_MAX_HYDRATION = 8640;//4일 버팀
export constexpr int PLAYER_THIRSTY_HYDRATION= 2160*3;
export constexpr int PLAYER_VERY_THIRSTY_HYDRATION = 2160 * 2;
export constexpr int PLAYER_DEHYDRATION_HYDRATION = 2160 * 1;

export constexpr float THIRST_SPEED = 1.5;


export constexpr int PLAYER_MAX_FATIGUE = 4320;//최대 3일 안 자고 버팀
export constexpr int PLAYER_TIRED_FATIGUE = 3360;//16시간 지나면 피곤해짐
export constexpr int PLAYER_VERY_TIRED_FATIGUE = 2400;
export constexpr int PLAYER_EXHAUSTED_FATIGUE = 1440;

export constexpr float FATIGUE_SPEED = 1.0;

export constexpr int MAX_BATCH = 4096;

export constexpr int MAX_CIRCUIT_LOOP_COUNT = 50; //전자회로가 트랜지스터 게이트로 인해 중복 업데이트될 때의 한계값



export namespace mulCol
{
    constexpr SDL_Color day = { 255,255,255,255 };
    constexpr SDL_Color dawn = { 0,0,100,30 };
    constexpr SDL_Color sunfall = { 121,78,59,100 };
    constexpr SDL_Color night = { 0,0,100,100 };
}

export namespace col
{
    constexpr SDL_Color black = { 0x00, 0x00, 0x00 };
    constexpr SDL_Color yellow = { 0xff,0xff,0x00 };
    constexpr SDL_Color brown = { 0x5c,0x33,0x17 };
    constexpr SDL_Color gray = { 0x63,0x63,0x63 };
    constexpr SDL_Color green = { 0x00,0x6e,0x00 };
    constexpr SDL_Color blueberry = { 0x64,0x64,0xff };
    constexpr SDL_Color red = { 0xf9,0x29,0x29 };
    constexpr SDL_Color white = { 0xff,0xff,0xff };
    constexpr SDL_Color lightGray = { 0x96,0x96,0x96 };
    constexpr SDL_Color blue = { 0x21,0x4a,0xea };
    constexpr SDL_Color yellowGreen = { 0x3a, 0xf5, 0x43 };
    constexpr SDL_Color monaLisa = { 0xff,0x96,0x96 };
    constexpr SDL_Color bondiBlue = { 0x00,0x96,0xb4 };
    constexpr SDL_Color hotPink = { 0x8b,0x3a,0x62 };
    constexpr SDL_Color pink = { 0xfe,0x00,0xfe };
    constexpr SDL_Color skyBlue = { 0x00,0xf0,0xff };
    constexpr SDL_Color blueDart = { 0x4e,0x8e,0xd2 };
    constexpr SDL_Color orange = { 0xf2, 0x65, 0x22 };
    constexpr SDL_Color cyan = { 0x00,0xa3,0xd2 };
};

export namespace lowCol
{
    constexpr SDL_Color black = { 0x00,0x00,0x00 };
    constexpr SDL_Color white = { 0xff,0xff,0xff };
    constexpr SDL_Color red = { 0xd0,0x3f,0x3f };
    constexpr SDL_Color orange = { 0xd0,0x7a,0x3f };
    constexpr SDL_Color yellow = { 0xd0,0xc3,0x3f };
    constexpr SDL_Color green = { 0x75,0xd0,0x3f };
    constexpr SDL_Color mint = { 0x3f,0xd0,0x7f };
    constexpr SDL_Color skyBlue = { 0x3f,0xba,0xd0 };
    constexpr SDL_Color deepBlue = { 0x20,0x50,0xa8 };
    constexpr SDL_Color blue = { 0x2b,0x81,0xe8 };
    constexpr SDL_Color purple = { 0x43,0x3e,0x8e };
    constexpr SDL_Color pink = { 0xbe,0x3f,0xd0 };
    constexpr SDL_Color crimson = { 0xd0,0x3f,0x89 };
};

export enum class act
{
    null,       //일반 게임플레이
    blank,      //빈칸
    status,     //상태창
    ability,    //특수능력
    equipment,  //인벤토리
    bionic,     //바이오닉
    profic,      //재능
    runMode,    //달리기 모드
    skill,      //스킬
    quest,      //퀘스트
    menu,       //메뉴

    identify,   //감정
    vehicle,    //차량
    god, //신앙
    map, //지도

    closeDoor, //문닫기


    armor,  //레이드아머
    cooking,    //요리
    loot,       //줍기
    pick,       //넣기(가방에)
    wield,      //들기
    equip,      //장비
    pickSelect, //줍기(선택)
    selectAll,  //선택(전부) 필요없음
    searching,  //검색
    sorting,    //정렬
    select,     //선택
    eat,        //먹기
    drink,     //마시기
    apply,      //사용
    selectMode, //선택 모드
    droping,    //버리기
    throwing,   //던지기
    dirSelect,  //방향선택
    coordSelect,//위치선택
    mutation,   //돌연변이
    craft,      //조합
    construct,  //건축
    open,       //가방열기
    test,       //테스트 기능
    insert,     //아이템 넣기(가방에 옮기기)
    reload,     //아이템 장전
    reloadBulletToMagazine, //탄환 장전
    unloadBulletFromMagazine, //탄환 분리
    reloadMagazine, //탄창 장전, 총과 탄창 양쪽에 있으며 기능이 다름
    unloadMagazine, //탄창 분리
    reloadBulletToGun, //탄환 장전
    unloadBulletFromGun, //탄환 분리

    turnLeft,//좌회전
    wait,//1턴대기
    turnRight,//우회전
    startEngine,//엔신시동
    stopEngine,//엔진끄기
    shiftGear,//기어변경
    accel,//액셀
    brake,//브레이크
    autoGear,//자동변속기

    collectiveLever,
    cyclicLever,
    rpmLever,
    tailRotorPedalL,
    tailRotorPedalR,

    confirm,//확인
    rotate,//회전
    cancel,//취소

    quickSlot,
    skillActive,
    skillToggle,

    inspect,
    unbox,
    pull,

    climb,
    swim,
    ride,

    phone,
    message,
    camera,
    internet,

    settings,
    saveAndQuit,

    toggleOn,
    toggleOff,

    vehicleRepair,
    vehicleDetach,

    headlight,

    drawLiquid,

    sleep,

    drinkFloorWater, //바닥물 마시기

    dump, //쏟기 (포켓 내부에 있는 아이템을 바닥에 쏟음)

    propCarry, //프롭 들기
    propInstall, //프롭 설치

    propTurnOn, //프롭 켜기
    propTurnOff, //프롭 끄기

    connectPlusZ, //위층의 타일과 연결
    connectMinusZ, //아래층의 타일과 연결

    toggleCrossCable,
};

export namespace humanCustom
{
    enum class skin
    {
        null,
        white,
        yellow,
        brown,
        black,
        closed,
    };

    enum class eyes
    {
        null,
        closed,
        red,
        redHalf,
        blue,
        blueHalf,
        black,
        blackHalf,
        
    };


    enum class scar
    {
        null,
        darkCircles
    };

    enum class beard
    {
        null,
        mustache
    };

    enum class hair
    {
        null,
        commaBlack,
        bob1Black,
        ponytail,
        middlePart,
    };

    enum class horn
    {
        null,
        coverRed,
    };


};

export enum class fovFlag
{
    white,
    black,
    gray
};

export enum class weatherFlag
{
    sunny,
    cloudy,
    rain,
    storm,
    snow,
};



/*
* 파츠는 0번이 메인이고 0번이 0이 되면 사망함
* 데미지를 가하면 1번 이상의 파츠인덱스에 누적되는데 이때 0번에도 같은 데미지가 들어감, 기타 출혈같은 요소로도 HP가 사라짐
*/
export namespace partType
{
    constexpr unsigned __int8 null = 0;
    constexpr unsigned __int8 head = 1;
    constexpr unsigned __int8 torso = 2;
    constexpr unsigned __int8 larm = 3;
    constexpr unsigned __int8 rarm = 4;
    constexpr unsigned __int8 lleg = 5;
    constexpr unsigned __int8 rleg = 6;
    constexpr unsigned __int8 tail = 7;

    constexpr unsigned __int8 turret = 8;
    constexpr unsigned __int8 camera = 9;
    constexpr unsigned __int8 body = 10;
    constexpr unsigned __int8 lCaterpillar = 11;
    constexpr unsigned __int8 rCaterpillar = 12;
};

export namespace wound
{
    constexpr int pierce = 0; //관통상
    constexpr int cut = 1; //절단상
    constexpr int bash = 2; //타박상
    constexpr int fracture = 3; //골절 : 피격시 타박상 포인트가 높을 경우 발생, 부목으로만 치료 가능 20% 고정 손실
    constexpr int burn = 4; //화상 : 치료 불가능, 의사에게 가야함
    constexpr int frostbite = 5; //동상 : 따뜻한 곳에 가면 아주 조금씩 재생됨
    constexpr int shock = 6; //쇼크 : 정신적인 충격, 머리에만 발생함
};

export namespace proficFlag
{
    constexpr int fighting = 0;
    constexpr int dodging = 1;
    constexpr int stealth = 2;
    constexpr int throwing = 3;
    constexpr int unarmedCombat = 4;
    constexpr int piercingWeapon = 5;
    constexpr int cuttingWeapon = 6;
    constexpr int bashingWeapon = 7;
    constexpr int archery = 8;
    constexpr int gun = 9;
    constexpr int electronics = 10;
    constexpr int chemistry = 11;
    constexpr int mechanics = 12;
    constexpr int computer = 13;
    constexpr int medicine = 14;
    constexpr int cooking = 15;
    constexpr int fabrication = 16;
    constexpr int social = 17;
    constexpr int architecture = 18;
    constexpr int invocations = 19;
};

export enum class msgFlag
{
    deact,
    normal,
    input,
};

export enum class itemFlag
{
    TWOHANDED,
    CANEQUIP,
    NONSTACK,
    GUN,
    MAGAZINE,
    AMMO,
    BOOKMARK1,
    BOOKMARK2,
    BOOKMARK3,
    BOOKMARK4,
    BOOKMARK5,
    BOOKMARK6,
    POWERED,//동력공구
    VFRAME,//차량프레임 : 설치 가능
    RAIDARMOR,//아머프레임 : 설치 가능
    WHITEFILTER,
    GRAYFILTER,//검색 및 조합 GUI에서 아이템을 회색으로 표시하게 만듬
    BLACKFILTER,//조합 GUI에서 미해당 아이템을 검은색으로 표시하게 만듬(미표시)
    COORDCRAFT,//좌표조합, 건축물이나 차량 등
    LIQUID, //액체 상
    GAS, // 기체 상
    VPART, //차량부품
    TRANSPARENT_WALL, //투명벽

    VPART_WALL_CONNECT,//차벽같이 주변에 연결되는 16타일
    VPART_DIR_DEPEND,//방향에 의존하여 16방향이 존재하는 프롭
    VPART_DOOR_OPEN,
    VPART_DOOR_CLOSE,

    VEH_ROOF,

    LIGHT_ON,
    LIGHT_OFF,

    TIRE_NORMAL,
    TIRE_STEER,

    PROP, //설치 가능한 아이템
    PROP_BIG,//48px가 아니라 80px*80px의 giantVehicleset 참조함
    PROP_WALKABLE,//이동가능 프롭
    PROP_BLOCKER,//시야방해 프롭
    PROP_DEPTH_LOWER,//기존 설치물(&엔티티)들과 겹쳐지지 않고 바닥에 깔리는 설치물


    PIPE_CNCT_RIGHT,
    PIPE_CNCT_UP,
    PIPE_CNCT_LEFT,
    PIPE_CNCT_DOWN,

    CABLE_CNCT_RIGHT,
    CABLE_CNCT_UP,
    CABLE_CNCT_LEFT,
    CABLE_CNCT_DOWN,

    CONVEYOR_CNCT_RIGHT,
    CONVEYOR_CNCT_UP,
    CONVEYOR_CNCT_LEFT,
    CONVEYOR_CNCT_DOWN,

    CABLE,
    PIPE,
    CONVEYOR,

    RAIL,
    RAIL_CNCT_UP,
    RAIL_CNCT_DOWN,
    RAIL_CNCT_LEFT,
    RAIL_CNCT_RIGHT,

    //선로전환기에서 고정된 레일
    RAIL_INPUT_UP,
    RAIL_INPUT_DOWN,
    RAIL_INPUT_LEFT,
    RAIL_INPUT_RIGHT,

    RAIL_BUFFER,

    NOT_RECIPE, //조합법에 따로 추가되지 않음(회전된 설치물같은 중복 설치물들)

    /// 여기서부터 새로 추가해야됨
    TREE, //나무
    PLANT, //식물
    PLANT_SEASON_DEPENDENT, //계절의존식물
    PLANT_GROWTH_DEPENDENT, //성장의존식물
    MUSHROOM, //버섯
    FLOOR, //바닥타일
    WALL, //벽
    CEIL, //천장
    WATER_SHALLOW, //얕은물
    WATER_DEEP, //깊은물
    FRESHWATER, //담수
    SEAWATER, //해수
    TILE_SEASON, //계절에 따라 변하는 타일
    DOOR,
    UPSTAIR,
    DOWNSTAIR,
    SIGN,//나무표지판같이 문구를 저장하는 객체


    DOOR_CLOSE,
    DOOR_OPEN,

    TRAIN_WHEEL, //열차바퀴

    PROP_GAS_OBSTACLE_ON,
    PROP_GAS_OBSTACLE_OFF,
    WALL_GAS_PERMEABLE,

    POCKET,
    CAN_CLIMB,

    SPR_TH_WEAPON,
    NO_HAIR_HELMET,//머리카락이 안보이는 헬멧

    BOW,
    CROSSBOW,

    STUMP,

    TOGGLE_ON,
    TOGGLE_OFF,
    HAS_TOGGLE_SPRITE,

    CANCRAFT,//조합할 수 있는 아이템인지

    HEADLIGHT,
    SHIELD,
    CONTAINER_LIQ,

    VPART_NOT_WALKABLE,

    ENGINE_GASOLINE,
    ENGINE_DIESEL,
    ENGINE_ELECTRIC,

    CAN_EAT,
    CAN_DRINK,

    CONTAINER_FLEX, //내부에 들어있는 아이템에 따라 부피가 더해짐

    BURIED, //땅에 매립된 부품
    
    WIELD_NORMAL_DISPLAY,//이게 있으면 손에 들면 보임(무기같은 이미 이미지가 장비 제외)

    LIQ_COL_RED,
    LIQ_COL_BLUE,
    LIQ_COL_YELLOW,
    LIQ_COL_WHITE,
    LIQ_COL_GRAY,
    LIQ_COL_BLACK,

    CONTAINER_TRANSPARENT,
    CONTAINER_TRANSLUCENT,

    PROP_POWER_OFF,//가솔린 발전기나 무선 장치 등등
    PROP_POWER_ON,
    PROP_NEXT_TURN_POWER_OFF, //택트스위치 다음 턴 종료 플래그

    CIRCUIT, //전자회로 관련 부품

	CABLE_BEHIND, //케이블이 프롭 뒤에 그려짐

    VOLTAGE_SOURCE, //전력 공급원(발전기 등)
    VOLTAGE_OUTPUT_RIGHT,
	VOLTAGE_OUTPUT_UP,
    VOLTAGE_OUTPUT_LEFT,
    VOLTAGE_OUTPUT_DOWN,

    CABLE_Z_ASCEND, //위층의 현재 타일과 연결된 케이블
    CABLE_Z_DESCEND, //아래층의 현재 타일과 연결된 케이블

    HAS_GROUND,

    MULTI_GND, //GND를 여러 방향에 독립적으로 가지고 있는 경우

    TRANSISTOR, //아직 미사용
    LOGIC_GATE, //BFS에 포함되기만 해도 loadSet에 포함되는 플래그(2개 이상의 경로 가짐)

    CROSSED_CABLE,
};

export enum class walkFlag
{
    walk,
    run,
    crouch,
    crawl,
    wade,
    swim,
};

export namespace toolQuality
{
    constexpr int none = 0;
    constexpr int screwDriving = 1;
    constexpr int drilling = 2;
    constexpr int welding = 3;
    constexpr int soldering = 4;
    constexpr int cutting = 5;
    constexpr int sawing = 6;
    constexpr int hammering = 7;
    constexpr int digging = 8;
    constexpr int sewing = 9;
    constexpr int distillation = 10;
    constexpr int boiling = 11;
    constexpr int frying = 12;
    constexpr int roasting = 13;
    constexpr int boltTurning = 14;
    constexpr int woodCutting = 15;
    constexpr int metalCutting = 16;
    constexpr int metalDrilling = 17;
    constexpr int refrigeration = 18;
    constexpr int heating = 19;
};

export enum class itemAct
{
    pick = 0,
    wield = 1,
    drop = 2,
    pitch = 3,

    wear = 4,
    takeOff = 5,
    repair = 6,
    disasm = 7,

    apply = 8,
    eat = 9,
    read = 10,
    reload = 11,
    unload = 12,
    open = 13,
};


export enum class playerSpriteCategory
{
    oneHanded = 0,
    twoHanded = 2,
    sitOneHanded = 4,
    sitTwoHanded = 6,
    runOneHanded = 8,
    runTwoHanded = 10,
};

export enum class btn
{
    keypad6 = 0,
    keypad9 = 1,
    keypad8 = 2,
    keypad7 = 3,
    keypad4 = 4,
    keypad1 = 5,
    keypad2 = 6,
    keypad3 = 7,
    keypad5 = 8,
    a = 9,
    b = 10,
    x = 11,
    y = 12,
    l1 = 13,
    r1 = 14,
    l2 = 15,
    r2 = 16,
};



export enum class storageType
{
    null,//임시용
    equip,//장비
    stack,//스택
    pocket,//아이템의 내부 주머니
    temp,//임시용
    recipe,//레시피(플레이어용)
};

export enum class aniFlag
{
    null,
    move,
    atk,
    felling,
    miningWall,
    diggingWall,
    drop,
    winUnfoldOpen,
    winUnfoldClose,
    winSlipOpen,
    winSlipClose,
    throwing,
    popUpLetterbox, //GUI에 존재하는 HUD 전용 애니메이션
    popDownLetterbox,
    popUpSingleLetterbox,

    shotSingle,

    shotBurst,
    shotAuto,
    shotArrow,
    propRush,
    minecartRush,
    quickSlotPopLeft,
    quickSlotPopRight,

    fireStorm,
    roll,
    leap,

    treeFalling,

    entityThrow,

    faint,
    dropInventory,

    propTurnOnOff, //프롭 켜기/끄기
    changePropDelay, //딜레이 조정
};

export enum class input
{
    mouse,
    touch,
    gamepad
};

export enum class msgOption
{
    okay,
    cancel,
    yes,
    no,
    confirm,
    left,
    right
};

export enum class tabFlag
{
    attackNearby,
    closeWin,
    back,
    confirm,
    aim,

};

export enum class sortFlag
{
    null,
    weightDescend,
    weightAscend,
    volumeDescend,
    volumeAscend,
    equip,
};



export enum class equipHandFlag
{
    none,
    normal,
    left,
    right,
    both,
};

export enum class turn
{
    playerInput,
    playerAnime,
    VehicleAI,
    VehicleAnime,
    monsterAI,
    monsterAnime,
};

export namespace keyboardIndex
{
    constexpr int tab = 1;
    constexpr int tabPressed = 2;
    constexpr int m = 3;
    constexpr int mPressed = 4;
    constexpr int enter = 5;
    constexpr int enterPressed = 6;
}

export namespace keyIcon
{
    constexpr int blankRect = 0;

    constexpr int keyboard_1 = 1;
    constexpr int keyboard_2 = 2;
    constexpr int keyboard_3 = 3;
    constexpr int keyboard_4 = 4;
    constexpr int keyboard_5 = 5;
    constexpr int keyboard_6 = 6;
    constexpr int keyboard_7 = 7;
    constexpr int keyboard_8 = 8;
    constexpr int keyboard_9 = 9;
    constexpr int keyboard_0 = 10;

    constexpr int keyboard_Num1 = 11;
    constexpr int keyboard_Num2 = 12;
    constexpr int keyboard_Num3 = 13;
    constexpr int keyboard_Num4 = 14;
    constexpr int keyboard_Num5 = 15;
    constexpr int keyboard_Num6 = 16;
    constexpr int keyboard_Num7 = 17;
    constexpr int keyboard_Num8 = 18;
    constexpr int keyboard_Num9 = 19;
    constexpr int keyboard_Num0 = 20;

    constexpr int keyboard_F1 = 21;
    constexpr int keyboard_F2 = 22;
    constexpr int keyboard_F3 = 23;
    constexpr int keyboard_F4 = 24;
    constexpr int keyboard_F5 = 25;
    constexpr int keyboard_F6 = 26;
    constexpr int keyboard_F7 = 27;
    constexpr int keyboard_F8 = 28;
    constexpr int keyboard_F9 = 29;
    constexpr int keyboard_F10 = 30;
    constexpr int keyboard_F11 = 31;
    constexpr int keyboard_F12 = 32;

    constexpr int keyboard_H = 33;
    constexpr int keyboard_J = 34;
    constexpr int keyboard_K = 35;
    constexpr int keyboard_L = 36;
    constexpr int keyboard_Y = 37;
    constexpr int keyboard_U = 38;
    constexpr int keyboard_B = 39;
    constexpr int keyboard_N = 40;
    constexpr int keyboard_A = 41;
    constexpr int keyboard_S = 42;
    constexpr int keyboard_Z = 43;
    constexpr int keyboard_X = 44;

    constexpr int keyboard_Tab = 45;
    constexpr int keyboard_Enter = 46;
    constexpr int keyboard_PgUp = 47;
    constexpr int keyboard_PgDn = 48;

    constexpr int pad_X = 49;
    constexpr int pad_Y = 50;
    constexpr int pad_A = 51;
    constexpr int pad_B = 52;
    constexpr int pad_L = 53;
    constexpr int pad_R = 54;
    constexpr int pad_LStick = 55;
    constexpr int pad_ZL = 56;
    constexpr int pad_ZR = 57;

    constexpr int pad_Right = 58;
    constexpr int pad_Up = 59;
    constexpr int pad_Left = 60;
    constexpr int pad_Down = 61;

    constexpr int keyboard_C = 62;
    constexpr int keyboard_V = 63;

    constexpr int keyboard_LShift = 64;
    constexpr int keyboard_RShift = 65;
    constexpr int keyboard_M = 66;

    constexpr int duelSense_L1 = 80;
    constexpr int duelSense_L2 = 81;
    constexpr int duelSense_R1 = 82;
    constexpr int duelSense_R2 = 83;

    constexpr int duelSense_X = 84;
    constexpr int duelSense_CIR = 85;
    constexpr int duelSense_RECT = 86;
    constexpr int duelSense_TRI = 87;
    constexpr int duelSense_LStick = 88;
    constexpr int duelSense_RStick = 89;

    constexpr int duelSense_RIGHT = 90;
    constexpr int duelSense_UP = 91;
    constexpr int duelSense_LEFT = 92;
    constexpr int duelSense_DOWN = 93;
    constexpr int duelSense_OPTIONS = 94;
    constexpr int duelSense_SHARE = 95;

    ////////

    constexpr int joyCon_L = 96;
    constexpr int joyCon_ZL = 97;
    constexpr int joyCon_R = 98;
    constexpr int joyCon_ZR = 99;

    constexpr int joyCon_A = 100;
    constexpr int joyCon_B = 101;
    constexpr int joyCon_X = 102;
    constexpr int joyCon_Y = 103;
    constexpr int joyCon_LStick = 104;
    constexpr int joyCon_RStick = 105;

    constexpr int joyCon_RIGHT = 106;
    constexpr int joyCon_UP = 107;
    constexpr int joyCon_LEFT = 108;
    constexpr int joyCon_DOWN = 109;
    constexpr int joyCon_PLUS = 110;
    constexpr int joycon_MINUS = 111;


};

export namespace sprInf
{
    constexpr int walk = 0;
    constexpr int run = 6;
    constexpr int sit = 12;
    constexpr int crawl = 18;
};

export enum class itemCategory
{
    equipment,
    foods,
    tools,
    tech,
    consumables,
    vehicles,
    structures,
    materials,
};

export enum class itemSubcategory
{
    equipment_melee,
    equipment_ranged,
    equipment_firearms,
    equipment_throwing,
    equipment_clothing,

    foods_cooked,       // 요리 (김치찌개, 스테이크, 스파게티, 케밥 등)
    foods_processed,    // 가공식품 (편의점 샌드위치, 빵 등)
    foods_preserved,    // 보존식품 (갈치통조림, 건빵 등)
    foods_drinks,       // 음료 (물, 콜라)
    foods_ingredients,  // 재료 (달걀, 식용유, 소금 등)

    consumable_medicine,
    consumable_ammo,
    consumable_fuel,
    consumable_etc,

    tools_hand,          // 수공구 (망치, 톱 등)
    tools_power,         // 전동공구 (전동드릴, 그라인더 등)
    tools_containers,    // 컨테이너 (배낭, 비커 등)
    tools_etc,          // 기타 도구 (나침반, 지도 등)

    tech_bionics,        // 바이오닉
    tech_powerArmor,    // 파워아머

    vehicle_frames,//프레임
    vehicle_power,//파워 : 가솔린, 디젤, 전기모터
    vehicle_exteriors,//외장 : 바퀴, 차문, 트렁크문
    vehicle_parts,//부품 : 기타

    structure_walls,
    structure_floors,
    structure_ceilings,
    structure_props,

    material_metals,       // 금속류 (철, 납, 아연 등)
    material_organic,      // 유기물 (가죽, 나무, 시체 등)
    material_components,   // 부품류 (파이프, 전선, 회로 등)
    material_chemicals,    // 화학물질 (산, 연료, 화약 등)
    material_etc,
};

export enum class godFlag
{
    none,
    teshub,
    buddha,
    jesus,
    amaterasu,
    yudi,
    ra,
};

export namespace entityCategory
{
    constexpr int none = 0;
    constexpr int human = 1;
    constexpr int zombie = 2;
    constexpr int robot = 3;
    constexpr int animal = 4;
};

export namespace bulletFlag
{
    constexpr int normal = 0;
    constexpr int tracer = 1;
    constexpr int ap = 2;
};

export namespace weaponMode
{
    constexpr int none = 0;
    constexpr int safeMode = 1;
    constexpr int semiAutoMode = 2;
    constexpr int burstMode = 3;
    constexpr int autoMode = 4;
};

//입은 데미지의 종류
export enum class dmgFlag
{
    none,
    pierce,
    cut,
    bash,
    fire,
    ice,
    elec,
    corr,
    rad
};

export enum class humanPartFlag
{
    head,
    torso,
    lArm,
    rArm,
    lLeg,
    rLeg,
};

//플레이어가 취할 수 있는 공격의 타입, Aim의 5가지 종류(관통, 절단, 타격, 사격, 투척)
export enum class atkType
{
    pierce,
    cut,
    bash,
    shot,
    throwing,
};

export namespace UNI
{
    constexpr int NUL = 0;   // Null char
    constexpr int SOH = 1;   // Start of Heading
    constexpr int STX = 2;   // Start of Text
    constexpr int ETX = 3;   // End of Text
    constexpr int EOT = 4;   // End of Transmission
    constexpr int ENQ = 5;   // Enquiry
    constexpr int ACK = 6;   // Acknowledgment
    constexpr int BEL = 7;   // Bell
    constexpr int BACKSPACE = 8;    // Back Space
    constexpr int TAB = 9;   // Horizontal Tab
    constexpr int LF = 10;   // Line Feed
    constexpr int VT = 11;   // Vertical Tab
    constexpr int FF = 12;   // Form Feed
    constexpr int CR = 13;   // Carriage Return
    constexpr int SO = 14;   // Shift Out / X-On
    constexpr int SI = 15;   // Shift In / X-Off
    constexpr int DLE = 16;  // Data Line Escape
    constexpr int DC1 = 17;  // Device Control 1 (oft. XON)
    constexpr int DC2 = 18;  // Device Control 2
    constexpr int DC3 = 19;  // Device Control 3 (oft. XOFF)
    constexpr int DC4 = 20;  // Device Control 4
    constexpr int NAK = 21;  // Negative Acknowledgement
    constexpr int SYN = 22;  // Synchronous Idle
    constexpr int ETB = 23;  // End of Transmit Block
    constexpr int CAN = 24;  // Cancel
    constexpr int EM = 25;   // End of Medium
    constexpr int SUB = 26;  // Substitute
    constexpr int ESC = 27;  // Escape
    constexpr int FS = 28;   // File Separator
    constexpr int GS = 29;   // Group Separator
    constexpr int RS = 30;   // Record Separator
    constexpr int US = 31;   // Unit Separator
    constexpr int SPACE = 32;
    constexpr int EXCLAMATION_MARK = 33;  // '!'
    constexpr int DOUBLE_QUOTES = 34;  // '"'
    constexpr int HASH = 35;  // '#'
    constexpr int DOLLAR_SIGN = 36;  // '$'
    constexpr int PERCENT_SIGN = 37;  // '%'
    constexpr int AMPERSAND = 38;  // '&'
    constexpr int APOSTROPHE = 39;  // '''
    constexpr int LEFT_PARENTHESIS = 40;  // '('
    constexpr int RIGHT_PARENTHESIS = 41;  // ')'
    constexpr int ASTERISK = 42;  // '*'
    constexpr int PLUS_SIGN = 43;  // '+'
    constexpr int COMMA = 44;  // ','
    constexpr int MINUS_SIGN = 45;  // '-'
    constexpr int PERIOD = 46;  // '.'
    constexpr int SLASH = 47;  // '/'
    constexpr int ZERO = 48;
    constexpr int ONE = 49;
    constexpr int TWO = 50;
    constexpr int THREE = 51;
    constexpr int FOUR = 52;
    constexpr int FIVE = 53;
    constexpr int SIX = 54;
    constexpr int SEVEN = 55;
    constexpr int EIGHT = 56;
    constexpr int NINE = 57;
    constexpr int COLON = 58;  // ':'
    constexpr int SEMICOLON = 59;  // ';'
    constexpr int LESS_THAN_SIGN = 60;  // '<'
    constexpr int EQUAL_SIGN = 61;  // '='
    constexpr int GREATER_THAN_SIGN = 62;  // '>'
    constexpr int QUESTION_MARK = 63;  // '?'
    constexpr int AT_SIGN = 64;  // '@'
    constexpr int A = 65;
    constexpr int B = 66;
    constexpr int C = 67;
    constexpr int D = 68;
    constexpr int E = 69;
    constexpr int F = 70;
    constexpr int G = 71;
    constexpr int H = 72;
    constexpr int I = 73;
    constexpr int J = 74;
    constexpr int K = 75;
    constexpr int L = 76;
    constexpr int M = 77;
    constexpr int N = 78;
    constexpr int O = 79;
    constexpr int P = 80;
    constexpr int Q = 81;
    constexpr int R = 82;
    constexpr int S = 83;
    constexpr int T = 84;
    constexpr int U = 85;
    constexpr int V = 86;
    constexpr int W = 87;
    constexpr int X = 88;
    constexpr int Y = 89;
    constexpr int Z = 90;
    constexpr int LEFT_SQUARE_BRACKET = 91;  // '['
    constexpr int BACKSLASH = 92;  // '\'
    constexpr int RIGHT_SQUARE_BRACKET = 93;  // ']'
    constexpr int CARET = 94;  // '^'
    constexpr int UNDERSCORE = 95;  // '_'
    constexpr int GRAVE_ACCENT = 96;  // '`'
    constexpr int a = 97;
    constexpr int b = 98;
    constexpr int c = 99;
    constexpr int d = 100;
    constexpr int e = 101;
    constexpr int f = 102;
    constexpr int g = 103;
    constexpr int h = 104;
    constexpr int i = 105;
    constexpr int j = 106;
    constexpr int k = 107;
    constexpr int l = 108;
    constexpr int m = 109;
    constexpr int n = 110;
    constexpr int o = 111;
    constexpr int p = 112;
    constexpr int q = 113;
    constexpr int r = 114;
    constexpr int s = 115;
    constexpr int t = 116;
    constexpr int u = 117;
    constexpr int v = 118;
    constexpr int w = 119;
    constexpr int x = 120;
    constexpr int y = 121;
    constexpr int z = 122;
    constexpr int LEFT_CURLY_BRACKET = 123;  // '{'
    constexpr int VERTICAL_BAR = 124;  // '|'
    constexpr int RIGHT_CURLY_BRACKET = 125;  // '}'
    constexpr int TILDE = 126;  // '~'
    constexpr int DEL = 127;   // Delete
    constexpr int MIDDLE_DOT = 183;
};


export enum class gearFlag
{
    park,
    reverse,
    neutral,
    drive,
};

export enum class chunkFlag
{
    none,
    freshwater,//담수
    seawater,//해수
    meadow,//초원
    underground,//지하
    dirt,//흙
    //도로
    //도시
};

export namespace chunkCol
{
    constexpr SDL_Color seawater = { 0x16,0x21,0xff };
    constexpr SDL_Color river = { 0x9d,0xa2,0xfb };
    constexpr SDL_Color city = { 0xa2,0xa2,0xa2 };
    constexpr SDL_Color land = { 0x59,0xc6,0x82 };
};



export enum class seasonFlag
{
    spring,
    summer,
    autumn,
    winter,
};

export enum class vehFlag
{
    none,
    car,
    heli,
    minecart,
    train,
    ship,
};


export namespace itemRefCode
{
    constexpr int metalFrame = 48;
    constexpr int dirt = 109;
    constexpr int grass = 220;
    constexpr int blackAsphalt = 296;
    constexpr int yellowAsphalt = 377;

    constexpr int railRL = 320;
    constexpr int railTB = 321;
    constexpr int railBR = 322;
    constexpr int railTR = 323;
    constexpr int railTL = 324;
    constexpr int railBL = 325;

    constexpr int water = 71;


    constexpr int railSwitchEN = 326;
    constexpr int railSwitchES = 327;
    constexpr int railSwitchNW = 328;
    constexpr int railSwitchNE = 329;
    constexpr int railSwitchWN = 330;
    constexpr int railSwitchWS = 331;
    constexpr int railSwitchSW = 332;
    constexpr int railSwitchSE = 333;

    constexpr int wideRailHTop = 399;
    constexpr int wideRailHMid = 400;
    constexpr int wideRailHBot = 401;

    constexpr int wideRailVLeft = 402;
    constexpr int wideRailVMid = 403;
    constexpr int wideRailVRight = 404;

    constexpr int shallowFreshWater = 225;
    constexpr int deepFreshWater = 226;

    constexpr int shallowSeaWater = 231;
    constexpr int deepSeaWater = 232;

    constexpr int sandFloor = 381;

    constexpr int pickaxe = 388;
    constexpr int fellingAxe = 391;

    constexpr int dirtWall = 302;
    constexpr int stoneWall = 397;
    constexpr int glassWall = 114;
    constexpr int wireFence = 376;

    constexpr int katana = 103;

    constexpr int minecart = 405;
    constexpr int minecartController = 406;

    constexpr int arrowQuiver = 408;
    constexpr int boltQuiver = 409;

    constexpr int gasoline = 433;
    constexpr int diesel = 434;
    constexpr int electricity = 45;

    constexpr int gasolineGeneratorR = 458;
    constexpr int gasolineGeneratorT = 459;
    constexpr int gasolineGeneratorL = 460;
    constexpr int gasolineGeneratorB = 461;

    constexpr int dieselGeneratorR = 463;
	constexpr int dieselGeneratorT = 464;
	constexpr int dieselGeneratorL = 465;
    constexpr int dieselGeneratorB = 466;

	constexpr int solarGeneratorR = 467;
	constexpr int solarGeneratorT = 468;
	constexpr int solarGeneratorL = 469;
	constexpr int solarGeneratorB = 470;

    constexpr int steamGenerator = 471;

    constexpr int powerBankR = 472;
    constexpr int powerBankL = 473;

    constexpr int copperCable = 480;
    constexpr int silverCable = 482;

    constexpr int bollardLight = 118;

    constexpr int leverRL = 149;
    constexpr int leverUD = 150;

    constexpr int tactSwitchRL = 151;
    constexpr int tactSwitchUD = 152;

    constexpr int pressureSwitchRL = 163;
    constexpr int pressureSwitchUD = 164;

    constexpr int transistorR = 155;
    constexpr int transistorU = 156;
    constexpr int transistorL = 157;
    constexpr int transistorD = 158;

    constexpr int relayR = 165;
    constexpr int relayU = 166;
    constexpr int relayL = 167;
    constexpr int relayD = 168;

    constexpr int andGateR = 488;
    constexpr int andGateL = 489;

    constexpr int orGateR = 490;
    constexpr int orGateL = 491;

    constexpr int xorGateR = 492;
    constexpr int xorGateL = 493;

    constexpr int notGateR = 494;
    constexpr int notGateL = 495;

    constexpr int srLatchR = 496;
    constexpr int srLatchL = 497;

    constexpr int delayR = 498;
    constexpr int delayL = 499;


};

export namespace entityRefCode
{
    constexpr int none = 0;
    constexpr int player = 1;
    constexpr int zombieA = 2;
    constexpr int horse = 3;
};

export namespace skillRefCode
{
    constexpr int roll = 32;
    constexpr int leap = 33;
}

export namespace connectFlag
{
    constexpr int cross = 0;
    constexpr int none = 1;
    constexpr int ULD = 2;
    constexpr int LD = 3;
    constexpr int RLD = 4;
    constexpr int RD = 5;
    constexpr int RUD = 6;
    constexpr int RU = 7;
    constexpr int RUL = 8;
    constexpr int UL = 9;
    constexpr int L = 10;
    constexpr int D = 11;
    constexpr int R = 12;
    constexpr int U = 13;
    constexpr int RL = 14;
    constexpr int UD = 15;
}

export enum class skillSrc
{
    GENERAL,
    BIONIC,
    MUTATION,
    MAGIC,
};

export enum class skillType
{
    ACTIVE,
    PASSIVE,
    TOGGLE,
};

export enum class quickSlotFlag
{
    NONE,
    SKILL,
    ITEM,
};

export enum class CoordSelectFlag
{
    NONE,
    SINGLE_TARGET_SKILL,
    FIRESTORM,
};

export enum class flameFlag
{
    NONE,
    SMALL,
    NORMAL,
    BIG,
};

export enum class gasFlag
{
    NONE,
    SMALL,
    NORMAL,
    BIG,
};

export enum statusEffectFlag
{
    none = -1,
    confused = 0,
    bleeding = 1,
    hungry = 2,
    dehydrated = 3,
    blind = 4,
    tired = 5,
    exhausted = 6,

    run = 7,
    crouch = 8,
    crawl = 9,
};

export enum charSprIndex
{
    WALK = 0,
    WALK_2H = 3,

    RUN = 6,
    RUN_2H = 9,

    CROUCH = 12,
    CROUCH_2H = 15,

    CRAWL = 18,

    SIT = 21,
    HOVER = 22,
    SUPINE = 23,

    DASH = 24,

    LATK1 = 28,
    LATK2 = 29,
    RATK1 = 30,
    RATK2 = 31,
    AIM_RIFLE = 32,
    AIM_RIFLE_CROUCH = 33,
    LCAST = 34,
    LCAST_CROUCH = 35,

    RCAST = 36,
    RCAST_CROUCH = 37,
    BCAST = 38,
    BCAST_CROUCH = 39,
    MINING1 = 40,
    MINING2 = 41,

    LAND = 42,
    AIM_PISTOL = 43,
    AIM_PISTOL_CROUCH = 44,
    AIM_PISTOL_CRAWL = 45,

    CRAFT1 = 46,
    CRAFT2 = 47,
};

export enum class dmgAniFlag
{
    none,
    dodged,
};

export enum class ridingFlag
{
    none,
    horse,
    wyvern,
    dolphin,
};

export enum class relationFlag
{
    neutral,
    hostile,
    friendly,
};

export enum class particleFlag
{
    parabolic,
    leaf,
};



//전방선언
export struct ItemData;
export class ItemPocket;
export class ItemStack;
export class Entity;
export class Monster;
export class Vehicle;
export class Prop;
export class World;
export class Chunk;
export class Loot;
export class Equip;
export class LIght;

export class statusEffect
{
public:
    statusEffectFlag effectType;
    float duration;
};

export class gasData
{
public:
    int gasCode;
    int gasVol;
    bool operator==(const gasData& other) const
    {
        return gasCode == other.gasCode && gasVol == other.gasVol;
    }
};

namespace std
{
    template<>
    struct hash<gasData>
    {
        std::size_t operator()(const gasData& g) const
        {
            return std::hash<int>()(g.gasCode);
        }
    };
}


