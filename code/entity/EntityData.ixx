export module EntityData;

import std;
import constVar;
import Sprite;
import SkillData;
import ItemPocket;
import statusEffect;

//__int8 : -128~127
//__int16 : -32768 ~32767
//__int32 : –2,147,483,648 ~2,147,483,647


export struct EntityData_Base
{
    std::wstring name;
    unsigned __int16 entityCode = 1;
    Sprite* entitySpr = nullptr;
    unsigned __int16 tooltipIndex = 0;
    unsigned __int8 category = 0;
    unsigned __int8 bodyTemplate = 0;
    __int16 temparature = 20;
    unsigned __int32 weight = 1000;
    unsigned __int32 volume = 1000;
    std::vector<unsigned __int16> material;
    unsigned __int8 HD = 1;
    //방어 계열 변수
    __int32 maxHP = 100;
    __int16 maxMP = 100;
    __int16 maxSTA = 60;

    std::vector<std::array<__int16, 2>> enc;
    __int16 rPierce = 0;
    __int16 rCut = 0;
    __int16 rBash = 0;
    __int16 sh = 0;
    __int16 ev = 0;
    __int16 rFire = 0;
    __int16 rCold = 0;
    __int16 rElec = 0;
    __int16 rCorr = 0;
    __int16 rRad = 0;
    std::vector<unsigned __int16> bionicList;
    unsigned __int16 corpseItemCode = 0;
    int eyeSight = 8; //기본 시야


    std::vector<SkillData> skillList;

    __int8 hpBarHeight = -12;

    relationFlag relation = relationFlag::hostile;
    bool isHumanCustomSprite = false;

    __int8 atkSpr1 = -1;
    __int8 atkSpr2 = -1;
};

export struct EntityData : public EntityData_Base
{

    
    ////////////////////////////////csv에 없는 데이터/////////////////////////////////////
    __int32 HP = 100;
    __int32 fakeHP = 100;
    unsigned __int8 fakeHPAlpha = 255;
    __int16 displayHPBarCount = 0;
    __int16 alphaHPBar = 0;

    __int16 MP = 100;
    __int16 STA = 60;
    __int16 fakeMP = 100;
    unsigned __int8 fakeMPAlpha = 255;
    __int16 recentDmg = 0;
    __int16 recentDmgTimer = 0;
    unsigned __int8 statStr = 5;
    unsigned __int8 statInt = 5;
    unsigned __int8 statDex = 5;
    bool isPlayer = false;
    __int8 direction = 0;
    std::unique_ptr<ItemPocket> equipment;
    std::array<int, TALENT_SIZE> proficExp = { 0, }; //경험치
    std::array<int, TALENT_SIZE> proficApt = { 0,0,+1,-2,0,0,0,-1,0, }; //적성
    std::array<int, TALENT_SIZE> proficFocus = { 0, }; //집중도 0:미분배, 1:소분배, 2:일반분배
    __int16 sprIndex = 0;
    __int16 sprIndexInfimum = 0;
    bool sprFlip = false;
    float sprAngle = 0.0f; //스프라이트의 회전각도
    int jumpOffsetY = 0;
    humanCustom::skin skin = humanCustom::skin::null;
    humanCustom::eyes eyes = humanCustom::eyes::null;
    humanCustom::scar scar = humanCustom::scar::null;
    humanCustom::beard beard = humanCustom::beard::null;
    humanCustom::hair hair = humanCustom::hair::null;
    humanCustom::horn horn = humanCustom::horn::null;

    std::vector<statusEffect> statusEffectVec;

    walkFlag walkMode = walkFlag::walk;
    double gridMoveSpd = 3.0;//그리드와 그리드 사이를 넘어갈 때의 속도

    bool isEyesHalf = false; //눈을 반쯤 뜬 상태
    bool isEyesClose = false; //눈을 감은 상태

    EntityData cloneEntity() const;
    EntityData();
    virtual ~EntityData();
    EntityData(EntityData&&);
    EntityData& operator=(EntityData&&) = default;
    EntityData(const EntityData&) = delete;
    EntityData& operator=(const EntityData&) = delete;
};