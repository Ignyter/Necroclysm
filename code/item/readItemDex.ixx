#define _SILENCE_CXX17_CODECVT_HEADER_DEPRECATION_WARNING
#define _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
#include <SDL3/SDL.h>

export module readItemDex;

import std;
import util;
import constVar;
import globalVar;
import textureVar;

namespace csvItem
{
    constexpr int nativeName = 0;
    constexpr int name = 1;
    constexpr int dir = 2;
    constexpr int itemCode = 3;
    constexpr int sprIndex = 4;
    constexpr int nativeDescript = 5;
    constexpr int descript = 6;
    constexpr int category = 7;
    constexpr int subcategory = 8;
    constexpr int flag = 9;
    constexpr int weight = 10;
    constexpr int volume = 11;
    constexpr int material = 12;
    constexpr int tileSprIndex = 13;
    constexpr int tileConnectGroup = 14;
    constexpr int propSprIndex = 15;
    constexpr int propMaxHP = 16;
    constexpr int propRPierce = 17;
    constexpr int propRCut = 18;
    constexpr int propRBash = 19;
    constexpr int propDrawPriority = 20;
    constexpr int propWIPSprIndex = 21;
    constexpr int calorie = 22;
    constexpr int hydration = 23;
    constexpr int hydrationPer = 24;
    constexpr int toolQuality = 25; // A_X 여기서 X는 레벨, 즉 1_3은 망치질 3단계
    constexpr int craftMinUnit = 26;
    constexpr int craftTime = 27;
    constexpr int recipe = 28;
    constexpr int recipeQualityNeed = 29;
    constexpr int recipeProficNeed = 30;
    constexpr int disassy = 31;
    constexpr int disassyQualityNeed = 32;
    constexpr int disassyProficNeed = 33;
    constexpr int repair = 34;
    constexpr int repairQualityNeed = 35;
    constexpr int repairProficNeed = 36;
    constexpr int pocketMaxVolume = 37;
    constexpr int pocketMaxNumber = 38;
    constexpr int pocketOnlyItem = 39;

    // Torso (몸통)
    constexpr int rPierceTorso = 40;
    constexpr int rCutTorso = 41;
    constexpr int rBashTorso = 42;
    constexpr int encTorso = 43;

    // Head (머리)
    constexpr int rPierceHead = 44;
    constexpr int rCutHead = 45;
    constexpr int rBashHead = 46;
    constexpr int encHead = 47;

    // Left Arm (왼팔)
    constexpr int rPierceLArm = 48;
    constexpr int rCutLArm = 49;
    constexpr int rBashLArm = 50;
    constexpr int encLArm = 51;

    // Right Arm (오른팔)
    constexpr int rPierceRArm = 52;
    constexpr int rCutRArm = 53;
    constexpr int rBashRArm = 54;
    constexpr int encRArm = 55;

    // Left Leg (왼다리)
    constexpr int rPierceLLeg = 56;
    constexpr int rCutLLeg = 57;
    constexpr int rBashLLeg = 58;
    constexpr int encLLeg = 59;

    // Right Leg (오른다리)
    constexpr int rPierceRLeg = 60;
    constexpr int rCutRLeg = 61;
    constexpr int rBashRLeg = 62;
    constexpr int encRLeg = 63;

    constexpr int SH = 64;
    constexpr int EV = 65;
    constexpr int rFire = 66;
    constexpr int rCold = 67;
    constexpr int rElec = 68;
    constexpr int rCorr = 69;
    constexpr int rRad = 70;
    constexpr int bionicIndex = 71;
    constexpr int durability = 72;
    constexpr int pierce = 73;
    constexpr int cut = 74;
    constexpr int bash = 75;
    constexpr int bulletPierce = 76;
    constexpr int bulletCut = 77;
    constexpr int bulletBash = 78;
    constexpr int bulletStoppingPower = 79;
    constexpr int bulletJam = 80;
    constexpr int bulletType = 81;
    constexpr int bulletRange = 82;
    constexpr int gunDmg = 83;
    constexpr int gunAccInit = 84;
    constexpr int gunAccSpd = 85;
    constexpr int gunAccMax = 86;
    constexpr int gunRebound = 87;
    constexpr int gunShotSpd = 88;
    constexpr int gunReloadSpd = 89;
    constexpr int gunJam = 90;
    constexpr int gunMod = 91;
    constexpr int gunBalance = 92;
    constexpr int gunRange = 93;
    constexpr int meleeAtkSpd = 94;
    constexpr int meleeAtkAcc = 95;
    constexpr int meleeBalance = 96;
    constexpr int meleeRange = 97;
    constexpr int modeTemplate = 98;
    constexpr int throwAtk = 99;
    constexpr int throwType = 100;
    constexpr int throwAtkAcc = 101;
    constexpr int throwBalance = 102;
    constexpr int throwRange = 103;
    constexpr int throwStoppingPower = 104;
    constexpr int bookIndex = 105;
    constexpr int equipSprName = 106;
    constexpr int equipPriority = 107;
    constexpr int flipEquipSprName = 108;
    constexpr int flipEquipPriority = 109;
    constexpr int leftWieldSpr = 110;
    constexpr int leftWieldPriority = 111;
    constexpr int rightWieldSpr = 112;
    constexpr int rightWieldPriority = 113;
    constexpr int dirChangeItemCode = 114;
    constexpr int lightRange = 115;
    constexpr int lightIntensity = 116;
    constexpr int lightR = 117;
    constexpr int lightG = 118;
    constexpr int lightB = 119;
    constexpr int lightDelX = 120;
    constexpr int lightDelY = 121;
    constexpr int animeSize = 122;
    constexpr int animeFPS = 123;
    constexpr int randomPropSprSize = 124;
    constexpr int growthThreshold = 125;
    constexpr int molecularWeight = 126;
    constexpr int liqColorR = 127;
    constexpr int liqColorG = 128;
    constexpr int liqColorB = 129;
    constexpr int gasColorR = 130;
    constexpr int gasColorG = 131;
    constexpr int gasColorB = 132;
    constexpr int gasAlphaMax = 133;

    constexpr int propInstallCode = 134; //이게 0이 아닐 경우 주변 타일에 설치 가능
    constexpr int propUninstallCode = 135; //이게 0이 아닐 경우 프롭을 손에 드는 행위 등이 가능

    constexpr int electricResistance = 136;
    constexpr int electricMaxPower = 137;

    constexpr int gndUsePower = 138; //전자기기 소비전력

    constexpr int gndUsePowerRight = 139;
    constexpr int gndUsePowerUp = 140;
    constexpr int gndUsePowerLeft = 141;
    constexpr int gndUsePowerDown = 142;
};

export int readItemDex(const wchar_t* file)
{
    std::wifstream in(file);
    std::wstring str;
    std::wstring strFragment;//표 한 칸의 문자열이 저장되는 객체, 매번 초기화됨

    if (in.is_open())
    {
        in.imbue(std::locale(in.getloc(), new std::codecvt_utf8<wchar_t, 0x10ffff, std::consume_header>));
        in.seekg(0, std::ios::end);
        long long size = in.tellg();
        str.resize(size);
        in.seekg(0, std::ios::beg);
        in.read(&str[0], size);
        in.close();
        //읽기 종료

        //배열의 가로 사이즈를 구한다.
        int csvWidth = 0;
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == UNI::TAB || str[i] == UNI::LF)//해당 문자가 쉼표(44)거나 라인피드(10)일 경우
            {
                csvWidth++;
                if (str[i] == UNI::LF)
                {
                    //prt(L"이 csv 파일의 가로사이즈는 %d이다!\n", csvWidth);
                    break;
                }
            }
        }

        // 3,5,7 형태의 문자열을 벡터에 순서대로 3->5->7 순서대로 입력해주는 람다함수
        auto valsToVec = [&](std::wstring strFragment, auto& container) {
            int val;
            for (int j = 0; j < strFragment.size(); j++)
            {
                if (strFragment[j] == UNI::SEMICOLON || j == strFragment.size() - 1)
                {
                    if (j == strFragment.size() - 1) { j++; } //마지막이면 j값을 1 더하여 보정

                    val = std::stoi(strFragment.substr(0, j));
                    strFragment.erase(0, j + 1);
                    j = 0;

                    container.push_back(val);
                }
            }
            };


        // 3x3,5x6,2x6 형태의 문자열을 pair 형태로 벡터에 순서대로 입력해주는 람다함수
        auto pairsToVec = [&](std::wstring strFragment, auto& container) {
            using T1 = decltype(container[0].first);
            using T2 = decltype(container[0].second);

            for (int j = 0; j < strFragment.size(); j++)
            {
                if (strFragment[j] == UNI::ASTERISK)
                {
                    container.push_back({ static_cast<T1>(std::stoi(strFragment.substr(0, j))), 0 });
                    strFragment.erase(0, j + 1);
                    j = 0;
                }

                if (strFragment[j] == UNI::SEMICOLON || j == strFragment.size() - 1)
                {
                    if (j == strFragment.size() - 1) { j++; } //마지막이면 j값을 1 더하여 보정

                    container.back().second = static_cast<T2>(std::stoi(strFragment.substr(0, j)));
                    strFragment.erase(0, j + 1);
                    j = 0;
                }
            }
            };


        int startPoint = -1;
        int endPoint = -1;
        int arrayCounter = 0;

        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == UNI::TAB || str[i] == 10)//ASCII로 44(,)와 또는 라인피드(\n)와 같을 때
            {
                if (i == str.size() - 1) { i++; }//마지막 문자면 endP-startP 보정을 위해 i를 1 더함.
                endPoint = i;

                strFragment = str.substr(startPoint, endPoint - startPoint);

                if (arrayCounter / csvWidth == itemDex.size() + 1) // 만약 벡터 크기(상하)가 부족하면 1칸 늘림
                {
                    ItemData newItem;
                    itemDex.push_back(std::move(newItem));
                }

                if (arrayCounter / (csvWidth) != 0)
                {
                    const int tgtIndex = arrayCounter / (csvWidth)-1;
                    const int columnIndex = arrayCounter % csvWidth;

                    switch (columnIndex)
                    {
                    case csvItem::nativeName:
                        break;
                    case csvItem::name:
                        itemDex[tgtIndex].name = strFragment;
                        break;
                    case csvItem::dir:
                        itemDex[tgtIndex].dir = strFragment;
                        break;
                    case csvItem::itemCode:
                        itemDex[tgtIndex].itemCode = wtoi(strFragment.c_str());
                        break;
                    case csvItem::sprIndex:
                        itemDex[tgtIndex].itemSprIndex = wtoi(strFragment.c_str());
                        break;
                    case csvItem::nativeDescript:
                        break;
                    case csvItem::descript:
                        itemTooltip.push_back(strFragment);
                        itemDex[tgtIndex].tooltipIndex = itemTooltip.size() - 1;
                        break;
                    case csvItem::category:
                        if (strFragment == L"EQUIPMENT") { itemDex[tgtIndex].category = itemCategory::equipment; }
                        else if (strFragment == L"FOODS") { itemDex[tgtIndex].category = itemCategory::foods; }
                        else if (strFragment == L"TOOLS") { itemDex[tgtIndex].category = itemCategory::tools; }
                        else if (strFragment == L"TECH") { itemDex[tgtIndex].category = itemCategory::tech; }
                        else if (strFragment == L"CONSUMABLES") { itemDex[tgtIndex].category = itemCategory::consumables; }
                        else if (strFragment == L"VEHICLES") { itemDex[tgtIndex].category = itemCategory::vehicles; }
                        else if (strFragment == L"STRUCTURES") { itemDex[tgtIndex].category = itemCategory::structures; }
                        else if (strFragment == L"MATERIALS") { itemDex[tgtIndex].category = itemCategory::materials; }
                        else { errorBox(L"error in readItemDex.ixx, csvItem::category"); }

                        break;
                    case csvItem::subcategory:
                        if (itemDex[tgtIndex].category == itemCategory::equipment)
                        {
                            if (strFragment == L"EQUIPMENT_MELEE") { itemDex[tgtIndex].subcategory = itemSubcategory::equipment_melee; }
                            else if (strFragment == L"EQUIPMENT_RANGED") { itemDex[tgtIndex].subcategory = itemSubcategory::equipment_ranged; }
                            else if (strFragment == L"EQUIPMENT_FIREARMS") { itemDex[tgtIndex].subcategory = itemSubcategory::equipment_firearms; }
                            else if (strFragment == L"EQUIPMENT_THROWING") { itemDex[tgtIndex].subcategory = itemSubcategory::equipment_throwing; }
                            else if (strFragment == L"EQUIPMENT_CLOTHING") { itemDex[tgtIndex].subcategory = itemSubcategory::equipment_clothing; }
                            else { errorBox(L"error in readItemDex.ixx, csvItem::subcategory(equipment)"); }
                        }
                        else if (itemDex[tgtIndex].category == itemCategory::foods)
                        {
                            if (strFragment == L"FOODS_COOKED") { itemDex[tgtIndex].subcategory = itemSubcategory::foods_cooked; }
                            else if (strFragment == L"FOODS_PROCESSED") { itemDex[tgtIndex].subcategory = itemSubcategory::foods_processed; }
                            else if (strFragment == L"FOODS_PRESERVED") { itemDex[tgtIndex].subcategory = itemSubcategory::foods_preserved; }
                            else if (strFragment == L"FOODS_DRINKS") { itemDex[tgtIndex].subcategory = itemSubcategory::foods_drinks; }
                            else if (strFragment == L"FOODS_INGREDIENTS") { itemDex[tgtIndex].subcategory = itemSubcategory::foods_ingredients; }
                            else { errorBox(L"error in readItemDex.ixx, csvItem::subcategory(foods)"); }
                        }
                        else if (itemDex[tgtIndex].category == itemCategory::tools)
                        {
                            if (strFragment == L"TOOLS_HAND") { itemDex[tgtIndex].subcategory = itemSubcategory::tools_hand; }
                            else if (strFragment == L"TOOLS_POWER") { itemDex[tgtIndex].subcategory = itemSubcategory::tools_power; }
                            else if (strFragment == L"TOOLS_CONTAINERS") { itemDex[tgtIndex].subcategory = itemSubcategory::tools_containers; }
                            else if (strFragment == L"TOOLS_ETC") { itemDex[tgtIndex].subcategory = itemSubcategory::tools_etc; }
                            else { errorBox(L"error in readItemDex.ixx, csvItem::subcategory(tools)"); }
                        }
                        else if (itemDex[tgtIndex].category == itemCategory::tech)
                        {
                            if (strFragment == L"TECH_BIONICS") { itemDex[tgtIndex].subcategory = itemSubcategory::tech_bionics; }
                            else if (strFragment == L"TECH_POWERARMOR") { itemDex[tgtIndex].subcategory = itemSubcategory::tech_powerArmor; }
                            else { errorBox(L"error in readItemDex.ixx, csvItem::subcategory(tech)"); }
                        }
                        else if (itemDex[tgtIndex].category == itemCategory::consumables)
                        {
                            if (strFragment == L"CONSUMABLE_MEDICINE") { itemDex[tgtIndex].subcategory = itemSubcategory::consumable_medicine; }
                            else if (strFragment == L"CONSUMABLE_AMMO") { itemDex[tgtIndex].subcategory = itemSubcategory::consumable_ammo; }
                            else if (strFragment == L"CONSUMABLE_FUEL") { itemDex[tgtIndex].subcategory = itemSubcategory::consumable_fuel; }
                            else if (strFragment == L"CONSUMABLE_ETC") { itemDex[tgtIndex].subcategory = itemSubcategory::consumable_etc; }
                            else { errorBox(L"error in readItemDex.ixx, csvItem::subcategory(consumables)"); }
                        }
                        else if (itemDex[tgtIndex].category == itemCategory::vehicles)
                        {
                            if (strFragment == L"VEHICLE_FRAMES") { itemDex[tgtIndex].subcategory = itemSubcategory::vehicle_frames; }
                            else if (strFragment == L"VEHICLE_POWER") { itemDex[tgtIndex].subcategory = itemSubcategory::vehicle_power; }
                            else if (strFragment == L"VEHICLE_EXTERIORS") { itemDex[tgtIndex].subcategory = itemSubcategory::vehicle_exteriors; }
                            else if (strFragment == L"VEHICLE_PARTS") { itemDex[tgtIndex].subcategory = itemSubcategory::vehicle_parts; }
                            else { errorBox(L"error in readItemDex.ixx, csvItem::subcategory(vehicles)"); }
                        }
                        else if (itemDex[tgtIndex].category == itemCategory::structures)
                        {
                            if (strFragment == L"STRUCTURE_WALLS") { itemDex[tgtIndex].subcategory = itemSubcategory::structure_walls; }
                            else if (strFragment == L"STRUCTURE_FLOORS") { itemDex[tgtIndex].subcategory = itemSubcategory::structure_floors; }
                            else if (strFragment == L"STRUCTURE_CEILINGS") { itemDex[tgtIndex].subcategory = itemSubcategory::structure_ceilings; }
                            else if (strFragment == L"STRUCTURE_PROPS") { itemDex[tgtIndex].subcategory = itemSubcategory::structure_props; }
                            else { errorBox(L"error in readItemDex.ixx, csvItem::subcategory(structures)"); }
                        }
                        else if (itemDex[tgtIndex].category == itemCategory::materials)
                        {
                            if (strFragment == L"MATERIAL_METALS") { itemDex[tgtIndex].subcategory = itemSubcategory::material_metals; }
                            else if (strFragment == L"MATERIAL_ORGANIC") { itemDex[tgtIndex].subcategory = itemSubcategory::material_organic; }
                            else if (strFragment == L"MATERIAL_COMPONENTS") { itemDex[tgtIndex].subcategory = itemSubcategory::material_components; }
                            else if (strFragment == L"MATERIAL_CHEMICALS") { itemDex[tgtIndex].subcategory = itemSubcategory::material_chemicals; }
                            else if (strFragment == L"MATERIAL_ETC") { itemDex[tgtIndex].subcategory = itemSubcategory::material_etc; }
                            else { errorBox(L"error in readItemDex.ixx, csvItem::subcategory(materials)"); }
                        }
                        else { errorBox(L"error in readItemDex.ixx, itemCategory"); }
                        break;
                    case csvItem::weight:
                        itemDex[tgtIndex].weight = wtoi(strFragment.c_str());
                        break;
                    case csvItem::volume:
                        itemDex[tgtIndex].originalVolume = wtoi(strFragment.c_str());
                        break;
                    case csvItem::material:
                        break;
                    case csvItem::flag:
                    {
                        static const std::unordered_map<std::wstring, itemFlag> flagMap{
                            {L"TWOHANDED", itemFlag::TWOHANDED},
                            {L"CANEQUIP", itemFlag::CANEQUIP},
                            {L"NONSTACK", itemFlag::NONSTACK},
                            {L"GUN", itemFlag::GUN},
                            {L"MAGAZINE", itemFlag::MAGAZINE},
                            {L"AMMO", itemFlag::AMMO},
                            {L"POWERED", itemFlag::POWERED},
                            {L"RAIDARMOR", itemFlag::RAIDARMOR},
                            {L"COORDCRAFT", itemFlag::COORDCRAFT},
                            {L"VFRAME", itemFlag::VFRAME},
                            {L"LIQUID", itemFlag::LIQUID},
                            {L"GAS", itemFlag::GAS},
                            {L"VPART", itemFlag::VPART},
                            {L"TRANSPARENT_WALL", itemFlag::TRANSPARENT_WALL},
                            {L"PROP_BIG", itemFlag::PROP_BIG},
                            {L"VEH_ROOF", itemFlag::VEH_ROOF},
                            {L"PROP_WALKABLE", itemFlag::PROP_WALKABLE},
                            {L"PROP_BLOCKER", itemFlag::PROP_BLOCKER},
                            {L"PROP_DEPTH_LOWER", itemFlag::PROP_DEPTH_LOWER},
                            {L"LIGHT_ON", itemFlag::LIGHT_ON},
                            {L"LIGHT_OFF", itemFlag::LIGHT_OFF},
                            {L"VPART_WALL_CONNECT", itemFlag::VPART_WALL_CONNECT},
                            {L"VPART_DIR_DEPEND", itemFlag::VPART_DIR_DEPEND},
                            {L"VPART_DOOR_OPEN", itemFlag::VPART_DOOR_OPEN},
                            {L"VPART_DOOR_CLOSE", itemFlag::VPART_DOOR_CLOSE},

                            {L"PIPE_CNCT_RIGHT", itemFlag::PIPE_CNCT_RIGHT},
                            {L"PIPE_CNCT_UP", itemFlag::PIPE_CNCT_UP},
                            {L"PIPE_CNCT_LEFT", itemFlag::PIPE_CNCT_LEFT},
                            {L"PIPE_CNCT_DOWN", itemFlag::PIPE_CNCT_DOWN},

                            {L"CABLE_CNCT_RIGHT", itemFlag::CABLE_CNCT_RIGHT},
                            {L"CABLE_CNCT_UP", itemFlag::CABLE_CNCT_UP},
                            {L"CABLE_CNCT_LEFT", itemFlag::CABLE_CNCT_LEFT},
                            {L"CABLE_CNCT_DOWN", itemFlag::CABLE_CNCT_DOWN},

                            {L"CONVEYOR_CNCT_RIGHT", itemFlag::CONVEYOR_CNCT_RIGHT},
                            {L"CONVEYOR_CNCT_UP", itemFlag::CONVEYOR_CNCT_UP},
                            {L"CONVEYOR_CNCT_LEFT", itemFlag::CONVEYOR_CNCT_LEFT},
                            {L"CONVEYOR_CNCT_DOWN", itemFlag::CONVEYOR_CNCT_DOWN},

                            {L"TIRE_NORMAL", itemFlag::TIRE_NORMAL},
                            {L"TIRE_STEER", itemFlag::TIRE_STEER},

                            {L"PROP", itemFlag::PROP},

                            {L"CABLE", itemFlag::CABLE},
                            {L"PIPE", itemFlag::PIPE},
                            {L"CONVEYOR", itemFlag::CONVEYOR},

                            {L"RAIL", itemFlag::RAIL},

                            {L"RAIL_CNCT_UP", itemFlag::RAIL_CNCT_UP},
                            {L"RAIL_CNCT_DOWN", itemFlag::RAIL_CNCT_DOWN},
                            {L"RAIL_CNCT_LEFT", itemFlag::RAIL_CNCT_LEFT},
                            {L"RAIL_CNCT_RIGHT", itemFlag::RAIL_CNCT_RIGHT},

                            {L"RAIL_INPUT_UP", itemFlag::RAIL_INPUT_UP},
                            {L"RAIL_INPUT_DOWN", itemFlag::RAIL_INPUT_DOWN},
                            {L"RAIL_INPUT_LEFT", itemFlag::RAIL_INPUT_LEFT},
                            {L"RAIL_INPUT_RIGHT", itemFlag::RAIL_INPUT_RIGHT},

                            {L"RAIL_BUFFER", itemFlag::RAIL_BUFFER},

                            {L"NOT_RECIPE", itemFlag::NOT_RECIPE},

                            {L"TREE", itemFlag::TREE},

                            {L"PLANT", itemFlag::PLANT},
                            {L"PLANT_SEASON_DEPENDENT", itemFlag::PLANT_SEASON_DEPENDENT},
                            {L"PLANT_GROWTH_DEPENDENT", itemFlag::PLANT_GROWTH_DEPENDENT},
                            {L"MUSHROOM", itemFlag::MUSHROOM},
                            {L"FLOOR", itemFlag::FLOOR},
                            {L"WALL", itemFlag::WALL},
                            {L"CEIL", itemFlag::CEIL},
                            {L"WATER_SHALLOW", itemFlag::WATER_SHALLOW},
                            {L"WATER_DEEP", itemFlag::WATER_DEEP},
                            {L"FRESHWATER", itemFlag::FRESHWATER},
                            {L"SEAWATER", itemFlag::SEAWATER},
                            {L"TILE_SEASON", itemFlag::TILE_SEASON},

                            {L"DOOR", itemFlag::DOOR},
                            {L"UPSTAIR", itemFlag::UPSTAIR},
                            {L"DOWNSTAIR", itemFlag::DOWNSTAIR},
                            {L"SIGN", itemFlag::SIGN},
                            {L"DOOR_CLOSE", itemFlag::DOOR_CLOSE},
                            {L"DOOR_OPEN", itemFlag::DOOR_OPEN},
                            {L"TRAIN_WHEEL", itemFlag::TRAIN_WHEEL},

                            {L"PROP_GAS_OBSTACLE_ON", itemFlag::PROP_GAS_OBSTACLE_ON},
                            {L"PROP_GAS_OBSTACLE_OFF", itemFlag::PROP_GAS_OBSTACLE_OFF},
                            {L"WALL_GAS_PERMEABLE", itemFlag::WALL_GAS_PERMEABLE},

                            {L"POCKET", itemFlag::POCKET},
                            {L"CAN_CLIMB", itemFlag::CAN_CLIMB},
                            {L"SPR_TH_WEAPON", itemFlag::SPR_TH_WEAPON},
                            {L"NO_HAIR_HELMET", itemFlag::NO_HAIR_HELMET},
                            {L"BOW", itemFlag::BOW},
                            {L"CROSSBOW", itemFlag::CROSSBOW},
                            {L"TOGGLE_ON", itemFlag::TOGGLE_ON},
                            {L"TOGGLE_OFF", itemFlag::TOGGLE_OFF},
                            {L"HAS_TOGGLE_SPRITE", itemFlag::HAS_TOGGLE_SPRITE},
                            {L"CANCRAFT", itemFlag::CANCRAFT},
                            {L"HEADLIGHT", itemFlag::HEADLIGHT},
                            {L"SHIELD", itemFlag::SHIELD},
                            {L"CONTAINER_LIQ", itemFlag::CONTAINER_LIQ},
                            {L"VPART_NOT_WALKABLE", itemFlag::VPART_NOT_WALKABLE},
                            {L"ENGINE_GASOLINE", itemFlag::ENGINE_GASOLINE},
                            {L"ENGINE_DIESEL", itemFlag::ENGINE_DIESEL},
                            {L"ENGINE_ELECTRIC", itemFlag::ENGINE_ELECTRIC},
                            {L"CAN_EAT", itemFlag::CAN_EAT},
                            {L"CAN_DRINK", itemFlag::CAN_DRINK},
                            {L"CONTAINER_FLEX", itemFlag::CONTAINER_FLEX},
                            {L"WIELD_NORMAL_DISPLAY", itemFlag::WIELD_NORMAL_DISPLAY},

                            {L"LIQ_COL_RED", itemFlag::LIQ_COL_RED},
                            {L"LIQ_COL_BLUE", itemFlag::LIQ_COL_BLUE},
                            {L"LIQ_COL_YELLOW", itemFlag::LIQ_COL_YELLOW},
                            {L"LIQ_COL_WHITE", itemFlag::LIQ_COL_WHITE},
                            {L"LIQ_COL_GRAY", itemFlag::LIQ_COL_GRAY},
                            {L"LIQ_COL_BLACK", itemFlag::LIQ_COL_BLACK},
                            {L"CONTAINER_TRANSPARENT", itemFlag::CONTAINER_TRANSPARENT},
                            {L"CONTAINER_TRANSLUCENT", itemFlag::CONTAINER_TRANSLUCENT},

                            {L"PROP_POWER_OFF", itemFlag::PROP_POWER_OFF},
                            {L"PROP_POWER_ON", itemFlag::PROP_POWER_ON},

                            {L"CIRCUIT", itemFlag::CIRCUIT},
                            {L"CABLE_BEHIND", itemFlag::CABLE_BEHIND},
                            {L"VOLTAGE_SOURCE", itemFlag::VOLTAGE_SOURCE},

                            { L"VOLTAGE_OUTPUT_RIGHT", itemFlag::VOLTAGE_OUTPUT_RIGHT },
                            { L"VOLTAGE_OUTPUT_UP", itemFlag::VOLTAGE_OUTPUT_UP },
                            { L"VOLTAGE_OUTPUT_LEFT", itemFlag::VOLTAGE_OUTPUT_LEFT },
                            { L"VOLTAGE_OUTPUT_DOWN", itemFlag::VOLTAGE_OUTPUT_DOWN },
                            { L"CABLE_Z_ASCEND", itemFlag::CABLE_Z_ASCEND },
                            { L"CABLE_Z_DESCEND", itemFlag::CABLE_Z_DESCEND },

                            { L"HAS_GROUND", itemFlag::HAS_GROUND },
                            { L"VOLTAGE_GND_UP", itemFlag::VOLTAGE_GND_UP },
                            { L"VOLTAGE_GND_DOWN", itemFlag::VOLTAGE_GND_DOWN },
                            { L"VOLTAGE_GND_LEFT", itemFlag::VOLTAGE_GND_LEFT },
                            { L"VOLTAGE_GND_RIGHT", itemFlag::VOLTAGE_GND_RIGHT },
                            { L"VOLTAGE_GND_ALL", itemFlag::VOLTAGE_GND_ALL },
                            
                            { L"TRANSISTOR", itemFlag::TRANSISTOR },
                            { L"LOGIC_GATE", itemFlag::LOGIC_GATE },
                        };

                        size_t pos = 0;
                        while (pos <= strFragment.size()) 
                        {
                            size_t sep = strFragment.find(UNI::SEMICOLON, pos);
                            std::wstring token = (sep == std::wstring::npos)
                                ? strFragment.substr(pos)
                                : strFragment.substr(pos, sep - pos);

                            if (!token.empty()) 
                            {
                                auto it = flagMap.find(token);
                                if (it != flagMap.end()) itemDex[tgtIndex].flag.push_back(it->second);
                                else errorBox(L"error in readItemDex.ixx, csvItem::flag, unknown itemFlag defined " + token);
                            }
                            if (sep == std::wstring::npos) break;
                            pos = sep + 1;
                        }
                        break;
                    }

                    case csvItem::calorie:
                        itemDex[tgtIndex].calorie = wtoi(strFragment.c_str());
                        break;
                    case csvItem::hydration:
                        itemDex[tgtIndex].hydration = wtoi(strFragment.c_str());    
                        break;
                    case csvItem::hydrationPer:
                        itemDex[tgtIndex].hydrationPerML = wtof(strFragment.c_str());
                        break;
                    case csvItem::toolQuality:
                    {
                        for (int j = 0; j < strFragment.size(); j++)
                        {
                            if (strFragment[j] == UNI::SEMICOLON || j == strFragment.size() - 1)
                            {
                                if (j == strFragment.size() - 1) { j++; } //마지막이면 j값을 1 더하여 보정

                                itemDex[tgtIndex].toolQuality.push_back(wtoi(strFragment.substr(0, j).c_str()));
                                strFragment.erase(0, j + 1);
                                j = 0;
                            }
                        }
                        break;
                    }
                    case csvItem::craftMinUnit:
                    {
                        itemDex[tgtIndex].craftMinUnit = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::craftTime:
                    {
                        itemDex[tgtIndex].craftTime = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::recipe:
                    {
                        pairsToVec(strFragment, itemDex[tgtIndex].recipe);
                        break;
                    }
                    case csvItem::recipeQualityNeed:
                    {
                        for (int j = 0; j < strFragment.size(); j++)
                        {
                            if (strFragment[j] == UNI::SEMICOLON || j == strFragment.size() - 1)
                            {
                                if (j == strFragment.size() - 1) { j++; } //마지막이면 j값을 1 더하여 보정

                                itemDex[tgtIndex].recipeQualityNeed.push_back(wtoi(strFragment.substr(0, j).c_str()));
                                strFragment.erase(0, j + 1);
                                j = 0;
                            }
                        }
                        break;
                    }
                    case csvItem::recipeProficNeed:
                    {
                        pairsToVec(strFragment, itemDex[tgtIndex].recipeProficNeed);
                        break;
                    }
                    case csvItem::disassy:
                    {
                        pairsToVec(strFragment, itemDex[tgtIndex].disassy);
                        break;
                    }
                    case csvItem::disassyQualityNeed:
                    {
                        for (int j = 0; j < strFragment.size(); j++)
                        {
                            if (strFragment[j] == UNI::SEMICOLON || j == strFragment.size() - 1)
                            {
                                if (j == strFragment.size() - 1) { j++; } //마지막이면 j값을 1 더하여 보정

                                itemDex[tgtIndex].disassyQualityNeed.push_back(wtoi(strFragment.substr(0, j).c_str()));
                                strFragment.erase(0, j + 1);
                                j = 0;
                            }
                        }
                        break;
                    }
                    case csvItem::disassyProficNeed:
                    {
                        pairsToVec(strFragment, itemDex[tgtIndex].disassyProficNeed);
                        break;
                    }
                    case csvItem::repair:
                    {
                        pairsToVec(strFragment, itemDex[tgtIndex].repair);
                        break;
                    }
                    case csvItem::repairQualityNeed:
                    {
                        for (int j = 0; j < strFragment.size(); j++)
                        {
                            if (strFragment[j] == UNI::SEMICOLON || j == strFragment.size() - 1)
                            {
                                if (j == strFragment.size() - 1) { j++; } //마지막이면 j값을 1 더하여 보정

                                itemDex[tgtIndex].repairQualityNeed.push_back(wtoi(strFragment.c_str()));
                                strFragment.erase(0, j + 1);
                                j = 0;
                            }
                        }
                        break;
                    }
                    case csvItem::repairProficNeed:
                    {
                        pairsToVec(strFragment, itemDex[tgtIndex].repairProficNeed);
                        break;
                    }
                    case csvItem::pocketMaxVolume:
                        itemDex[tgtIndex].pocketMaxVolume = wtoi(strFragment.c_str());
                        break;
                    case csvItem::pocketMaxNumber:
                        itemDex[tgtIndex].pocketMaxNumber = wtoi(strFragment.c_str());
                        break;
                    case csvItem::pocketOnlyItem:
                    {
                        valsToVec(strFragment, itemDex[tgtIndex].pocketOnlyItem);
                        break;
                    }
                    case csvItem::rPierceTorso:
                    {
                        itemDex[tgtIndex].rPierceTorso = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rCutTorso:
                    {
                        itemDex[tgtIndex].rCutTorso = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rBashTorso:
                    {
                        itemDex[tgtIndex].rBashTorso = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::encTorso:
                    {
                        itemDex[tgtIndex].encTorso = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rPierceHead:
                    {
                        itemDex[tgtIndex].rPierceHead = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rCutHead:
                    {
                        itemDex[tgtIndex].rCutHead = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rBashHead:
                    {
                        itemDex[tgtIndex].rBashHead = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::encHead:
                    {
                        itemDex[tgtIndex].encHead = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rPierceLArm:
                    {
                        itemDex[tgtIndex].rPierceLArm = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rCutLArm:
                    {
                        itemDex[tgtIndex].rCutLArm = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rBashLArm:
                    {
                        itemDex[tgtIndex].rBashLArm = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::encLArm:
                    {
                        itemDex[tgtIndex].encLArm = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rPierceRArm:
                    {
                        itemDex[tgtIndex].rPierceRArm = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rCutRArm:
                    {
                        itemDex[tgtIndex].rCutRArm = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rBashRArm:
                    {
                        itemDex[tgtIndex].rBashRArm = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::encRArm:
                    {
                        itemDex[tgtIndex].encRArm = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rPierceLLeg:
                    {
                        itemDex[tgtIndex].rPierceLLeg = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rCutLLeg:
                    {
                        itemDex[tgtIndex].rCutLLeg = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rBashLLeg:
                    {
                        itemDex[tgtIndex].rBashLLeg = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::encLLeg:
                    {
                        itemDex[tgtIndex].encLLeg = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rPierceRLeg:
                    {
                        itemDex[tgtIndex].rPierceRLeg = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rCutRLeg:
                    {
                        itemDex[tgtIndex].rCutRLeg = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::rBashRLeg:
                    {
                        itemDex[tgtIndex].rBashRLeg = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::encRLeg:
                    {
                        itemDex[tgtIndex].encRLeg = wtoi(strFragment.c_str());
                        break;
                    }
                    case csvItem::SH:
                        itemDex[tgtIndex].sh = wtoi(strFragment.c_str());
                        break;
                    case csvItem::EV:
                        itemDex[tgtIndex].ev = wtoi(strFragment.c_str());
                        break;
                    case csvItem::rFire:
                        itemDex[tgtIndex].rFire = wtoi(strFragment.c_str());
                        break;
                    case csvItem::rCold:
                        itemDex[tgtIndex].rCold = wtoi(strFragment.c_str());
                        break;
                    case csvItem::rElec:
                        itemDex[tgtIndex].rElec = wtoi(strFragment.c_str());
                        break;
                    case csvItem::rCorr:
                        itemDex[tgtIndex].rCorr = wtoi(strFragment.c_str());
                        break;
                    case csvItem::rRad:
                        itemDex[tgtIndex].rRad = wtoi(strFragment.c_str());
                        break;
                    case csvItem::bionicIndex:
                        itemDex[tgtIndex].bionicIndex = wtoi(strFragment.c_str());
                        break;
                    case csvItem::durability:
                        itemDex[tgtIndex].durablility = wtoi(strFragment.c_str());
                        break;
                    case csvItem::pierce:
                        itemDex[tgtIndex].pierce = wtoi(strFragment.c_str());
                        break;
                    case csvItem::cut:
                        itemDex[tgtIndex].cut = wtoi(strFragment.c_str());
                        break;
                    case csvItem::bash:
                        itemDex[tgtIndex].bash = wtoi(strFragment.c_str());
                        break;
                    case csvItem::bulletPierce:
                        itemDex[tgtIndex].bulletPierce = wtoi(strFragment.c_str());
                        break;
                    case csvItem::bulletCut:
                        itemDex[tgtIndex].bulletCut = wtoi(strFragment.c_str());
                        break;
                    case csvItem::bulletBash:
                        itemDex[tgtIndex].bulletBash = wtoi(strFragment.c_str());
                        break;
                    case csvItem::bulletStoppingPower:
                        itemDex[tgtIndex].bulletStoppingPower = wtof(strFragment.c_str());
                        break;
                    case csvItem::bulletJam:
                        itemDex[tgtIndex].bulletJam = wtoi(strFragment.c_str());
                        break;
                    case csvItem::bulletType:
                        if (strFragment == L"NORMAL") { itemDex[tgtIndex].bulletType = bulletFlag::normal; }
                        else if (strFragment == L"TRACER") { itemDex[tgtIndex].bulletType = bulletFlag::tracer; }
                        else if (strFragment == L"AP") { itemDex[tgtIndex].bulletType = bulletFlag::ap; }
                        else { errorBox(L"error in readItemDex.ixx, csvItem::bulletType"); }
                        break;
                    case csvItem::bulletRange:
                        itemDex[tgtIndex].bulletRange = wtoi(strFragment.c_str());
                        break;

                    case csvItem::gunDmg:
                        itemDex[tgtIndex].gunDmg = wtof(strFragment.c_str());
                        break;
                    case csvItem::gunAccInit:
                        itemDex[tgtIndex].gunAccInit = wtof(strFragment.c_str());
                        break;
                    case csvItem::gunAccSpd:
                        itemDex[tgtIndex].gunAccSpd = wtoi(strFragment.c_str());
                        break;
                    case csvItem::gunAccMax:
                        itemDex[tgtIndex].gunAccMax = wtof(strFragment.c_str());
                        break;
                    case csvItem::gunRebound:
                        itemDex[tgtIndex].gunRebound = wtoi(strFragment.c_str());
                        break;
                    case csvItem::gunShotSpd:
                        itemDex[tgtIndex].gunShotSpd = wtof(strFragment.c_str());
                        break;
                    case csvItem::gunReloadSpd:
                        itemDex[tgtIndex].gunReloadSpd = wtof(strFragment.c_str());
                        break;
                    case csvItem::gunJam:
                        itemDex[tgtIndex].gunJam = wtof(strFragment.c_str());
                        break;
                    case csvItem::gunMod:
                    {
                        valsToVec(strFragment, itemDex[tgtIndex].gunMod);
                        break;
                    }
                    case csvItem::gunBalance:
                        itemDex[tgtIndex].gunBalance = wtof(strFragment.c_str());
                        break;
                    case csvItem::gunRange:
                        itemDex[tgtIndex].gunRange = wtoi(strFragment.c_str());
                        break;


                    case csvItem::meleeAtkSpd:
                        itemDex[tgtIndex].meleeAtkSpd = wtof(strFragment.c_str());
                        break;
                    case csvItem::meleeAtkAcc:
                        itemDex[tgtIndex].meleeAtkAcc = wtof(strFragment.c_str());
                        break;
                    case csvItem::meleeBalance:
                        itemDex[tgtIndex].meleeBalance = wtof(strFragment.c_str());
                        break;
                    case csvItem::meleeRange:
                        itemDex[tgtIndex].meleeRange = wtoi(strFragment.c_str());
                        break;


                    case csvItem::modeTemplate:
                        itemDex[tgtIndex].modeTemplate = wtoi(strFragment.c_str());
                        if (itemDex[tgtIndex].modeTemplate == 1)
                        {
                            itemDex[tgtIndex].modeCurrent = weaponMode::semiAutoMode;
                        }
                        break;

                    case csvItem::throwAtk:
                        itemDex[tgtIndex].throwAtk = wtoi(strFragment.c_str());
                        break;
                    case csvItem::throwType:
                        itemDex[tgtIndex].throwType = wtoi(strFragment.c_str());
                        break;
                    case csvItem::throwAtkAcc:
                        itemDex[tgtIndex].throwAtkAcc = wtof(strFragment.c_str());
                        break;
                    case csvItem::throwBalance:
                        itemDex[tgtIndex].throwBalance = wtof(strFragment.c_str());
                        break;
                    case csvItem::throwRange:
                        itemDex[tgtIndex].throwRange = wtoi(strFragment.c_str());
                        break;
                    case csvItem::throwStoppingPower:
                        itemDex[tgtIndex].throwStoppingPower = wtof(strFragment.c_str());
                        break;
                    case csvItem::bookIndex:
                        itemDex[tgtIndex].bookIndex = wtoi(strFragment.c_str());
                        break;
                    case csvItem::propSprIndex:
                        itemDex[tgtIndex].propSprIndex = wtoi(strFragment.c_str());
                        break;

                    case csvItem::equipSprName:
                        errorBox(spr::spriteMapper.find(strFragment) == spr::spriteMapper.end(), L"이 아이템의 equip 이미지 파일이 spr::spriteMapper에 없음 : " + strFragment);
                        itemDex[tgtIndex].equipSpr = spr::spriteMapper[strFragment.c_str()];

                        if (itemDex[tgtIndex].checkFlag(itemFlag::HAS_TOGGLE_SPRITE))
                        {
                            itemDex[tgtIndex].equipSprToggleOn = spr::spriteMapper[(strFragment + L"On").c_str()];
                        }
                        break;
                    case csvItem::equipPriority:
                        itemDex[tgtIndex].equipPriority = wtoi(strFragment.c_str());
                        break;
                    case csvItem::flipEquipSprName:
                        errorBox(spr::spriteMapper.find(strFragment) == spr::spriteMapper.end(), L"이 아이템의 equip 이미지 파일이 spr::spriteMapper에 없음 : " + strFragment);
                        itemDex[tgtIndex].flipEquipSpr = spr::spriteMapper[strFragment.c_str()];

                        if (itemDex[tgtIndex].checkFlag(itemFlag::HAS_TOGGLE_SPRITE))
                        {
                            itemDex[tgtIndex].flipEquipSprToggleOn = spr::spriteMapper[(strFragment + L"On").c_str()];
                        }
                        break;
                    case csvItem::flipEquipPriority:
                        itemDex[tgtIndex].flipEquipPriority = wtoi(strFragment.c_str());
                        break;

                    case csvItem::leftWieldSpr:
                        errorBox(spr::spriteMapper.find(strFragment) == spr::spriteMapper.end(), L"이 아이템의 leftWield 이미지 파일이 spr::spriteMapper에 없음 : " + strFragment);
                        itemDex[tgtIndex].leftWieldSpr = spr::spriteMapper[strFragment.c_str()];
                        break;
                    case csvItem::leftWieldPriority:
                        itemDex[tgtIndex].leftWieldPriority = wtoi(strFragment.c_str());
                        break;

                    case csvItem::rightWieldSpr:
                        errorBox(spr::spriteMapper.find(strFragment) == spr::spriteMapper.end(), L"이 아이템의 rightWield 이미지 파일이 spr::spriteMapper에 없음 : " + strFragment);
                        itemDex[tgtIndex].rightWieldSpr = spr::spriteMapper[strFragment.c_str()];
                        break;
                    case csvItem::rightWieldPriority:
                        itemDex[tgtIndex].rightWieldPriority = wtoi(strFragment.c_str());
                        break;
                        ///////////////////////////////////////////
                    case csvItem::tileSprIndex:
                        itemDex[tgtIndex].tileSprIndex = wtoi(strFragment.c_str());
                        break;
                    case csvItem::tileConnectGroup:
                        itemDex[tgtIndex].tileConnectGroup = wtoi(strFragment.c_str());
                        break;
                    case csvItem::dirChangeItemCode:
                        itemDex[tgtIndex].dirChangeItemCode = wtoi(strFragment.c_str());
                        break;

                    case csvItem::lightRange:
                        itemDex[tgtIndex].lightRange = wtoi(strFragment.c_str());
                        break;
                    case csvItem::lightIntensity:
                        itemDex[tgtIndex].lightIntensity = wtoi(strFragment.c_str());
                        break;
                    case csvItem::lightR:
                        itemDex[tgtIndex].lightR = wtoi(strFragment.c_str());
                        break;
                    case csvItem::lightG:
                        itemDex[tgtIndex].lightG = wtoi(strFragment.c_str());
                        break;
                    case csvItem::lightB:
                        itemDex[tgtIndex].lightB = wtoi(strFragment.c_str());
                        break;
                    case csvItem::lightDelX:
                        itemDex[tgtIndex].lightDelX = wtoi(strFragment.c_str());
                        break;
                    case csvItem::lightDelY:
                        itemDex[tgtIndex].lightDelY = wtoi(strFragment.c_str());
                        break;

                    case csvItem::animeSize:
                        itemDex[tgtIndex].animeSize = wtoi(strFragment.c_str());
                        break;
                    case csvItem::animeFPS:
                        itemDex[tgtIndex].animeFPS = wtoi(strFragment.c_str());
                        break;
                    case csvItem::randomPropSprSize:
                        itemDex[tgtIndex].randomPropSprSize = wtoi(strFragment.c_str());
                        break;
                    case csvItem::growthThreshold:
                        break;
                    case csvItem::molecularWeight:
                        itemDex[tgtIndex].molecularWeight = wtoi(strFragment.c_str());
                        break;
                    case csvItem::liqColorR:
                        itemDex[tgtIndex].liqColorR = wtoi(strFragment.c_str());
                        break;
                    case csvItem::liqColorG:
                        itemDex[tgtIndex].liqColorG = wtoi(strFragment.c_str());
                        break;
                    case csvItem::liqColorB:
                        itemDex[tgtIndex].liqColorB = wtoi(strFragment.c_str());
                        break;
                    case csvItem::gasColorR:
                        itemDex[tgtIndex].gasColorR = wtoi(strFragment.c_str());
                        break;
                    case csvItem::gasColorG:
                        itemDex[tgtIndex].gasColorG = wtoi(strFragment.c_str());
                        break;
                    case csvItem::gasColorB:
                        itemDex[tgtIndex].gasColorB = wtoi(strFragment.c_str());
                        break;
                    case csvItem::gasAlphaMax:
                        itemDex[tgtIndex].gasAlphaMax = wtoi(strFragment.c_str());
                        break;
                    case csvItem::propMaxHP:
                        itemDex[tgtIndex].propMaxHP = wtoi(strFragment.c_str());
                        break;
                    case csvItem::propRPierce:
                        itemDex[tgtIndex].propRPierce = wtoi(strFragment.c_str());
                        break;
                    case csvItem::propRCut:
                        itemDex[tgtIndex].propRCut = wtoi(strFragment.c_str());
                        break;
                    case csvItem::propRBash:
                        itemDex[tgtIndex].propRBash = wtoi(strFragment.c_str());
                        break;
                    case csvItem::propDrawPriority:
                        itemDex[tgtIndex].propDrawPriority = wtoi(strFragment.c_str());
                        break;
                    case csvItem::propWIPSprIndex:
                        itemDex[tgtIndex].propWIPSprIndex = wtoi(strFragment.c_str());
                        break;
                    case csvItem::propInstallCode:
                        itemDex[tgtIndex].propInstallCode = wtoi(strFragment.c_str());
                        break;
                    case csvItem::propUninstallCode:
                        itemDex[tgtIndex].propUninstallCode = wtoi(strFragment.c_str());
                        break;
                    case csvItem::electricResistance:
                        itemDex[tgtIndex].electricResistance = wtof(strFragment.c_str());
                        break;
                    case csvItem::electricMaxPower:
						itemDex[tgtIndex].electricMaxPower = wtoi(strFragment.c_str());
                        break;
                    case csvItem::gndUsePower:
                        itemDex[tgtIndex].gndUsePower = wtoi(strFragment.c_str());
                        break;
                    case csvItem::gndUsePowerRight:
                        itemDex[tgtIndex].gndUsePowerRight = wtoi(strFragment.c_str());
                        break;
                    case csvItem::gndUsePowerUp:
                        itemDex[tgtIndex].gndUsePowerUp = wtoi(strFragment.c_str());
                        break;
                    case csvItem::gndUsePowerLeft:
                        itemDex[tgtIndex].gndUsePowerLeft = wtoi(strFragment.c_str());
                        break;
                    case csvItem::gndUsePowerDown:
                        itemDex[tgtIndex].gndUsePowerDown = wtoi(strFragment.c_str());
                        break;
                    default:
                        errorBox(L"readItemDex.ixx에서 오류 발생. csv의 잘못된 장소를 읽었다.");
                        break;
                    }

                    //prt(L"[문자열] %ws ", strFragment.c_str());
                    //\prt(L"를 (%d,%d)에 입력했다.\n", tgtIndex, arrayCounter % (csvWidth));
                }

                arrayCounter++;

                startPoint = -1;
                endPoint = -1;
                strFragment.clear();

                if (i != str.size() - 1)//만약 다음 글자가 연속으로 쉼표면 여백이므로 건너뜀
                {
                    while (str[i + 1] == UNI::TAB || str[i + 1] == 10)
                    {
                        arrayCounter++;
                        i++;
                    }
                }
            }
            else
            {
                if (startPoint == -1)
                {
                    startPoint = i;
                }
            }
        }
        //itemDex.erase(itemDex.begin());//0번째 라벨 삭제
        return 1;
    }
    else//읽기 실패
    {

        return 0;
    }
}


