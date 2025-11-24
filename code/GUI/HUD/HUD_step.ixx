import HUD;
import globalVar;
import wrapVar;
import Profic;
import Sleep;

void HUD::step()
{
    tabType = tabFlag::attackNearby;

    if (GUI::getLastGUI() == this)
    {
        gamepadStep();
        mouseStep();
    }
    //현재 수련 중인 재능이 없을 경우 강제로 재능 창을 열음
    if (Profic::ins() == nullptr)
    {
        for (int i = 0; i < TALENT_SIZE; i++)
        {
            if (PlayerPtr->entityInfo.proficFocus[i] > 0) { break; }
            if (i == TALENT_SIZE - 1)
            {
                new Profic();
                Profic::ins()->setWarningIndex(1);
            }
        }
    }
    if (doPopUpSingleHUD == true)
    {
        executePopUpSingle();
        doPopUpSingleHUD = false;
    }
    else if (doPopDownHUD == true)
    {
        executePopDown();
        doPopDownHUD = false;
    }

    static int hungryDisplayTimer = 0;
    static int thirstDisplayTimer = 0;
    static int fatigueDisplayTimer = 0;
    auto& statInfo = PlayerPtr->entityInfo.statusEffectVec;

    int currentHunger = hunger;
    int currentThirst = thirst;
    int currentFatigue = fatigue;

    // 배고픔 상태 관리
    if (fakeHunger < currentHunger && checkStatusEffect(statInfo, statusEffectFlag::hungry) == false)
    {
        hungryDisplayTimer = 0;
        statInfo.push_back({ statusEffectFlag::hungry, -1 });
    }

    if (fakeHunger < PLAYER_HUNGRY_CALORIE && checkStatusEffect(statInfo, statusEffectFlag::hungry) == false)
    {
        hungryDisplayTimer = 0;
        statInfo.push_back({ statusEffectFlag::hungry, -1 });
    }
    else if (fakeHunger >= PLAYER_HUNGRY_CALORIE&& checkStatusEffect(statInfo, statusEffectFlag::hungry) == true&& fakeHunger == currentHunger)
    {
        if (hungryDisplayTimer == 0) hungryDisplayTimer = 180;
        hungryDisplayTimer--;
        if (hungryDisplayTimer == 0)
        {
            eraseStatusEffect(statInfo, statusEffectFlag::hungry);
        }
    }

    // 갈증 상태 관리
    if (fakeThirst < currentThirst && checkStatusEffect(statInfo, statusEffectFlag::dehydrated) == false)
    {
        thirstDisplayTimer = 0;
        statInfo.push_back({ statusEffectFlag::dehydrated, -1 });
    }
    
    if (fakeThirst < PLAYER_THIRSTY_HYDRATION && checkStatusEffect(statInfo, statusEffectFlag::dehydrated) == false)
    {
        thirstDisplayTimer = 0;
        statInfo.push_back({ statusEffectFlag::dehydrated, -1 });
    }
    else if (fakeThirst >= PLAYER_THIRSTY_HYDRATION && checkStatusEffect(statInfo, statusEffectFlag::dehydrated) == true && fakeThirst == currentThirst)
    {
        if (thirstDisplayTimer == 0) thirstDisplayTimer = 180;
        thirstDisplayTimer--;
        if (thirstDisplayTimer == 0)
        {
            eraseStatusEffect(statInfo, statusEffectFlag::dehydrated);
        }
    }

    // 배고픔 상태 관리
    if (fakeHunger < currentHunger && checkStatusEffect(statInfo, statusEffectFlag::hungry) == false)
    {
        hungryDisplayTimer = 0;
        statInfo.push_back({ statusEffectFlag::hungry, -1 });
    }

    if(getLastGUI() == Sleep::ins() && checkStatusEffect(statInfo, statusEffectFlag::tired) == false)
    {
        fatigueDisplayTimer = 0;
        statInfo.push_back({ statusEffectFlag::tired, -1 });
    }

    if (fakeFatigue < PLAYER_TIRED_FATIGUE && checkStatusEffect(statInfo, statusEffectFlag::tired) == false)
    {
        fatigueDisplayTimer = 0;
        statInfo.push_back({ statusEffectFlag::tired, -1 });
    }
    else if (fakeFatigue >= PLAYER_TIRED_FATIGUE && checkStatusEffect(statInfo, statusEffectFlag::tired) == true && fakeFatigue == currentFatigue && getLastGUI() != Sleep::ins())
    {
        if (fatigueDisplayTimer == 0) fatigueDisplayTimer = 180;
        fatigueDisplayTimer--;
        if (fatigueDisplayTimer == 0)
        {
            eraseStatusEffect(statInfo, statusEffectFlag::tired);
        }
    }
}