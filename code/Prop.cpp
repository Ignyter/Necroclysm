import Prop;

#include <SDL3/SDL.h>

import util;
import globalVar;
import wrapVar;
import constVar;
import World;
import ItemStack;
import ItemPocket;
import ItemData;
import Ani;
import AI;
import Light;
import Coord;
import log;
import Drawable;
import Sticker;
import Particle;


Prop::Prop(Point3 inputCoor, int leadItemCode)
{
    leadItem = cloneFromItemDex(itemDex[leadItemCode], 1);
    setAniPriority(3);
    //prt(L"[Prop:constructor] 생성자가 호출되었다. 생성된 좌표는 %d,%d,%d이다.\n", inputCoor.x, inputCoor.y, inputCoor.z);
    setGrid(inputCoor.x, inputCoor.y, inputCoor.z);

    errorBox(TileProp(inputCoor.x, inputCoor.y, inputCoor.z) != nullptr, L"생성위치에 이미 설치물이 존재한다!");

    if (leadItem.checkFlag(itemFlag::LIGHT_ON))
    {
        leadItem.lightPtr = std::make_unique<Light>(inputCoor.x + leadItem.lightDelX, inputCoor.y + leadItem.lightDelY, inputCoor.z, leadItem.lightRange, leadItem.lightIntensity, SDL_Color{ leadItem.lightR,leadItem.lightG,leadItem.lightB });//임시로 이렇게 만들어놨음
        //PlayerPtr->updateVision(PlayerPtr->entityInfo.eyeSight);
    }


    deactivateAI();//차량을 제외하고 기본적으로 비활성화



    if (leadItem.randomPropSprSize != 1)
    {
        leadItem.propSprIndex += randomRange(0, leadItem.randomPropSprSize - 1);
    }




    //HP 설정
    leadItem.propHP = leadItem.propMaxHP;
    leadItem.propFakeHP = leadItem.propMaxHP;


}

Prop::~Prop()
{
    Point2 currentChunkCoord = World::ins()->changeToSectorCoord(getGridX(), getGridY());
    Chunk& currentChunk = World::ins()->getChunk(currentChunkCoord.x, currentChunkCoord.y, getGridZ());
    currentChunk.eraseProp(this);

    if (leadItem.checkFlag(itemFlag::CABLE))
    {
        if (leadItem.checkFlag(itemFlag::CABLE_Z_ASCEND))
        {
            Prop* abovePropPtr = TileProp(getGridX(), getGridY(), getGridZ() + 1);
            if (abovePropPtr != nullptr) abovePropPtr->leadItem.eraseFlag(itemFlag::CABLE_Z_DESCEND);
        }

        if (leadItem.checkFlag(itemFlag::CABLE_Z_DESCEND))
        {
            Prop* belowPropPtr = TileProp(getGridX(), getGridY(), getGridZ() - 1);
            if (belowPropPtr != nullptr) belowPropPtr->leadItem.eraseFlag(itemFlag::CABLE_Z_ASCEND);
        }
    }

    prt(L"[Prop:destructor] 소멸자가 호출되었다. \n");
}

void Prop::setGrid(int inputGridX, int inputGridY, int inputGridZ)
{
    Point2 prevChunkCoord = World::ins()->changeToSectorCoord(getGridX(), getGridY());
    Chunk& prevChunk = World::ins()->getChunk(prevChunkCoord.x, prevChunkCoord.y, getGridZ());
    prevChunk.eraseProp(this);

    Coord::setGrid(inputGridX, inputGridY, inputGridZ);

    Point2 currentChunkCoord = World::ins()->changeToSectorCoord(getGridX(), getGridY());
    Chunk& currentChunk = World::ins()->getChunk(currentChunkCoord.x, currentChunkCoord.y, getGridZ());
    currentChunk.addProp(this);
}

void Prop::updateSprIndex()
{
    bool topTile = false;
    bool botTile = false;
    bool leftTile = false;
    bool rightTile = false;


    if (leadItem.checkFlag(itemFlag::CABLE))//전선일 경우
    {
        auto wireCheck = [this](int dx, int dy) -> bool
            {
                Prop* tgtProp = TileProp(getGridX() + dx, getGridY() + dy, getGridZ());
                if (tgtProp != nullptr)
                {
                    if (tgtProp->leadItem.checkFlag(itemFlag::CABLE))//같은 전선일 경우
                    {
                        return true;
                    }
                    else
                    {
                        if ((dx == 0 && dy == -1) && tgtProp->leadItem.checkFlag(itemFlag::CABLE_CNCT_DOWN)) return true;
                        else if ((dx == 0 && dy == 1) && tgtProp->leadItem.checkFlag(itemFlag::CABLE_CNCT_UP)) return true;
                        else if ((dx == 1 && dy == 0) && tgtProp->leadItem.checkFlag(itemFlag::CABLE_CNCT_LEFT)) return true;
                        else if ((dx == -1 && dy == 0) && tgtProp->leadItem.checkFlag(itemFlag::CABLE_CNCT_RIGHT)) return true;
                        else return false;
                    }
                }
                else return false;
            };

        topTile = wireCheck(0, -1);
        botTile = wireCheck(0, 1);
        leftTile = wireCheck(-1, 0);
        rightTile = wireCheck(1, 0);
        leadItem.extraSprIndexSingle = connectGroupExtraIndex(topTile, botTile, leftTile, rightTile);
    }
    else if (leadItem.checkFlag(itemFlag::PIPE))//배관일 경우
    {
        auto pipeCheck = [=](int dx, int dy) -> bool
            {
                Prop* tgtProp = TileProp(getGridX() + dx, getGridY() + dy, getGridZ());
                if (tgtProp != nullptr)
                {
                    if (tgtProp->leadItem.checkFlag(itemFlag::PIPE))//같은 전선일 경우
                    {
                        return true;
                    }
                    else
                    {
                        if ((dx == 0 && dy == -1) && tgtProp->leadItem.checkFlag(itemFlag::PIPE_CNCT_DOWN)) return true;
                        else if ((dx == 0 && dy == 1) && tgtProp->leadItem.checkFlag(itemFlag::PIPE_CNCT_UP)) return true;
                        else if ((dx == 1 && dy == 0) && tgtProp->leadItem.checkFlag(itemFlag::PIPE_CNCT_LEFT)) return true;
                        else if ((dx == -1 && dy == 0) && tgtProp->leadItem.checkFlag(itemFlag::PIPE_CNCT_RIGHT)) return true;
                        else return false;
                    }
                }
                else return false;
            };

        topTile = pipeCheck(0, -1);
        botTile = pipeCheck(0, 1);
        leftTile = pipeCheck(-1, 0);
        rightTile = pipeCheck(1, 0);
        leadItem.extraSprIndexSingle = connectGroupExtraIndex(topTile, botTile, leftTile, rightTile);
    }
    else if (leadItem.checkFlag(itemFlag::CONVEYOR))//컨베이어 벨트일 경우
    {
        auto conveyorCheck = [=](int dx, int dy) -> bool
            {
                Prop* tgtProp = TileProp(getGridX() + dx, getGridY() + dy, getGridZ());
                if (tgtProp != nullptr)
                {
                    if (tgtProp->leadItem.checkFlag(itemFlag::CONVEYOR))//같은 전선일 경우
                    {
                        return true;
                    }
                    else
                    {
                        if ((dx == 0 && dy == -1) && tgtProp->leadItem.checkFlag(itemFlag::CONVEYOR_CNCT_DOWN)) return true;
                        else if ((dx == 0 && dy == 1) && tgtProp->leadItem.checkFlag(itemFlag::CONVEYOR_CNCT_UP)) return true;
                        else if ((dx == 1 && dy == 0) && tgtProp->leadItem.checkFlag(itemFlag::CONVEYOR_CNCT_LEFT)) return true;
                        else if ((dx == -1 && dy == 0) && tgtProp->leadItem.checkFlag(itemFlag::CONVEYOR_CNCT_RIGHT)) return true;
                        else return false;
                    }
                }
                else return false;
            };

        topTile = conveyorCheck(0, -1);
        botTile = conveyorCheck(0, 1);
        leftTile = conveyorCheck(-1, 0);
        rightTile = conveyorCheck(1, 0);
        leadItem.extraSprIndexSingle = connectGroupExtraIndex(topTile, botTile, leftTile, rightTile);
    }
    else if (leadItem.tileConnectGroup != -1)
    {
        auto sameCheck = [=](int dx, int dy) -> bool
            {
                Prop* tgtProp = TileProp(getGridX() + dx, getGridY() + dy, getGridZ());
                if (tgtProp != nullptr)
                {
                    if (tgtProp->leadItem.tileConnectGroup != 0)
                    {
                        if (tgtProp->leadItem.tileConnectGroup == leadItem.tileConnectGroup)
                        {
                            return true;
                        }
                        else return false;
                    }
                    else
                    {
                        if (tgtProp->leadItem.itemCode == leadItem.itemCode)
                        {
                            return true;
                        }
                        else return false;
                    }
                }
                else return false;
            };

        int x = getGridX();
        int y = getGridY();

        topTile = sameCheck(0, -1);
        botTile = sameCheck(0, 1);
        leftTile = sameCheck(-1, 0);
        rightTile = sameCheck(1, 0);
        leadItem.extraSprIndexSingle = connectGroupExtraIndex(topTile, botTile, leftTile, rightTile);
    }
}

bool Prop::runAI()
{
    //prt(L"[Prop:AI] ID : %p의 AI를 실행시켰다.\n", this);
    while (1)
    {

        //prt(L"[Prop:AI] ID : %p의 turnResource는 %f입니다.\n", this, getTurnResource());
        if (getTurnResource() >= 2.0)
        {
            clearTurnResource();
            addTurnResource(2.0);
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //위의 모든 패턴 조건을 만족하지않을시 return true
        //prt(L"[Prop:AI] AI가 true를 반환했다. AI를 종료합니다.\n");
        return true;
    }
}

bool Prop::runAnimation(bool shutdown)
{
    //prt(L"Prop %p의 runAnimation이 실행되었다.\n", this);
    if (getAniType() == aniFlag::propRush)
    {
        addTimer();
    }
    else if (getAniType() == aniFlag::drop)
    {
        addTimer();
        int pX = getX();
        int pY = getY();

        if (getTimer() == 1) setFakeY(-4);
        else if (getTimer() == 2) setFakeY(-5);
        else if (getTimer() == 4) setFakeY(-6);
        else if (getTimer() == 7) setFakeY(-7);
        else if (getTimer() == 10) setFakeY(-6);
        else if (getTimer() == 12) setFakeY(-5);
        else if (getTimer() == 13) setFakeY(-4);
        else if (getTimer() == 16)
        {
            setFakeY(0);
            resetTimer();
            setAniType(aniFlag::null);
            return true;
        }
    }
    else if (getAniType() == aniFlag::treeFalling)
    {
        addTimer();

        if (getTimer() == 1) leadItem.addFlag(itemFlag::STUMP);

        if (PlayerX() <= getGridX()) treeAngle += 0.7 + (float)(getTimer()) * 0.04;
        else treeAngle -= 0.7 + (float)(getTimer()) * 0.04;

        if (treeAngle >= 90.0 || treeAngle <= -90.0)
        {
            Point3 itemPos;
            ItemStack* itemPtr1;
            ItemStack* itemPtr2;
            if (treeAngle >= 90.0)
            {
                if (1/*TileWall(getGridX() + 1, getGridY(), getGridZ()) == 0*/) itemPos = { getGridX() + 1, getGridY(), getGridZ() };
                else  itemPos = { getGridX(), getGridY(), getGridZ() };

                createItemStack(itemPos, { {392,1} });
                for (int i = 0; i < 8; i++)
                {
                    new Particle(getX() + 16 + randomRange(-16, 16), getY() + randomRange(0, 8), randomRange(0, 7), randomRangeFloat(-1.2, 1.2), randomRangeFloat(-2.6, -3.2), 0.18, randomRange(25, 35));
                }
            }
            else
            {
                if (1/*TileWall(getGridX() - 1, getGridY(), getGridZ()) == 0*/) itemPos = { getGridX() - 1, getGridY(), getGridZ() };
                else  itemPos = { getGridX(), getGridY(), getGridZ() };

                createItemStack(itemPos, { {392,1} });
                for (int i = 0; i < 8; i++)
                {
                    new Particle(getX() - 16 + randomRange(-16, 16), getY() + randomRange(0, 8), randomRange(0, 7), randomRangeFloat(-1.2, 1.2), randomRangeFloat(-2.6, -3.2), 0.18, randomRange(25, 35));
                }
            }


            addAniUSetPlayer(TileItemStack(itemPos.x, itemPos.y, itemPos.z), aniFlag::drop);
            treeAngle = 0;
            resetTimer();
            setAniType(aniFlag::null);
            return true;
        }
    }
    return false;
}





void Prop::runPropFunc()
{
    if (runUsed) return;

    if (leadItem.electricUsePower > 0) finalLoadSet.insert(this);

    if (leadItem.checkFlag(itemFlag::CIRCUIT))
    {
        updateCircuitNetwork();
    }

    runUsed = true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




void Prop::propTurnOn()
{
    leadItem.eraseFlag(itemFlag::PROP_POWER_OFF);
    leadItem.addFlag(itemFlag::PROP_POWER_ON);

    int iCode = leadItem.itemCode;
    if (iCode == itemRefCode::bollardLight)
    {
        leadItem.lightPtr = std::make_unique<Light>(getGridX() + leadItem.lightDelX, getGridY() + leadItem.lightDelY, getGridZ(), leadItem.lightRange, leadItem.lightIntensity, SDL_Color{ leadItem.lightR,leadItem.lightG,leadItem.lightB });
    }

}

void Prop::propTurnOff()
{
    leadItem.eraseFlag(itemFlag::PROP_POWER_ON);
    leadItem.addFlag(itemFlag::PROP_POWER_OFF);

    int iCode = leadItem.itemCode;
    if (iCode == itemRefCode::bollardLight)
    {
        leadItem.lightPtr = nullptr;
    }
}