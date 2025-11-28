import Vehicle;

#include <SDL3/SDL.h>

import std;
import globalVar;
import wrapVar;
import textureVar;
import constVar;
import util;
import World;
import ItemPocket;
import ItemData;
import Entity;
import Player;
import AI;
import Light;
import Ani;
import Coord;
import Drawable;
import log;
import Prop;
import drawSprite;

Vehicle::Vehicle(int inputX, int inputY, int inputZ, int leadItemCode)
{
    trainWheelCenter = { inputX, inputY };

    setAniPriority(3);
    prt(L"[Vehicle:constructor] 생성자가 호출되었다. 생성된 좌표는 %d,%d,%d이다.\n", inputX, inputY, inputZ);
    setGrid(inputX, inputY, inputZ);

    errorBox(TileVehicle(inputX, inputY, inputZ) != nullptr, L"생성위치에 이미 프롭이 존재한다!");
    TileVehicle(inputX, inputY, inputZ) = this;

    partInfo[{inputX, inputY}] = std::make_unique<ItemPocket>(storageType::null);
    partInfo[{inputX, inputY}]->addItemFromDex(leadItemCode, 1);


    deactivateAI();//차량을 제외하고 기본적으로 비활성화
}

Vehicle::~Vehicle()
{
    Point2 currentChunkCoord = World::ins()->changeToSectorCoord(getGridX(), getGridY());
    Chunk& currentChunk = World::ins()->getChunk(currentChunkCoord.x, currentChunkCoord.y, getGridZ());
    currentChunk.eraseVehicle(this);

    prt(L"[Vehicle:destructor] 소멸자가 호출되었다. \n");
}



bool Vehicle::hasFrame(int inputX, int inputY)
{
    auto it = partInfo.find({ inputX, inputY });
    if (it != partInfo.end())
    {
        std::vector<ItemData>& vParts = partInfo[{inputX, inputY}]->itemInfo;
        for (int i = 0; i < vParts.size(); i++)
        {
            if (vParts[i].checkFlag(itemFlag::VFRAME)) return true;
        }
    }
    return false;
}


/////////////////////////////////////////※ 기존 프레임에 부품 추가////////////////////////////////////////////////////
void Vehicle::addPart(int inputX, int inputY, int dexIndex)
{
    errorBox(partInfo.find({ inputX, inputY }) == partInfo.end(), L"[Vehicle:addPart] 입력한 위치에 프레임이 존재하지 않는다.");

    ItemData inputPart = cloneFromItemDex(itemDex[dexIndex], 1);

    // 타이어의 경우 맨 앞에 추가 (기존 로직 유지)
    if (inputPart.checkFlag(itemFlag::TIRE_NORMAL) || inputPart.checkFlag(itemFlag::TIRE_STEER))
    {
        partInfo[{inputX, inputY}]->itemInfo.insert(partInfo[{inputX, inputY}]->itemInfo.begin(), std::move(inputPart));
    }
    else
    {
        // 새로운 부품의 priority
        int newPriority = inputPart.propDrawPriority;

        // priority에 따른 삽입 위치 찾기
        auto& itemVec = partInfo[{inputX, inputY}]->itemInfo;
        auto insertPos = itemVec.end();

        // 뒤에서부터 탐색하여 새 부품보다 낮거나 같은 priority를 가진 위치 찾기
        for (auto it = itemVec.rbegin(); it != itemVec.rend(); ++it)
        {
            if (it->propDrawPriority <= newPriority)
            {
                insertPos = it.base(); // reverse_iterator를 forward_iterator로 변환
                break;
            }
        }

        itemVec.insert(insertPos, std::move(inputPart));
    }

    // 열차바퀴 중심 설정
    if (inputPart.checkFlag(itemFlag::TRAIN_WHEEL)) updateTrainCenter();

    updateSpr();
}
void Vehicle::addPart(int inputX, int inputY, std::vector<int> dexVec)
{
    for (int i = 0; i < dexVec.size(); i++) addPart(inputX, inputY, dexVec[i]);
}

void Vehicle::erasePart(int inputX, int inputY, int index)
{
    if (partInfo[{ inputX, inputY }]->itemInfo[index].checkFlag(itemFlag::TRAIN_WHEEL)) updateTrainCenter();

    partInfo[{ inputX, inputY }]->eraseItemInfo(index);
}

//////////////////////////////////////////////※ 프레임 확장/////////////////////////////////////////////////////////
void Vehicle::extendPart(int inputX, int inputY, int inputItemCode)
{
    //십자 방향에 부품이 있는지 체크
    for (int i = 0; i < 4; i++)
    {
        int dir = 2 * i;
        int dx, dy;
        dir2Coord(dir, dx, dy);
        //존재할 경우
        if (partInfo.find({ inputX + dx, inputY + dy }) != partInfo.end()) break;
        errorBox(i == 3, L"[Vehicle:extendPart] 상하좌우에 프레임이 없는데 해당 타일로 확장을 시도했다.");
    }
    errorBox(partInfo.find({ inputX, inputY }) != partInfo.end(), L"[Vehicle:extendPart] 이미 이 프롭 프레임이 있는 좌표로 확장을 시도했다.");

    partInfo[{inputX, inputY}] = std::make_unique<ItemPocket>(storageType::null);
    partInfo[{inputX, inputY}]->addItemFromDex(inputItemCode);
    TileVehicle(inputX, inputY, getGridZ()) = this;

    //prt(L"[Vehicle:extendPart] %p 차량이 %d,%d 위치로 %d 아이템을 확장에 성공다.\n", inputX, inputY, inputItemCode);
    updateSpr();
}

void Vehicle::setGrid(int inputGridX, int inputGridY, int inputGridZ)
{
    Point2 prevChunkCoord = World::ins()->changeToChunkCoord(getGridX(), getGridY());
    Chunk& prevChunk = World::ins()->getChunk(prevChunkCoord.x, prevChunkCoord.y, getGridZ());
    prevChunk.eraseVehicle(this);

    Coord::setGrid(inputGridX, inputGridY, inputGridZ);

    Point2 currentChunkCoord = World::ins()->changeToChunkCoord(getGridX(), getGridY());
    Chunk& currentChunk = World::ins()->getChunk(currentChunkCoord.x, currentChunkCoord.y, getGridZ());
    currentChunk.addVehicle(this);
}

int Vehicle::getSprIndex(int inputX, int inputY)
{
    errorBox(partInfo[{inputX, inputY}]->itemInfo.size() == 0, L"[Vehicle:getSprIndex] The vehicle part are empty(itemInfo size is zero)");
    return getItemSprIndex(partInfo[{inputX, inputY}]->itemInfo[0]);
}

void Vehicle::rotatePartInfo(dir16 inputDir16)
{
    if (bodyDir != inputDir16)
    {
        std::unordered_map<Point2, std::unique_ptr<ItemPocket>, Point2::Hash> newPartInfo;
        auto currentCoordTransform = coordTransform[bodyDir];
        auto targetCoordTransform = coordTransform[inputDir16];
        for (int x = getGridX() - MAX_VEHICLE_SIZE / 2; x <= getGridX() + MAX_VEHICLE_SIZE / 2; x++)
        {
            for (int y = getGridY() - MAX_VEHICLE_SIZE / 2; y <= getGridY() + MAX_VEHICLE_SIZE / 2; y++)
            {
                if (partInfo.find({ x,y }) != partInfo.end())
                {
                    Point2 originCoord = currentCoordTransform[{x - getGridX(), y - getGridY()}];
                    Point2 dstCoord;
                    for (const auto& [coord, transformedCoord] : targetCoordTransform)
                    {
                        if (transformedCoord == originCoord)
                        {
                            dstCoord = coord;
                            break;
                        }
                    }
                    newPartInfo[{dstCoord.x + getGridX(), dstCoord.y + getGridY()}] = std::move(partInfo[{x, y}]);
                }
            }
        }
        partInfo = std::move(newPartInfo);
    }
}

std::unordered_set<Point2, Point2::Hash> Vehicle::getRotateShadow(dir16 inputDir16)
{
    if (bodyDir != inputDir16)
    {
        std::unordered_set<Point2, Point2::Hash> newPartInfo;
        auto currentCoordTransform = coordTransform[bodyDir];
        auto targetCoordTransform = coordTransform[inputDir16];
        for (int x = getGridX() - MAX_VEHICLE_SIZE / 2; x <= getGridX() + MAX_VEHICLE_SIZE / 2; x++)
        {
            for (int y = getGridY() - MAX_VEHICLE_SIZE / 2; y <= getGridY() + MAX_VEHICLE_SIZE / 2; y++)
            {
                if (partInfo.find({ x,y }) != partInfo.end())
                {
                    Point2 originCoord = currentCoordTransform[{x - getGridX(), y - getGridY()}];
                    Point2 dstCoord;
                    for (const auto& [coord, transformedCoord] : targetCoordTransform)
                    {
                        if (transformedCoord == originCoord)
                        {
                            dstCoord = coord;
                            break;
                        }
                    }
                    newPartInfo.insert({ dstCoord.x + getGridX(), dstCoord.y + getGridY() });
                }
            }
        }
        return newPartInfo;
    }
    else
    {
        std::unordered_set<Point2, Point2::Hash> newPartInfo;
        for (const auto& [pos, pocket] : partInfo)
        {
            newPartInfo.insert({ pos.x, pos.y });
        }
        return newPartInfo;
    }
}

void Vehicle::rotateEntityPtr(dir16 inputDir16)
{
    if (bodyDir != inputDir16)
    {
        std::unordered_map<Point2, std::unique_ptr<Entity>, Point2::Hash> entityWormhole; //엔티티를 새로운 좌표로 옮기기 전에 임시적으로 저장하는 컨테이너
        for (const auto& [pos, pocket] : partInfo)
        {
            if (TileEntity(pos.x, pos.y, getGridZ()) != nullptr)
            {
                entityWormhole[{pos.x, pos.y}] = std::move(World::ins()->getTile(pos.x, pos.y, getGridZ()).EntityPtr);
            }
        }

        auto currentCoordTransform = coordTransform[bodyDir];
        auto targetCoordTransform = coordTransform[inputDir16];
        for (int x = getGridX() - MAX_VEHICLE_SIZE / 2; x <= getGridX() + MAX_VEHICLE_SIZE / 2; x++)
        {
            for (int y = getGridY() - MAX_VEHICLE_SIZE / 2; y <= getGridY() + MAX_VEHICLE_SIZE / 2; y++)
            {
                if (partInfo.find({ x,y }) != partInfo.end())
                {
                    Point2 originCoord = currentCoordTransform[{x - getGridX(), y - getGridY()}];
                    Point2 dstCoord;
                    for (const auto& [coord, transformedCoord] : targetCoordTransform)
                    {
                        if (transformedCoord == originCoord)
                        {
                            dstCoord = coord;
                            break;
                        }
                    }

                    if (entityWormhole.find({ x, y }) != entityWormhole.end())
                    {
                        EntityPtrMove(std::move(entityWormhole[{x, y}]), { dstCoord.x + getGridX(), dstCoord.y + getGridY(), getGridZ() });
                    }
                }
            }
        }
    }
}

void Vehicle::rotate(dir16 inputDir16)
{
    if (bodyDir != inputDir16)
    {
        for (const auto& [pos, pocket] : partInfo)
        {
            TileVehicle(pos.x, pos.y, getGridZ()) = nullptr;
        }

        rotateEntityPtr(inputDir16);
        rotatePartInfo(inputDir16);

        for (const auto& [pos, pocket] : partInfo)
        {
            TileVehicle(pos.x, pos.y, getGridZ()) = this;
        }

        //회전하는 방향에 대해 바퀴 방향 재설정
        if (ACW2(bodyDir) == wheelDir) wheelDir = ACW2(inputDir16);
        else if (ACW(bodyDir) == wheelDir) wheelDir = ACW(inputDir16);
        else if (CW(bodyDir) == wheelDir) wheelDir = CW(inputDir16);
        else if (CW2(bodyDir) == wheelDir) wheelDir = CW2(inputDir16);
        else wheelDir = inputDir16;

        bodyDir = inputDir16;
    }
    //else errorBox(L"[Vehicle:roate] 이미 차량이 해당 방향을 향하고 있다.");
    updateSpr();
    updateHeadlight();

}

void Vehicle::updateSpr()
{
    dir16 refDir;
    switch (bodyDir)
    {
    case dir16::dir1:
    case dir16::dir1_5:
    case dir16::dir2:
    case dir16::dir2_5:
    case dir16::dir3:
        refDir = dir16::dir2;
        break;
    case dir16::dir3_5:
    case dir16::dir4:
    case dir16::dir4_5:
        refDir = dir16::dir4;
        break;
    case dir16::dir5:
    case dir16::dir5_5:
    case dir16::dir6:
    case dir16::dir6_5:
    case dir16::dir7:
        refDir = dir16::dir6;
        break;
    case dir16::dir7_5:
    case dir16::dir0:
    case dir16::dir0_5:
        refDir = dir16::dir0;
        break;
    }

    for (const auto& [pos, pocket] : partInfo)
    {
        int tgtX = pos.x;
        int tgtY = pos.y;
        int tgtRelX = tgtX - getGridX();
        int tgtRelY = tgtY - getGridY();
        ItemPocket* tgtPocket = pocket.get();
        for (int layer = 0; layer < tgtPocket->itemInfo.size(); layer++)
        {
            if (tgtPocket->itemInfo[layer].checkFlag(itemFlag::VPART_WALL_CONNECT))
            {
                auto checkWallGroup = [=](int dx, int dy)->bool
                    {
                        int currentGroup = tgtPocket->itemInfo[layer].tileConnectGroup;
                        Point2 value1 = coordTransform[bodyDir][{tgtRelX, tgtRelY}];
                        Point2 key1;
                        for (const auto& [coord, transformedCoord] : coordTransform[refDir])
                        {
                            if (transformedCoord == value1)
                            {
                                key1 = coord;
                            }
                        }
                        Point2 value2 = coordTransform[refDir][{key1.x + dx, key1.y + dy}];
                        Point2 key2;
                        for (const auto& [coord, transformedCoord] : coordTransform[bodyDir])
                        {
                            if (transformedCoord == value2)
                            {
                                key2 = coord;
                            }
                        }
                        if (partInfo.find({ getGridX() + key2.x, getGridY() + key2.y }) != partInfo.end())
                        {
                            std::vector<ItemData>& tgtItemInfo = partInfo[{getGridX() + key2.x, getGridY() + key2.y}]->itemInfo;
                            for (int i = 0; i < tgtItemInfo.size(); i++)
                            {
                                if (/*tgtItemInfo[i].checkFlag(itemFlag::PROP_WALL_CONNECT) && */tgtItemInfo[i].tileConnectGroup == currentGroup)
                                {
                                    return true;
                                }
                            }
                            return false;
                        }
                    };
                bool topTile = checkWallGroup(0, -1);
                bool botTile = checkWallGroup(0, 1);
                bool leftTile = checkWallGroup(-1, 0);
                bool rightTile = checkWallGroup(1, 0);
                int extraIndex = connectGroupExtraIndex(topTile, botTile, leftTile, rightTile);
                tgtPocket->itemInfo[layer].extraSprIndexSingle = extraIndex;
            }
            else if (tgtPocket->itemInfo[layer].checkFlag(itemFlag::VPART_DIR_DEPEND))
            {
                tgtPocket->itemInfo[layer].extraSprIndexSingle = dir16toInt16(bodyDir);
            }
            else if (tgtPocket->itemInfo[layer].checkFlag(itemFlag::TIRE_STEER))
            {
                tgtPocket->itemInfo[layer].extraSprIndexSingle = dir16toInt16(wheelDir);
            }
        }
    }
}

void Vehicle::shift(int dx, int dy)
{
    std::unordered_map<Point2, std::unique_ptr<Entity>, Point2::Hash> entityWormhole;//엔티티를 새로운 좌표로 옮기기 전에 임시적으로 저장하는 컨테이너

    for (const auto& [pos, pocket] : partInfo)
    {
        TileVehicle(pos.x, pos.y, getGridZ()) = nullptr;
        if (TileEntity(pos.x, pos.y, getGridZ()) != nullptr)
        {
            entityWormhole[{pos.x, pos.y}] = std::move(World::ins()->getTile(pos.x, pos.y, getGridZ()).EntityPtr);
        }
    }

    //엔티티 옮기기
    for (const auto& [pos, pocket] : partInfo)
    {
        TileVehicle(pos.x + dx, pos.y + dy, getGridZ()) = this;
        if (entityWormhole.find({ pos.x, pos.y }) != entityWormhole.end())
        {
            EntityPtrMove(std::move(entityWormhole[{pos.x, pos.y}]), { pos.x + dx, pos.y + dy, getGridZ() });
        }
    }

    std::unordered_map<Point2, std::unique_ptr<ItemPocket>, Point2::Hash> shiftPartInfo;
    for (auto& [pos, pocket] : partInfo)
    {
        shiftPartInfo[{pos.x + dx, pos.y + dy}] = std::move(pocket);
    }
    partInfo = std::move(shiftPartInfo);


    setGrid(getGridX() + dx, getGridY() + dy, getGridZ());
    updateHeadlight();
}

void Vehicle::zShift(int dz)
{
    std::unordered_map<Point2, std::unique_ptr<Entity>, Point2::Hash> entityWormhole;//엔티티를 새로운 좌표로 옮기기 전에 임시적으로 저장하는 컨테이너

    for (const auto& [pos, pocket] : partInfo)
    {
        TileVehicle(pos.x, pos.y, getGridZ()) = nullptr;
        if (TileEntity(pos.x, pos.y, getGridZ()) != nullptr)
        {
            entityWormhole[{pos.x, pos.y}] = std::move(World::ins()->getTile(pos.x, pos.y, getGridZ()).EntityPtr);
        }
    }

    //엔티티 옮기기
    for (const auto& [pos, pocket] : partInfo)
    {
        TileVehicle(pos.x, pos.y, getGridZ() + dz) = this;
        if (entityWormhole.find({ pos.x, pos.y }) != entityWormhole.end())
        {
            World::ins()->getTile(pos.x, pos.y, getGridZ() + dz).EntityPtr = std::move(entityWormhole[{pos.x, pos.y}]);
            TileEntity(pos.x, pos.y, getGridZ() + dz)->setGrid(pos.x, pos.y, getGridZ() + dz);//위치 그리드 변경
        }
    }

    setGrid(getGridX(), getGridY(), getGridZ() + dz);
    updateHeadlight();

};

bool Vehicle::colisionCheck(dir16 inputDir16, int dx, int dy)
{
    auto rotatedPartInfo = getRotateShadow(inputDir16);
    for (const auto& pos : rotatedPartInfo)
    {
        //벽 충돌 체크
        if (TileWall(pos.x + dx, pos.y + dy, getGridZ()) != 0) return true;

        if (TileProp(pos.x + dx, pos.y + dy, getGridZ()) != nullptr)
        {
            if (TileProp(pos.x + dx, pos.y + dy, getGridZ())->leadItem.checkFlag(itemFlag::PROP_DEPTH_LOWER) == false)
            {
                return true;
            }
        }

        //프롭 충돌 체크
        Vehicle* targetPtr = TileVehicle(pos.x + dx, pos.y + dy, getGridZ());
        if (targetPtr != nullptr && targetPtr != this) return true;
    }
    return false;
}

bool Vehicle::colisionCheck(int dx, int dy)//해당 dx,dy만큼 이동했을 때 prop이 벽 또는 기존의 Vehicle과 충돌하는지
{
    for (const auto& [pos, pocket] : partInfo)
    {
        //벽 충돌 체크
        if (TileWall(pos.x + dx, pos.y + dy, getGridZ()) != 0) return true;

        if (TileProp(pos.x + dx, pos.y + dy, getGridZ()) != nullptr)
        {
            if (TileProp(pos.x + dx, pos.y + dy, getGridZ())->leadItem.checkFlag(itemFlag::PROP_DEPTH_LOWER) == false)
            {
                return true;
            }
        }

        //프롭 충돌 체크
        Vehicle* targetPtr = TileVehicle(pos.x + dx, pos.y + dy, getGridZ());
        if (targetPtr != nullptr && targetPtr != this)
        {
            prt(L"(%d,%d)만큼 이동했을 때 포인터 %p와 충돌했다.\n", dx, dy, targetPtr);
            return true;
        }
    }
    return false;
}

//bool colisionCheck(dir16 inputDir16)
//{
//    return colisionCheck(inputDir16, 0, 0);
//}

void Vehicle::rush(int dx, int dy)
{
    if (dx == 0 && dy == 0) return;
    //iAmDictator();

    setDelGrid(dx, dy);
    shift(getDelGridX(), getDelGridY());
    cameraFix = false;
    setFakeX(-getDelX());
    setFakeY(-getDelY());
    extraRenderVehList.push_back(this);
    for (const auto& [pos, pocket] : partInfo)
    {
        if (auto e = TileEntity(pos.x, pos.y, getGridZ()))
        {
            e->setFakeX(-getDelX());
            e->setFakeY(-getDelY());
            extraRenderEntityList.push_back((e));
        }
    }

    updateHeadlight(getClosestGridWithFake());
    if (TileVehicle(PlayerX(), PlayerY(), PlayerZ()) == this) PlayerPtr->updateVision(PlayerPtr->entityInfo.eyeSight, getClosestGridWithFake().x, getClosestGridWithFake().y);

    addAniUSetMonster(this, aniFlag::propRush);
}

void Vehicle::centerShift(int dx, int dy, int dz)
{
}

void Vehicle::updateHeadlight()
{
    for (const auto& [pos, pocket] : partInfo)
    {
        for (int i = 0; i < pocket->itemInfo.size(); i++)
        {
            if (pocket->itemInfo[i].checkFlag(itemFlag::HEADLIGHT))
            {
                if (headlightOn)
                {
                    if (pocket->itemInfo[i].lightPtr != nullptr)
                    {
                        Light* thisLight = pocket->itemInfo[i].lightPtr.get();
                        thisLight->dir = bodyDir;
                        thisLight->moveLight(pos.x, pos.y, getGridZ());
                    }
                }
            }
        }
    }
}

void Vehicle::updateHeadlight(Point3 fakeCoor) //코어가 해당 위치에 가정하고 계산합니다.
{
    for (const auto& [pos, pocket] : partInfo)
    {
        for (int i = 0; i < pocket->itemInfo.size(); i++)
        {
            if (pocket->itemInfo[i].checkFlag(itemFlag::HEADLIGHT))
            {
                if (headlightOn)
                {
                    if (pocket->itemInfo[i].lightPtr != nullptr)
                    {
                        Light* thisLight = pocket->itemInfo[i].lightPtr.get();
                        thisLight->dir = bodyDir;
                        int revX = pos.x - getGridX();
                        int revY = pos.y - getGridY();

                        thisLight->moveLight(fakeCoor.x + revX, fakeCoor.y + revY, getGridZ());
                    }
                }
            }
        }
    }
}

bool Vehicle::runAnimation(bool shutdown)
{
    //prt(L"Vehicle %p의 runAnimation이 실행되었다.\n", this);
    //move는 사실상 pulled 카트 용도로만 사용됨
    if (getAniType() == aniFlag::move)//만약 플레이어 인스턴스의 좌표와 목적좌표가 다를 경우
    {
        // 1 / 60초마다 runAnimation이 실행됨
        addTimer();
        const double spd = pullMoveSpd;
        if (getFakeX() > 0)
        {
            addFakeX(-spd);
            if (getFakeX() < 0) setFakeX(0);
        }
        else if (getFakeX() < 0)
        {
            addFakeX(+spd);
            if (getFakeX() > 0) setFakeX(0);
        }

        if (getFakeY() > 0)
        {
            addFakeY(-spd);
            if (getFakeY() < 0) setFakeY(0);
        }
        else if (getFakeY() < 0)
        {
            addFakeY(+spd);
            if (getFakeY() > 0) setFakeY(0);
        }

        if (std::abs(getIntegerFakeX()) == 0 && std::abs(getIntegerFakeY()) == 0)
        {
            resetTimer();
            setAniType(aniFlag::null);
            setFakeX(0);
            setFakeY(0);
            return true;
        }
    }
    else if (getAniType() == aniFlag::propRush)
    {
        addTimer();

        //if (getTimer() > 300) return true;

        {
            static float totalDist = 0;
            static float totalMove = 0;
            static std::vector<Point2> lineRevPath;
            static Point3 startPoint;
            static int lineCheck = 0;
            static Point3 currentCoreGrid;

            if (getTimer() == 1)
            {
                lineRevPath.clear();
                makeLine(lineRevPath, getDelGridX(), getDelGridY());

                totalDist = std::sqrt(std::pow(getDelX(), 2) + std::pow(getDelY(), 2));
                totalMove = 0;
                lineCheck = 0;

                currentCoreGrid = getClosestGridWithFake();
                startPoint = currentCoreGrid;
            }

            float spd = 3.0;
            float xSpd, ySpd;
            int relX = getDelX();
            int relY = getDelY();
            float dist = std::sqrt(std::pow(relX, 2) + std::pow(relY, 2));
            float cosVal = relX / dist;
            float sinVal = relY / dist;

            xSpd = spd * cosVal;
            ySpd = spd * sinVal;

            setFakeX(getFakeX() + xSpd);
            setFakeY(getFakeY() + ySpd);
            //prt(L"x방향의 속도를 %f, y방향의 속도를 %f만큼 더했다.현재의 fake 좌표는 (%f,%f)이다.\n", xSpd, ySpd, getFakeX(), getFakeY());

            if (xSpd > 0 && getFakeX() > 0) { setFakeX(0); }
            if (xSpd < 0 && getFakeX() < 0) { setFakeX(0); }
            if (ySpd > 0 && getFakeY() > 0) { setFakeY(0); }
            if (ySpd < 0 && getFakeY() < 0) { setFakeY(0); }

            for (const auto& [pos, pocket] : partInfo)
            {
                if (TileEntity(pos.x, pos.y, getGridZ()) != nullptr)
                {
                    TileEntity(pos.x, pos.y, getGridZ())->setFakeX(getFakeX());
                    TileEntity(pos.x, pos.y, getGridZ())->setFakeY(getFakeY());
                }
            }


            if (currentCoreGrid != getClosestGridWithFake())
            {
                currentCoreGrid = getClosestGridWithFake();
                for (int i = 0; i < lineRevPath.size(); i++)
                {
                    if (currentCoreGrid.x == startPoint.x + lineRevPath[i].x && currentCoreGrid.y == startPoint.y + lineRevPath[i].y)
                    {
                        for (int j = lineCheck; j <= i; j++)
                        {
                            updateHeadlight({ startPoint.x + lineRevPath[i].x,startPoint.y + lineRevPath[i].y,getGridZ() });
                            if (TileVehicle(PlayerX(), PlayerY(), PlayerZ()) == this)
                                PlayerPtr->updateVision(PlayerPtr->entityInfo.eyeSight, startPoint.x + (PlayerX() - getGridX()) + lineRevPath[i].x, startPoint.y + (PlayerY() - getGridY()) + lineRevPath[i].y);
                            lineCheck++;
                        }
                    }
                }
            }

            cameraX = PlayerPtr->getX() + PlayerPtr->getIntegerFakeX();
            cameraY = PlayerPtr->getY() + PlayerPtr->getIntegerFakeY();

            if (getFakeX() == 0 && getFakeY() == 0)//도착
            {
                //prt(L"도착했다! 현재의 fake 좌표는 (%f,%f)이다.\n", getFakeX(), getFakeY());

                extraRenderEntityList.clear();
                setDelGrid(0, 0);
                setFakeX(0);
                setFakeY(0);
                for (const auto& [pos, pocket] : partInfo)//엔티티 페이크 설정
                {
                    if (TileEntity(pos.x, pos.y, getGridZ()) != nullptr)
                    {
                        Entity* tgtEntity = TileEntity(pos.x, pos.y, getGridZ());
                        tgtEntity->setFakeX(0);
                        tgtEntity->setFakeY(0);
                        tgtEntity->setDelGrid(0, 0);
                    }
                }

                cameraFix = true;
                PlayerPtr->updateVision(PlayerPtr->entityInfo.eyeSight);
                PlayerPtr->updateMinimap();
                resetTimer();
                setAniType(aniFlag::null);
                extraRenderVehList.erase(std::find(extraRenderVehList.begin(), extraRenderVehList.end(), this));
                for (const auto& [pos, pocket] : partInfo)
                {
                    Drawable* iPtr = TileEntity(pos.x, pos.y, getGridZ());
                    if (iPtr != nullptr)
                    {
                        auto eraseIt = std::find(extraRenderEntityList.begin(), extraRenderEntityList.end(), iPtr);
                        if (eraseIt != extraRenderEntityList.end()) extraRenderEntityList.erase(eraseIt);
                    }
                }
                return true;
            }
        }
    }
    else if (getAniType() == aniFlag::minecartRush)
    {
        addTimer();

        if (singleRailMoveVec.size() > 0)
        {
            if (singleRailMoveVec[0] == dir16::dir0) setFakeX(getIntegerFakeX() + 4.0);
            else if (singleRailMoveVec[0] == dir16::dir2) setFakeY(getIntegerFakeY() - 4.0);
            else if (singleRailMoveVec[0] == dir16::dir4) setFakeX(getIntegerFakeX() - 4.0);
            else if (singleRailMoveVec[0] == dir16::dir6) setFakeY(getIntegerFakeY() + 4.0);

            for (const auto& [pos, pocket] : partInfo)
            {
                if (TileEntity(pos.x, pos.y, getGridZ()) != nullptr)
                {
                    TileEntity(pos.x, pos.y, getGridZ())->setFakeX(getIntegerFakeX());
                    TileEntity(pos.x, pos.y, getGridZ())->setFakeY(getIntegerFakeY());
                }
            }

            if (getTimer() == 1)
            {
                if (gearState == gearFlag::drive) bodyDir = singleRailMoveVec[0];
                else if (gearState == gearFlag::reverse) bodyDir = reverse(singleRailMoveVec[0]);
            }

            // prt(L"[Vehicle : train %p ] 타이머 %d : 연산 후의 fake 좌표는 (%d,%d)이다.\n", this, getTimer(),getIntegerFakeX(), getIntegerFakeY());

            cameraX = PlayerPtr->getX() + PlayerPtr->getIntegerFakeX();
            cameraY = PlayerPtr->getY() + PlayerPtr->getIntegerFakeY();

            if (getTimer() >= 4)
            {
                PlayerPtr->updateVision(PlayerPtr->entityInfo.eyeSight, PlayerX() + (PlayerPtr->getIntegerFakeX() / 16), PlayerY() + (PlayerPtr->getIntegerFakeY() / 16));
                //prt(L"[Vehicle : train %p ] 카운터가 4보다 커져 fake 좌표가 초기화되었다.\n", this);
                singleRailMoveVec.erase(singleRailMoveVec.begin());
                resetTimer();
            }
        }
        else
        {
            //prt(L"[Vehicle : train %p ] 이동이 전부 완료된 후의 페이크 좌표는 (%f,%f)이다.\n", this, getFakeX(), getFakeY());
            extraRenderVehList.clear();
            extraRenderEntityList.clear();
            PlayerPtr->updateVision(PlayerPtr->entityInfo.eyeSight);
            PlayerPtr->updateMinimap();
            cameraFix = true;
            resetTimer();
            return true;
        }


    }
    return false;
}

void Vehicle::updateTrainCenter()
{
    std::vector<Point2> trainWheelList;

    for (const auto& [pos, pocket] : partInfo)
    {
        ItemPocket* pocketPtr = pocket.get();
        for (int i = 0; i < pocketPtr->itemInfo.size(); i++)
        {
            if (pocketPtr->itemInfo[i].checkFlag(itemFlag::TRAIN_WHEEL))
            {
                if (std::find(trainWheelList.begin(), trainWheelList.end(), pos) == trainWheelList.end()) //열차 바퀴 좌표가 중복된 값이 없으면
                {
                    trainWheelList.push_back({ pos.x, pos.y });
                }
            }
        }
    }
    //trainWheelList의 중간좌표를 구함
    trainWheelCenter = calcMidpoint(trainWheelList);
}

int Vehicle::getGasolineFuel()
{
    int gasolineNumber = 0;
    for (const auto& [pos, pocket] : partInfo)
    {
        ItemPocket* partPtr = pocket.get();
        for (int i = 0; i < partPtr->itemInfo.size(); i++)
        {
            ItemPocket* tankPtr = partPtr->itemInfo[i].pocketPtr.get();
            if (tankPtr != nullptr)
            {
                for (int j = 0; j < tankPtr->itemInfo.size(); j++)
                {
                    if (tankPtr->itemInfo[j].itemCode == itemRefCode::gasoline)
                    {
                        gasolineNumber += tankPtr->itemInfo[j].number;
                    }
                }
            }
        }
    }
    return gasolineNumber;
}

int Vehicle::getDiselFuel()
{
    int diselNumber = 0;
    for (const auto& [pos, pocket] : partInfo)
    {
        ItemPocket* partPtr = pocket.get();
        for (int i = 0; i < partPtr->itemInfo.size(); i++)
        {
            ItemPocket* tankPtr = partPtr->itemInfo[i].pocketPtr.get();
            if (tankPtr != nullptr)
            {
                for (int j = 0; j < tankPtr->itemInfo.size(); j++)
                {
                    if (tankPtr->itemInfo[j].itemCode == itemRefCode::diesel)
                    {
                        diselNumber += tankPtr->itemInfo[j].number;
                    }
                }
            }
        }
    }
    return diselNumber;
}

int Vehicle::getElectricityFuel()
{
    int electricityNumber = 0;
    for (const auto& [pos, pocket] : partInfo)
    {
        ItemPocket* partPtr = pocket.get();
        for (int i = 0; i < partPtr->itemInfo.size(); i++)
        {
            ItemPocket* tankPtr = partPtr->itemInfo[i].pocketPtr.get();
            if (tankPtr != nullptr)
            {
                for (int j = 0; j < tankPtr->itemInfo.size(); j++)
                {
                    if (tankPtr->itemInfo[j].itemCode == itemRefCode::electricity)
                    {
                        electricityNumber += tankPtr->itemInfo[j].number;
                    }
                }
            }
        }
    }
    return electricityNumber;
}

ItemData* Vehicle::getMainEngine()
{
    for (const auto& [pos, pocket] : partInfo)
    {
        ItemPocket* pocketPtr = pocket.get();
        for (int i = 0; i < pocketPtr->itemInfo.size(); i++)
        {
            if (pocketPtr->itemInfo[i].checkFlag(itemFlag::ENGINE_GASOLINE)) return &pocketPtr->itemInfo[i];
            else if (pocketPtr->itemInfo[i].checkFlag(itemFlag::ENGINE_DIESEL)) return &pocketPtr->itemInfo[i];
            else if (pocketPtr->itemInfo[i].checkFlag(itemFlag::ENGINE_ELECTRIC)) return &pocketPtr->itemInfo[i];
        }
    }
    return nullptr;
}

int Vehicle::getEngineFuel()
{
    int fuelNumber = 0;

    if (getMainEngine() == nullptr) return 0;
    else
    {
        if (getMainEngine()->checkFlag(itemFlag::ENGINE_GASOLINE)) fuelNumber = getGasolineFuel();
        else if (getMainEngine()->checkFlag(itemFlag::ENGINE_DIESEL)) fuelNumber = getDiselFuel();
        else if (getMainEngine()->checkFlag(itemFlag::ENGINE_ELECTRIC)) fuelNumber = getElectricityFuel();
    }
    return fuelNumber;
}


void Vehicle::useEngineFuel(int fuelAmount)
{
    int fuelNumber = 0;
    int targetFuelCode = 0;

    if (getMainEngine() == nullptr) return;
    else
    {
        if (getMainEngine()->checkFlag(itemFlag::ENGINE_GASOLINE)) targetFuelCode = itemRefCode::gasoline;
        else if (getMainEngine()->checkFlag(itemFlag::ENGINE_DIESEL)) targetFuelCode = itemRefCode::diesel;
        else if (getMainEngine()->checkFlag(itemFlag::ENGINE_ELECTRIC)) targetFuelCode = itemRefCode::electricity;
    }

    for (const auto& [pos, pocket] : partInfo)
    {
        int eraseNumber = 0;
        ItemPocket* partPtr = pocket.get();
        for (int i = 0; i < partPtr->itemInfo.size(); i++)
        {
            ItemPocket* tankPtr = partPtr->itemInfo[i].pocketPtr.get();
            if (tankPtr != nullptr)
            {
                for (int j = 0; j < tankPtr->itemInfo.size(); j++)
                {
                    if (tankPtr->itemInfo[j].itemCode == targetFuelCode)
                    {
                        if (eraseNumber < fuelAmount)
                        {
                            if (tankPtr->itemInfo[j].number > fuelAmount - eraseNumber)
                            {
                                tankPtr->itemInfo[j].number -= fuelAmount - eraseNumber;
                                eraseNumber = fuelAmount;
                            }
                            else
                            {
                                eraseNumber += tankPtr->itemInfo[j].number;
                                tankPtr->itemInfo.erase(tankPtr->itemInfo.begin() + j);
                                j--;
                            }
                        }
                        else break;
                    }
                }
            }
        }
    }
}

void Vehicle::drawSelf()
{
    std::vector<Point2> rotorList;
    int tileSize = 16 * zoomScale;
    auto drawVehicleComponent = [=](Vehicle* vPtr, int tgtX, int tgtY, int layer, int alpha)
        {
            SDL_Rect dst;
            dst.x = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX) - ((16 * zoomScale) / 2);
            dst.y = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY) - ((16 * zoomScale) / 2);
            dst.w = tileSize;
            dst.h = tileSize;

            setZoom(zoomScale);
            SDL_SetTextureAlphaMod(spr::propset->getTexture(), alpha); //텍스쳐 투명도 설정
            SDL_SetTextureBlendMode(spr::propset->getTexture(), SDL_BLENDMODE_BLEND); //블렌드모드 설정
            int sprIndex = vPtr->partInfo[{tgtX, tgtY}]->itemInfo[layer].propSprIndex + vPtr->partInfo[{tgtX, tgtY}]->itemInfo[layer].extraSprIndexSingle + 16 * vPtr->partInfo[{tgtX, tgtY}]->itemInfo[layer].extraSprIndex16;

            if (vPtr->partInfo[{tgtX, tgtY}]->itemInfo[layer].itemCode == itemRefCode::minecart)
            {
                if (bodyDir == dir16::dir0 || bodyDir == dir16::dir4) sprIndex += 0;
                else sprIndex += 1;
            }


            drawSpriteCenter
            (
                spr::propset,
                sprIndex,
                dst.x + dst.w / 2 + zoomScale * vPtr->getIntegerFakeX(),
                dst.y + dst.h / 2 + zoomScale * vPtr->getIntegerFakeY()
            );
            SDL_SetTextureAlphaMod(spr::propset->getTexture(), 255); //텍스쳐 투명도 설정


            if (vPtr->partInfo[{tgtX, tgtY}]->itemInfo[layer].pocketPtr != nullptr)
            {
                ItemPocket* pocketPtr = vPtr->partInfo[{tgtX, tgtY}]->itemInfo[layer].pocketPtr.get();
                if (pocketPtr->itemInfo.size() > 0)
                {
                    drawSpriteCenter
                    (
                        spr::itemset,
                        getItemSprIndex(pocketPtr->itemInfo[pocketPtr->itemInfo.size() - 1]),
                        dst.x + dst.w / 2 + zoomScale * vPtr->getIntegerFakeX(),
                        dst.y + dst.h / 2 + zoomScale * vPtr->getIntegerFakeY()
                    );
                }
            }

            setZoom(1.0);
        };


    for (const auto& [pos, pocket] : this->partInfo)
    {
        ////////////////////////////////일반 차량부품/////////////////////////////////////////////////
        for (int layer = 0; layer < pocket->itemInfo.size(); layer++)
        {
            //바닥프롭,천장프롭 플래그가 없는 일반 프롭일 경우
            if (!pocket->itemInfo[layer].checkFlag(itemFlag::VEH_ROOF))
            {
                drawVehicleComponent(this, pos.x, pos.y, layer, 255);
            }
        }

        ////////////////////////////////천장 차량부품////////////////////////////////////////////////////
        int propCeilAlpha = 255;
        if (TileVehicle(PlayerX(), PlayerY(), PlayerZ()) == this) propCeilAlpha = 50;

        for (int layer = 0; layer < pocket->itemInfo.size(); layer++)
        {
            if (pocket->itemInfo[layer].checkFlag(itemFlag::VEH_ROOF))
            {
                if (pocket->itemInfo[layer].itemCode == 314)
                {
                    rotorList.push_back({ pos.x, pos.y });
                }
                else drawVehicleComponent(this, pos.x, pos.y, layer, propCeilAlpha);
            }
        }
    }

    for (int i = 0; i < rotorList.size(); i++)
    {
        int propCeilAlpha = 255;
        if (TileVehicle(PlayerX(), PlayerY(), PlayerZ()) == this) propCeilAlpha = 50;

        int tgtX = rotorList[i].x;
        int tgtY = rotorList[i].y;
        SDL_Rect dst;
        dst.x = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX) - ((16 * zoomScale) / 2);
        dst.y = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY) - ((16 * zoomScale) / 2);
        dst.w = tileSize;
        dst.h = tileSize;

        setZoom(zoomScale);
        SDL_SetTextureAlphaMod(spr::mainRotor->getTexture(), propCeilAlpha); //텍스쳐 투명도 설정
        SDL_SetTextureBlendMode(spr::mainRotor->getTexture(), SDL_BLENDMODE_BLEND); //블렌드모드 설정
        //int sprIndex = vPtr->partInfo[{tgtX, tgtY}]->itemInfo[layer].propSprIndex + vPtr->partInfo[{tgtX, tgtY}]->itemInfo[layer].extraSprIndexSingle + 16 * vPtr->partInfo[{tgtX, tgtY}]->itemInfo[layer].extraSprIndex16;
        drawSpriteCenter
        (
            spr::mainRotor,
            0,
            dst.x + dst.w / 2 + zoomScale * getIntegerFakeX(),
            dst.y + dst.h / 2 + zoomScale * getIntegerFakeY()
        );
        SDL_SetTextureAlphaMod(spr::mainRotor->getTexture(), 255); //텍스쳐 투명도 설정
        setZoom(1.0);
    }
};