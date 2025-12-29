module;

#include <SDL3/SDL.h>

export module wrapVar;

import std;
import util;
import globalVar;
import constVar;
import ItemData;
import ItemPocket;
import ItemStack;
import World;
import Prop;
import Vehicle;
import Entity;
import Monster;
import Player;
import log;
import statusEffect;

export inline int PlayerX() { return PlayerPtr->getGridX(); }
export inline int PlayerY() { return PlayerPtr->getGridY(); }
export inline int PlayerZ() { return PlayerPtr->getGridZ(); }

export inline const unsigned __int16 TileFloor(int x, int y, int z) { return World::ins()->getTile(x, y, z).floor; }

export inline const bool TileSnow(int x, int y, int z) { return World::ins()->getTile(x, y, z).hasSnow; }

export inline const unsigned __int16 TileWall(int x, int y, int z) { return World::ins()->getTile(x, y, z).wall; }

export inline const bool ExistWall(int x, int y, int z) { return (World::ins()->getTile(x, y, z).wall != 0); }

export inline void setWall(Point3 coord, int val)
{
    World::ins()->getTile(coord.x, coord.y, coord.z).setWall(val);
};

export inline void setFloor(Point3 coord, int val)
{
    World::ins()->getTile(coord.x, coord.y, coord.z).setFloor(val);
};

export inline Entity* TileEntity(int x, int y, int z) { return (Entity*&)World::ins()->getTile(x, y, z).EntityPtr; }
export inline void EntityPtrMove(Point3 startCoor, Point3 endCoor)
{
    // 1단계: 시작 좌표의 EntityPtr 존재 확인
    auto& startTile = World::ins()->getTile(startCoor);
    errorBox(startTile.EntityPtr == nullptr,
        L"EntityPtrMove: 시작 좌표에 EntityPtr이 없습니다. startCoor=(" +
        std::to_wstring(startCoor.x) + L"," + std::to_wstring(startCoor.y) + L"," + std::to_wstring(startCoor.z) + L")");

    // 2단계: 목적지 좌표의 EntityPtr이 비어있는지 확인
    auto& endTile = World::ins()->getTile(endCoor);
    errorBox(endTile.EntityPtr != nullptr,
        L"EntityPtrMove: 목적지에 이미 EntityPtr이 있습니다. endCoor=(" +
        std::to_wstring(endCoor.x) + L"," + std::to_wstring(endCoor.y) + L"," + std::to_wstring(endCoor.z) + L")");

    // 3단계: 이동 실행
    endTile.EntityPtr = std::move(startTile.EntityPtr);

    // 4단계: 이동 후 EntityPtr 존재 확인
    errorBox(endTile.EntityPtr == nullptr,
        L"EntityPtrMove: 이동 후 EntityPtr이 nullptr입니다. endCoor=(" +
        std::to_wstring(endCoor.x) + L"," + std::to_wstring(endCoor.y) + L"," + std::to_wstring(endCoor.z) + L")");

    // 5단계: setGrid 호출
    endTile.EntityPtr->setGrid(endCoor.x, endCoor.y, endCoor.z);
    endTile.EntityPtr->pullEquipLights();
}
export inline void EntityPtrMove(std::unique_ptr<Entity> inputPtr, Point3 endCoor)
{
    World::ins()->getTile(endCoor).EntityPtr = std::move(inputPtr);
    World::ins()->getTile(endCoor).EntityPtr->setGrid(endCoor.x, endCoor.y, endCoor.z);
    World::ins()->getTile(endCoor).EntityPtr->pullEquipLights();
    
}
export inline Prop* TileProp(int x, int y, int z) { return World::ins()->getTile(x, y, z).PropPtr.get(); }
export inline Prop* TileProp(Point3 pt) { return World::ins()->getTile(pt.x, pt.y, pt.z).PropPtr.get(); }
export inline Vehicle*& TileVehicle(int x, int y, int z) { return (Vehicle*&)World::ins()->getTile(x, y, z).VehiclePtr; }
export inline Vehicle*& TileVehicle(Point3 pt) { return (Vehicle*&)World::ins()->getTile(pt.x, pt.y, pt.z).VehiclePtr; }

export inline ItemStack* TileItemStack(int x, int y, int z) { return World::ins()->getTile(x, y, z).ItemStackPtr.get(); }
export inline ItemStack* TileItemStack(Point3 pt) { return World::ins()->getTile(pt.x, pt.y, pt.z).ItemStackPtr.get(); }

export inline fovFlag& TileFov(int x, int y, int z) { return static_cast<fovFlag&>(World::ins()->getTile(x, y, z).fov); }

export inline void createMonster(Point3 inputCoor, int inputEntityCode)
{
    World::ins()->getTile(inputCoor).EntityPtr = std::make_unique<Monster>(inputEntityCode, inputCoor.x, inputCoor.y, inputCoor.z);
}

export inline void createItemStack(Point3 inputCoor)
{
    World::ins()->getTile(inputCoor).ItemStackPtr = std::make_unique<ItemStack>(inputCoor);
}

export inline void createItemStack(Point3 inputCoor, std::vector<std::pair<int, int>> inputItems)
{
    World::ins()->getTile(inputCoor).ItemStackPtr = std::make_unique<ItemStack>(inputCoor, inputItems);
}

export inline void destroyItemStack(Point3 inputCoor)
{
    World::ins()->getTile(inputCoor).ItemStackPtr.reset();
}

export inline void destroyProp(Point3 inputCoor)
{
    World::ins()->getTile(inputCoor).PropPtr.reset();
}


export inline void createProp(Point3 inputCoor, int inputItemCode)
{
    World::ins()->getTile(inputCoor).PropPtr = std::make_unique<Prop>(inputCoor, inputItemCode);

    World::ins()->getTile(inputCoor).PropPtr->updateSprIndex();

    //주변 타일을 분석해 extraIndex(연결) 설정
    int dx = 0;
    int dy = 0;
    for (int i = 0; i < 8; i++)
    {
        dir2Coord(i, dx, dy);
        Prop* targetProp = TileProp(inputCoor.x + dx, inputCoor.y + dy, inputCoor.z);
        if (targetProp != nullptr) targetProp->updateSprIndex();
    }
}
export inline void createFlame(Point3 inputCoor, flameFlag inputFlag) { World::ins()->getTile(inputCoor).flamePtr = std::make_unique<Flame>(inputCoor, inputFlag); }

export inline void DestroyWall(int x, int y, int z) { World::ins()->getTile(x, y, z).destoryWall(); }

export inline bool isWalkable(Point3 coord)
{
    if (TileWall(coord.x, coord.y, coord.z) != 0) return false;
    else if (TileProp(coord.x, coord.y, coord.z) != nullptr && TileProp(coord.x, coord.y, coord.z)->leadItem.checkFlag(itemFlag::PROP_WALKABLE) == false) return false;
    else if (TileEntity(coord.x, coord.y, coord.z) != nullptr) return false;
    else if (TileVehicle(coord.x, coord.y, coord.z) != nullptr)
    {
        ItemPocket* targetPocket = TileVehicle(coord.x, coord.y, coord.z)->partInfo[{coord.x, coord.y}].get();
        for (int i = 0; i < targetPocket->itemInfo.size(); i++)
        {
            if (targetPocket->itemInfo[i].checkFlag(itemFlag::VPART_NOT_WALKABLE)) return false;
        }
    }
    else if (TileFloor(coord.x, coord.y, coord.z) == 0) return false; //바닥이 없는 경우

    return true;
};

export inline bool isRayBlocker(Point3 coord)
{
    if (TileWall(coord.x, coord.y, coord.z) != 0 && itemDex[TileWall(coord.x, coord.y, coord.z)].checkFlag(itemFlag::TRANSPARENT_WALL) == false) return true;
    else if (TileProp(coord.x, coord.y, coord.z) != nullptr && TileProp(coord.x, coord.y, coord.z)->leadItem.checkFlag(itemFlag::PROP_BLOCKER) == true) return true;
    else return false;
};

export float getMouseX()
{
    float px, py;
    SDL_GetMouseState(&px, &py);
    float renderX, renderY;
    SDL_RenderCoordinatesFromWindow(renderer, px, py, &renderX, &renderY);
    return renderX;
}

export float getMouseY()
{
    float px, py;
    SDL_GetMouseState(&px, &py);
    float renderX, renderY;
    SDL_RenderCoordinatesFromWindow(renderer, px, py, &renderX, &renderY);
    return renderY;
}

export Point2 getAbsMouseGrid()
{
    int cameraGridX, cameraGridY;
    if (cameraX >= 0) cameraGridX = cameraX / 16;
    else cameraGridX = -1 + cameraX / 16;
    if (cameraY >= 0) cameraGridY = cameraY / 16;
    else cameraGridY = -1 + cameraY / 16;

    int camDelX = cameraX - (16 * cameraGridX + 8);
    int camDelY = cameraY - (16 * cameraGridY + 8);

    int revX, revY, revGridX, revGridY;
    if (option::inputMethod == input::touch)
    {
        revX = static_cast<int>(event.tfinger.x * cameraW) - (cameraW / 2);
        revY = static_cast<int>(event.tfinger.y * cameraH) - (cameraH / 2);
    }
    else
    {
        revX = static_cast<int>(getMouseX()) - (cameraW / 2);
        revY = static_cast<int>(getMouseY()) - (cameraH / 2);
    }
    revX += sgn(revX) * (8 * zoomScale) + camDelX;
    revGridX = revX / (16 * zoomScale);
    revY += sgn(revY) * (8 * zoomScale) + camDelY;
    revGridY = revY / (16 * zoomScale);

    return { cameraGridX + revGridX, cameraGridY + revGridY };
}

/*******************************************************************************
* 부피 관련 변수들
* 부피는 CONTAINER_FLEX로 인해 가변적이므로 반드시 래퍼 함수를 거쳐야 함
* ItemData의 originalVolume을 사용하는 코드가 이 래퍼함수 이외에 존재하면 제거할 것
 *******************************************************************************/

export int getVolume(const ItemData& inputData)
{
    int baseVolume = inputData.originalVolume;

    if (inputData.checkFlag(itemFlag::CONTAINER_FLEX) && inputData.pocketPtr != nullptr)
    {
        for (const auto& item : inputData.pocketPtr->itemInfo)
        {
            baseVolume += getVolume(item) * item.number;
        }
    }

    return baseVolume;
}

export void sortVolumeDescend(std::vector<ItemData>& inputInfo, int startIndex, int endIndex)
{
    std::sort(inputInfo.begin() + startIndex, inputInfo.begin() + endIndex + 1,
        [](ItemData& a, ItemData& b)
        {
            return (getVolume(a) > getVolume(b));
        }
    );
}
export void sortVolumeDescend(std::vector<ItemData>& inputInfo) { sortVolumeDescend(inputInfo, 0, inputInfo.size() - 1); }

export void sortVolumeAscend(std::vector<ItemData>& inputInfo, int startIndex, int endIndex)
{
    std::sort(inputInfo.begin() + startIndex, inputInfo.begin() + endIndex + 1,
        [](ItemData& a, ItemData& b)
        {
            return (getVolume(a) < getVolume(b));
        }
    );
}
export void sortVolumeAscend(std::vector<ItemData>& inputInfo) { sortVolumeAscend(inputInfo, 0, inputInfo.size() - 1); }


export bool checkStatusEffect(std::vector<statusEffect>& inputStatus, statusEffectFlag inputFlag)
{
    for (const auto& effect : inputStatus)
    {
        if (effect.effectType == inputFlag) return true;
    }
    return false;
}

export void eraseStatusEffect(std::vector<statusEffect>& inputStatus, statusEffectFlag inputFlag)
{
    for (auto it = inputStatus.begin(); it != inputStatus.end();)
    {
        if (it->effectType == inputFlag)
        {
            it = inputStatus.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

export int getItemSprIndex(ItemData& inputData)
{
    if ((inputData.itemCode == itemRefCode::arrowQuiver || inputData.itemCode == itemRefCode::boltQuiver)&& inputData.pocketPtr != nullptr)
    {
        std::vector<ItemData>& pocketInfo = inputData.pocketPtr.get()->itemInfo;
        
        int num = inputData.pocketPtr.get()->getPocketNumber();
        if (num == 0) return itemDex[inputData.itemCode].itemSprIndex;
        else if (num == 1) return itemDex[inputData.itemCode].itemSprIndex + 1;
        else return itemDex[inputData.itemCode].itemSprIndex + 2;
    }
    else if (inputData.checkFlag(itemFlag::CONTAINER_LIQ) && inputData.checkFlag(itemFlag::CONTAINER_TRANSPARENT) && inputData.pocketPtr != nullptr)//투명 액체 용기
    {
        std::vector<ItemData>& pocketInfo = inputData.pocketPtr.get()->itemInfo;

        if (pocketInfo.size() > 0)
        {
            if(pocketInfo[0].checkFlag(itemFlag::LIQ_COL_RED))  return inputData.itemSprIndex+2;
            else if(pocketInfo[0].checkFlag(itemFlag::LIQ_COL_BLUE)) return inputData.itemSprIndex+3;
            else if(pocketInfo[0].checkFlag(itemFlag::LIQ_COL_YELLOW)) return inputData.itemSprIndex+4;
            else if(pocketInfo[0].checkFlag(itemFlag::LIQ_COL_WHITE)) return inputData.itemSprIndex+5;
            else if(pocketInfo[0].checkFlag(itemFlag::LIQ_COL_GRAY)) return inputData.itemSprIndex+6;
            else if(pocketInfo[0].checkFlag(itemFlag::LIQ_COL_BLACK)) return inputData.itemSprIndex+7;
            else return inputData.itemSprIndex + 8;
        }
        else return inputData.itemSprIndex;
    }
    else if (inputData.checkFlag(itemFlag::CONTAINER_LIQ) && inputData.checkFlag(itemFlag::CONTAINER_TRANSLUCENT) && inputData.pocketPtr != nullptr)//반투명   
    {
        std::vector<ItemData>& pocketInfo = inputData.pocketPtr.get()->itemInfo;

        if (pocketInfo.size() > 0)
        {
            return inputData.itemSprIndex + 1;
        }
        else return inputData.itemSprIndex;
    }
    else return inputData.itemSprIndex;
}

export void changePlayerWalkMode(walkFlag inputMode)
{
    auto& pStatus = PlayerPtr->entityInfo.statusEffectVec;

    //pStatus에서 run, crouch, crawl 모두 제거
    pStatus.erase(std::remove_if(pStatus.begin(), pStatus.end(),
        [](statusEffect& effect)
        {
            return effect.effectType == statusEffectFlag::run ||
                effect.effectType == statusEffectFlag::crouch ||
                effect.effectType == statusEffectFlag::crawl;
        }),
        pStatus.end());

    if(inputMode == walkFlag::run) pStatus.push_back({statusEffectFlag::run, -1});
    else if(inputMode == walkFlag::crouch) pStatus.push_back({statusEffectFlag::crouch, -1});
    else if (inputMode == walkFlag::crawl) pStatus.push_back({ statusEffectFlag::crawl, -1 });

    PlayerPtr->entityInfo.walkMode = inputMode;
}