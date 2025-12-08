#include <SDL3/SDL.h>

export module renderTile;

import std;
import util;
import globalVar;
import wrapVar;
import constVar;
import textureVar;
import checkCursor;
import ItemStack;
import Entity;
import World;
import Player;
import drawSprite;
import drawText;
import Damage;
import Corpse;
import Craft;
import Vehicle;
import GUI;
import Prop;
import globalTime;
import Drawable;
import TileData;
import Flame;
import HUD;
import Bullet;
import Particle;
import Footprint;
import Wave;
import Wake;

SDL_Rect dst, renderRegion;
int tileSize, cameraGridX, cameraGridY, renderRangeW, renderRangeH, pZ;

Point2 vertices[MAX_BATCH];
int indices[MAX_BATCH];
SDL_Color rectColors[MAX_BATCH];
Uint8 batchAlphas[MAX_BATCH];

void analyseRender();
void drawTiles();
void drawCorpses();
void drawFloorProp();
void drawItems();
void drawEntities();
void drawDamages();
void drawBullets();
void drawParticles();
void drawMulFogs();
void drawFogs();
void drawMarkers();
void drawDebug();


// 차량과 엔티티는 중복을 허용하면 안됨
std::vector<Point2> tileList, itemList, floorPropList, gasList, blackFogList, grayFogList, lightFogList, flameList, allTileList, mulFogList, wallHPList;
std::unordered_set<Point2, Point2::Hash> lightFogSet, shallowSeaWaves, deepSeaWaves, deepFreshWaves;
std::vector<Drawable*> renderVehList, renderEntityList;
std::unordered_set<Point2, Point2::Hash> raySet;

export __int64 renderTile()
{
    __int64 timeStampStart = getNanoTimer();

    tileSize = 16 * zoomScale;
    cameraGridX = (cameraX - 8) / (16);
    cameraGridY = (cameraY - 8) / (16);
    renderRangeW = 3 + (cameraW + extraCameraLength) / tileSize;
    renderRangeH = 3 + (cameraH + extraCameraLength) / tileSize;
    pZ = PlayerZ();
    renderRegion = { cameraGridX - (renderRangeW / 2), cameraGridY - (renderRangeH / 2), renderRangeW, renderRangeH };

    tileList.clear();
    itemList.clear();
    floorPropList.clear();
    renderVehList.clear();
    renderEntityList.clear();
    gasList.clear();
    blackFogList.clear();
    grayFogList.clear();
    lightFogList.clear();
    lightFogSet.clear();
    flameList.clear();
    mulFogList.clear();
    shallowSeaWaves.clear();
    deepSeaWaves.clear();
    wallHPList.clear();

    if (rangeRay)
    {
        
        raySet.clear();

        if (checkCursor(&tab) == false && checkCursor(&letterbox) == false)
        {
            if (option::inputMethod == input::mouse)
            {
                int revGridX = getAbsMouseGrid().x - PlayerX();
                int revGridY = getAbsMouseGrid().y - PlayerY();
                makeLine(raySet, getAbsMouseGrid().x - PlayerX(), getAbsMouseGrid().y - PlayerY());
                if (revGridX > 0) PlayerPtr->setDirection(0);
                else if (revGridX < 0) PlayerPtr->setDirection(4);
            }
        }
        
    }

    auto PROFILE = [](auto&& f) -> __int64
        {
            const __int64 start = getNanoTimer();
            std::forward<decltype(f)>(f)();
            return getNanoTimer() - start;
        };

    dur::analysis = PROFILE([] { analyseRender(); });
    dur::tile = PROFILE([] { drawTiles(); });
    dur::corpse = PROFILE([] { drawCorpses(); });
    dur::floorProp = PROFILE([] { drawFloorProp(); });
    dur::item = PROFILE([] { drawItems(); });
    dur::entity = PROFILE([] { drawEntities(); });
    dur::damage = PROFILE([] { drawDamages(); });
    dur::bullet = PROFILE([] { drawBullets(); });
    dur::particle = PROFILE([] { drawParticles(); });
    dur::mulFog = PROFILE([] { drawMulFogs(); });
    dur::fog = PROFILE([] { drawFogs(); });
    dur::marker = PROFILE([] { drawMarkers(); });
    dur::debug = PROFILE([] { drawDebug(); });

    return (getNanoTimer() - timeStampStart);
}




void analyseRender()
{
    // 루프 분석
    for (int tgtY = renderRegion.y - 1; tgtY < renderRegion.y + renderRegion.h + 1; tgtY++)
    {
        for (int tgtX = renderRegion.x - 1; tgtX < renderRegion.x + renderRegion.w + 1; tgtX++)
        {
            TileData* thisTile = &World::ins()->getTile(tgtX, tgtY, pZ);

            // 바깥쪽이면 타일캐시만 추가하고 거기에 있는 내용은 렌더링하지 않음
            if (tgtX < renderRegion.x || tgtX >= renderRegion.x + renderRegion.w) continue;
            if (tgtY < renderRegion.y || tgtY >= renderRegion.y + renderRegion.h) continue;

            // 바닥과 벽
            if (thisTile->fov != fovFlag::black)
            {
                tileList.push_back({ tgtX, tgtY });
                if (thisTile->wall != 0 && thisTile->displayHPBarCount>0)
                {
                    wallHPList.push_back({ tgtX, tgtY });
                }

                switch (thisTile->floor)
                {
                case itemRefCode::shallowSeaWater:
                    for (int dir = 0; dir < 8; dir++)
                    {
                        int dx, dy;
                        dir2Coord(dir, dx, dy);
                        if (World::ins()->getTile(tgtX + dx, tgtY + dy, pZ).floor != itemRefCode::deepSeaWater)
                        {
                            shallowSeaWaves.insert({ tgtX + dx, tgtY + dy });
                        }
                    }
                    break;
                case itemRefCode::deepSeaWater:
                    for (int dir = 0; dir < 8; dir++)
                    {
                        int dx, dy;
                        dir2Coord(dir, dx, dy);
                        if (World::ins()->getTile(tgtX + dx, tgtY + dy, pZ).floor != itemRefCode::deepSeaWater)
                        {
                            deepSeaWaves.insert({ tgtX + dx, tgtY + dy });
                        }
                    }
                    break;
                case itemRefCode::deepFreshWater:
                    for (int dir = 0; dir < 8; dir++)
                    {
                        int dx, dy;
                        dir2Coord(dir, dx, dy);
                        if (World::ins()->getTile(tgtX + dx, tgtY + dy, pZ).floor == itemRefCode::shallowFreshWater)
                        {
                            deepFreshWaves.insert({ tgtX + dx, tgtY + dy });
                        }
                    }
                    break;
                }

            }

            // 바닥프롭
            Prop* fpPtr = thisTile->PropPtr.get();
            if (fpPtr != nullptr && fpPtr->leadItem.checkFlag(itemFlag::PROP_DEPTH_LOWER)) floorPropList.push_back({ tgtX, tgtY });

            // 아이템
            if (thisTile->fov == fovFlag::white && TileItemStack(tgtX, tgtY, pZ) != nullptr) itemList.push_back({ tgtX, tgtY });

            // 화염
            Flame* flamePtr = thisTile->flamePtr.get();
            if (flamePtr != nullptr) flameList.push_back({ tgtX, tgtY });

            // 차량
            Drawable* vPtr = (Drawable*)((Vehicle*)(thisTile->VehiclePtr));
            if (vPtr != nullptr) renderVehList.push_back(vPtr);

            // 플레이어와 겹치는 일반설치물
            Prop* pPtr = thisTile->PropPtr.get();
            if (pPtr != nullptr && pPtr->leadItem.checkFlag(itemFlag::PROP_DEPTH_LOWER) == false) renderEntityList.push_back((Drawable*)pPtr);

            // 일반 객체
            Drawable* ePtr = (Drawable*)(thisTile->EntityPtr.get());
            if (ePtr != nullptr) renderEntityList.push_back(ePtr);

            // 가스
            if (thisTile->gasVec.size() > 0) gasList.push_back({ tgtX, tgtY });

            // 안개
            if (thisTile->fov == fovFlag::black) blackFogList.push_back({ tgtX, tgtY });
            else
            {
                mulFogList.push_back({ tgtX,tgtY });

                if (thisTile->fov == fovFlag::gray) grayFogList.push_back({ tgtX, tgtY });
                else
                {
                    if (thisTile->lightVec.size() > 0)
                    {
                        lightFogList.push_back({ tgtX, tgtY });
                        lightFogSet.insert({ tgtX, tgtY });
                    }
                }
            }
        }
    }

    for (auto it = extraRenderVehList.begin(); it != extraRenderVehList.end(); it++)
    {
        int exVehSize = extraRenderVehList.size(); // 메모리 누수 체크용
        (void)exVehSize;
        renderVehList.push_back(*it);
    }
    for (auto it = extraRenderEntityList.begin(); it != extraRenderEntityList.end(); it++)
    {
        int exEntitySize = extraRenderEntityList.size(); // 메모리 누수 체크용
        (void)exEntitySize;
        renderEntityList.push_back(*it);
    }
}

void drawTiles()
{
    SDL_Texture* tileTexture = spr::tileset->getTexture();
    int tileTextureW = spr::tileset->getW();
    int tileTextureH = spr::tileset->getH();

    
    int tileCounter = 0;

    for (const auto& elem : tileList)
    {
        int tgtX = elem.x;
        int tgtY = elem.y;
        const TileData* thisTile = &World::ins()->getTile(tgtX, tgtY, PlayerZ());
        const TileData* topTile = &World::ins()->getTile(tgtX, tgtY - 1, PlayerZ());
        const TileData* botTile = &World::ins()->getTile(tgtX, tgtY + 1, PlayerZ());
        const TileData* leftTile = &World::ins()->getTile(tgtX - 1, tgtY, PlayerZ());
        const TileData* rightTile = &World::ins()->getTile(tgtX + 1, tgtY, PlayerZ());

        setZoom(zoomScale);

        int dirCorrection = 0;
        int tileAniExtraIndex16 = 0;
        int tileAniExtraIndexSingle = 0;
        if (itemDex[thisTile->floor].tileConnectGroup != -1)
        {
            bool topCheck, botCheck, leftCheck, rightCheck;
            if (itemDex[thisTile->floor].tileConnectGroup == 0)
            {
                int currentTileFloor = thisTile->floor;
                int topTileFloor = topTile->floor;
                int botTileFloor = botTile->floor;
                int leftTileFloor = leftTile->floor;
                int rightTileFloor = rightTile->floor;

                topCheck = currentTileFloor == topTileFloor;
                botCheck = currentTileFloor == botTileFloor;
                leftCheck = currentTileFloor == leftTileFloor;
                rightCheck = currentTileFloor == rightTileFloor;
            }
            else
            {
                int currentTileGroup = itemDex[thisTile->floor].tileConnectGroup;
                int topTileGroup = itemDex[topTile->floor].tileConnectGroup;
                int botTileGroup = itemDex[botTile->floor].tileConnectGroup;
                int leftTileGroup = itemDex[leftTile->floor].tileConnectGroup;
                int rightTileGroup = itemDex[rightTile->floor].tileConnectGroup;

                topCheck = currentTileGroup == topTileGroup;
                botCheck = currentTileGroup == botTileGroup;
                leftCheck = currentTileGroup == leftTileGroup;
                rightCheck = currentTileGroup == rightTileGroup;
            }

            dirCorrection = connectGroupExtraIndex(topCheck, botCheck, leftCheck, rightCheck);

            if (itemDex[thisTile->floor].animeSize > 1)
            {
                int animeFPS = itemDex[thisTile->floor].animeFPS;
                int animeSize = itemDex[thisTile->floor].animeSize;
                tileAniExtraIndex16 = getMilliTimer() / animeFPS % animeSize;
            }
        }
        else
        {
            if (itemDex[thisTile->floor].animeSize > 1)
            {
                int animeFPS = itemDex[thisTile->floor].animeFPS;
                int animeSize = itemDex[thisTile->floor].animeSize;
                tileAniExtraIndexSingle = getMilliTimer() / animeFPS % animeSize;
            }
        }

        int sprIndex = itemDex[thisTile->floor].tileSprIndex + itemDex[thisTile->floor].extraSprIndexSingle + 16 * itemDex[thisTile->floor].extraSprIndex16;
        sprIndex += 16 * tileAniExtraIndex16 + tileAniExtraIndexSingle;
        if (thisTile->floor == 0) sprIndex = 506;

        if (thisTile->floor == 220)
        {
            if (getSeason() == seasonFlag::winter) sprIndex += 16;
            else if (getSeason() == seasonFlag::summer) sprIndex += 32;
        }


        vertices[tileCounter] = 
        { 
            cameraW / 2 + static_cast<int>(zoomScale * (16 * tgtX + 8 - cameraX)), 
            cameraH / 2 + static_cast<int>(zoomScale * (16 * tgtY + 8 - cameraY)) 
        };
        indices[tileCounter] = sprIndex + dirCorrection;
        batchAlphas[tileCounter] = 255;
        tileCounter++;



        // 눈
        if (thisTile->hasSnow == true)
        {
            setZoom(zoomScale);

            int dirCorrection = 0;

            bool topCheck, botCheck, leftCheck, rightCheck;

            int currentSnow = thisTile->hasSnow;
            int topTileSnow = topTile->hasSnow;
            int botTileSnow = botTile->hasSnow;
            int leftTileSnow = leftTile->hasSnow;
            int rightTileSnow = rightTile->hasSnow;

            topCheck = currentSnow == topTileSnow;
            botCheck = currentSnow == botTileSnow;
            leftCheck = currentSnow == leftTileSnow;
            rightCheck = currentSnow == rightTileSnow;

            dirCorrection = connectGroupExtraIndex(topCheck, botCheck, leftCheck, rightCheck);

            vertices[tileCounter] = 
            { 
                cameraW / 2 + static_cast<int>(zoomScale * (16 * tgtX + 8 - cameraX)), 
                cameraH / 2 + static_cast<int>(zoomScale * (16 * tgtY + 8 - cameraY)) 
            };
            indices[tileCounter] = 1712 + dirCorrection;
            batchAlphas[tileCounter] = 255;
            tileCounter++;
        }


        // 발자국
        if (Footprint::map.find({ tgtX, tgtY, PlayerZ() }) != Footprint::map.end())
        {
            for (const auto& address : Footprint::map[{tgtX, tgtY, PlayerZ()}])
            {
                vertices[tileCounter] =
                {
                cameraW / 2 + static_cast<int>(zoomScale * (16 * tgtX + 8 - cameraX)),
                cameraH / 2 + static_cast<int>(zoomScale * (16 * tgtY + 8 - cameraY))
                };

                indices[tileCounter] = 1952 + address->sprIndex;
                batchAlphas[tileCounter] = address->alpha;
                tileCounter++;
            }
        }



        if(thisTile->wall!=0)
        {
            setZoom(zoomScale);
            int dirCorrection = 0;
            if (itemDex[thisTile->wall].tileConnectGroup != -1)
            {
                bool topCheck, botCheck, leftCheck, rightCheck;
                if (itemDex[thisTile->wall].tileConnectGroup == 0)
                {
                    int currentTileWall = thisTile->wall;
                    int topTileWall = topTile->wall;
                    int botTileWall = botTile->wall;
                    int leftTileWall = leftTile->wall;
                    int rightTileWall = rightTile->wall;

                    topCheck = currentTileWall == topTileWall;
                    botCheck = currentTileWall == botTileWall;
                    leftCheck = currentTileWall == leftTileWall;
                    rightCheck = currentTileWall == rightTileWall;
                }
                else
                {
                    int currentTileGroup = itemDex[thisTile->wall].tileConnectGroup;
                    int topTileGroup = itemDex[topTile->wall].tileConnectGroup;
                    int botTileGroup = itemDex[botTile->wall].tileConnectGroup;
                    int leftTileGroup = itemDex[leftTile->wall].tileConnectGroup;
                    int rightTileGroup = itemDex[rightTile->wall].tileConnectGroup;

                    topCheck = (currentTileGroup == topTileGroup);
                    botCheck = (currentTileGroup == botTileGroup);
                    leftCheck = (currentTileGroup == leftTileGroup);
                    rightCheck = (currentTileGroup == rightTileGroup);
                }

                dirCorrection = connectGroupExtraIndex(topCheck, botCheck, leftCheck, rightCheck);
            }

            vertices[tileCounter] =
            {
                cameraW / 2 + static_cast<int>(zoomScale * (16 * tgtX + 8 - cameraX)),
                cameraH / 2 + static_cast<int>(zoomScale * (16 * tgtY + 8 - cameraY))
            };
            indices[tileCounter] = itemDex[thisTile->wall].tileSprIndex + dirCorrection;
            batchAlphas[tileCounter] = 255;
            tileCounter++;


        }

        // 스킬 범위 그리기
        if (rangeSet.find({ tgtX, tgtY }) != rangeSet.end())
        {
            bool rightCheck = rangeSet.find({ tgtX + 1, tgtY }) != rangeSet.end();
            bool topCheck = rangeSet.find({ tgtX, tgtY - 1 }) != rangeSet.end();
            bool leftCheck = rangeSet.find({ tgtX - 1, tgtY }) != rangeSet.end();
            bool botCheck = rangeSet.find({ tgtX, tgtY + 1 }) != rangeSet.end();
            int dirCorrection = connectGroupExtraIndex(topCheck, botCheck, leftCheck, rightCheck);

            vertices[tileCounter] =
            {
                cameraW / 2 + static_cast<int>(zoomScale * (16 * tgtX + 8 - cameraX)),
                cameraH / 2 + static_cast<int>(zoomScale * (16 * tgtY + 8 - cameraY))
            };
            indices[tileCounter] = 1440 + dirCorrection;
            batchAlphas[tileCounter] = 255;
            tileCounter++;
        }

        if (rangeRay && raySet.find({ tgtX - PlayerX(),tgtY - PlayerY() }) != raySet.end())
        {
            vertices[tileCounter] =
            {
                cameraW / 2 + static_cast<int>(zoomScale * (16 * tgtX + 8 - cameraX)),
                cameraH / 2 + static_cast<int>(zoomScale * (16 * tgtY + 8 - cameraY))
            };
            indices[tileCounter] = 0;
            batchAlphas[tileCounter] = 150;
            tileCounter++;
        }

        if (rangeSet.size() > 0 && rangeSet.find({ tgtX, tgtY }) != rangeSet.end())
        {
            if (getAbsMouseGrid().x == tgtX && getAbsMouseGrid().y == tgtY)
            {
                vertices[tileCounter] =
                {
                    cameraW / 2 + static_cast<int>(zoomScale * (16 * tgtX + 8 - cameraX)),
                    cameraH / 2 + static_cast<int>(zoomScale * (16 * tgtY + 8 - cameraY))
                };
                indices[tileCounter] = 0;
                batchAlphas[tileCounter] = 150;
                tileCounter++;
            }
        }
    }

    for(auto elem : shallowSeaWaves)
    {
        int tgtX = elem.x;
        int tgtY = elem.y;
        const TileData* thisTile = &World::ins()->getTile(tgtX, tgtY, PlayerZ());
        const TileData* topTile = &World::ins()->getTile(tgtX, tgtY - 1, PlayerZ());
        const TileData* botTile = &World::ins()->getTile(tgtX, tgtY + 1, PlayerZ());
        const TileData* leftTile = &World::ins()->getTile(tgtX - 1, tgtY, PlayerZ());
        const TileData* rightTile = &World::ins()->getTile(tgtX + 1, tgtY, PlayerZ());
        int animeExtraIndex = 32 * ((SDL_GetTicks() / 300) % 7);

        if(thisTile->floor != itemRefCode::shallowSeaWater)
        {
            bool topCheck = topTile->floor == itemRefCode::shallowSeaWater;
            bool botCheck = botTile->floor == itemRefCode::shallowSeaWater;
            bool leftCheck = leftTile->floor == itemRefCode::shallowSeaWater;
            bool rightCheck = rightTile->floor == itemRefCode::shallowSeaWater;

            Uint8 alpha = 205;
            int extraIndex = 224;
            auto addWave = [&](int inputIndex)
                {
                    vertices[tileCounter] =
                    {
                        cameraW / 2 + static_cast<int>(zoomScale * (16 * tgtX + 8 - cameraX)),
                        cameraH / 2 + static_cast<int>(zoomScale * (16 * tgtY + 8 - cameraY))
                    };
                    indices[tileCounter] = inputIndex + animeExtraIndex + extraIndex;
                    batchAlphas[tileCounter] = alpha;
                    tileCounter++;
                };

            if (topCheck && botCheck && leftCheck && rightCheck) addWave(1526);
            else if (topCheck && botCheck && rightCheck) addWave(1520); // →
            else if (leftCheck && botCheck && rightCheck) addWave(1523); //↑
            else if (botCheck && rightCheck && topCheck) addWave(1522); //←
            else if (rightCheck && topCheck && leftCheck) addWave(1521); // ↓
            else if (topCheck && botCheck) addWave(1524);
            else if (rightCheck && leftCheck) addWave(1525);
            else if (rightCheck && topCheck) addWave(1505);
            else if (topCheck && leftCheck)  addWave(1507);
            else if (leftCheck && botCheck)  addWave(1509);
            else if (botCheck && rightCheck) addWave(1511);
            else if (topCheck)  addWave(1506);
            else if (botCheck)  addWave(1510);
            else if (leftCheck) addWave(1508);
            else if (rightCheck)addWave(1504);


            const TileData* topRightTile = &World::ins()->getTile(tgtX + 1, tgtY - 1, PlayerZ());
            const TileData* topLeftTile = &World::ins()->getTile(tgtX - 1, tgtY - 1, PlayerZ());
            const TileData* botLeftTile = &World::ins()->getTile(tgtX - 1, tgtY + 1, PlayerZ());
            const TileData* botRightTile = &World::ins()->getTile(tgtX + 1, tgtY + 1, PlayerZ());

            bool topRightCheck = topRightTile->floor == itemRefCode::shallowSeaWater;
            bool topLeftCheck = topLeftTile->floor == itemRefCode::shallowSeaWater;
            bool botLeftCheck = botLeftTile->floor == itemRefCode::shallowSeaWater;
            bool botRightCheck = botRightTile->floor == itemRefCode::shallowSeaWater;

            if (topRightCheck && !topCheck && !rightCheck) addWave(1514);
            if (topLeftCheck && !topCheck && !leftCheck) addWave(1515);
            if (botLeftCheck && !botCheck && !leftCheck) addWave(1512);
            if (botRightCheck && !botCheck && !rightCheck) addWave(1513);
        }
    }

    for (auto elem : deepSeaWaves)
    {
        int tgtX = elem.x;
        int tgtY = elem.y;

        const TileData* thisTile = &World::ins()->getTile(tgtX, tgtY, PlayerZ());
        const TileData* topTile = &World::ins()->getTile(tgtX, tgtY - 1, PlayerZ());
        const TileData* botTile = &World::ins()->getTile(tgtX, tgtY + 1, PlayerZ());
        const TileData* leftTile = &World::ins()->getTile(tgtX - 1, tgtY, PlayerZ());
        const TileData* rightTile = &World::ins()->getTile(tgtX + 1, tgtY, PlayerZ());
        int animeExtraIndex = 32 * ((SDL_GetTicks() / 300) % 7);
        if (thisTile->floor == itemRefCode::shallowSeaWater) animeExtraIndex = 0;

        {
            bool topCheck = topTile->floor == itemRefCode::deepSeaWater;
            bool botCheck = botTile->floor == itemRefCode::deepSeaWater;
            bool leftCheck = leftTile->floor == itemRefCode::deepSeaWater;
            bool rightCheck = rightTile->floor == itemRefCode::deepSeaWater;
            auto addWave = [&](int inputIndex)
                {
                    vertices[tileCounter] =
                    {
                        cameraW / 2 + static_cast<int>(zoomScale * (16 * tgtX + 8 - cameraX)),
                        cameraH / 2 + static_cast<int>(zoomScale * (16 * tgtY + 8 - cameraY))
                    };
                    indices[tileCounter] = inputIndex + animeExtraIndex;
                    batchAlphas[tileCounter] = 200;
                    tileCounter++;
                };

            if (topCheck && botCheck && leftCheck && rightCheck) addWave(1526);
            else if (topCheck && botCheck && rightCheck) addWave(1520); // →
            else if (leftCheck && botCheck && rightCheck) addWave(1523); //↑
            else if (botCheck && rightCheck && topCheck) addWave(1522); //←
            else if (rightCheck && topCheck && leftCheck) addWave(1521); // ↓
            else if (topCheck && botCheck) addWave(1524);
            else if (rightCheck && leftCheck) addWave(1525);
            else if (rightCheck && topCheck) addWave(1505);
            else if (topCheck && leftCheck)  addWave(1507);
            else if (leftCheck && botCheck)  addWave(1509);
            else if (botCheck && rightCheck) addWave(1511);
            else if (topCheck)  addWave(1506);
            else if (botCheck)  addWave(1510);
            else if (leftCheck) addWave(1508);
            else if (rightCheck)addWave(1504);

            const TileData* topRightTile = &World::ins()->getTile(tgtX + 1, tgtY - 1, PlayerZ());
            const TileData* topLeftTile = &World::ins()->getTile(tgtX - 1, tgtY - 1, PlayerZ());
            const TileData* botLeftTile = &World::ins()->getTile(tgtX - 1, tgtY + 1, PlayerZ());
            const TileData* botRightTile = &World::ins()->getTile(tgtX + 1, tgtY + 1, PlayerZ());

            bool topRightCheck = topRightTile->floor == itemRefCode::deepSeaWater;
            bool topLeftCheck = topLeftTile->floor == itemRefCode::deepSeaWater;
            bool botLeftCheck = botLeftTile->floor == itemRefCode::deepSeaWater;
            bool botRightCheck = botRightTile->floor == itemRefCode::deepSeaWater;

            bool topCheckSw = topTile->floor == itemRefCode::shallowSeaWater;
            bool botCheckSw = botTile->floor == itemRefCode::shallowSeaWater;
            bool leftCheckSw = leftTile->floor == itemRefCode::shallowSeaWater;
            bool rightCheckSw = rightTile->floor == itemRefCode::shallowSeaWater;
            if (topRightCheck && (!topCheck && !topCheckSw) && (!rightCheck && !rightCheckSw)) addWave(1514);
            if (topLeftCheck && (!topCheck && !topCheckSw) && (!leftCheck && !leftCheckSw)) addWave(1515);
            if (botLeftCheck && (!botCheck && !botCheckSw) && (!leftCheck && !leftCheckSw)) addWave(1512);
            if (botRightCheck && (!botCheck && !botCheckSw) && (!rightCheck && !rightCheckSw)) addWave(1513);
        }
    }

    for (auto elem : deepFreshWaves)
    {
        int tgtX = elem.x;
        int tgtY = elem.y;

        const TileData* thisTile = &World::ins()->getTile(tgtX, tgtY, PlayerZ());
        const TileData* topTile = &World::ins()->getTile(tgtX, tgtY - 1, PlayerZ());
        const TileData* botTile = &World::ins()->getTile(tgtX, tgtY + 1, PlayerZ());
        const TileData* leftTile = &World::ins()->getTile(tgtX - 1, tgtY, PlayerZ());
        const TileData* rightTile = &World::ins()->getTile(tgtX + 1, tgtY, PlayerZ());
        int animeExtraIndex = 0;

        {
            bool topCheck = topTile->floor == itemRefCode::deepFreshWater;
            bool botCheck = botTile->floor == itemRefCode::deepFreshWater;
            bool leftCheck = leftTile->floor == itemRefCode::deepFreshWater;
            bool rightCheck = rightTile->floor == itemRefCode::deepFreshWater;
            auto addWave = [&](int inputIndex)
                {
                    vertices[tileCounter] =
                    {
                        cameraW / 2 + static_cast<int>(zoomScale * (16 * tgtX + 8 - cameraX)),
                        cameraH / 2 + static_cast<int>(zoomScale * (16 * tgtY + 8 - cameraY))
                    };
                    indices[tileCounter] = inputIndex + animeExtraIndex + 496;
                    batchAlphas[tileCounter] = 200;
                    tileCounter++;
                };

            if (topCheck && botCheck && leftCheck && rightCheck) addWave(1526);
            else if (topCheck && botCheck && rightCheck) addWave(1520); // →
            else if (leftCheck && botCheck && rightCheck) addWave(1523); //↑
            else if (botCheck && rightCheck && topCheck) addWave(1522); //←
            else if (rightCheck && topCheck && leftCheck) addWave(1521); // ↓
            else if (topCheck && botCheck) addWave(1524);
            else if (rightCheck && leftCheck) addWave(1525);
            else if (rightCheck && topCheck) addWave(1505);
            else if (topCheck && leftCheck)  addWave(1507);
            else if (leftCheck && botCheck)  addWave(1509);
            else if (botCheck && rightCheck) addWave(1511);
            else if (topCheck)  addWave(1506);
            else if (botCheck)  addWave(1510);
            else if (leftCheck) addWave(1508);
            else if (rightCheck)addWave(1504);

            const TileData* topRightTile = &World::ins()->getTile(tgtX + 1, tgtY - 1, PlayerZ());
            const TileData* topLeftTile = &World::ins()->getTile(tgtX - 1, tgtY - 1, PlayerZ());
            const TileData* botLeftTile = &World::ins()->getTile(tgtX - 1, tgtY + 1, PlayerZ());
            const TileData* botRightTile = &World::ins()->getTile(tgtX + 1, tgtY + 1, PlayerZ());

            bool topRightCheck = topRightTile->floor == itemRefCode::deepFreshWater;
            bool topLeftCheck = topLeftTile->floor == itemRefCode::deepFreshWater;
            bool botLeftCheck = botLeftTile->floor == itemRefCode::deepFreshWater;
            bool botRightCheck = botRightTile->floor == itemRefCode::deepFreshWater;

            bool topCheckSw = topTile->floor == itemRefCode::shallowFreshWater;
            bool botCheckSw = botTile->floor == itemRefCode::shallowFreshWater;
            bool leftCheckSw = leftTile->floor == itemRefCode::shallowFreshWater;
            bool rightCheckSw = rightTile->floor == itemRefCode::shallowFreshWater;
            if (topRightCheck && (!topCheck && !topCheckSw) && (!rightCheck && !rightCheckSw)) addWave(1514);
            if (topLeftCheck && (!topCheck && !topCheckSw) && (!leftCheck && !leftCheckSw)) addWave(1515);
            if (botLeftCheck && (!botCheck && !botCheckSw) && (!leftCheck && !leftCheckSw)) addWave(1512);
            if (botRightCheck && (!botCheck && !botCheckSw) && (!rightCheck && !rightCheckSw)) addWave(1513);
        }
    }

    for (auto elem : Wave::list)
    {
        int tgtX = elem->getGridX();
        int tgtY = elem->getGridY();
        vertices[tileCounter] =
        {
            cameraW / 2 + static_cast<int>(zoomScale * (16 * tgtX + 8 - cameraX)),
            cameraH / 2 + static_cast<int>(zoomScale * (16 * tgtY + 8 - cameraY))
        };
        

        if(elem->lifetime <6) indices[tileCounter] = 2018;
        else if(elem->lifetime <12) indices[tileCounter] = 2017;
        else indices[tileCounter] = 2016;

        if (TileFloor(tgtX, tgtY, PlayerZ()) == itemRefCode::shallowFreshWater) indices[tileCounter] += 3;

        
        batchAlphas[tileCounter] = elem->alpha;
        tileCounter++;
    }

    for (auto elem : Wake::list)
    {
        int tgtX = elem->getGridX();
        int tgtY = elem->getGridY();
        vertices[tileCounter] =
        {
            cameraW / 2 + static_cast<int>(zoomScale * (16 * tgtX + 8 - cameraX)),
            cameraH / 2 + static_cast<int>(zoomScale * (16 * tgtY + 8 - cameraY))
        };

        if (elem->lifetime < 6) indices[tileCounter] = 2032 + 32 + elem->dir;
        else if (elem->lifetime < 12) indices[tileCounter] = 2032 + 16 + elem->dir;
        else indices[tileCounter] = 2032 + elem->dir;

        if (TileFloor(tgtX, tgtY, PlayerZ()) == itemRefCode::deepFreshWater) indices[tileCounter] += 8;


        batchAlphas[tileCounter] = elem->alpha;
        tileCounter++;
    }

    drawSpriteBatchCenter(spr::tileset, vertices, indices, batchAlphas, tileCounter);
    
    
    for (const auto& elem : wallHPList)// 벽 HP 표기
    {
        int tgtX = elem.x;
        int tgtY = elem.y;
        TileData& t = World::ins()->getTile(tgtX, tgtY, PlayerZ());

        if (t.wallFakeHP > t.wallHP) t.wallFakeHP -= ((float)t.wallMaxHP / 100.0);
        else if (t.wallFakeHP < t.wallHP) t.wallFakeHP = t.wallHP;

        if (t.wallFakeHP != t.wallHP)
        {
            if (t.alphaFakeHPBar > 20) t.alphaFakeHPBar -= 20;
            else
            {
                t.alphaFakeHPBar = 0;
                t.wallFakeHP = t.wallHP;
            }
        }
        else t.alphaFakeHPBar = 0;

        if (t.displayHPBarCount > 1) t.displayHPBarCount--;
        else if (t.displayHPBarCount == 1)
        {
            t.alphaHPBar -= 10;
            if (t.alphaHPBar <= 0)
            {
                t.alphaHPBar = 0;
                t.displayHPBarCount = 0;
            }
        }

        int drawingX = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX);
        int drawingY = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY);
        draw3pxGauge(
            drawingX - (int)(8 * zoomScale),
            drawingY,
            zoomScale,
            (float)t.wallHP / (float)t.wallMaxHP,
            t.alphaHPBar,
            lowCol::red,
            (float)t.wallFakeHP / (float)t.wallMaxHP,
            t.alphaFakeHPBar
        );
    }
    
    
    setZoom(1.0);
}

void drawCorpses()
{
    for (const auto& elem : Corpse::list)
    {
        Corpse* adr = elem;
        if (pZ == adr->getZ()) // 플레이어의 z축과 시체의 z축이 같을 경우
        {
            setZoom(zoomScale);
            drawSpriteCenter(adr->getSprite(), adr->getSprIndex(), (cameraW / 2) + zoomScale * (adr->getX() - cameraX), (cameraH / 2) + zoomScale * (adr->getY() - cameraY));
            setZoom(1.0);
        }
    }
}

void drawFloorProp()
{
    // 바닥 설치물 그리기
    for (const auto& elem : floorPropList)
    {
        int tgtX = elem.x;
        int tgtY = elem.y;
        Prop* iPtr = TileProp(tgtX, tgtY, PlayerZ());
        iPtr->drawSelf();
    }
}


void drawItems()
{
    for (const auto& elem : itemList)
    {
        int tgtX = elem.x;
        int tgtY = elem.y;

        ItemStack* address = TileItemStack(tgtX, tgtY, pZ);
        std::vector<ItemData>& pocketInfo = address->getPocket()->itemInfo;
        setZoom(zoomScale);
        drawSpriteCenter
        (
            spr::itemset,
            getItemSprIndex(pocketInfo[pocketInfo.size() - 1]),
            (cameraW / 2) + zoomScale * (address->getX() - cameraX + address->getIntegerFakeX()),
            (cameraH / 2) + zoomScale * (address->getY() - cameraY + address->getIntegerFakeY())
        );
        setZoom(1.0);
    }
}

void drawEntities()
{
    // 화염 그리기
    for (const auto& elem : flameList)
    {
        int tgtX = elem.x;
        int tgtY = elem.y;
        Flame* tgtFlame = (&World::ins()->getTile(tgtX, tgtY, PlayerZ()))->flamePtr.get();

        SDL_Rect dst;
        dst.x = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX) - ((16 * zoomScale) / 2);
        dst.y = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY) - ((16 * zoomScale) / 2);
        dst.w = tileSize;
        dst.h = tileSize;

        setZoom(zoomScale);
        SDL_SetTextureAlphaMod(spr::propset->getTexture(), 255);
        SDL_SetTextureBlendMode(spr::propset->getTexture(), SDL_BLENDMODE_BLEND);
        int sprIndex = 0;
        int animeVal = timer::timer600 % 30;
        if (animeVal < 6) sprIndex += 0;
        else if (animeVal < 12) sprIndex += 1;
        else if (animeVal < 18) sprIndex += 2;
        else if (animeVal < 24) sprIndex += 3;
        else sprIndex += 4;
        sprIndex += tgtFlame->sprRandomStart;
        sprIndex = sprIndex % 5;

        drawSpriteCenter
        (
            spr::flameSet,
            tgtFlame->sprInfimum + sprIndex,
            dst.x + dst.w / 2 + zoomScale,
            dst.y + dst.h / 2 + zoomScale
        );
        SDL_SetTextureAlphaMod(spr::propset->getTexture(), 255);
        setZoom(1.0);
    }

    std::vector<Point2> rotorList;
    // 차량그리기
    std::unordered_set<Drawable*> vehCache;
    for (const auto& elem : renderVehList)
    {
        if (vehCache.find(elem) == vehCache.end())
        {
            elem->drawSelf();
            vehCache.insert(elem);
        }
    }
    // 엔티티&일반설치물 그리기
    std::unordered_set<Drawable*> entityCache;
    for (const auto& elem : renderEntityList)
    {
        if (entityCache.find(elem) == entityCache.end())
        {
            elem->drawSelf();
            entityCache.insert(elem);
        }
    }

    // 헬기 로터 그리기
    for (int i = 0; i < rotorList.size(); i++)
    {
        int tgtX = rotorList[i].x;
        int tgtY = rotorList[i].y;
        Vehicle* vPtr = TileVehicle(tgtX, tgtY, PlayerZ());

        SDL_Rect dst;
        dst.x = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX) - ((16 * zoomScale) / 2);
        dst.y = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY) - ((16 * zoomScale) / 2);
        dst.w = tileSize;
        dst.h = tileSize;

        setZoom(zoomScale);
        if (TileVehicle(PlayerX(), PlayerY(), PlayerZ()) == vPtr)
        {
            SDL_SetTextureAlphaMod(spr::mainRotor->getTexture(), 50);
            SDL_SetTextureBlendMode(spr::mainRotor->getTexture(), SDL_BLENDMODE_BLEND);
        }
        drawSpriteCenter
        (
            spr::mainRotor,
            0,
            dst.x + dst.w / 2 + zoomScale * vPtr->getIntegerFakeX(),
            dst.y + dst.h / 2 + zoomScale * vPtr->getIntegerFakeY()
        );
        SDL_SetTextureAlphaMod(spr::mainRotor->getTexture(), 255);
        setZoom(1.0);
    }

    // 조종 중인 차량의 마커 그리기
    if (ctrlVeh != nullptr)
    {
        Vehicle* vPtr = ctrlVeh;
        for (auto it = vPtr->partInfo.begin(); it != vPtr->partInfo.end(); it++)
        {
            int tgtX = it->first.x;
            int tgtY = it->first.y;

            for (int layer = 0; layer < vPtr->partInfo[{tgtX, tgtY}]->itemInfo.size(); layer++)
            {
                if (vPtr->partInfo[{tgtX, tgtY}]->itemInfo[layer].checkFlag(itemFlag::TIRE_STEER))
                {
                    SDL_Rect dst;
                    dst.x = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX) - ((16 * zoomScale) / 2);
                    dst.y = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY) - ((16 * zoomScale) / 2);
                    dst.w = tileSize;
                    dst.h = tileSize;

                    setZoom(zoomScale);
                    SDL_SetTextureAlphaMod(spr::propset->getTexture(), 150);
                    SDL_SetTextureBlendMode(spr::propset->getTexture(), SDL_BLENDMODE_BLEND);
                    int sprIndex = vPtr->partInfo[{tgtX, tgtY}]->itemInfo[layer].propSprIndex + vPtr->partInfo[{tgtX, tgtY}]->itemInfo[layer].extraSprIndexSingle + 16 * vPtr->partInfo[{tgtX, tgtY}]->itemInfo[layer].extraSprIndex16;
                    drawSpriteCenter
                    (
                        spr::propset,
                        sprIndex,
                        dst.x + dst.w / 2 + zoomScale * vPtr->getIntegerFakeX(),
                        dst.y + dst.h / 2 + zoomScale * vPtr->getIntegerFakeY()
                    );
                    SDL_SetTextureAlphaMod(spr::dirMarker->getTexture(), 150);
                    SDL_SetTextureBlendMode(spr::dirMarker->getTexture(), SDL_BLENDMODE_BLEND);
                    drawSpriteCenter
                    (
                        spr::dirMarker,
                        128 + dir16toInt16(vPtr->wheelDir),
                        dst.x + dst.w / 2 + zoomScale * vPtr->getIntegerFakeX(),
                        dst.y + dst.h / 2 + zoomScale * vPtr->getIntegerFakeY()
                    );
                    SDL_SetTextureAlphaMod(spr::propset->getTexture(), 255);
                    SDL_SetTextureAlphaMod(spr::dirMarker->getTexture(), 255);
                    setZoom(1.0);
                }
                else if (vPtr->isPowerCart)
                {
                    SDL_Rect dst;
                    dst.x = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX) - ((16 * zoomScale) / 2);
                    dst.y = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY) - ((16 * zoomScale) / 2);
                    dst.w = tileSize;
                    dst.h = tileSize;

                    setZoom(zoomScale);
                    SDL_SetTextureAlphaMod(spr::dirMarker->getTexture(), 110);
                    SDL_SetTextureBlendMode(spr::dirMarker->getTexture(), SDL_BLENDMODE_BLEND);
                    drawSpriteCenter
                    (
                        spr::dirMarker,
                        224 + dir16toInt16(vPtr->bodyDir),
                        dst.x + dst.w / 2 + zoomScale * vPtr->getIntegerFakeX(),
                        dst.y + dst.h / 2 + zoomScale * vPtr->getIntegerFakeY()
                    );
                    SDL_SetTextureAlphaMod(spr::dirMarker->getTexture(), 255);
                    setZoom(1.0);
                }
            }

            // 플레이어 속도 표현
            if (tgtX == PlayerX() && tgtY == PlayerY())
            {
                if (vPtr->spdVec.isZeroVec() == false)
                {
                    SDL_Rect dst;
                    dst.x = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX) - ((16 * zoomScale) / 2);
                    dst.y = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY) - ((16 * zoomScale) / 2);
                    dst.w = tileSize;
                    dst.h = tileSize;

                    setZoom(zoomScale);
                    SDL_SetTextureAlphaMod(spr::dirMarker->getTexture(), 180);
                    SDL_SetTextureBlendMode(spr::dirMarker->getTexture(), SDL_BLENDMODE_BLEND);
                    int spdExtraIndex = 0;
                    if (vPtr->spdVec.getLength() < 5) spdExtraIndex = 0;
                    else if (vPtr->spdVec.getLength() < 10) spdExtraIndex = 1;
                    else if (vPtr->spdVec.getLength() < 15) spdExtraIndex = 2;
                    else spdExtraIndex = 3;
                    drawSpriteCenter
                    (
                        spr::dirMarker,
                        160 + getNearDir16(vPtr->spdVec) + 16 * spdExtraIndex,
                        dst.x + dst.w / 2 + zoomScale * vPtr->getIntegerFakeX(),
                        dst.y + dst.h / 2 + zoomScale * vPtr->getIntegerFakeY()
                    );
                    SDL_SetTextureAlphaMod(spr::dirMarker->getTexture(), 255);
                    setZoom(1.0);
                }
            }
        }
    }
}

void drawDamages()
{
    for (int i = 0; i < Damage::list.size(); i++)
    {
        Damage* address = Damage::list[i];
        setFontSize(10 * zoomScale);
        int drawingX = (cameraW / 2) + zoomScale * (address->getX() - cameraX);
        int drawingY = (cameraH / 2) + zoomScale * (address->getY() - cameraY);
        setZoom(zoomScale);
        SDL_SetTextureAlphaMod(address->getSprite()->getTexture(), address->getAlpha());
        drawSpriteCenter(address->getSprite(), 0, drawingX, drawingY);
        setZoom(1.0);
    }
}

void drawBullets()
{
    for (int i = 0; i < Bullet::list.size(); i++)
    {
        Bullet* address = Bullet::list[i];
        setZoom(zoomScale);
        drawSpriteCenter
        (
            address->sprite,
            address->sprIndex,
            (cameraW / 2) + zoomScale * (address->getX() - cameraX + address->getIntegerFakeX()),
            (cameraH / 2) + zoomScale * (address->getY() - cameraY + address->getIntegerFakeY())
        );
        setZoom(1.0);
    }
}

void drawParticles()
{
    for (int i = 0; i < Particle::list.size(); i++)
    {
        Particle* address = Particle::list[i];
        setZoom(zoomScale);
        SDL_SetTextureAlphaMod(address->sprite->getTexture(), address->alpha);
        drawSpriteCenter
        (
            address->sprite,
            address->sprIndex,
            (cameraW / 2) + zoomScale * (address->getX() - cameraX + address->getIntegerFakeX()),
            (cameraH / 2) + zoomScale * (address->getY() - cameraY + address->getIntegerFakeY())
        );
        SDL_SetTextureAlphaMod(address->sprite->getTexture(), 255);
        setZoom(1.0);
    }
}

void drawMulFogs()
{
    struct TimeColor
    {
        float time;
        SDL_Color color;
    };

    static const std::vector<TimeColor> timeColors =
    {
        {0.0f,  {0, 0, 59, 150}},
        {6.0f,  {0, 0, 59, 150}},
        {8.0f,  { 0,0,49,50}},
        {10.0f, {0, 0, 0, 0}},
        {17.0f, {0, 0, 0, 0}},
        {18.0f, {121, 78, 59, 130}},
        {18.5f, {0, 0, 59, 150}},
        {24.0f, {0, 0, 59, 150}}
    };

    float currentTime = getHour() + getMin() / 60.0f;

    SDL_Color mulLightColor = { 0, 0, 0, 0 };
    for (size_t i = 0; i < timeColors.size() - 1; ++i)
    {
        if (currentTime >= timeColors[i].time && currentTime < timeColors[i + 1].time)
        {
            float t1 = timeColors[i].time;
            float t2 = timeColors[i + 1].time;
            float ratio = (currentTime - t1) / (t2 - t1);

            const SDL_Color& c1 = timeColors[i].color;
            const SDL_Color& c2 = timeColors[i + 1].color;

            mulLightColor =
            {
                (Uint8)(c1.r + (c2.r - c1.r) * ratio),
                (Uint8)(c1.g + (c2.g - c1.g) * ratio),
                (Uint8)(c1.b + (c2.b - c1.b) * ratio),
                (Uint8)(c1.a + (c2.a - c1.a) * ratio)
            };
            break;
        }
    }

    int mulFogCounter = 0;
    for (const auto& fog : mulFogList)
    {
        if (lightFogSet.find({ fog.x,fog.y }) != lightFogSet.end()) continue;

        int screenX = cameraW / 2 + zoomScale * ((16 * fog.x + 8) - cameraX) - (8 * zoomScale);
        int screenY = cameraH / 2 + zoomScale * ((16 * fog.y + 8) - cameraY) - (8 * zoomScale);
        vertices[mulFogCounter] = { screenX, screenY };


        rectColors[mulFogCounter] = mulLightColor;



        mulFogCounter++;
    }

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_MUL);
    drawRectBatch(16, 16, rectColors, vertices, mulFogCounter, zoomScale);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
}

void drawFogs()
{
    for (const auto& elem : gasList)
    {
        int tgtX = elem.x;
        int tgtY = elem.y;
        SDL_Rect dst;
        dst.x = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX) - ((16 * zoomScale) / 2);
        dst.y = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY) - ((16 * zoomScale) / 2);
        dst.w = tileSize;
        dst.h = tileSize;

        setZoom(zoomScale);
        TileData* thisTile = &World::ins()->getTile(tgtX, tgtY, PlayerZ());

        for (int j = 0; j < thisTile->gasVec.size(); j++)
        {
            int alpha = 255 * (1 - std::exp(-0.03 * thisTile->gasVec[j].gasVol));
            SDL_SetTextureAlphaMod(spr::steamEffect1->getTexture(), std::min(255, alpha));
            SDL_SetTextureBlendMode(spr::steamEffect1->getTexture(), SDL_BLENDMODE_BLEND);

            SDL_SetTextureColorMod(spr::steamEffect1->getTexture(),
                itemDex[thisTile->gasVec[j].gasCode].gasColorR,
                itemDex[thisTile->gasVec[j].gasCode].gasColorG,
                itemDex[thisTile->gasVec[j].gasCode].gasColorB);
            int sprIndex = 0;//1초에 5번해야됨, 600 10
            sprIndex += thisTile->randomVal;
            if (timer::timer600 % 60 < 10) sprIndex += 0;
            else if (timer::timer600 % 60 < 20) sprIndex += 1;
            else if (timer::timer600 % 60 < 30) sprIndex += 2;
            else if (timer::timer600 % 60 < 40) sprIndex += 3;
            else if (timer::timer600 % 60 < 50) sprIndex += 4;
            else sprIndex += 5;
            sprIndex += 2 * j;
            sprIndex = sprIndex % 6;
            drawSpriteCenter
            (
                spr::steamEffect1,
                sprIndex,
                dst.x + dst.w / 2,
                dst.y + dst.h / 2
            );
        }
        SDL_SetTextureAlphaMod(spr::steamEffect1->getTexture(), 255);
        setZoom(1.0);
    }

    int fogCounter = 0;
    for (const auto& elem : blackFogList)
    {
        vertices[fogCounter] = {
            cameraW / 2 + static_cast<int>(zoomScale * ((16 * elem.x + 8) - cameraX) - ((16 * zoomScale) / 2)),
            cameraH / 2 + static_cast<int>(zoomScale * ((16 * elem.y + 8) - cameraY) - ((16 * zoomScale) / 2))
        };
        rectColors[fogCounter] = { 0x16, 0x16, 0x16, 255 };
        fogCounter++;

    }

    for (const auto& elem : grayFogList)
    {
        vertices[fogCounter] = {
            cameraW / 2 + static_cast<int>(zoomScale * ((16 * elem.x + 8) - cameraX) - ((16 * zoomScale) / 2)),
            cameraH / 2 + static_cast<int>(zoomScale * ((16 * elem.y + 8) - cameraY) - ((16 * zoomScale) / 2))
        };
        rectColors[fogCounter] = { 0x16, 0x16, 0x16, 185 };
        fogCounter++;
    }

    drawRectBatch(16, 16, rectColors, vertices, fogCounter, zoomScale);

    int lightCounter = 0;
    for (const auto& elem : lightFogList)
    {
        int tgtX = elem.x;
        int tgtY = elem.y;
        int posX = cameraW / 2 + static_cast<int>(zoomScale * ((16 * elem.x + 8) - cameraX) - ((16 * zoomScale) / 2));
        int posY = cameraH / 2 + static_cast<int>(zoomScale * ((16 * elem.y + 8) - cameraY) - ((16 * zoomScale) / 2));
        TileData* thisTile = &World::ins()->getTile(tgtX, tgtY, PlayerZ());
        for (int i = 0; i < thisTile->lightVec.size(); i++)
        {
            //vertices[lightCounter] = { posX, posY };
            //rectColors[lightCounter] = { 0x16, 0x16, 0x16, 200 };
            //lightCounter++;

            vertices[lightCounter] = { posX, posY };
            rectColors[lightCounter] = { thisTile->lightVec[i].r, thisTile->lightVec[i].g, thisTile->lightVec[i].b, thisTile->lightVec[i].a };
            lightCounter++;
        }
    }

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_ADD);
    drawRectBatch(16, 16, rectColors, vertices, lightCounter, zoomScale);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
}

void drawMarkers()
{
    auto drawEplsionText = [](auto* spr, int spriteIndex, int x, int y) {
        SDL_SetTextureBlendMode(spr->getTexture(), SDL_BLENDMODE_BLEND);
        SDL_SetTextureColorMod(spr->getTexture(), 255, 255, 255);
        SDL_SetTextureAlphaMod(spr->getTexture(), 0);

        drawSprite(spr, spriteIndex, x + zoomScale, y);
        drawSprite(spr, spriteIndex, x, y - zoomScale);
        drawSprite(spr, spriteIndex, x - zoomScale, y);
        drawSprite(spr, spriteIndex, x, y + zoomScale);

        SDL_SetTextureColorMod(spr->getTexture(), 255, 255, 255);
        SDL_SetTextureAlphaMod(spr->getTexture(), 255);
        drawSprite(spr, spriteIndex, x, y);
        SDL_SetTextureColorMod(spr->getTexture(), 255, 255, 255);
        };

    if (option::inputMethod == input::mouse)
    {
        if (isPlayerMoving == false && turnCycle == turn::playerInput)
        {
            if (checkCursor(&letterbox) == false 
                && checkCursor(&tab) == false 
                && checkCursor(&letterboxPopUpButton) == false 
                && checkCursor(&quickSlotRegion) == false
                && checkCursor(&minimapRegion) == false
                )
            {
                if (GUI::getLastGUI() == HUD::ins())
                {
                    int tgtX = getAbsMouseGrid().x;
                    int tgtY = getAbsMouseGrid().y;

                    dst.x = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX) - ((16 * zoomScale) / 2);
                    dst.y = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY) - ((16 * zoomScale) / 2);
                    dst.w = tileSize;
                    dst.h = tileSize;
                    setZoom(zoomScale);
                    drawSpriteCenter
                    (
                        spr::whiteMarker,
                        0,
                        dst.x + dst.w / 2,
                        dst.y + dst.h / 2
                    );
                    setZoom(1.0);
                }
            }
        }
    }

    for (int i = 1; i < aStarTrail.size(); i++)
    {
        int tgtX = aStarTrail[i].x;
        int tgtY = aStarTrail[i].y;
        dst.x = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX) - ((16 * zoomScale) / 2);
        dst.y = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY) - ((16 * zoomScale) / 2);
        dst.w = tileSize;
        dst.h = tileSize;
        setZoom(zoomScale);
        drawSpriteCenter
        (
            spr::trail,
            0,
            dst.x + dst.w / 2,
            dst.y + dst.h / 2
        );
        setZoom(1.0);
    }

    // 게임패드 화이트마커 그리기
    if (option::inputMethod == input::gamepad)
    {
        if (PlayerPtr->getAniType() == aniFlag::null)
        {
            if (gamepadWhiteMarker.z == PlayerZ())
            {
                if (std::abs(gamepadWhiteMarker.x - PlayerX()) <= MARKER_LIMIT_DIST)
                {
                    if (std::abs(gamepadWhiteMarker.y - PlayerY()) <= MARKER_LIMIT_DIST)
                    {
                        int tgtX = gamepadWhiteMarker.x;
                        int tgtY = gamepadWhiteMarker.y;
                        dst.x = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX) - ((16 * zoomScale) / 2);
                        dst.y = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY) - ((16 * zoomScale) / 2);
                        dst.w = tileSize;
                        dst.h = tileSize;
                        setZoom(zoomScale);
                        drawSpriteCenter
                        (
                            spr::whiteMarker,
                            0,
                            dst.x + dst.w / 2,
                            dst.y + dst.h / 2
                        );
                        setZoom(1.0);
                    }
                }
            }
        }
    }
whiteMarkerEnd:
    return;
}

void drawDebug()
{
    if (debug::chunkLineDraw)
    {
        for (const auto& elem : tileList)
        {
            int tgtX = elem.x;
            int tgtY = elem.y;

            dst.x = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX) - ((16 * zoomScale) / 2);
            dst.y = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY) - ((16 * zoomScale) / 2);
            dst.w = tileSize;
            dst.h = tileSize;

            if (tgtX % 13 == 0) drawLine(dst.x, dst.y, dst.x, dst.y + dst.h, col::red);
            else if (tgtX % 13 == 12)  drawLine(dst.x + dst.w, dst.y, dst.x + dst.w, dst.y + dst.h, col::red);

            if (tgtY % 13 == 0) drawLine(dst.x, dst.y, dst.x + dst.w, dst.y, col::red);
            else if (tgtY % 13 == 12)  drawLine(dst.x, dst.y + dst.h, dst.x + dst.w, dst.y + dst.h, col::red);

            if (std::abs(tgtX % 13) == 6 && std::abs(tgtY % 13) == 6)
            {
                setFontSize(20);

                drawTextCenter(L"CHUNK", dst.x + dst.w / 2, dst.y + dst.h / 2 - 12, col::red);

                int cx, cy;
                World::ins()->changeToChunkCoord(tgtX, tgtY, cx, cy);
                std::wstring chunkName = L"";
                chunkName += std::to_wstring(cx);
                chunkName += L",";
                chunkName += std::to_wstring(cy);
                chunkName += L",";
                chunkName += std::to_wstring(PlayerZ());
                drawTextCenter(chunkName, dst.x + dst.w / 2, dst.y + dst.h / 2 + 12, col::red);
            }
        }
    }
}
