import Player;

#include <SDL3/SDL.h>

import std;
import util;
import Entity;
import World;
import globalVar;
import wrapVar;
import textureVar;
import constVar;
import log;
import TileData;
import ItemPocket;
import ItemData;
import nanoTimer;
import globalTime;
import Footprint;
import GameOver;
import turnWait;
import Wave;
import Wake;

Player::Player(int gridX, int gridY, int gridZ) : Entity(1, gridX, gridY, gridZ)//생성자입니다.
{
	static Player* ptr = this;
	prt(L"[디버그] 플레이어 생성 완료 ID : %p\n", this);

	entityInfo.skin = humanCustom::skin::yellow;
	entityInfo.eyes = humanCustom::eyes::blue;
	entityInfo.hair = humanCustom::hair::bob1Black;

	entityInfo.isPlayer = true;

	int i = 0;

	//getEquipPtr()->addItemFromDex(itemRefCode::katana);
	//getEquipPtr()->itemInfo[i++].equipState = equipHandFlag::both;

	getEquipPtr()->addItemFromDex(2);
	getEquipPtr()->itemInfo[i++].equipState = equipHandFlag::normal;

	getEquipPtr()->addItemFromDex(290);
	getEquipPtr()->itemInfo[i++].equipState = equipHandFlag::normal;

	getEquipPtr()->addItemFromDex(105);
	getEquipPtr()->itemInfo[i++].equipState = equipHandFlag::normal;

	getEquipPtr()->addItemFromDex(106);
	getEquipPtr()->itemInfo[i++].equipState = equipHandFlag::normal;

	getEquipPtr()->addItemFromDex(107);
	getEquipPtr()->itemInfo[i++].equipState = equipHandFlag::normal;

	updateStatus();

	//방독면
	//getEquipPtr()->addItemFromDex(374);
	//getEquipPtr()->itemInfo[i++].equipState = equipHandFlag::normal;

	addSkill(30);
	quickSlot[2] = { quickSlotFlag::SKILL, 30 };

	addSkill(32);
	quickSlot[0] = { quickSlotFlag::SKILL,32 };

	addSkill(33);
	quickSlot[1] = { quickSlotFlag::SKILL, 33 };

	for (int i = 0; i < TALENT_SIZE; i++) entityInfo.proficApt[i] = 2.0;
}
Player::~Player()
{
	prt(L"Player : 소멸자가 호출되었습니다..\n");
}


void Player::startAtk(int inputGridX, int inputGridY, int inputGridZ, aniFlag inputAniType)
{
	Entity::startAtk(inputGridX, inputGridY, inputGridZ, inputAniType);
	addAniUSetPlayer(this, inputAniType);
}

void Player::startAtk(int inputGridX, int inputGridY, int inputGridZ) { startAtk(inputGridX, inputGridY, inputGridZ, aniFlag::atk); }


void Player::startMove(int inputDir)
{
	if (PlayerPtr->getAniType() == aniFlag::null)
	{
		//errorBox(PlayerPtr->getAniType() == aniFlag::null, L"Player's startMove activated while player's aniFlag is not null.");
		errorBox(((PlayerPtr)->getX() - 8) % 16 != 0, L"This instance moved from non-integer coordinates.");

		int dx, dy;
		dir2Coord(inputDir, dx, dy);
		Player* player = PlayerPtr;
		//걸을 수 있는 타일이면
		if (isWalkable({ PlayerX() + dx, PlayerY() + dy, PlayerZ() }))
		{
			player->setDirection(inputDir);
			if (TileSnow(PlayerX(), PlayerY(), PlayerZ()) || TileFloor(PlayerX(),PlayerY(),PlayerZ()) == itemRefCode::sandFloor)
			{
				new Footprint(getGridX(), getGridY(), getGridZ(), entityInfo.direction);
			}
			else if (TileFloor(PlayerX(), PlayerY(), PlayerZ()) == itemRefCode::deepFreshWater || TileFloor(PlayerX(), PlayerY(), PlayerZ()) == itemRefCode::deepSeaWater)
			{
				new Wake(getGridX(), getGridY(), getGridZ(), entityInfo.direction);
			}

			player->move(inputDir, false);
			turnCycle = turn::playerAnime;
		}
		else
		{
			player->setDirection(inputDir);
			if (TileEntity(player->getGridX() + dx, player->getGridY() + dy, player->getGridZ()) != nullptr)
			{
				player->startAtk(player->getGridX() + dx, player->getGridY() + dy, player->getGridZ());
				turnWait(1.0);
				PlayerPtr->deactAStarDst();
			}
		}
	}
}

void Player::updateMinimap()
{
	auto timeStampStart = getNanoTimer();


	// 현재 플레이어가 있는 청크 좌표 계산
	int playerChunkX, playerChunkY;
	World::ins()->changeToChunkCoord(getGridX(), getGridY(), playerChunkX, playerChunkY);

	if (ctrlVeh == nullptr)
	{
		SDL_SetRenderTarget(renderer, texture::minimap);
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
		SDL_RenderClear(renderer);

		for (int dx = -(MINIMAP_DIAMETER / 2); dx <= (MINIMAP_DIAMETER / 2); dx++)
		{
			for (int dy = -(MINIMAP_DIAMETER / 2); dy <= (MINIMAP_DIAMETER / 2); dy++)
			{
				if (isCircle(MINIMAP_DIAMETER / 2, dx, dy))
				{
					SDL_Color ptCol;
					const TileData* tgtTile = &World::ins()->getTile(getGridX() + dx, getGridY() + dy, getGridZ());
					if (tgtTile->fov == fovFlag::white || tgtTile->fov == fovFlag::gray)
					{
						//floor
						switch (tgtTile->floor)
						{
						case 0:
							break;
						default:
							ptCol = { 112,112, 112 };
							break;
						}
						//wall
						switch (tgtTile->wall)
						{
						case 0:
							break;
						default:
							ptCol = { 29,29, 29 };
							break;
						}
						//prop
						if (tgtTile->PropPtr != nullptr) ptCol = lowCol::yellow;
						//vehicle
						if (tgtTile->VehiclePtr != nullptr) ptCol = lowCol::orange;
						drawPoint(dx + (MINIMAP_DIAMETER / 2), dy + (MINIMAP_DIAMETER / 2), ptCol);
						if (tgtTile->fov == fovFlag::gray) drawPoint(dx + (MINIMAP_DIAMETER / 2), dy + (MINIMAP_DIAMETER / 2), col::black, 100);
					}
					else drawPoint(dx + (MINIMAP_DIAMETER / 2), dy + (MINIMAP_DIAMETER / 2), col::black);
				}
			}
		}
		SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
		SDL_RenderPoint(renderer, (MINIMAP_DIAMETER / 2), (MINIMAP_DIAMETER / 2));
	}
	else
	{
		SDL_SetRenderTarget(renderer, texture::navimap);
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
		SDL_RenderClear(renderer);

		for (int dx = -(NAVIMAP_WIDTH / 2); dx <= (NAVIMAP_WIDTH / 2); dx++)
		{
			for (int dy = -(NAVIMAP_HEIGHT / 2); dy <= (NAVIMAP_HEIGHT / 2); dy++)
			{
				SDL_Color ptCol;
				const TileData* tgtTile = &World::ins()->getTile(getGridX() + dx, getGridY() + dy, getGridZ());
				ptCol = { 255,255, 255 };
				drawPoint(dx + (NAVIMAP_WIDTH / 2), dy + (NAVIMAP_HEIGHT / 2), ptCol);
				if (tgtTile->fov == fovFlag::white || tgtTile->fov == fovFlag::gray)
				{
					//floor
					switch (tgtTile->floor)
					{
					case 0:
						break;
					default:
						ptCol = { 112,112, 112 };
						break;
					}
					//wall
					switch (tgtTile->wall)
					{
					case 0:
						break;
					default:
						ptCol = { 29,29, 29 };
						break;
					}
					//prop
					if (tgtTile->PropPtr != nullptr) ptCol = lowCol::yellow;
					//vehicle
					if (tgtTile->VehiclePtr != nullptr) ptCol = lowCol::orange;
					drawPoint(dx + (NAVIMAP_WIDTH / 2), dy + (NAVIMAP_HEIGHT / 2), ptCol);
					if (tgtTile->fov == fovFlag::gray) drawPoint(dx + (NAVIMAP_WIDTH / 2), dy + (NAVIMAP_HEIGHT / 2), col::black, 100);
				}
				else drawPoint(dx + (NAVIMAP_WIDTH / 2), dy + (NAVIMAP_HEIGHT / 2), col::black);
			}
		}
		SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
		SDL_RenderPoint(renderer, (NAVIMAP_WIDTH / 2), (NAVIMAP_HEIGHT / 2));
	}
	SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, 0xff);
	SDL_SetRenderTarget(renderer, nullptr);
}



void Player::updateVision(int range, int cx, int cy)
{
	__int64 timeStampStart = getNanoTimer();
	//prt(L"[updateVision] %d,%d에서 시야업데이트가 진행되었다.\n",cx,cy);

	int correctionRange = range;
	if (getHour() >= 6 && getHour() < 18) correctionRange = range;
	else correctionRange = myMax(1,range - 2);

	//줌스케일이 최대일 때 45칸 정도가 최대로 들어옴
	for (int i = cx - DARK_VISION_RADIUS; i <= cx + DARK_VISION_RADIUS; i++)
	{
		for (int j = cy - DARK_VISION_RADIUS; j <= cy + DARK_VISION_RADIUS; j++)
		{
			if (TileFov(i, j, getGridZ()) == fovFlag::white) TileFov(i, j, getGridZ()) = fovFlag::gray;
		}
	}

	std::vector<Point2> tasksVec;
	tasksVec.reserve((2 * DARK_VISION_RADIUS + 1) * (2 * DARK_VISION_RADIUS + 1));
	for (int tgtX = cx - DARK_VISION_RADIUS; tgtX <= cx + DARK_VISION_RADIUS; tgtX++)
	{
		for (int tgtY = cy - DARK_VISION_RADIUS; tgtY <= cy + DARK_VISION_RADIUS; tgtY++)
		{
			tasksVec.push_back({ tgtX,tgtY });
		}
	}

	auto rayCastingWorker = [=](int cx, int cy, const std::vector<Point2>& points, int correctionRange)
		{
			for (const auto& point : points)
			{
				if (isCircle(correctionRange, point.x - cx, point.y - cy)) rayCasting(cx, cy, point.x, point.y);
				else if (isCircle(DARK_VISION_RADIUS, point.x - cx, point.y - cy)) rayCastingDark(cx, cy, point.x, point.y);
			}
		};

	int numThreads = threadPoolPtr->getAvailableThreads();
	int chunkSize = tasksVec.size() / numThreads;
	for (int i = 0; i < numThreads; i++) {
		std::vector<Point2>::iterator startPoint = tasksVec.begin() + i * chunkSize;

		std::vector<Point2>::iterator endPoint;
		if (i == numThreads - 1) endPoint = tasksVec.end(); //만약 마지막 스레드일 경우 벡터의 끝을 강제로 설정
		else endPoint = startPoint + chunkSize;
		std::vector<Point2> chunk(startPoint, endPoint);

		threadPoolPtr->addTask([=]() {
			rayCastingWorker(cx, cy, chunk, correctionRange);
			});
	}

	threadPoolPtr->waitForThreads();

    //std::wprintf(L"[updateVision] %d,%d에서 시야업데이트가 완료되었다. 소요시간 : %lf ms\n", cx, cy, (getNanoTimer() - timeStampStart) / 1000000.0);
}


void Player::updateVision(int range) {
	updateVision(range, getGridX(), getGridY());
}

void Player::updateVision() 
{
	updateVision(entityInfo.eyeSight, getGridX(), getGridY());
}

void Player::updateNearbyChunk(int range)
{
    //std::wprintf(L"updateNearbyChunk 호출됨. 플레이어 좌표 : %d,%d,%d\n", getGridX(), getGridY(), getGridZ());
	int chunkX, chunkY;
	World::ins()->changeToChunkCoord(getGridX(), getGridY(), chunkX, chunkY);
	for (int y = chunkY - range; y <= chunkY + range; y++)
	{
		for (int x = chunkX - range; x <= chunkX + range; x++)
		{
			if (World::ins()->existChunk(x, y, getGridZ()) == false)
			{
				World::ins()->createChunk(x, y, getGridZ());
			}
		}
	}

	for (int y = chunkY - range; y <= chunkY + range; y++)
	{
		for (int x = chunkX - range; x <= chunkX + range; x++)
		{
			if (World::ins()->existChunk(x, y, getGridZ() + 1) == false)
			{
				World::ins()->createChunk(x, y, getGridZ() + 1);
			}
		}
	}

	for (int y = chunkY - range; y <= chunkY + range; y++)
	{
		for (int x = chunkX - range; x <= chunkX + range; x++)
		{
			if (World::ins()->existChunk(x, y, getGridZ() - 1) == false)
			{
				World::ins()->createChunk(x, y, getGridZ() - 1);
			}
		}
	}

	World::ins()->deactivate();

	//std::wprintf(L"▼청크 활성화▼\n");
	for (int x = chunkX - 2; x <= chunkX + 2; x++)
	{
		for (int y = chunkY - 2; y <= chunkY + 2; y++)
		{
            //std::wprintf(L"(%d,%d) ", x, y);
			//if (y == chunkY + 2) std::wprintf(L"\n");
			World::ins()->activate(x, y, PlayerPtr->getGridZ());
		}
	}
}

void Player::setGrid(int inputGridX, int inputGridY, int inputGridZ)
{
	Coord::setGrid(inputGridX, inputGridY, inputGridZ);

	Point2 sectorXY = World::ins()->changeToSectorCoord(getGridX(), getGridY());
	for (int dir = -1; dir <= 7; dir++)
	{
		int dx, dy;
		dir2Coord(dir, dx, dy);
		if (World::ins()->isEmptySector(sectorXY.x + dx, sectorXY.y + dy, getGridZ()) == true) World::ins()->createSector(sectorXY.x + dx, sectorXY.y + dy, getGridZ());
	}
	updateNearbyChunk(CHUNK_LOADING_RANGE);
}

void Player::endMove()//aStar로 인해 이동이 끝났을 경우
{

	if (PlayerPtr->entityInfo.walkMode == walkFlag::run)
	{
		entityInfo.STA -= 7;
		if (entityInfo.STA < 0)
		{
			entityInfo.STA = 0;
            changePlayerWalkMode(walkFlag::walk);
		}
	}


	if (itemDex[TileFloor(getGridX(), getGridY(), getGridZ())].checkFlag(itemFlag::WATER_SHALLOW))
	{
        changePlayerWalkMode(walkFlag::wade);
	}
	else if (itemDex[TileFloor(getGridX(), getGridY(), getGridZ())].checkFlag(itemFlag::WATER_DEEP))
	{
        changePlayerWalkMode(walkFlag::swim);
	}
	else if (entityInfo.walkMode == walkFlag::swim || entityInfo.walkMode == walkFlag::wade)
	{
		changePlayerWalkMode(walkFlag::walk);
	}

	if(TileFloor(getGridX(),getGridY(),getGridZ()) == itemRefCode::shallowFreshWater ||
	   TileFloor(getGridX(),getGridY(),getGridZ()) == itemRefCode::shallowSeaWater)
	{
		new Wave(getGridX(), getGridY(), getGridZ());
    }


	
	updateVision(entityInfo.eyeSight);
	updateMinimap();
	if (getHasAStarDst())
	{
		if (getAStarDstX() == getGridX() && getAStarDstY() == getGridY())
		{
			PlayerPtr->deactAStarDst();
			aStarTrail.clear();
		}
	}
}

void Player::death()
{
	GameOver::create(L"HP가 0이 되어 사망했다.");
}

int Player::checkItemSur(int index)//주변에 있는 타일을 포함해 아이템을 가지고 있는지 조사
{
	int itemNumber = 0;
	//주변 9타일의 아이템스택 검사
	for (int i = 0; i < 9; i++)
	{
		ItemStack* ptr = TileItemStack(getGridX(), getGridY(), getGridZ());
	}
	//자기 자신의 장비 검사 
	{
		itemNumber++;
	}
	return itemNumber;
}
void Player::eraseItemSur(int index, int number) //주변객체를 중심으로 총 9칸
{
	int residue = number;
	//주변 9타일의 아이템스택 검사
	for (int i = 0; i < 9; i++)
	{
		residue--;
		if (residue == 0) { return; }
	}
	//자기 자신의 장비 검사 
	{
		residue--;
		if (residue == 0) { return; }
	}
}
int Player::checkToolQualitySur(int index) //없으면 0 반환, 있으면 공구레벨 반환
{
	int itemNumber = 0;
	//주변 9타일의 아이템스택 검사
	for (int i = 0; i < 9; i++)
	{
		itemNumber++;
	}
	//자기 자신의 장비 검사 
	{
		itemNumber++;
	}
	return itemNumber;
}



// Torso (몸통) 저항
int Player::getResPierceTorso()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rPierceTorso;
	return totalVal;
}

int Player::getResCutTorso()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rCutTorso;
	return totalVal;
}

int Player::getResBashTorso()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rBashTorso;
	return totalVal;
}

// Head (머리) 저항
int Player::getResPierceHead()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rPierceHead;
	return totalVal;
}

int Player::getResCutHead()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rCutHead;
	return totalVal;
}

int Player::getResBashHead()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rBashHead;
	return totalVal;
}

// Left Arm (왼팔) 저항
int Player::getResPierceLArm()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rPierceLArm;
	return totalVal;
}

int Player::getResCutLArm()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rCutLArm;
	return totalVal;
}

int Player::getResBashLArm()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rBashLArm;
	return totalVal;
}

// Right Arm (오른팔) 저항
int Player::getResPierceRArm()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rPierceRArm;
	return totalVal;
}

int Player::getResCutRArm()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rCutRArm;
	return totalVal;
}

int Player::getResBashRArm()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rBashRArm;
	return totalVal;
}

// Left Leg (왼다리) 저항
int Player::getResPierceLLeg()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rPierceLLeg;
	return totalVal;
}

int Player::getResCutLLeg()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rCutLLeg;
	return totalVal;
}

int Player::getResBashLLeg()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rBashLLeg;
	return totalVal;
}

// Right Leg (오른다리) 저항
int Player::getResPierceRLeg()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rPierceRLeg;
	return totalVal;
}

int Player::getResCutRLeg()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rCutRLeg;
	return totalVal;
}

int Player::getResBashRLeg()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rBashRLeg;
	return totalVal;
}

////////////////////////////////////////////////////////////////////////////////

int Player::getSH()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].sh;
	return totalVal;
}

int Player::getEV()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].ev;
	return totalVal;
}

int Player::getResFire()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rFire;
	return totalVal;
}

int Player::getResCold()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rCold;
	return totalVal;
}

int Player::getResElec()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rElec;
	return totalVal;
}

int Player::getResCorr()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rCorr;
	return totalVal;
}

int Player::getResRad()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].rRad;
	return totalVal;
}

// Torso (몸통) 방해도
int Player::getEncTorso()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].encTorso;
	return totalVal;
}

// Head (머리) 방해도
int Player::getEncHead()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].encHead;
	return totalVal;
}

// Left Arm (왼팔) 방해도
int Player::getEncLArm()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].encLArm;
	return totalVal;
}

// Right Arm (오른팔) 방해도
int Player::getEncRArm()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].encRArm;
	return totalVal;
}

// Left Leg (왼다리) 방해도
int Player::getEncLLeg()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].encLLeg;
	return totalVal;
}

// Right Leg (오른다리) 방해도
int Player::getEncRLeg()
{
	std::vector<ItemData>& equip = entityInfo.equipment.get()->itemInfo;
	int totalVal = 0;
	for (int i = 0; i < equip.size(); i++) totalVal += equip[i].encRLeg;
	return totalVal;
}

