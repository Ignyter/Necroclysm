#define SNOW(x, y, z) (World::ins()->getTile(x, y, z).hasSnow = true)
#include <sol/sol.hpp>

export module startArea;

import util;
import globalVar;
import wrapVar;
import HUD;
import Player;
import ItemData;
import ItemPocket;
import ItemStack;
import World;
import Vehicle;
import Prop;
import Monster;

export void startArea()
{
	new HUD();

	World::ins()->getTile(0, 0, 0).EntityPtr = std::make_unique<Player>(0, 0, 0);
	PlayerPtr = (Player*)TileEntity(0, 0, 0);
	PlayerPtr->setGrid(0, 0, 0);
	PlayerPtr->setDstGrid(0, 0);

	Point2 sectorXY = World::ins()->changeToSectorCoord(PlayerX(), PlayerY());
	for (int dx = -2; dx <= 2; dx++)
	{
		for (int dy = -2; dy <= 2; dy++)
		{
			if (World::ins()->isEmptySector(sectorXY.x + dx, sectorXY.y + dy, PlayerZ()) == true) World::ins()->createSector(sectorXY.x + dx, sectorXY.y + dy, PlayerZ());
		}
	}

	PlayerPtr->entityInfo.statusEffectVec.push_back({ statusEffectFlag::hungry, -1 });
	PlayerPtr->entityInfo.statusEffectVec.push_back({ statusEffectFlag::dehydrated, -1 });
	PlayerPtr->entityInfo.statusEffectVec.push_back({ statusEffectFlag::tired, -1 });



	createItemStack({ 2, 1, 0 }, {
{2, 1}, {0, 5}, {23, 1}, {24, 10}, {1, 4}, {0, 1},
{3, 1}, {12, 1}, {13, 1}, {14, 1}, {15, 1}, {16, 1},
{17, 1}, {18, 1}, {4, 1}, {5, 8}, {88, 1}, {89, 1000},
{91, 1000},{itemRefCode::gasoline, 1000}, {82, 1},{389,1}, {386,2}, {387,1},{441,99}
		}
	);

	createItemStack({ 12, 17, 0 }, {
		{454, 1}, {455, 1}, {456, 1}, {60, 10}, {61, 4}, {62, 1},
		{63, 1}, {64, 1}, {65, 1}, {66, 1}, {67, 1}
		}
	);




	//의약품 상자
	{
		createItemStack({ 0, -19, 0 }, { { 452,1 } });
		ItemPocket* aidKitInside = TileItemStack({ 0, -19, 0 })->getPocket()->itemInfo[0].pocketPtr.get();
		aidKitInside->addItemFromDex({ { 442, 1 }, { 443, 1 }, { 446, 4 }, { 447, 10 }, { 449, 1 }, { 451, 20 } });
		std::vector<ItemData>& targetItemInfo = aidKitInside->itemInfo;
		for (int i = 0; i < targetItemInfo.size(); i++)
		{
			if (targetItemInfo[i].itemCode == 442)
			{
				targetItemInfo[i].pocketPtr->addItemFromDex(444, 9);
			}
			else if (targetItemInfo[i].itemCode == 443)
			{
				targetItemInfo[i].pocketPtr->addItemFromDex(445, 100);
			}
			else if (targetItemInfo[i].itemCode == 449)
			{
				targetItemInfo[i].pocketPtr->addItemFromDex(450, 30);
			}
		}
	}


	createItemStack({ -5, 2, 0 }, { {373, 1},{475, 1},{476, 1},{477, 1},{478, 1} });//페트병
	

	createItemStack({ 2, 8, 0 }, { {itemRefCode::arrowQuiver,1} });//화살통
	createItemStack({ 2, 9, 0 }, { {itemRefCode::boltQuiver,1} });//볼트통

	//활과 석궁
	createItemStack({ 3, 8, 0 }, { {383,1} });
	createItemStack({ 4, 8, 0 }, { {385,30} });
	createItemStack({ 3, 9, 0 }, { {382,1} });
	createItemStack({ 4, 9, 0 }, { {384,30} });

	createMonster({ 8,8,0 }, 5);//허수아비

	createItemStack({ 7, -4, 0 }, { {391,1} }); //벌목도끼

	createItemStack({ -5, 1, 0 }, { {394,1} }); //낚시대

	createItemStack({ -3, -4, -1 }, { {itemRefCode::pickaxe,1} });//곡괭이
	createItemStack({ -2, -4, -1 }, { {393,1} });//광부헬멧
	createItemStack({ -1, -4, -1 }, { {395,1} });//삽



	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////▼아이템 레시피 추가////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	for (int i = 1; i <= 491; i++)
	{
		if (itemDex[i].name != L"?")
		{
			if (itemDex[i].checkFlag(itemFlag::CANCRAFT) && getItemSprIndex(itemDex[i]) != 1)
			{
				ItemPocket::unlockRecipes.get()->addRecipe(i);
			}
		}
	}

	//타일 세팅
	{
		for (int dx = -30; dx <= 30; dx++)
		{
			for (int dy = -30; dy <= 30; dy++)
			{
				setFloor({ dx,dy,0 }, itemRefCode::dirt);
			}
		}
		for (int dx = -30; dx <= 30; dx++)
		{
			for (int dy = -30; dy <= -2; dy++)
			{
				setFloor({ dx,dy,0 }, itemRefCode::grass);
			}
		}
		for (int dx = -6; dx >= -14; dx--)
		{
			for (int dy = -30; dy <= 30; dy++)
			{
				if (dx == -10 && ((dy + 30) % 6 < 3)) setFloor({ dx,dy,0 }, itemRefCode::yellowAsphalt); //노랑아스팔트
				else setFloor({ dx,dy,0 }, itemRefCode::blackAsphalt); //검정아스팔트
			}
		}

		//집 바닥 타일
		for (int dx = 0; dx < 5; dx++)
		{
			for (int dy = 0; dy < 4; dy++)
			{
				setFloor({ -5 + dx, -5 + dy, 0 }, 292);
			}
		}
		createProp({ 2, -1, 0 }, 297);//표지판
		//유리벽 설치
		setWall({ 2,-4,0 }, 114);
		setWall({ 2,-3,0 }, 114);
		setWall({ 2,-2,0 }, 114);
		setWall({ 4,-4,0 }, 114);
		setWall({ 4,-3,0 }, 114);
		//얕은 물 타일(연못)
		setFloor({ -3,0,0 }, itemRefCode::shallowFreshWater);
		setFloor({ -4,0,0 }, itemRefCode::deepFreshWater);
		setFloor({ -3,1,0 }, itemRefCode::shallowFreshWater);
		setFloor({ -4,1,0 }, itemRefCode::deepFreshWater);
		setFloor({ -4,2,0 }, itemRefCode::shallowFreshWater);
		int startX = -33;
		int startY = 36;
		for (int dy = 0; dy <= 30; dy++)
		{
			for (int dx = 0; dx <= 60; dx++)
			{
				itemRefCode::sandFloor;
				if (dy <= 1) setFloor({ startX + dx, startY + dy, 0 }, itemRefCode::shallowSeaWater);//얕은해수
				else if (dy <= 5) setFloor({ startX + dx, startY + dy, 0 }, itemRefCode::sandFloor);//모래
				else if (dy <= 7) setFloor({ startX + dx, startY + dy, 0 }, itemRefCode::shallowSeaWater);//얕은해수
				else setFloor({ startX + dx, startY + dy, 0 }, itemRefCode::deepSeaWater);//깊은해수
			}
		}

		setFloor({ -7,41,0 }, itemRefCode::shallowSeaWater);
		setFloor({ 0,38,0 }, itemRefCode::shallowSeaWater);
		setFloor({ 1,38,0 }, itemRefCode::shallowSeaWater);
		setFloor({ 1,36,0 }, itemRefCode::deepSeaWater);
		setFloor({ 4,41,0 }, itemRefCode::shallowSeaWater);
		setFloor({ 5,41,0 }, itemRefCode::shallowSeaWater);
		setFloor({ 6,41,0 }, itemRefCode::shallowSeaWater);
		setFloor({ 5,43,0 }, itemRefCode::deepSeaWater);
		setFloor({ 6,43,0 }, itemRefCode::deepSeaWater);
		//하단연못
		setFloor({ 5,8,0 }, itemRefCode::shallowFreshWater);
		setFloor({ 5,9,0 }, itemRefCode::shallowFreshWater);
		for (int dx = -3; dx <= 2; dx++)
		{
			setFloor({ 5 + dx,10,0 }, itemRefCode::shallowFreshWater);
			setFloor({ 5 + dx,11,0 }, itemRefCode::shallowFreshWater);
		}

		//오솔길
		setFloor({ -3,-1,0 }, 293);
		setFloor({ -2,-1,0 }, 293);
		setFloor({ -2,0,0 }, 293);
		setFloor({ -1,0,0 }, 293);
		setFloor({ 0,0,0 }, 293);
		setFloor({ 1,0,0 }, 293);
		setFloor({ 2,0,0 }, 293);
		setFloor({ 2,1,0 }, 293);
		setFloor({ 3,1,0 }, 293);
		setFloor({ 4,1,0 }, 293);
		setFloor({ 4,2,0 }, 293);
		for (int i = 0; i < 9; i++) setFloor({ 5 + i,2,0 }, 293);
	}

	createProp({ 10, -8, 0 }, 96);//냉장고 설치
	ItemPocket* refri = TileProp(10, -8, 0)->leadItem.pocketPtr.get();
	refri->addItemFromDex({ { 4, 1 },{ 12, 1 },{ 410, 1 },{ 414, 1 },{ 415, 1 },{ 417, 1 } });
	createProp({ 9, -8, 0 }, 427);//탄통 설치
	TileProp(9, -8, 0)->leadItem.pocketPtr->addItemFromDex({ { 5,99},{ 15,99},{ 16,99},{ 17,99} ,{ 411,99},{ 412,99},{ 413,99},{ 416,99} ,{ 418,99 },{ 419,99 },{ 420,99 },{ 421,99 },{ 422,99 },{ 423,99 },{ 424,30 },{ 425,30 },{ 426,30 },{ 13,1 },{ 14,1 },{ 428,1 },{ 429,1 },{ 430,1 } });

	//나무벽 설치
	//집 하단 5타일
	setWall({ -1,-2,0 }, 375);
	setWall({ -2,-2,0 }, 375);
	createProp({ -3, -2, 0 }, 291);//나무문 설치
	setWall({ -4,-2,0 }, 375);
	setWall({ -5,-2,0 }, 375);
	//집 우측 4타일
	setWall({ -1,-3,0 }, 375);
	setWall({ -1,-4,0 }, 114);
	setWall({ -1,-5,0 }, 375);
	setWall({ -1,-6,0 }, 375);
	//집 좌측 4타일
	setWall({ -5,-3,0 }, 375);
	setWall({ -5,-4,0 }, 114);
	setWall({ -5,-5,0 }, 375);
	setWall({ -5,-6,0 }, 375);
	//잡 상단 중앙 3타일
	setWall({ -2,-6,0 }, 375);
	setWall({ -3,-6,0 }, 375);
	setWall({ -4,-6,0 }, 375);

	createProp({ -4, -5, 0 }, 295);//책장
	createProp({ -2, -5, 0 }, 294);//침대

	createProp({ -4, -3, 0 }, 298);//상승계단
	for (int dx = -1; dx <= 1; dx++)
	{
		for (int dy = -1; dy <= 1; dy++)
		{
			setFloor({ -3 + dx, -4 + dy, 1 }, 292);
		}
	}

	createProp({ -2, -3, 0 }, 299);//하강계단


	//철조망
	for (int i = 0; i < 17; i++)//상단
	{
		setWall({ -5 + i, -9, 0 }, 376);
	}

	for (int i = 0; i < 20; i++)//우측
	{
		if (-8 + i != 1 && -8 + i != 2 && -8 + i != 3)
		{
			setWall({ 11, -8 + i, 0 }, 376);
		}
	}

	for (int i = 0; i < 17; i++)//상단
	{
		setWall({ 11 - i, 12, 0 }, 376);
	}

	//철조망 우측 입구 전통등 2개
	createProp({ 12, 0, 0 }, 118);//볼라드등
	createProp({ 12, 4, 0 }, 118);//볼라드등


	//철조망 아래 선로

	createProp({ -2, 15, 0 }, itemRefCode::railBR);
	for (int i = 0; i < 11; i++)  createProp({ -1 + i, 15, 0 }, itemRefCode::railRL);
	createProp({ 10, 15, 0 }, itemRefCode::railSwitchWS);
	for (int i = 0; i < 7; i++)  createProp({ 10, 16 + i, 0 }, itemRefCode::railTB);
	createProp({ 10, 23, 0 }, itemRefCode::railTL);
	for (int i = 0; i < 6; i++)  createProp({ 9 - i, 23, 0 }, itemRefCode::railRL);
	createProp({ 3, 23, 0 }, itemRefCode::railTR);
	for (int i = 0; i < 3; i++) createProp({ 3, 22 - i, 0 }, itemRefCode::railTB);
	createProp({ 3, 19, 0 }, itemRefCode::railBL);
	for (int i = 0; i < 4; i++) createProp({ 2 - i, 19, 0 }, itemRefCode::railRL);
	createProp({ -2, 19, 0 }, itemRefCode::railTR);
	for (int i = 0; i < 3; i++) createProp({ -2, 18 - i, 0 }, itemRefCode::railTB);

	for (int i = 0; i < 5; i++)  createProp({ 11 + i, 15, 0 }, itemRefCode::railRL);

	//지하
	{
		for (int dx = -1; dx <= 1; dx++)
		{
			for (int dy = -2; dy <= 2; dy++)
			{
				DestroyWall(-2 + dx, -3 + dy, -1);
			}
		}

		for (int dx = -4; dx >= -20; dx--)
		{
			for (int dy = -6; dy <= 20; dy++)
			{
				setWall({ dx, dy, -1 }, 397);
			}
		}

		createProp({ - 2, - 5, -1 }, 211);//전통등
		{
			int cx = -1;
			int cy = -1;

			for (int i = 1; i < 9; i++) DestroyWall(cx, cy + i, -1);
			createProp({ cx, cy + 1, -1 }, 291);//나무문 설치

			for (int i = 1; i < 9; i++) DestroyWall(cx + i, cy, -1);
			createProp({ cx + 1, cy, -1 }, 291);
			Prop* door2 = TileProp(cx + 1, cy, -1);//나무문 설치
			door2->leadItem.extraSprIndexSingle = 2;

			int aisleEndX = cx + 8;
			int aisleEndY = cy;
			for (int dx = -3; dx <= 3; dx++)
			{
				for (int dy = -50; dy <= 40; dy++)
				{
					DestroyWall(aisleEndX + dx, aisleEndY + dy, -1);
				}
			}

			int aisleEndX2 = cx + 8 + 12;
			int aisleEndY2 = cy;
			for (int dx = -4; dx <= 4; dx++)
			{
				for (int dy = -40; dy <= 40; dy++)
				{
					DestroyWall(aisleEndX2 + dx, aisleEndY2 + dy, -1);
				}
			}

			int aisleEndX3 = cx + 8;
			int aisleEndY3 = cy - 14;
			for (int dx = -40; dx <= 40; dx++)
			{
				for (int dy = -3; dy <= 3; dy++)
				{
					DestroyWall(aisleEndX3 + dx, aisleEndY3 + dy, -1);
				}
			}

			int aisleEndX4 = 19;
			int aisleEndY4 = 12;
			for (int dx = -15; dx <= 0; dx++)
			{
				for (int dy = -3; dy <= 3; dy++)
				{
					DestroyWall(aisleEndX4 + dx, aisleEndY4 + dy, -1);
				}
			}


			int cursorX = aisleEndX;
			int cursorY = aisleEndY + 12;

			//for (int i = 0; i <= 78; i++)
			//{
			//	createProp(6, 39 - i, -1, itemRefCode::wideRailVLeft);
			//	createProp(7, 39 - i, -1, itemRefCode::wideRailVMid);
			//	createProp(8, 39 - i, -1, itemRefCode::wideRailVRight);
			//	cursorY--;
			//}

			for (int i = 0; i <= 78; i++)
			{
				createProp({ 6, 39 - i, -1 }, itemRefCode::wideRailVLeft);
				createProp({ 7, 39 - i, -1 }, itemRefCode::wideRailVMid);
				createProp({ 8, 39 - i, -1 }, itemRefCode::wideRailVRight);
				cursorY--;
			}


			//지하철 설치
			int vX = 7;
			int vY = -1;
			Vehicle* myTrainPower = new Vehicle(vX, vY, -1, itemRefCode::metalFrame);//차량 설치
			{
				myTrainPower->name = L"동력차";
				myTrainPower->vehType = vehFlag::train;
				myTrainPower->isPowerTrain = true;

				///////////////////////차량 기초 프레임//////////////////////////////////////
				myTrainPower->extendPart(vX, vY - 1, itemRefCode::metalFrame);
				myTrainPower->extendPart(vX - 1, vY - 1, itemRefCode::metalFrame);
				myTrainPower->extendPart(vX + 1, vY - 1, itemRefCode::metalFrame);
				myTrainPower->extendPart(vX + 2, vY - 1, itemRefCode::metalFrame);
				myTrainPower->extendPart(vX - 2, vY - 1, itemRefCode::metalFrame);

				myTrainPower->extendPart(vX - 1, vY - 2, itemRefCode::metalFrame);
				myTrainPower->extendPart(vX, vY - 2, itemRefCode::metalFrame);
				myTrainPower->extendPart(vX + 1, vY - 2, itemRefCode::metalFrame);
				myTrainPower->extendPart(vX + 2, vY - 2, itemRefCode::metalFrame);
				myTrainPower->extendPart(vX - 2, vY - 2, itemRefCode::metalFrame);

				myTrainPower->extendPart(vX - 1, vY - 3, itemRefCode::metalFrame);
				myTrainPower->extendPart(vX, vY - 3, itemRefCode::metalFrame);
				myTrainPower->extendPart(vX + 1, vY - 3, itemRefCode::metalFrame);
				myTrainPower->extendPart(vX + 2, vY - 3, itemRefCode::metalFrame);
				myTrainPower->extendPart(vX - 2, vY - 3, itemRefCode::metalFrame);

				myTrainPower->extendPart(vX - 1, vY, itemRefCode::metalFrame);
				myTrainPower->extendPart(vX + 1, vY, itemRefCode::metalFrame);
				myTrainPower->extendPart(vX + 2, vY, itemRefCode::metalFrame);
				myTrainPower->extendPart(vX - 2, vY, itemRefCode::metalFrame);

				for (int i = 1; i < 5; i++)
				{
					myTrainPower->extendPart(vX - 1, vY + i, itemRefCode::metalFrame);
					myTrainPower->extendPart(vX, vY + i, itemRefCode::metalFrame);
					myTrainPower->extendPart(vX + 1, vY + i, itemRefCode::metalFrame);
					myTrainPower->extendPart(vX + 2, vY + i, itemRefCode::metalFrame);
					myTrainPower->extendPart(vX - 2, vY + i, itemRefCode::metalFrame);
				}

				int topLeftX = vX - 2;
				int topLeftY = vY - 3;

				for (int x = topLeftX; x <= topLeftX + 4; x++)
				{
					for (int y = topLeftY; y <= topLeftY + 7; y++)
					{
						if ((x == topLeftX || x == topLeftX + 4) || (y == topLeftY || y == topLeftY + 7))
						{
							if (x == topLeftX + 2 && y == topLeftY + 7)  myTrainPower->addPart(x, y, { 120 });
							else if (y == topLeftY + 4)  myTrainPower->addPart(x, y, { 120 });
							else if (y == topLeftY) myTrainPower->addPart(x, y, { 121 });
							else myTrainPower->addPart(x, y, { 119 });
						}
						else if ((y == topLeftY + 2))
						{
							if (x == topLeftX + 2) myTrainPower->addPart(x, y, { 120 });
							else myTrainPower->addPart(x, y, { 119 });
						}
						else
						{
							myTrainPower->addPart(x, y, { 122 });
						}
					}
				}

				myTrainPower->addPart(topLeftX + 2, topLeftY + 1, { 313 });


				myTrainPower->addPart(topLeftX + 1, topLeftY + 3, { 123 });
				myTrainPower->addPart(topLeftX + 1, topLeftY + 5, { 123 });
				myTrainPower->addPart(topLeftX + 1, topLeftY + 6, { 123 });

				myTrainPower->addPart(topLeftX + 3, topLeftY + 3, { 123 });
				myTrainPower->addPart(topLeftX + 3, topLeftY + 5, { 123 });
				myTrainPower->addPart(topLeftX + 3, topLeftY + 6, { 123 });
			}

			//지하철(화물칸) 설치

			Vehicle* myTrain = new Vehicle(7, 7, -1, itemRefCode::metalFrame);//차량 설치

			{
				int vX = 7;
				int vY = 7;
				myTrain->vehType = vehFlag::train;

				///////////////////////차량 기초 프레임//////////////////////////////////////
				myTrain->extendPart(vX, vY - 1, itemRefCode::metalFrame);
				myTrain->extendPart(vX - 1, vY - 1, itemRefCode::metalFrame);
				myTrain->extendPart(vX + 1, vY - 1, itemRefCode::metalFrame);
				myTrain->extendPart(vX + 2, vY - 1, itemRefCode::metalFrame);
				myTrain->extendPart(vX - 2, vY - 1, itemRefCode::metalFrame);

				myTrain->extendPart(vX - 1, vY - 2, itemRefCode::metalFrame);
				myTrain->extendPart(vX, vY - 2, itemRefCode::metalFrame);
				myTrain->extendPart(vX + 1, vY - 2, itemRefCode::metalFrame);
				myTrain->extendPart(vX + 2, vY - 2, itemRefCode::metalFrame);
				myTrain->extendPart(vX - 2, vY - 2, itemRefCode::metalFrame);

				myTrain->extendPart(vX - 1, vY - 3, itemRefCode::metalFrame);
				myTrain->extendPart(vX, vY - 3, itemRefCode::metalFrame);
				myTrain->extendPart(vX + 1, vY - 3, itemRefCode::metalFrame);
				myTrain->extendPart(vX + 2, vY - 3, itemRefCode::metalFrame);
				myTrain->extendPart(vX - 2, vY - 3, itemRefCode::metalFrame);

				myTrain->extendPart(vX - 1, vY, itemRefCode::metalFrame);
				myTrain->extendPart(vX + 1, vY, itemRefCode::metalFrame);
				myTrain->extendPart(vX + 2, vY, itemRefCode::metalFrame);
				myTrain->extendPart(vX - 2, vY, itemRefCode::metalFrame);

				for (int i = 1; i < 5; i++)
				{
					myTrain->extendPart(vX - 1, vY + i, itemRefCode::metalFrame);
					myTrain->extendPart(vX, vY + i, itemRefCode::metalFrame);
					myTrain->extendPart(vX + 1, vY + i, itemRefCode::metalFrame);
					myTrain->extendPart(vX + 2, vY + i, itemRefCode::metalFrame);
					myTrain->extendPart(vX - 2, vY + i, itemRefCode::metalFrame);
				}

				int topLeftX = vX - 2;
				int topLeftY = vY - 3;

				for (int x = topLeftX; x <= topLeftX + 4; x++)
				{
					for (int y = topLeftY; y <= topLeftY + 7; y++)
					{
						if ((x == topLeftX || x == topLeftX + 4) || (y == topLeftY || y == topLeftY + 7))
						{
							if (x == topLeftX + 2)  myTrain->addPart(x, y, { 120 });
							else if (y == topLeftY + 4)  myTrain->addPart(x, y, { 120 });
							else myTrain->addPart(x, y, { 119 });
						}
						else
						{
							myTrain->addPart(x, y, { 122 });
						}
					}
				}

				myTrain->addPart(topLeftX + 1, topLeftY + 1, { 123 });
				myTrain->addPart(topLeftX + 1, topLeftY + 2, { 123 });
				myTrain->addPart(topLeftX + 1, topLeftY + 3, { 123 });
				myTrain->addPart(topLeftX + 1, topLeftY + 5, { 123 });
				myTrain->addPart(topLeftX + 1, topLeftY + 6, { 123 });

				myTrain->addPart(topLeftX + 3, topLeftY + 1, { 123 });
				myTrain->addPart(topLeftX + 3, topLeftY + 2, { 123 });
				myTrain->addPart(topLeftX + 3, topLeftY + 3, { 123 });
				myTrain->addPart(topLeftX + 3, topLeftY + 5, { 123 });
				myTrain->addPart(topLeftX + 3, topLeftY + 6, { 123 });
			}
			myTrainPower->rearTrain = myTrain;


			//for (int targetY = endY; targetY >= endY - 19; targetY--)
			//{
			//	createProp(endX + 3, targetY, pZ - 1, 303);//나무문 설치
			//}

			//createProp(endX + 3, endY - 20, pZ - 1, 317);//나무문 설치

			//for (int targetY = endY - 21; targetY >= endY - 50; targetY--)
			//{
			//	createProp(endX + 3, targetY, pZ - 1, 303);//나무문 설치
			//}

			//for (int targetX = endX + 1; targetX >= endX - 30; targetX--)
			//{
			//	createProp(targetX, endY - 20, pZ - 1, 303);//나무문 설치
			//}
		}
	}




	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////▼설치물 추가////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	//createProp({pX + 1 - 1, pY - 2, pZ}, 234);//벚꽃낙엽 설치
	//createProp({pX + 1, pY - 2 + 1, pZ}, 234);//벚꽃낙엽 설치
	//createProp({pX + 6, pY - 5, pZ}, 234);//벚꽃낙엽 설치
	//createProp({pX + 7, pY - 3, pZ}, 234);//벚꽃낙엽 설치
	//createProp({pX + 7, pY - 4, pZ}, 234);//벚꽃낙엽 설치
	//createProp({pX + 5, pY - 1 + 1, pZ}, 234);//벚꽃낙엽 설치
	//createProp({pX + 5 - 1, pY - 1, pZ}, 234);//벚꽃낙엽 설치
	//createProp({pX + 5 - 1, pY - 1 + 1, pZ}, 234);//벚꽃낙엽 설치
	//createProp({pX + 7 - 1, pY + 1, pZ}, 234);//벚꽃낙엽 설치


	/*
	for (int x = 0; x <= 10; x++) {
		for (int y = -8; y <= -2; y++) {
			SNOW(x, y, 0);
		}
	}

	for (int x = 4; x <= 10; x++) SNOW(x, -1, 0);

	for (int x = 6; x <= 10; x++) SNOW(x, 0, 0);

	SNOW(3, 4, 0);
	SNOW(3, 5, 0);
	SNOW(4, 5, 0);

	for (int x = 5; x <= 13; x++) {
		for (int y = 4; y <= 7; y++) {
			SNOW(x, y, 0);
		}
	}

	for (int x = -5; x <= 1; x++) {
		for (int y = 4; y <= 11; y++) {
			SNOW(x, y, 0);
		}
	}

	for (int x = 2; x <= 3; x++) {
		for (int y = 6; y <= 8; y++) {
			SNOW(x, y, 0);
		}
	}

	for (int x = -2; x <= 1; x++) {
		for (int y = 2; y <= 3; y++) {
			SNOW(x, y, 0);
		}
	}

	SNOW(-3, 3, 0);

	for (int x = 11; x <= 13; x++) {
		for (int y = -8; y <= -1; y++) {
			SNOW(x, y, 0);
		}
	}

	for (int x = -6; x <= -1; x++) {
		for (int y = -9; y <= -7; y++) {
			SNOW(x, y, 0);
		}
	}

	SNOW(6, 3, 0);
	SNOW(9, 3, 0);
	SNOW(10, 3, 0);
	SNOW(-5, 3, 0);
	SNOW(7, 1, 0);
	SNOW(-5, 2, 0);
	*/

	//for (int y = -7; y <= -3; y++) SNOW(14, y, 0);


	createProp({ 1, -3, 0 }, 117);//나무 설치
	createProp({ 3, -2, 0 }, 239);//나무 설치
	createProp({ 5, -1, 0 }, 247);//나무 설치
	createProp({ 0, -5, 0 }, 238);//나무 설치
	createProp({ 7, 1, 0 }, 237);//나무 설치
	createProp({ 4, -5, 0 }, 248);//나무 설치
	createProp({ 9, -4, 0 }, 237);//나무 설치
	createProp({ 10, -1, 0 }, 244);//사과나무 설치
	createProp({ -2, 39, 0 }, 242);//야자나무 설치
	createProp({ 0, -20, 0 }, 237);//나무 설치
	createProp({ -4, 5, 0 }, 245);//사과나무 설치
	createProp({ 3, 3, 0 }, 338);//고철 설치
	createProp({ 4, 3, 0 }, 338);//고철 설치
	createProp({ 5, 3, 0 }, 338);//고철 설치
	createProp({ 4, 4, 0 }, 338);//고철 설치
	createProp({ 10, 11, 0 }, 338);//고철 설치
	createProp({ 10, 10, 0 }, 338);//고철 설치
	createProp({ 10, 9, 0 }, 338);//고철 설치
	createProp({ 9, 11, 0 }, 338);//고철 설치
	createProp({ 9, 10, 0 }, 338);//고철 설치
	//잔디

	setFloor({ -2, 3, 0 }, itemRefCode::grass);
	setFloor({ -1, 3, 0 }, itemRefCode::grass);
	setFloor({ 0, 3, 0 }, itemRefCode::grass);
	setFloor({ 1, 3, 0 }, itemRefCode::grass);
	setFloor({ -2, 4, 0 }, itemRefCode::grass);
	setFloor({ -1, 4, 0 }, itemRefCode::grass);
	setFloor({ 0, 4, 0 }, itemRefCode::grass);
	setFloor({ 1, 4, 0 }, itemRefCode::grass);
	createProp({ -2, 3, 0 }, 270);//꽃 설치
	createProp({ -1, 3, 0 }, 265);//꽃 설치
	createProp({ 0, 3, 0 }, 266);//꽃 설치
	createProp({ 1, 3, 0 }, 267);//꽃 설치
	createProp({ -2, 4, 0 }, 271);//꽃 설치
	createProp({ -1, 4, 0 }, 268);//꽃 설치
	createProp({ 0, 4, 0 }, 269);//꽃 설치
	createProp({ 1, 4, 0 }, 270);//꽃 설치
	createProp({ 6, -4, 0 }, 270);//꽃 설치
	createProp({ 0, -1, 0 }, 211);//전통 등 설치
	createProp({ 4, 0, 0 }, 211);//볼라드 등 설치

	//울타리 설치
	createProp({ -3, 2, 0 }, 206);
	createProp({ -2, 2, 0 }, 206);
	createProp({ -1, 2, 0 }, 206);
	createProp({ 0, 2, 0 }, 206);
	createProp({ 1, 2, 0 }, 206);
	createProp({ 2, 2, 0 }, 206);
	createProp({ 3, 2, 0 }, 206);
	createProp({ -3, 3, 0 }, 206);
	createProp({ -3, 4, 0 }, 206);
	createProp({ 2, 3, 0 }, 206);
	createProp({ 2, 4, 0 }, 206);
	createProp({ -3, 5, 0 }, 206);
	createProp({ -2, 5, 0 }, 206);
	createProp({ -1, 5, 0 }, 206);
	createProp({ 0, 5, 0 }, 206);
	createProp({ 1, 5, 0 }, 206);
	createProp({ 2, 5, 0 }, 206);



	//배관 설치
	createProp({ 3, 6, 0 }, 144);
	createProp({ 4, 6, 0 }, 144);
	createProp({ 5, 6, 0 }, 144);
	createProp({ 6, 6, 0 }, 144);
	createProp({ 5, 7, 0 }, 144);

	//종교


	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	int vX = -8;
	int vY = +3;
	Vehicle* myCar = new Vehicle(vX, vY, 0, itemRefCode::metalFrame);//차량 설치
	myCar->name = L"SUV";
	myCar->vehType = vehFlag::car;

	///////////////////////차량 기초 프레임//////////////////////////////////////
	myCar->extendPart(vX, vY - 1, itemRefCode::metalFrame);
	myCar->extendPart(vX - 1, vY - 1, itemRefCode::metalFrame);
	myCar->extendPart(vX + 1, vY - 1, itemRefCode::metalFrame);
	myCar->extendPart(vX + 2, vY - 1, itemRefCode::metalFrame);
	myCar->extendPart(vX - 1, vY - 2, itemRefCode::metalFrame);
	myCar->extendPart(vX, vY - 2, itemRefCode::metalFrame);
	myCar->extendPart(vX + 1, vY - 2, itemRefCode::metalFrame);
	myCar->extendPart(vX + 2, vY - 2, itemRefCode::metalFrame);
	myCar->extendPart(vX - 1, vY, itemRefCode::metalFrame);
	myCar->extendPart(vX + 1, vY, itemRefCode::metalFrame);
	myCar->extendPart(vX + 2, vY, itemRefCode::metalFrame);
	myCar->extendPart(vX - 1, vY + 1, itemRefCode::metalFrame);
	myCar->extendPart(vX, vY + 1, itemRefCode::metalFrame);
	myCar->extendPart(vX + 1, vY + 1, itemRefCode::metalFrame);
	myCar->extendPart(vX + 2, vY + 1, itemRefCode::metalFrame);
	myCar->extendPart(vX - 1, vY + 2, itemRefCode::metalFrame);
	myCar->extendPart(vX, vY + 2, itemRefCode::metalFrame);
	myCar->extendPart(vX + 1, vY + 2, itemRefCode::metalFrame);
	myCar->extendPart(vX + 2, vY + 2, itemRefCode::metalFrame);
	myCar->extendPart(vX - 1, vY + 3, itemRefCode::metalFrame);
	myCar->extendPart(vX, vY + 3, itemRefCode::metalFrame);
	myCar->extendPart(vX + 1, vY + 3, itemRefCode::metalFrame);
	myCar->extendPart(vX + 2, vY + 3, itemRefCode::metalFrame);

	myCar->extendPart(vX - 1, vY - 3, 130);
	myCar->extendPart(vX, vY - 3, 130);
	myCar->extendPart(vX + 1, vY - 3, 130);
	myCar->extendPart(vX + 2, vY - 3, 130);
	//////////////////////////▼최상단 4타일////////////////////////////////////
	myCar->addPart(vX - 1, vY - 2, { 142,119,126 });
	myCar->addPart(vX, vY - 2, { 119 });
	myCar->addPart(vX + 1, vY - 2, { 119 });
	myCar->addPart(vX + 2, vY - 2, { 142,119,126 });
	//////////////////////////▼중상단 4타일////////////////////////////////////
	myCar->addPart(vX - 1, vY - 1, 121);
	myCar->addPart(vX, vY - 1, { 121,100 });
	myCar->addPart(vX + 1, vY - 1, 121);
	myCar->addPart(vX + 2, vY - 1, 121);
	////////////////////////////////▼운전석 4타일///////////////////////////////
	myCar->addPart(vX - 1, vY, { 120 });
	myCar->addPart(vX, vY, { 122, 123, 99, 128 });
	myCar->addPart(vX + 1, vY, { 122, 123, 128 });
	myCar->addPart(vX + 2, vY, { 120 });
	//////////////////////////▼운전석 아래 통로 4타일/////////////////////////////
	myCar->addPart(vX - 1, vY + 1, { 119 });
	myCar->addPart(vX, vY + 1, { 122, 128 });
	myCar->addPart(vX + 1, vY + 1, { 122, 128,129 });
	myCar->addPart(vX + 2, vY + 1, { 119 });
	///////////////////////////////▼뒷자석 4타일/////////////////////
	myCar->addPart(vX - 1, vY + 2, { 120, 101 });
	{
		ItemPocket* partPocket = myCar->partInfo[{vX - 1, vY + 2}].get();
		for (int i = 0; i < partPocket->itemInfo.size(); i++)
		{
			if (partPocket->itemInfo[i].itemCode == 101)
			{
				partPocket->itemInfo[i].pocketPtr->addItemFromDex(itemRefCode::gasoline, 900);
			}
		}
	}
	myCar->addPart(vX, vY + 2, { 122, 123, 128 });
	myCar->addPart(vX + 1, vY + 2, { 122, 123, 128 });
	myCar->addPart(vX + 2, vY + 2, { 120 });
	///////////////////////////////▼최후방 4타일///////////////////////////
	myCar->addPart(vX - 1, vY + 3, { 119,127 });
	myCar->addPart(vX, vY + 3, { 124 });
	myCar->addPart(vX + 1, vY + 3, { 124 });
	myCar->addPart(vX + 2, vY + 3, { 119,127 });



	///////////////////오토바이///////////////////////////////////////////
	Vehicle* myMoto = new Vehicle(vX + 6, vY + 5, 0, itemRefCode::metalFrame);
	myMoto->extendPart(vX + 6, vY + 4, itemRefCode::metalFrame);
	myMoto->extendPart(vX + 6, vY + 6, itemRefCode::metalFrame);

	myMoto->addPart(vX + 6, vY + 4, { 102,134 });
	myMoto->addPart(vX + 6, vY + 5, 132);
	myMoto->addPart(vX + 6, vY + 6, 102);

	////////////////////자전거////////////////////////////////////////////
	Vehicle* myBike = new Vehicle(vX + 9, vY + 4, 0, itemRefCode::metalFrame);
	myBike->extendPart(vX + 9, vY + 3, itemRefCode::metalFrame);
	myBike->extendPart(vX + 9, vY + 5, itemRefCode::metalFrame);

	myBike->addPart(vX + 9, vY + 3, { 102,133 });
	myBike->addPart(vX + 9, vY + 4, { 135, 132 });
	myBike->addPart(vX + 9, vY + 5, { 102,136 });


	////////////////////////////////////////////////////////////////////////////
	{
		int cx = 15;
		int cy = 0;
		Vehicle* myHeli = new Vehicle(cx, cy, 0, itemRefCode::metalFrame);
		myHeli->name = L"헬기";
		myHeli->vehType = vehFlag::heli;
		myHeli->addPart(cx, cy, { 311 });

		myHeli->extendPart(cx, cy - 1, itemRefCode::metalFrame);
		myHeli->extendPart(cx, cy + 1, itemRefCode::metalFrame);
		myHeli->extendPart(cx, cy + 2, itemRefCode::metalFrame);
		myHeli->extendPart(cx, cy + 3, itemRefCode::metalFrame);
		myHeli->extendPart(cx, cy + 4, itemRefCode::metalFrame);
		myHeli->extendPart(cx, cy + 5, itemRefCode::metalFrame);

		myHeli->extendPart(cx + 1, cy - 1, itemRefCode::metalFrame);
		myHeli->extendPart(cx + 1, cy, itemRefCode::metalFrame);
		myHeli->extendPart(cx + 1, cy + 1, itemRefCode::metalFrame);
		myHeli->extendPart(cx + 1, cy + 2, itemRefCode::metalFrame);

		myHeli->extendPart(cx - 1, cy - 1, itemRefCode::metalFrame);
		myHeli->extendPart(cx - 1, cy, itemRefCode::metalFrame);
		myHeli->extendPart(cx - 1, cy + 1, itemRefCode::metalFrame);
		myHeli->extendPart(cx - 1, cy + 2, itemRefCode::metalFrame);

		myHeli->addPart(cx + 1, cy - 1, { 121 });
		myHeli->addPart(cx, cy - 1, { 121 });
		myHeli->addPart(cx - 1, cy - 1, { 121 });

		myHeli->addPart(cx + 1, cy, { 120 });
		myHeli->addPart(cx, cy, { 122,123, 128 });
		myHeli->addPart(cx - 1, cy, { 120 });

		myHeli->addPart(cx + 1, cy + 1, { 119 });
		myHeli->addPart(cx, cy + 1, { 122, 128, 314 });
		myHeli->addPart(cx - 1, cy + 1, { 119 });

		myHeli->addPart(cx + 1, cy + 2, { 119 });
		myHeli->addPart(cx, cy + 2, { 119 });
		myHeli->addPart(cx - 1, cy + 2, { 119 });

		myHeli->addPart(cx, cy + 3, { 119 });
		myHeli->addPart(cx, cy + 4, { 119 });
		myHeli->addPart(cx, cy + 5, { 119, 315 });
	}

	//수레 3종
	Vehicle* cart1 = new Vehicle(10, 5, 0, 378);
	cart1->vehType = vehFlag::car;

	Vehicle* cart2 = new Vehicle(8, 5, 0, 379);
	cart2->vehType = vehFlag::car;

	Vehicle* cart3 = new Vehicle(6, 5, 0, 137);
	cart3->vehType = vehFlag::car;

	Vehicle* cart4 = new Vehicle(7, -5, 0, 378);
	cart4->vehType = vehFlag::car;

	//광차
	Vehicle* minecart1 = new Vehicle(3, 15, 0, 405);
	minecart1->vehType = vehFlag::minecart;
	minecart1->addPart(3, 15, { itemRefCode::minecartController });
	minecart1->bodyDir = dir16::dir0;
	minecart1->isPowerCart = true;

	Vehicle* minecart2 = new Vehicle(2, 15, 0, 405);
	minecart2->vehType = vehFlag::minecart;
	minecart2->bodyDir = dir16::dir0;

	Vehicle* minecart3 = new Vehicle(1, 15, 0, 405);
	minecart3->vehType = vehFlag::minecart;
	minecart3->bodyDir = dir16::dir0;

	minecart1->rearCart = minecart2;
	minecart2->rearCart = minecart3;



	///////////////////////////////////////////////////////////////


//타일 세팅 - 섬 모양
//타일 세팅 - 섬 모양
//타일 세팅 - 섬 모양
//타일 세팅 - 섬 모양
	{
		// 더 넓은 범위로 타일 설정
		for (int dx = -60; dx <= 60; dx++)
		{
			for (int dy = -60; dy <= 60; dy++)
			{
				float distance = sqrt(dx * dx + dy * dy);

				// 여러 노이즈를 조합해서 더 자연스러운 해안선 만들기
				float noise1 = sin(dx * 0.08f) * cos(dy * 0.12f) * 4.0f;
				float noise2 = sin(dx * 0.15f + dy * 0.1f) * 2.5f;
				float noise3 = cos(dx * 0.05f) * sin(dy * 0.07f) * 3.0f;
				float totalNoise = noise1 + noise2 + noise3;

				// 타원형 기본 모양 (가로세로 비율 조정)
				float ellipseX = dx / 1.4f;
				float ellipseY = dy / 1.1f;
				float ellipseDistance = sqrt(ellipseX * ellipseX + ellipseY * ellipseY);

				// 최종 거리 계산
				float finalDistance = ellipseDistance + totalNoise;

				if (finalDistance <= 30) // 중심부 - 땅/잔디
				{
					if (finalDistance <= 20)
					{
						setFloor({ dx, dy, 0 }, itemRefCode::grass);
					}
					else
					{
						setFloor({ dx, dy, 0 }, itemRefCode::dirt);
					}
				}
				else if (finalDistance <= 38) // 해변 - 모래
				{
					setFloor({ dx, dy, 0 }, itemRefCode::sandFloor);
				}
				else if (finalDistance <= 41) // 얕은 바다 (범위 축소: 38~41, 약 3타일)
				{
					setFloor({ dx, dy, 0 }, itemRefCode::shallowSeaWater);
				}
				else // 깊은 바다
				{
					setFloor({ dx, dy, 0 }, itemRefCode::deepSeaWater);
				}
			}
		}

		// 철조망 내부 지역을 예전처럼 흙으로 다시 설정
		for (int dx = -30; dx <= 30; dx++)
		{
			for (int dy = -30; dy <= 30; dy++)
			{
				// 철조망 내부 범위 (대략적으로)
				if (dx >= -5 && dx <= 11 && dy >= -9 && dy <= 12)
				{
					setFloor({ dx, dy, 0 }, itemRefCode::dirt);
				}
			}
		}

		// 철조망 내부에서 북쪽 부분만 잔디로 (예전처럼)
		for (int dx = -30; dx <= 30; dx++)
		{
			for (int dy = -30; dy <= -2; dy++)
			{
				// 철조망 내부의 북쪽 부분
				if (dx >= -5 && dx <= 11 && dy >= -9 && dy <= -2)
				{
					setFloor({ dx, dy, 0 }, itemRefCode::grass);
				}
			}
		}

		// 도로 다시 설정 (기존 도로를 위로 1칸 이동)
		for (int dx = -6; dx >= -14; dx--)
		{
			for (int dy = -31; dy <= 29; dy++)
			{
				if (dx == -10 && ((dy + 31) % 6 < 3)) setFloor({ dx, dy, 0 }, itemRefCode::yellowAsphalt);
				else setFloor({ dx, dy, 0 }, itemRefCode::blackAsphalt);
			}
		}

		// 집 바닥 타일 (기존 유지)
		for (int dx = 0; dx < 5; dx++)
		{
			for (int dy = 0; dy < 4; dy++)
			{
				setFloor({ -5 + dx, -5 + dy, 0 }, 292);
			}
		}

		// 레일 구간을 확실히 땅으로 설정 (더 부드럽게)
		for (int x = -5; x <= 20; x++)
		{
			for (int y = 12; y <= 26; y++)
			{
				float distFromRail = abs(y - 19) + abs(x - 7.5f) * 0.5f;
				if (distFromRail <= 8)
				{
					setFloor({ x, y, 0 }, itemRefCode::dirt);
				}
			}
		}

		// 기존 연못 유지
		setFloor({ -3, 0, 0 }, itemRefCode::shallowFreshWater);
		setFloor({ -4, 0, 0 }, itemRefCode::deepFreshWater);
		setFloor({ -3, 1, 0 }, itemRefCode::shallowFreshWater);
		setFloor({ -4, 1, 0 }, itemRefCode::deepFreshWater);
		setFloor({ -4, 2, 0 }, itemRefCode::shallowFreshWater);

		// 하단연못 유지
		setFloor({ 5, 8, 0 }, itemRefCode::shallowFreshWater);
		setFloor({ 5, 9, 0 }, itemRefCode::shallowFreshWater);
		for (int dx = -3; dx <= 2; dx++)
		{
			setFloor({ 5 + dx, 10, 0 }, itemRefCode::shallowFreshWater);
			setFloor({ 5 + dx, 11, 0 }, itemRefCode::shallowFreshWater);
		}

		// 오솔길 유지
		setFloor({ -3, -1, 0 }, 293);
		setFloor({ -2, -1, 0 }, 293);
		setFloor({ -2, 0, 0 }, 293);
		setFloor({ -1, 0, 0 }, 293);
		setFloor({ 0, 0, 0 }, 293);
		setFloor({ 1, 0, 0 }, 293);
		setFloor({ 2, 0, 0 }, 293);
		setFloor({ 2, 1, 0 }, 293);
		setFloor({ 3, 1, 0 }, 293);
		setFloor({ 4, 1, 0 }, 293);
		setFloor({ 4, 2, 0 }, 293);
		for (int i = 0; i < 9; i++) setFloor({ 5 + i, 2, 0 }, 293);
	}

	World::ins()->createSector(0, 0, 0);

	PlayerPtr->updateVision(PlayerPtr->entityInfo.eyeSight);

};