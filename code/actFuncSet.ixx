export module actFuncSet;

import std;
import util;
import globalVar;
import wrapVar;
import World;
import Entity;
import ItemData;
import ItemStack;
import ItemPocket;
import log;
import Msg;
import Lst;
import Player;
import Prop;
import CoordSelect;

//액트가 실행되는 환경은 3가지 경우가 가능
// 0:기본 HUD, 1:Loot, 2:Equip 

export enum class actEnv
{
	HUD,
	Loot,
	Equip,
	Inventory
};

export namespace actFunc
{
	//장전 : 총이나 탄창에 사용, 자기 자신의 탄환을 채워넣음
	export Corouter reloadSelf(actEnv envType, ItemPocket* reloadItemPocket, int reloadItemCursor)
	{
		prt(L"executeReloadSelf이 실행되었다.\n");
		int targetLootCursor = reloadItemCursor;
		std::vector<std::wstring> bulletList;
		ItemPocket* equipPtr = PlayerPtr->getEquipPtr();
		std::vector<ItemPocket*> targetSearchPtr;
		std::vector<ItemData>& equipInfo = PlayerPtr->getEquipPtr()->itemInfo;

		if (reloadItemPocket->itemInfo[targetLootCursor].checkFlag(itemFlag::BOW))
		{
			for (int j = 0; j < equipInfo.size(); j++)
			{
				if (equipInfo[j].itemCode == itemRefCode::arrowQuiver)
				{
					if (equipInfo[j].pocketPtr.get()->getPocketNumber() > 0)
					{
						equipInfo[j].pocketPtr.get()->transferItem(reloadItemPocket->itemInfo[targetLootCursor].pocketPtr.get(), 0, 1);
						updateLog(sysStr[325]);//화살을 시위에 걸었다.
						co_return;
					}
				}
			}
		}
		else if (reloadItemPocket->itemInfo[targetLootCursor].checkFlag(itemFlag::CROSSBOW))
		{
			for (int j = 0; j < equipInfo.size(); j++)
			{
				if (equipInfo[j].itemCode == itemRefCode::boltQuiver)
				{
					if (equipInfo[j].pocketPtr.get()->getPocketNumber() > 0)
					{
						equipInfo[j].pocketPtr.get()->transferItem(reloadItemPocket->itemInfo[targetLootCursor].pocketPtr.get(), 0, 1);
						updateLog(sysStr[326]);//볼트를 장전했다.
						co_return;
					}
				}
			}
		}

		//탐사할 타일 추가 (장비, 주변타일 9칸)
		{
			//장비타일
			targetSearchPtr.push_back(equipPtr);
			//바닥타일(주변9타일)
			for (int dir = -1; dir < 8; dir++)
			{
				int dx = 0, dy = 0;
				dir2Coord(dir, dx, dy);

				ItemStack* stack = TileItemStack(PlayerX() + dx, PlayerY() + dy, PlayerZ());
				if (stack != nullptr)
				{
					ItemPocket* lootPtr = stack->getPocket();
					targetSearchPtr.push_back(lootPtr);
				}
			}
		}

		for (int j = 0; j < targetSearchPtr.size(); j++)
		{
			//장비 중인 아이템에서 bulletList(또는 magazine) 추가
			for (int i = 0; i < targetSearchPtr[j]->itemInfo.size(); i++)
			{
				//만약 이 아이템에 넣을 수 있는 아이템코드가 equip에 있는 아이템과 같으면
				if (std::find(reloadItemPocket->itemInfo[targetLootCursor].pocketOnlyItem.begin(), reloadItemPocket->itemInfo[targetLootCursor].pocketOnlyItem.end(), targetSearchPtr[j]->itemInfo[i].itemCode) != reloadItemPocket->itemInfo[targetLootCursor].pocketOnlyItem.end())
				{
					bulletList.push_back(targetSearchPtr[j]->itemInfo[i].name);
				}
			}
		}

		if (bulletList.size() == 0) //넣을만한 포켓을 찾지 못했을 경우
		{
			updateLog(sysStr[96]);//이 아이템을 넣을만한 포켓이 없다.
			co_return;
		}

		////////////////////////////////////////////////////////////////////

		new Lst(sysStr[95], sysStr[94], bulletList);//넣기, 넣을 포켓을 선택해주세요.
		co_await std::suspend_always();

		////////////////////////////////////////////////////////////////////
		if (coAnswer.empty() == false)
		{
			int counter = 0;
			for (int j = 0; j < targetSearchPtr.size(); j++)
			{
				for (int i = 0; i < targetSearchPtr[j]->itemInfo.size(); i++)
				{
					//만약 이 아이템에 넣을 수 있는 아이템코드가 equip에 있는 아이템과 같으면
					if (std::find(reloadItemPocket->itemInfo[targetLootCursor].pocketOnlyItem.begin(), reloadItemPocket->itemInfo[targetLootCursor].pocketOnlyItem.end(), targetSearchPtr[j]->itemInfo[i].itemCode) != reloadItemPocket->itemInfo[targetLootCursor].pocketOnlyItem.end())
					{
						if (counter == wtoi(coAnswer.c_str()))
						{
							//넣을 수 있는만큼 가득 넣음
							targetSearchPtr[j]->transferItem
							(
								reloadItemPocket->itemInfo[targetLootCursor].pocketPtr.get(),
								i,
								1//일단은 전부 넣는걸로
							);

							co_return;
						}
						counter++;
					}
				}
			}
		}

	}

	export Corouter reloadOther(actEnv envType, ItemPocket* reloadItemPocket, int reloadItemCursor)//삽탄 : 총알에 사용, 이 탄환을 넣을 수 있는 탄창 리스트를 표시하고 거기에 넣음
	{
		//탄창이 장착한 총에 바로 넣는 기능 추가? -> 탄창을 안 빼고 총알을 넣는게 현실적으로 가능할리가 없다

		prt(L"executeReloadOther이 실행되었다.\n");
		int targetLootCursor = reloadItemCursor;
		std::vector<std::wstring> pocketList;
		ItemPocket* equipPtr = PlayerPtr->getEquipPtr();
		std::vector<ItemPocket*> targetSearchPtr;

		//1. 탐사할 타일 추가 (장비, 주변타일 9칸)
		{
			targetSearchPtr.push_back(equipPtr);
			//바닥타일(주변9타일)
			for (int dir = -1; dir < 8; dir++)
			{
				int dx = 0, dy = 0;
				dir2Coord(dir, dx, dy);
				ItemStack* stack = TileItemStack(PlayerX() + dx, PlayerY() + dy, PlayerZ());
				if (stack != nullptr)
				{
					ItemPocket* lootPtr = stack->getPocket();
					targetSearchPtr.push_back(lootPtr);
				}
			}
		}

		//2. 주변 타일과 장비 포인터들을 보관 중인 targetSearchPtr에서 리로드 가능한 아이템의 이름들을 수집
		for (int j = 0; j < targetSearchPtr.size(); j++)
		{
			std::vector<ItemData>& targetItemInfo = targetSearchPtr[j]->itemInfo;
			for (int i = 0; i < targetItemInfo.size(); i++)
			{
				if (targetItemInfo[i].pocketMaxNumber > 0 
					&& targetItemInfo[i].pocketPtr->countPocketItemNumber() < targetItemInfo[i].pocketMaxNumber)
				{
					if (std::find(
						targetItemInfo[i].pocketOnlyItem.begin(), 
						targetItemInfo[i].pocketOnlyItem.end(), 
						reloadItemPocket->itemInfo[targetLootCursor].itemCode) != targetItemInfo[i].pocketOnlyItem.end())
					{
						pocketList.push_back(targetItemInfo[i].name);
					}
				}
			}
		}

		if (pocketList.size() == 0) //넣을만한 포켓을 찾지 못했을 경우
		{
			//이 아이템을 넣을만한 포켓이 없다.
			updateLog(sysStr[96]);
			co_return;
		}

		////////////////////////////////////////////////////////////////////

		new Lst(sysStr[95], sysStr[94], pocketList);//넣기, 넣을 포켓을 선택해주세요.
		co_await std::suspend_always();

		////////////////////////////////////////////////////////////////////
		if (coAnswer.empty() == false)
		{
			int counter = 0;
			for (int j = 0; j < targetSearchPtr.size(); j++)
			{
				for (int i = 0; i < targetSearchPtr[j]->itemInfo.size(); i++)
				{
					std::vector<ItemData>& targetItemInfo = targetSearchPtr[j]->itemInfo;
					if (targetItemInfo[i].pocketMaxNumber > 0 
						&& targetItemInfo[i].pocketPtr->countPocketItemNumber() < targetItemInfo[i].pocketMaxNumber)
					{
						if (std::find(targetItemInfo[i].pocketOnlyItem.begin(), targetItemInfo[i].pocketOnlyItem.end(), reloadItemPocket->itemInfo[targetLootCursor].itemCode) != targetItemInfo[i].pocketOnlyItem.end())
						{
							if (counter == wtoi(coAnswer.c_str()))
							{
								reloadItemPocket->transferItem
								(
									targetItemInfo[i].pocketPtr.get(),
									targetLootCursor,
									1
								);

								co_return;
							}

							counter++;
						}
					}
				}
			}

		}
	}


	export void unload(ItemPocket* unloadItemPocket, int unloadItemCursor)//장전해제 : 타겟아이템에 들어있는 아이템을 드랍하거나 인벤토리에 넣는다.
	{
		int targetLootCursor = unloadItemCursor;
		ItemPocket* targetPocket = unloadItemPocket->itemInfo[targetLootCursor].pocketPtr.get();
		std::unique_ptr<ItemPocket> drop = std::make_unique<ItemPocket>(storageType::null);
		for (int i = 0; i < targetPocket->itemInfo.size(); i++) { targetPocket->transferItem(drop.get(), i, targetPocket->itemInfo[i].number); }
		PlayerPtr->drop(drop.get());
	}

	export void closeDoor(int tgtX, int tgtY, int tgtZ)
	{
		Prop* tgtProp = TileProp(tgtX, tgtY, tgtZ);
		tgtProp->leadItem.eraseFlag(itemFlag::DOOR_OPEN);
		tgtProp->leadItem.addFlag(itemFlag::DOOR_CLOSE);

		if (tgtProp->leadItem.checkFlag(itemFlag::PROP_GAS_OBSTACLE_OFF))
		{
			tgtProp->leadItem.eraseFlag(itemFlag::PROP_GAS_OBSTACLE_OFF);
			tgtProp->leadItem.addFlag(itemFlag::PROP_GAS_OBSTACLE_ON);
		}

		tgtProp->leadItem.eraseFlag(itemFlag::PROP_WALKABLE);
		tgtProp->leadItem.addFlag(itemFlag::PROP_BLOCKER);
		tgtProp->leadItem.extraSprIndexSingle--;
		PlayerPtr->updateVision(PlayerPtr->entityInfo.eyeSight);
	}

	export void closeVDoor(int tgtX, int tgtY, int tgtZ)
	{
		ItemPocket* tgtPocket = TileVehicle(tgtX, tgtY, PlayerZ())->partInfo[{tgtX, tgtY }].get();
		for (int i = 0; i < tgtPocket->itemInfo.size(); i++)
		{
			if (tgtPocket->itemInfo[i].checkFlag(itemFlag::VPART_DOOR_OPEN))
			{
				tgtPocket->itemInfo[i].eraseFlag(itemFlag::VPART_DOOR_OPEN);
				tgtPocket->itemInfo[i].addFlag(itemFlag::VPART_DOOR_CLOSE);

				tgtPocket->itemInfo[i].addFlag(itemFlag::VPART_NOT_WALKABLE);

				if (tgtPocket->itemInfo[i].checkFlag(itemFlag::PROP_GAS_OBSTACLE_OFF))
				{
					tgtPocket->itemInfo[i].eraseFlag(itemFlag::PROP_GAS_OBSTACLE_OFF);
					tgtPocket->itemInfo[i].addFlag(itemFlag::PROP_GAS_OBSTACLE_ON);
				}
				tgtPocket->itemInfo[i].propSprIndex -= 16;
				PlayerPtr->updateVision(PlayerPtr->entityInfo.eyeSight);
			}
		}

    }

	export void toggle(ItemData& inputItem)
	{
		if (inputItem.itemCode == itemRefCode::minerHelmet)
		{
			if (inputItem.checkFlag(itemFlag::TOGGLE_OFF))
			{
				if (inputItem.pocketPtr != nullptr && inputItem.pocketPtr->itemInfo.size() == 1)
				{
					if (inputItem.pocketPtr->itemInfo[0].powerStorage != 0.0)
					{
						inputItem.eraseFlag(itemFlag::TOGGLE_OFF);
						inputItem.addFlag(itemFlag::TOGGLE_ON);

						inputItem.lightPtr = std::make_unique<Light>(PlayerX(), PlayerY(), PlayerZ(), 8, 110, SDL_Color{ 150, 150, 250 });
						inputItem.itemSprIndex += 1;
						PlayerPtr->updateVision();
						updateLog(L"The headlamp comes on.");
					}
					else updateLog(L"The inserted battery is depleted.");
				}
				else updateLog(L"No battery inserted in the headlamp.");
			}
			else if (inputItem.checkFlag(itemFlag::TOGGLE_ON))
			{
				inputItem.eraseFlag(itemFlag::TOGGLE_ON);
				inputItem.addFlag(itemFlag::TOGGLE_OFF);

				inputItem.lightPtr.reset();
				inputItem.itemSprIndex -= 1;

				PlayerPtr->updateVision();
				updateLog(L"The headlamp goes off.");

			}
		}
	}

	export void drinkBottle(ItemData& inputData)
	{
		errorBox(inputData.pocketPtr == nullptr, L"drinkBottle: inputData.pocketPtr is nullptr.");
		errorBox(inputData.pocketPtr->itemInfo.size() == 0, L"drinkBottle: inputData.pocketPtr->itemInfo.size() is 0.");

		int needHydration = PLAYER_MAX_HYDRATION - thirst;

		if (needHydration <= 0)
		{
			updateLog(L"You're not thirsty.");
			return;
		}

		for (int i = 0; i < inputData.pocketPtr->itemInfo.size(); i++)
		{
			if (inputData.pocketPtr->itemInfo[i].itemCode == itemRefCode::water)
			{
				int hydrationPerWater = inputData.pocketPtr->itemInfo[i].hydrationPerML;
				int waterCount = inputData.pocketPtr->itemInfo[i].number;
				int waterNeeded = (needHydration + hydrationPerWater - 1) / hydrationPerWater;
				int waterToConsume = myMin(waterNeeded, waterCount);
				int actualHydration = waterToConsume * hydrationPerWater;
				actualHydration = myMin(actualHydration, needHydration);

				thirst += actualHydration;
				if (thirst > PLAYER_MAX_HYDRATION) thirst = PLAYER_MAX_HYDRATION;

				inputData.pocketPtr->subtractItemIndex(i, waterToConsume);
				updateLog(L"You drink from the bottle. Your thirst is quenched.");
				return;
			}
		}

		updateLog(L"The bottle is empty.");
	}

	export void eatFood(ItemPocket* inputPocket, int inputCursor)
	{
		errorBox(inputPocket == nullptr, L"eatFood: inputPocket is nullptr.");
		errorBox(inputCursor < 0 || inputCursor >= inputPocket->itemInfo.size(), L"eatFood: inputCursor is out of bounds.");

		ItemData& targetItem = inputPocket->itemInfo[inputCursor];

		// 아이템의 칼로리 확인
		int itemCalorie = targetItem.calorie;

		// 현재 허기 상태와 최대 허기 수치 확인
		int needCalorie = PLAYER_MAX_CALORIE - hunger;

		if (needCalorie <= 0)
		{
			updateLog(L"You're too full to eat anymore.");
			return;
		}

		// 칼로리 회복
		hunger += itemCalorie;
		if (hunger > PLAYER_MAX_CALORIE)
		{
			hunger = PLAYER_MAX_CALORIE;
		}

		// 아이템 1개 제거
		inputPocket->subtractItemIndex(inputCursor, 1);

		updateLog(L"You eat the food. Your hunger is satisfied.");
	}

	export void spillPocket(ItemData& inputData)
	{
		ItemPocket* pPtr = inputData.pocketPtr.get();
		errorBox(pPtr == nullptr, L"spillPocket: inputData.pocketPtr is nullptr.");
		errorBox(pPtr->itemInfo.size() == 0, L"spillPocket: inputData.pocketPtr->itemInfo.size() is 0.");

		std::wstring itemName = inputData.name;
		Point3 playerPos = { PlayerX(), PlayerY(), PlayerZ() };

		// 플레이어 위치에 ItemStack이 있는지 확인
		ItemStack* existingStack = TileItemStack(playerPos.x, playerPos.y, playerPos.z);

		if (existingStack == nullptr)
		{
			// ItemStack이 없으면 새로 생성
			createItemStack(playerPos);
			existingStack = TileItemStack(playerPos.x, playerPos.y, playerPos.z);
			errorBox(existingStack == nullptr, L"spillPocket: Failed to create ItemStack.");
		}

		ItemPocket* targetPocket = existingStack->getPocket();

		// 포켓의 모든 아이템을 바닥의 ItemStack으로 이동
		while (pPtr->itemInfo.size() > 0)
		{
			// 첫 번째 아이템의 전체 수량을 이동
			int itemCount = pPtr->itemInfo[0].number;
			pPtr->transferItem(targetPocket, 0, itemCount);
		}

		std::wstring logText = replaceStr(sysStr[297],L"(%container)" , itemName);
		updateLog(logText);
	}

	Corouter executeWield(ItemPocket* targetPocket, int targetPocketCursor)
	{
		ItemData& tgtItem = targetPocket->itemInfo[targetPocketCursor];
		ItemPocket* equipPtr = PlayerPtr->getEquipPtr();
		if (tgtItem.checkFlag(itemFlag::TWOHANDED)) //양손장비일 경우
		{
			std::wstring logStr = replaceStr(sysStr[331], L"(%item)", tgtItem.name);
			updateLog(logStr);
			bool isWield = false;
			std::unique_ptr<ItemPocket> drop = std::make_unique<ItemPocket>(storageType::null);
			for (int i = equipPtr->itemInfo.size() - 1; i >= 0; i--)
			{
				if (equipPtr->itemInfo[i].equipState == equipHandFlag::left || equipPtr->itemInfo[i].equipState == equipHandFlag::right || equipPtr->itemInfo[i].equipState == equipHandFlag::both)
				{
					equipPtr->transferItem(drop.get(), i, 1);
					isWield = true;
				}
			}
			if (isWield == true) { PlayerPtr->drop(drop.get()); }

			int returnIndex = targetPocket->transferItem(equipPtr, targetPocketCursor, 1);
			equipPtr->itemInfo[returnIndex].equipState = equipHandFlag::both; //양손
			equipPtr->sortEquip();
		}
		else
		{
			bool hasLeft = false;
			bool hasRight = false;
			for (int i = 0; i < equipPtr->itemInfo.size(); i++)
			{
				switch (equipPtr->itemInfo[i].equipState)
				{
				case equipHandFlag::left:
					hasLeft = true;
					break;
				case equipHandFlag::right:
					hasRight = true;
					break;
				case equipHandFlag::both:
					hasLeft = true;
					hasRight = true;
					break;
				}
			}

			if (hasLeft == true && hasRight == true)
			{
				//왼손, 오른손
				std::vector<std::wstring> choiceVec = { sysStr[49], sysStr[50] };
				//선택, 어느 손에 들까?
				new Msg(msgFlag::normal, sysStr[98], sysStr[99], choiceVec);
				co_await std::suspend_always();
				if (coAnswer.empty()) co_return;

				equipHandFlag handDir = equipHandFlag::none;
				if (coAnswer == sysStr[49])//왼손
				{
					handDir = equipHandFlag::left;
				}
				else//오른손
				{
					handDir = equipHandFlag::right;
				}

				//왼손 아이템 떨구기
				std::unique_ptr<ItemPocket> drop = std::make_unique<ItemPocket>(storageType::null);
				for (int i = equipPtr->itemInfo.size() - 1; i >= 0; i--)
				{
					if (equipPtr->itemInfo[i].equipState == handDir)
					{
						equipPtr->transferItem(drop.get(), i, 1);
						break;
					}
				}
				//양손 아이템 떨구기
				for (int i = equipPtr->itemInfo.size() - 1; i >= 0; i--)
				{
					if (equipPtr->itemInfo[i].equipState == equipHandFlag::both)
					{
						equipPtr->transferItem(drop.get(), i, 1);
						break;
					}
				}
				PlayerPtr->drop(drop.get());

				int returnIndex = targetPocket->transferItem(equipPtr, targetPocketCursor, 1);

				std::wstring logStr = replaceStr(sysStr[331], L"(%item)", equipPtr->itemInfo[returnIndex].name);
				updateLog(logStr);

				equipPtr->itemInfo[returnIndex].equipState = handDir;
				equipPtr->sortEquip();
			}
			else if (hasLeft == false && hasRight == false)
			{
				//왼손, 오른손
				std::vector<std::wstring> choiceVec = { sysStr[49], sysStr[50] };
				//선택, 어느 손에 들까?
				new Msg(msgFlag::normal, sysStr[98], sysStr[99], choiceVec);
				co_await std::suspend_always();
				if (coAnswer.empty()) co_return;

				equipHandFlag handDir = equipHandFlag::none;
				if (coAnswer == sysStr[49])//왼손
				{
					handDir = equipHandFlag::left;
				}
				else//오른손
				{
					handDir = equipHandFlag::right;
				}

				int returnIndex = targetPocket->transferItem(equipPtr, targetPocketCursor, 1);

				std::wstring logStr = replaceStr(sysStr[331], L"(%item)", equipPtr->itemInfo[returnIndex].name);
				updateLog(logStr);

				equipPtr->itemInfo[returnIndex].equipState = handDir;
				equipPtr->sortEquip();
			}
			else if (hasLeft == false && hasRight == true)//왼손에 들기
			{
				int returnIndex = targetPocket->transferItem(equipPtr, targetPocketCursor, 1);

				std::wstring logStr = replaceStr(sysStr[331], L"(%item)", equipPtr->itemInfo[returnIndex].name);
				updateLog(logStr);

				equipPtr->itemInfo[returnIndex].equipState = equipHandFlag::left;
				equipPtr->sortEquip();


			}
			else//오른손에 들기
			{
				int returnIndex = targetPocket->transferItem(equipPtr, targetPocketCursor, 1);

				std::wstring logStr = replaceStr(sysStr[331], L"(%item)", equipPtr->itemInfo[returnIndex].name);
				updateLog(logStr);

				equipPtr->itemInfo[returnIndex].equipState = equipHandFlag::right;
				equipPtr->sortEquip();


			}
		}
		for (int i = 0; i < equipPtr->itemInfo.size(); i++)
		{
			if (equipPtr->itemInfo[i].lightPtr != nullptr)
			{
				equipPtr->itemInfo[i].lightPtr.get()->setGrid(PlayerX(), PlayerY(), PlayerZ());
			}
		}
		PlayerPtr->pullEquipLights();
		PlayerPtr->updateStatus();
	}

	Corouter executeThrowing(ItemPocket* inputPocket, int inputIndex)//던지기
	{
		new CoordSelect(sysStr[131]);
		rangeRay = true;

		co_await std::suspend_always();

		if (coAnswer.empty())
		{
			updateLog(sysStr[330]);
			rangeRay = false;
			co_return;
		}

		std::wstring targetStr = coAnswer;
		int targetX = wtoi(targetStr.substr(0, targetStr.find(L",")).c_str());
		targetStr.erase(0, targetStr.find(L",") + 1);
		int targetY = wtoi(targetStr.substr(0, targetStr.find(L",")).c_str());
		targetStr.erase(0, targetStr.find(L",") + 1);
		int targetZ = wtoi(targetStr.c_str());

		if (targetX == PlayerX() && targetY == PlayerY() && targetZ == PlayerZ());
		else PlayerPtr->setDirection(getIntDegree(PlayerX(), PlayerY(), targetX, targetY));

		prt(L"executeThrowing에서 사용한 좌표의 값은 (%d,%d,%d)이다.\n", targetX, targetY, targetZ);

		std::unique_ptr<ItemPocket> throwing = std::make_unique<ItemPocket>(storageType::null);
		std::wstring logStr = replaceStr(L"You throw the (%item).", L"(%item)", inputPocket->itemInfo[inputIndex].name);
		updateLog(logStr);
		inputPocket->transferItem(throwing.get(), inputIndex, 1);
		PlayerPtr->throwing(std::move(throwing), targetX, targetY);
		PlayerPtr->updateStatus();

		rangeRay = false;
	}

	void executeEquip(ItemPocket* sourcePocket, int sourceIndex)
	{
		updateLog(replaceStr(sysStr[125], L"(%item)", sourcePocket->itemInfo[sourceIndex].name)); // (%item)를(을) 장착했다.

		ItemPocket* equipPtr = PlayerPtr->getEquipPtr();
		int returnIndex = sourcePocket->transferItem(equipPtr, sourceIndex, 1);
		equipPtr->itemInfo[returnIndex].equipState = equipHandFlag::normal;

		PlayerPtr->pullEquipLights();
		PlayerPtr->updateStatus();
	}


	//배터리 장착 : 전자기기에 사용, 자신에게 배터리를 추가함
	export Corouter insertBattery(actEnv envType, ItemPocket* targetItemPocket, int targetItemCursor)
	{
		prt(L"insertBattery가 실행되었다.\n");

		std::vector<std::wstring> batteryList;
		ItemPocket* equipPtr = PlayerPtr->getEquipPtr();
		std::vector<std::pair<ItemPocket*, ItemStack*>> targetSearchPtr; //ItemPocket과 소유 ItemStack(없으면 nullptr)

		//탐사할 타일 추가 (장비, 주변타일 9칸)
		{
			//장비타일 (ItemStack 없음)
			targetSearchPtr.push_back({ equipPtr, nullptr });
			//바닥타일(주변9타일)
			for (int dir = -1; dir < 8; dir++)
			{
				int dx = 0, dy = 0;
				dir2Coord(dir, dx, dy);

				ItemStack* stack = TileItemStack(PlayerX() + dx, PlayerY() + dy, PlayerZ());
				if (stack != nullptr)
				{
					ItemPocket* lootPtr = stack->getPocket();
					targetSearchPtr.push_back({ lootPtr, stack });
				}
			}
		}

		//주변에서 battery 또는 batteryPack 찾기
		for (int j = 0; j < targetSearchPtr.size(); j++)
		{
			for (int i = 0; i < targetSearchPtr[j].first->itemInfo.size(); i++)
			{
				int itemCode = targetSearchPtr[j].first->itemInfo[i].itemCode;
				if (itemCode == itemRefCode::battery || itemCode == itemRefCode::batteryPack)
				{
					batteryList.push_back(targetSearchPtr[j].first->itemInfo[i].name);
				}
			}
		}

		if (batteryList.size() == 0)
		{
			updateLog(sysStr[344]);//주변에 배터리가 없다.
			co_return;
		}

		////////////////////////////////////////////////////////////////////

		new Lst(sysStr[342], sysStr[345], batteryList);//배터리 장착, 장착할 배터리를 선택해주세요.
		co_await std::suspend_always();

		////////////////////////////////////////////////////////////////////
		if (coAnswer.empty() == false)
		{
			int counter = 0;
			for (int j = 0; j < targetSearchPtr.size(); j++)
			{
				for (int i = 0; i < targetSearchPtr[j].first->itemInfo.size(); i++)
				{
					int itemCode = targetSearchPtr[j].first->itemInfo[i].itemCode;
					if (itemCode == itemRefCode::battery || itemCode == itemRefCode::batteryPack)
					{
						if (counter == wtoi(coAnswer.c_str()))
						{
							//배터리를 전자기기에 장착
							targetSearchPtr[j].first->transferItem
							(
								targetItemPocket->itemInfo[targetItemCursor].pocketPtr.get(),
								i,
								1
							);
							updateLog(sysStr[346]);//배터리를 장착했다.
							co_return;
						}
						counter++;
					}
				}
			}
		}
	}

	//배터리 분리 : 전자기기 내부에 들어있는 배터리를 분리한다
	export void removeBattery(ItemPocket* unloadItemPocket, int unloadItemCursor)
	{
		int targetLootCursor = unloadItemCursor;
		ItemPocket* targetPocket = unloadItemPocket->itemInfo[targetLootCursor].pocketPtr.get();
		if (targetPocket->itemInfo.size() == 0)
		{
			updateLog(sysStr[348]);//분리할 배터리가 없다.
			return;
		}
		std::unique_ptr<ItemPocket> drop = std::make_unique<ItemPocket>(storageType::null);
		for (int i = 0; i < targetPocket->itemInfo.size(); i++) { targetPocket->transferItem(drop.get(), i, targetPocket->itemInfo[i].number); }
		PlayerPtr->drop(drop.get());
		updateLog(sysStr[347]);//배터리를 분리했다.
	}
};