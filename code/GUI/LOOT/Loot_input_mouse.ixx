#include <SDL3/SDL.h>
#define CORO(func) delete coFunc; coFunc = new Corouter(func); (*coFunc).run();

import Loot;
import std;
import util;
import Player;
import checkCursor;
import globalVar;
import wrapVar;
import actFuncSet;

void Loot::clickUpGUI()
{
	if (checkCursor(&tab) == true)// 탭박스
	{
		executeTab();
		return;
	}
	else if (checkCursor(&lootLabel))
	{
		if (checkCursor(&lootLabelSelect))
		{
			executeSelectAll();
		}
		else if (checkCursor(&lootLabelName))
		{
			CORO(executeSearch());
			//lootPocket->sortPocket(sortFlag::null);
			//lootScroll = 0;
		}
		else if (checkCursor(&lootLabelQuantity))
		{
			executeSort();
		}
	}
	else if (checkCursor(&lootBase)) //아이템 클릭 -> 에러 파트
	{
		if (checkCursor(&pocketLeft))
		{
			if (pocketCursor != 0) { pocketCursor--; }
		}
		else if (checkCursor(&pocketRight))
		{
			int numberOfBag = 0;
			ItemPocket* equipPtr = PlayerPtr->getEquipPtr();
			for (int i = 0; i < equipPtr->itemInfo.size(); i++)
			{
				if (equipPtr->itemInfo[i].pocketPtr !=nullptr)
				{
					numberOfBag++;
				}
			}
			if (pocketCursor != numberOfBag - 1) { pocketCursor++; }
		}
		else if (checkCursor(&lootBtn))
		{
			executePickSelect();
		}
		else
		{

			//만약 아이템을 클릭했으면 커서를 그 아이템으로 옮김, 다른 곳 누르면 -1로 바꿈
			for (int i = 0; i < LOOT_ITEM_MAX; i++)
			{
				if (lootPocket->itemInfo.size() - 1 >= i)
				{
					if (checkCursor(&lootItemRect[i]))
					{
						if (lootCursor != lootScroll + i) //새로운 커서 생성
						{
							lootCursor = lootScroll + i;
							updateBarAct();
						}
						else //커서 삭제
						{
							lootCursor = -1;
							barAct = actSet::null;
						}
						return;
					}
				}
			}

			//아이템 좌측 셀렉트 클릭
			for (int i = 0; i < LOOT_ITEM_MAX; i++)
			{
				if (checkCursor(&lootItemSelectRect[i]))
				{
					if (lootPocket->itemInfo.size() - 1 >= i)
					{
						if (lootPocket->itemInfo[i + lootScroll].lootSelect == 0)
						{
							if (option::inputMethod == input::mouse)
							{
								executeSelectItem(i + lootScroll);
							}
							else if (option::inputMethod == input::touch)
							{
								executeSelectItem(i + lootScroll);
							}
						}
						else
						{
							lootPocket->itemInfo[i + lootScroll].lootSelect = 0;
						}
					}
				}
			}
		}
	}
	else if (checkCursor(&letterbox)) //버튼은 return 없음
	{
		for (int i = 0; i < barAct.size(); i++) // 하단 UI 터치 이벤트
		{
			if (checkCursor(&barButton[i]))
			{
				switch (barAct[i])
				{
				case act::pick://넣기
					executePick();
					break;
				case act::equip://장비
					actFunc::executeEquip(lootPocket, lootCursor);
					break;
				case act::wield://들기
					CORO(actFunc::executeWield(lootPocket, lootCursor));
					break;
					//case act::insert:
					//	CORO(executeInsert());
					//	break;
				case act::reloadBulletToMagazine:
				case act::reloadBulletToGun:
					if (lootPocket->itemInfo[lootCursor].checkFlag(itemFlag::MAGAZINE))
					{
						CORO(actFunc::reloadSelf(actEnv::Loot, lootPocket, lootCursor));
					}
					else if (lootPocket->itemInfo[lootCursor].checkFlag(itemFlag::AMMO))
					{
						CORO(actFunc::reloadOther(actEnv::Loot, lootPocket, lootCursor));
					}
					else if (lootPocket->itemInfo[lootCursor].checkFlag(itemFlag::GUN))
					{
						CORO(actFunc::reloadSelf(actEnv::Equip, lootPocket, lootCursor));
					}
					break;
				case act::reloadMagazine:
					//총에서 사용하는 경우와 탄창에서 사용하는 경우가 다름
					//총에서 사용하면 자기 자신에게 장전함(self)
					//탄창에 사용하면 다른 타일의 총에게 장비함
					if (lootPocket->itemInfo[lootCursor].checkFlag(itemFlag::MAGAZINE))
					{
						CORO(actFunc::reloadOther(actEnv::Loot, lootPocket, lootCursor));
					}
					else
					{
						CORO(actFunc::reloadSelf(actEnv::Loot, lootPocket, lootCursor));
					}
					break;
				case act::unloadMagazine:
				case act::unloadBulletFromMagazine:
				case act::unloadBulletFromGun:
					actFunc::unload(lootPocket, lootCursor);
					break;
				case act::toggleOff:
				case act::toggleOn:
					actFunc::toggle(lootPocket->itemInfo[lootCursor]);
					updateBarAct();
					return;
				case act::drink:
					actFunc::drinkBottle(lootPocket->itemInfo[lootCursor]);
					close(aniFlag::null);
					return;
				case act::eat:
					actFunc::eatFood(lootPocket, lootCursor);
					updateBarAct();
					return;
				case act::dump:
					actFunc::spillPocket(lootPocket->itemInfo[lootCursor]);
					updateBarAct();
					return;
				case act::insertBattery:
					CORO(actFunc::insertBattery(actEnv::Loot, lootPocket, lootCursor));
					break;
				case act::removeBattery:
					actFunc::removeBattery(lootPocket, lootCursor);
					updateBarAct();
					return;
				}
			}
		}
	}

	//위의 모든 경우에서 return을 받지 못했으면 버튼 이외를 누른 것이므로 커서를 -1로 복구
	{
		lootCursor = -1;
		barAct = actSet::null;
	}
}
void Loot::clickMotionGUI(int dx, int dy)
{
	if (checkCursor(&lootBase))
	{
		if (click == true)
		{
			int scrollAccelConst = 20; // 가속상수, 작아질수록 스크롤 속도가 빨라짐
			lootScroll = initLootScroll + dy / scrollAccelConst;
			if (abs(dy / scrollAccelConst) >= 1)
			{
				deactClickUp = true;
				itemListColorLock = true;
			}
		}
	}
}
void Loot::clickDownGUI()
{
	//아이템 좌측 셀렉트 클릭
	selectTouchTime = getMilliTimer();
	initLootScroll = lootScroll;
	initPocketCursor = pocketCursor;
}

void Loot::clickRightGUI()
{
	//아이템 좌측 셀렉트 우클릭
	for (int i = 0; i < LOOT_ITEM_MAX; i++)
	{
		if (checkCursor(&lootItemSelectRect[i]))
		{
			if (lootPocket->itemInfo.size() - 1 >= i)
			{
				if (lootPocket->itemInfo[i + lootScroll].lootSelect == 0)
				{
					CORO(executeSelectItemEx(i + lootScroll));
				}
				else
				{
					lootPocket->itemInfo[i + lootScroll].lootSelect = 0;
				}
			}
		}
	}
}