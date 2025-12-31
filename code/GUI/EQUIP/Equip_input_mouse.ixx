#include <SDL3/SDL.h>
#define CORO(func) delete coFunc; coFunc = new Corouter(func); (*coFunc).run();

import util;
import Equip;
import checkCursor;
import globalVar;
import wrapVar;
import actFuncSet;
import ItemData;
import log;

void Equip::clickUpGUI()
{
	if (checkCursor(&tab) == true)// 탭박스
	{
		executeTab();
		return;
	}
	else if (checkCursor(&equipArea)) //아이템 클릭 -> 에러 파트
	{
		//아이템 메인 클릭
		//만약 아이템을 클릭했으면 커서를 그 아이템으로 옮김, 다른 곳 누르면 -1로 바꿈
		for (int i = 0; i < EQUIP_ITEM_MAX; i++)
		{
			if (equipPtr->itemInfo.size() - 1 >= i)
			{
				if (checkCursor(&equipItemRect[i]))
				{
					if (equipCursor != equipScroll + i) //새로운 커서 생성
					{
						equipCursor = equipScroll + i;
						updateBarAct();
					}
					else //커서 삭제
					{
						equipCursor = -1;
						barAct = actSet::null;
					}
					return;
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
				case act::droping:
				{
					executeDroping();
					break;
				}
				case act::throwing:
				{
					CORO(actFunc::executeThrowing(equipPtr, equipCursor));
					close(aniFlag::null);
					return;

				}
				case act::open:
				{
					executeOpen();
					break;
				}
				case act::reload:
				{
					CORO(executeReload());
					break;
				}
				case act::reloadBulletToMagazine:
				case act::reloadBulletToGun:
					if (equipPtr->itemInfo[equipCursor].checkFlag(itemFlag::MAGAZINE))
					{
						CORO(actFunc::reloadSelf(actEnv::Equip, equipPtr, equipCursor));
					}
					else if (equipPtr->itemInfo[equipCursor].checkFlag(itemFlag::AMMO))
					{
						CORO(actFunc::reloadOther(actEnv::Equip, equipPtr, equipCursor));
					}
					else if (equipPtr->itemInfo[equipCursor].checkFlag(itemFlag::GUN))
					{
						CORO(actFunc::reloadSelf(actEnv::Equip, equipPtr, equipCursor));
					}
					break;
				case act::reloadMagazine:
					//총에서 사용하는 경우와 탄창에서 사용하는 경우가 다름
					//총에서 사용하면 자기 자신에게 장전함(self)
					//탄창에 사용하면 다른 타일의 총에게 장비함
					if (equipPtr->itemInfo[equipCursor].checkFlag(itemFlag::MAGAZINE))
					{
						CORO(actFunc::reloadOther(actEnv::Equip, equipPtr, equipCursor));
					}
					else
					{
						CORO(actFunc::reloadSelf(actEnv::Equip, equipPtr, equipCursor));
					}
					break;
				case act::unloadMagazine:
				case act::unloadBulletFromMagazine:
				case act::unloadBulletFromGun:
				{
					actFunc::unload(equipPtr, equipCursor);
				}
				break;
				case act::toggleOff:
				case act::toggleOn:
					actFunc::toggle(equipPtr->itemInfo[equipCursor]);
					updateBarAct();
					return;
				case act::drink:
					actFunc::drinkBottle(equipPtr->itemInfo[equipCursor]);
					updateBarAct();
					close(aniFlag::null);
					return;
				case act::eat:
					actFunc::eatFood(equipPtr,equipCursor);
					updateBarAct();
					return;
				case act::dump:
					actFunc::spillPocket(equipPtr->itemInfo[equipCursor]);
					updateBarAct();
					return;
				case act::propInstall:
				{
					CORO(executePropInstall());
					break;
				}
				case act::insertBattery:
					CORO(actFunc::insertBattery(actEnv::Equip, equipPtr, equipCursor));
					break;
				case act::removeBattery:
					actFunc::removeBattery(equipPtr, equipCursor);
					updateBarAct();
					return;
				}



				if (equipPtr->itemInfo.size() == 0)
				{
					close(aniFlag::null);
					return;
				}

				if (equipPtr->itemInfo.size() - 1 <= equipScroll + equipScrollSize)
				{
					equipScroll = equipPtr->itemInfo.size() - equipScrollSize;
					if (equipScroll < 0) { equipScroll = 0; }
				}
				break;
			}
		}
	}

	//위의 모든 경우에서 return을 받지 못했으면 버튼 이외를 누른 것이므로 커서를 -1로 복구
	{
		equipCursor = -1;
		barAct = actSet::null;
	}
}
void Equip::clickMotionGUI(int dx, int dy)
{
}
void Equip::clickDownGUI()
{
}