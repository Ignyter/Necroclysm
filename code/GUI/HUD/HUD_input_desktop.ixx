#define CORO(func) delete coFunc; coFunc = new Corouter(func); (*coFunc).run();

import HUD;

#include <SDL3/SDL.h>

import std;
import util;
import checkCursor;
import globalVar;
import wrapVar;
import Player;
import World;
import Vehicle;
import log;
import Prop;
import ContextMenu;
import Entity;
import Aim;
import useSkill;
import ItemData;
import ItemPocket;
import SystemMenu;

static bool btnPressed = false;

void HUD::clickDownGUI()
{
	executedHold = false;

	for (int i = 0; i < QUICK_SLOT_MAX; i++)
	{
		if (checkCursor(&quickSlotBtn[i]))
		{
			if (quickSlot[i].first != quickSlotFlag::NONE)
			{
				dragQuickSlotTarget = i;
			}
		}
	}
}
void HUD::clickMotionGUI(int dx, int dy)
{
	if (click == true && (event.button.button == SDL_BUTTON_MIDDLE||option::inputMethod == input::touch) )
	{
		const int maxDist = 160;
		int prevCameraX = cameraX, prevCameraY = cameraY;
		cameraFix = false;
		cameraX -= ((getMouseX() - prevMouseX4Motion) / 2);
		cameraY -= ((getMouseY() - prevMouseY4Motion) / 2);
		disableClickUp4Motion = true;

		if (std::abs(PlayerPtr->getX() - cameraX) > maxDist) cameraX = prevCameraX;
		if (std::abs(PlayerPtr->getY() - cameraY) > maxDist) cameraY = prevCameraY;
	}
}
void HUD::clickUpGUI()
{
	if (disableClickUp4Motion && (event.button.button == SDL_BUTTON_MIDDLE || option::inputMethod == input::touch))
	{
		disableClickUp4Motion = false;
		return;
	}

	if (executedHold) return;
	

	if (checkCursor(&letterboxPopUpButton) == true)//팝업 기능
	{
		if (y == 0) { executePopUp(); }
		else { executePopDown(); }
	}
	else if (checkCursor(&tabSmallBox) == true)
	{
		findAndOpenAim();
	}
	else if (checkCursor(&tab) == true) executeTab();
	else if (checkCursor(&quickSlotPopBtn)) quickSlotToggle();
	else if (checkCursor(&letterbox))
	{
		for (int i = 0; i < barAct.size(); i++) // 하단 UI 터치 이벤트
		{
			if (checkCursor(&barButton[i]))
			{
				clickLetterboxBtn(barAct[i]);
			}
		}
	}
	else if (checkCursor(&quickSlotRegion))//퀵슬롯 이벤트 터치
	{
		for (int i = 0; i < 8; i++)
		{
			if (checkCursor(&quickSlotBtn[i]))
			{
				prt(L"%d번 스킬 슬롯을 눌렀다!\n", i + 1);
				if (quickSlot[i].first != quickSlotFlag::NONE)
				{
					if (dragQuickSlotTarget == i)
					{
						CORO(useSkill(quickSlot[i].second));
					}
				}
			}
		}
	}
	else if (checkCursor(&minimapRegion));
	else//타일터치
	{
		if (dragQuickSlotTarget == -1)
		{
			prt(L"[HUD] 절대좌표 (%d,%d) 타일을 터치했다.\n", clickTile.x, clickTile.y);
			clickTile = { getAbsMouseGrid().x,getAbsMouseGrid().y };
			tileTouch(clickTile.x, clickTile.y);
		}
	}

	if (dragQuickSlotTarget != -1)
	{
		if (checkCursor(&quickSlotRegion) == false)
		{
			quickSlot[dragQuickSlotTarget].first = quickSlotFlag::NONE;
			quickSlot[dragQuickSlotTarget].second = -1;
		}
		else
		{
			for (int i = 0; i < QUICK_SLOT_MAX; i++)
			{
				if (checkCursor(&quickSlotBtn[i]))
				{
					// 드래그한 슬롯의 정보 백업
					quickSlotFlag dragFlag = quickSlot[dragQuickSlotTarget].first;
					int dragIndex = quickSlot[dragQuickSlotTarget].second;

					// 드롭 대상 슬롯의 정보 백업
					quickSlotFlag targetFlag = quickSlot[i].first;
					int targetIndex = quickSlot[i].second;

					// 드롭 대상 슬롯에 스킬이 있으면 교환, 없으면 단순 이동
					if (targetFlag != quickSlotFlag::NONE)
					{
						// 서로 교환
						quickSlot[i].first = dragFlag;
						quickSlot[i].second = dragIndex;

						quickSlot[dragQuickSlotTarget].first = targetFlag;
						quickSlot[dragQuickSlotTarget].second = targetIndex;
					}
					else
					{
						// 빈 슬롯으로 이동 (기존 로직)
						for (int j = 0; j < QUICK_SLOT_MAX; j++)
						{
							if (quickSlot[j].first == dragFlag && quickSlot[j].second == dragIndex)
							{
								quickSlot[j].first = quickSlotFlag::NONE;
								quickSlot[j].second = -1;
							}
						}

						quickSlot[i].first = dragFlag;
						quickSlot[i].second = dragIndex;
					}
					break;
				}
			}
		}
	}

	dragQuickSlotTarget = -1;
}

void HUD::mouseStep()
{
	//홀드 이벤트
	if (dtClickStack >= 1000) //1초간 누르고 있을 경우
	{
		if (checkCursor(&letterbox) == false && checkCursor(&tab) == false && checkCursor(&letterboxPopUpButton) == false && checkCursor(&tabSmallBox) == false)
		{
			//터치한 좌표를 얻어내는 부분
			// prt(L"1초 이상 눌렀다.\n");
			int revX, revY, revGridX, revGridY;
			revX = clickDownPoint.x - (cameraW / 2);
			//revY = clickDownPoint.y - (cameraH / 2);
			//revX += sgn(revX) * (8 * zoomScale);
			//revGridX = revX / (16 * zoomScale);
			//revY += sgn(revY) * (8 * zoomScale);
			//revGridY = revY / (16 * zoomScale);
			//dtClickStack = -1;
			//executedHold = true;
			////advancedTileTouch(PlayerX() + revGridX, PlayerY() + revGridY);
		}
	}
}

void HUD::clickRightGUI()
{
	updateLog(L"[HUD] Right click event triggered.");

	if (checkCursor(&quickSlotRegion) == true)
	{
		for (int i = 0; i < QUICK_SLOT_MAX; i++)
		{
			if (checkCursor(&quickSlotBtn[i]))
			{
				quickSlot[i].first = quickSlotFlag::NONE;
				quickSlot[i].second = -1;
			}
		}
	}
	else
	{
		if(option::inputMethod==input::mouse) openContextMenu(getAbsMouseGrid());
	}

	
}
void HUD::clickHoldGUI()
{
	if (option::inputMethod == input::touch)
	{
		updateLog(L"[HUD] Touch hold event triggered.");
		openContextMenu(touchStartGrid);//터치이벤트가 끝나고 스텝이벤트라서 absMouse 함수 사용 금지(쓰레기값 출력됨)
	}
}


void HUD::keyDownGUI()
{
	switch (event.key.key)
	{
	case SDLK_TAB:
		executeTab();
		break;

	case SDLK_RETURN:
		if (!btnPressed)
		{
			if (y == 0) { executePopUp(); }
			else { executePopDown(); }
		}
		break;
	}

	btnPressed = true;
}

void HUD::keyUpGUI()
{
	btnPressed = false;
	switch (event.key.key)
	{
	case SDLK_1:
		if (quickSlot[0].first == quickSlotFlag::SKILL) CORO(useSkill(quickSlot[0].second));
		break;
	case SDLK_2:
		if (quickSlot[1].first == quickSlotFlag::SKILL) CORO(useSkill(quickSlot[1].second));
		break;
	case SDLK_3:
		if (quickSlot[2].first == quickSlotFlag::SKILL) CORO(useSkill(quickSlot[2].second));
		break;
	case SDLK_4:
		if (quickSlot[3].first == quickSlotFlag::SKILL) CORO(useSkill(quickSlot[3].second));
		break;
	case SDLK_5:
		if (quickSlot[4].first == quickSlotFlag::SKILL) CORO(useSkill(quickSlot[4].second));
		break;
	case SDLK_6:
		if (quickSlot[5].first == quickSlotFlag::SKILL) CORO(useSkill(quickSlot[5].second));
		break;
	case SDLK_7:
		if (quickSlot[6].first == quickSlotFlag::SKILL) CORO(useSkill(quickSlot[6].second));
		break;
	case SDLK_8:
		if (quickSlot[7].first == quickSlotFlag::SKILL) CORO(useSkill(quickSlot[7].second));
		break;
	case SDLK_ESCAPE:
		new SystemMenu();
		break;

	}
}