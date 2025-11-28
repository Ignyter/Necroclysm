#include <SDL3/SDL.h>

export module Aim;

import std;
import util;
import GUI;
import textureVar;
import wrapVar;
import drawText;
import drawSprite;
import globalVar;
import wrapVar;
import checkCursor;
import drawWindow;
import Player;
import World;
import Entity;
import log;
import ItemData;
import replaceStr;
import turnWait;

export class Aim : public GUI
{
private:
	inline static Aim* ptr = nullptr;
	SDL_Rect aimBase;
	std::vector<Point3> aimTrailRev;
	Point3 aimCoord = { 0,0,0 };

	double aimAcc = 0;
	double fakeAimAcc = aimAcc;
	int aimStack = 0;
	atkType targetAtkType = atkType::shot;

	bool deactClickUp = false;
public:
	Aim() : GUI(false)
	{
		//1개 이상의 메시지 객체 생성 시의 예외 처리
		errorBox(ptr != nullptr, L"More than one message instance was generated.");
		ptr = this;

		//메세지 박스 렌더링
		changeXY(cameraW / 2, cameraH / 2, true);

		


		ItemPocket* pEquip = PlayerPtr->getEquipPtr();
		if (pEquip->itemInfo[0].checkFlag(itemFlag::CROSSBOW)) PlayerPtr->setSpriteIndex(charSprIndex::AIM_RIFLE);
		else if (pEquip->itemInfo[0].checkFlag(itemFlag::BOW)) PlayerPtr->setSpriteIndex(charSprIndex::AIM_RIFLE);
		else if (pEquip->itemInfo[0].checkFlag(itemFlag::GUN) && pEquip->itemInfo[0].checkFlag(itemFlag::SPR_TH_WEAPON)) PlayerPtr->setSpriteIndex(charSprIndex::AIM_RIFLE);

		

		int pX = PlayerX(), pY = PlayerY(), pZ = PlayerZ();
		
		if(PlayerPtr->entityInfo.sprFlip == false) aimCoord = { pX + 1, pY, pZ };
		else  aimCoord = { pX - 1, pY, pZ };

		Entity* nearTarget = nullptr;
		double hiDist = 99999;
		const int searchRange = 10;
		for (int dx = -searchRange; dx <= searchRange; dx++)
		{
			for (int dy = -searchRange; dy <= searchRange; dy++)
			{
				if (TileFov(pX + dx, pY + dy, pZ) == fovFlag::white)
				{
					if (TileEntity(pX + dx, pY + dy, pZ) != nullptr)
					{
						if (TileEntity(pX + dx, pY + dy, pZ) != PlayerPtr)
						{
							if (std::sqrt(dx * dx + dy * dy) < hiDist)
							{
								nearTarget = TileEntity(pX + dx, pY + dy, pZ);
								hiDist = std::sqrt(dx * dx + dy * dy);
							}
						}
					}
				}
			}
		}

		if (nearTarget != nullptr)
		{
			changeAimTarget(nearTarget->getGridX(), nearTarget->getGridY());
		}
	}
	~Aim()
	{
		PlayerPtr->setSpriteIndex(charSprIndex::WALK);
		ptr = nullptr;
	}
	static Aim* ins() { return ptr; }
	void changeXY(int inputX, int inputY, bool center)
	{
		aimBase = { 0, 0, 650, 376 };

		if (center == false)
		{
			aimBase.x += inputX;
			aimBase.y += inputY;
		}
		else
		{
			aimBase.x += inputX - aimBase.w / 2;
			aimBase.y += inputY - aimBase.h / 2;
		}



		if (center == false)
		{
			x = inputX;
			y = inputY;
		}
		else
		{
			x = inputX - aimBase.w / 2;
			y = inputY - aimBase.h / 2;
		}

	}
	void drawGUI()
	{
		if (getStateDraw() == false) { return; }

		SDL_Rect dst;

		setFontSize(12);


		if (aimAcc != 0 && turnCycle == turn::playerInput)
		{
			std::wstring accStr = decimalCutter(fakeAimAcc * 100.0, 1);

			int iconIndex = 0;
			int currentBullet = 0;
			int maxBullet = 0;
			std::vector<ItemData>& equipInfo = PlayerPtr->getEquipPtr()->itemInfo;
			//손에 든 장비 찾기
			int weaponIdx = -1;
			for (int i = 0; i < equipInfo.size(); ++i)
			{
				if (equipInfo[i].equipState == equipHandFlag::none) continue;
				weaponIdx = i;                                   

				if (equipInfo[i].checkFlag(itemFlag::BOW))      iconIndex = 95;
				else if (equipInfo[i].checkFlag(itemFlag::CROSSBOW)) iconIndex = 96;
				else if (equipInfo[i].checkFlag(itemFlag::GUN))      iconIndex = 97;
				break;                                              
			}


			if (weaponIdx != -1)
			{
				//무기 내부의 탄 계산
				ItemData& weapon = equipInfo[weaponIdx];
				currentBullet += getBulletNumber(weapon);
				for (unsigned short code : weapon.pocketOnlyItem)
				{
					if (itemDex[code].checkFlag(itemFlag::AMMO)) maxBullet += weapon.pocketMaxNumber;
					else if (itemDex[code].checkFlag(itemFlag::MAGAZINE)) 
					{
						if (weapon.pocketPtr && !weapon.pocketPtr->itemInfo.empty()) maxBullet += weapon.pocketPtr->itemInfo[0].pocketMaxNumber;
						else maxBullet += itemDex[code].pocketMaxNumber;
					}
				}

				//무기를 제외한 장비들의 탄 계산
				for (int j = 0; j < equipInfo.size(); ++j)
				{
					if (j == weaponIdx)                    continue;
					ItemData& itm = equipInfo[j];

					if (!itm.checkFlag(itemFlag::MAGAZINE)) continue;
					if (itm.pocketOnlyItem.empty())        continue;

					bool sameAmmo = std::any_of(
						weapon.pocketOnlyItem.begin(), weapon.pocketOnlyItem.end(),
						[&](unsigned short c) { return c == itm.pocketOnlyItem[0]; });

					if (!sameAmmo) continue;

					currentBullet += getBulletNumber(itm);
					maxBullet += itm.pocketMaxNumber;
				}
			}

			std::wstring bulletStr = std::to_wstring(currentBullet)+L"/" + std::to_wstring(maxBullet);
			accStr += L"%";
			//if(aimStack>0) accStr += L" (" + std::to_wstring(aimStack) + L")";

			if (zoomScale == 1.0)
			{
				setFontSize(10);
				int textX = cameraW / 2;
				int textY = cameraH / 2 - 22;
				drawTextOutlineCenter(accStr, textX, textY, col::white);

				setFontSize(8);
				int textX2 = cameraW / 2;
				int textY2 = cameraH / 2 - 32;
				drawTextOutlineCenter(bulletStr, textX2, textY2, col::white);

				drawSpriteCenter(spr::icon16, iconIndex, textX2 - queryTextWidth(bulletStr) / 2.0 - 5, textY2 - 1);
			}
			else if (zoomScale == 2.0)
			{
				setFontSize(10);
				int textX = cameraW / 2;
				int textY = cameraH / 2 - 40;
				drawTextOutlineCenter(accStr, textX, textY, col::white);

				setFontSize(8);
				int textX2 = cameraW / 2;
				int textY2 = cameraH / 2 - 50;
				drawTextOutlineCenter(bulletStr, textX2, textY2, col::white);

				drawSpriteCenter(spr::icon16, iconIndex, textX2 - queryTextWidth(bulletStr) / 2.0 - 5, textY2 - 1);
			}
			else if (zoomScale == 3.0)
			{
				setFontSize(12);
				int textX = cameraW / 2;
				int textY = cameraH / 2 - 58;
				drawTextOutlineCenter(accStr, textX, textY, col::white);

				setFontSize(10);
				int textX2 = textX + 5;
				int textY2 = textY - 15;
				drawTextOutlineCenter(bulletStr, textX2, textY2, col::white);

				setZoom(2.0);
				drawSpriteCenter(spr::icon16, iconIndex, textX2 - queryTextWidth(bulletStr) / 2.0 - 10, textY2 - 1);
				setZoom(1.0);
			}
			else if (zoomScale == 4.0)
			{
				setFontSize(12);
				int textX = cameraW / 2;
				int textY = cameraH / 2 - 58 - 15;
				drawTextOutlineCenter(accStr, textX, textY, col::white);

				setFontSize(10);
				int textX2 = textX + 5;
				int textY2 = textY - 15;
				drawTextOutlineCenter(bulletStr, textX2, textY2, col::white);

				setZoom(2.0);
				drawSpriteCenter(spr::icon16, iconIndex, textX2 - queryTextWidth(bulletStr) / 2.0 - 10, textY2 - 1);
				setZoom(1.0);
			}
			else if (zoomScale == 5.0)
			{
				setFontSize(12);
				int textX = cameraW / 2;
				int textY = cameraH / 2 - 58 - 34;
				drawTextOutlineCenter(accStr, textX, textY, col::white);

				setFontSize(10);
				int textX2 = textX + 5;
				int textY2 = textY - 15;
				drawTextOutlineCenter(bulletStr, textX2, textY2, col::white);

				setZoom(2.0);
				drawSpriteCenter(spr::icon16, iconIndex, textX2 - queryTextWidth(bulletStr) / 2.0 - 10, textY2 - 1);
				setZoom(1.0);
			}
		}


		if(checkCursor(&tab)==false && checkCursor(&letterbox) == false && checkCursor(&quickSlotRegion) == false)
		{
			double tileSize = 16 * zoomScale;
			int tgtX = getAbsMouseGrid().x;
			int tgtY = getAbsMouseGrid().y;
			dst.x = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX) - ((16 * zoomScale) / 2);
			dst.y = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY) - ((16 * zoomScale) / 2);
			dst.w = tileSize;
			dst.h = tileSize;



			setZoom(zoomScale);
			SDL_SetTextureAlphaMod(spr::aimMarkerWhite->getTexture(), 150); //텍스쳐 투명도 설정
			drawSpriteCenter
			(
				spr::aimMarkerWhite,
				0,
				dst.x + dst.w / 2,
				dst.y + dst.h / 2
			);
			SDL_SetTextureAlphaMod(spr::aimMarkerWhite->getTexture(), 255); //텍스쳐 투명도 설정
			setZoom(1.0);
		}

		{
			int tgtX = aimCoord.x;
			int tgtY = aimCoord.y;
			dst.x = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX) - ((16 * zoomScale) / 2);
			dst.y = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY) - ((16 * zoomScale) / 2);
			dst.w = 16.0 * zoomScale;
			dst.h = 16.0 * zoomScale;
			setZoom(zoomScale);


			int sprIndex = 0;
			if (SDL_GetTicks() % 800 < 200) sprIndex = 0;
			else if (SDL_GetTicks() % 800 < 400) sprIndex = 1;
			else if (SDL_GetTicks() % 800 < 600) sprIndex = 2;
			else sprIndex = 1;

			drawSpriteCenter
			(
				spr::aimMarkerTmp,
				sprIndex,
				dst.x + dst.w / 2,
				dst.y + dst.h / 2
			);
			setZoom(1.0);
		}

		for (int i = 0; i < aimTrailRev.size(); i++)
		{
			int tgtX = aimTrailRev[i].x;
			int tgtY = aimTrailRev[i].y;
			dst.x = cameraW / 2 + zoomScale * ((16 * tgtX + 8) - cameraX) - ((16 * zoomScale) / 2);
			dst.y = cameraH / 2 + zoomScale * ((16 * tgtY + 8) - cameraY) - ((16 * zoomScale) / 2);
			dst.w = 16.0 * zoomScale;
			dst.h = 16.0 * zoomScale;
			setZoom(zoomScale);
			drawSpriteCenter
			(
				spr::trail,
				1,
				dst.x + dst.w / 2,
				dst.y + dst.h / 2
			);
			setZoom(1.0);
		}


		//사념파
		{
			int yOffset = 44;
			drawFillRect(SDL_Rect{ cameraW / 2 - 150, 80 + yOffset, 300, 60 }, col::black, 200);

			// 오른쪽 페이드
			{
				int rectX = cameraW / 2 + 150;
				for (int i = 0; i < 40; i++)
				{
					SDL_Rect floatRect = { rectX, 80 + yOffset, 5, 60 };
					drawFillRect(floatRect, col::black, 200 - 5 * i);
					rectX += 5;
				}
			}

			// 왼쪽 페이드
			{
				int rectX = cameraW / 2 - 150;
				for (int i = 0; i < 40; i++)
				{
					rectX -= 5;
					SDL_Rect floatRect = { rectX, 80 + yOffset, 5, 60 };
					drawFillRect(floatRect, col::black, 200 - 5 * i);
				}
			}

			setFontSize(24);
			drawTextCenter(L"사격할 위치를 선택해주세요.", cameraW / 2, 109 + yOffset);
		}
	}

	void changeAimTarget(int tgtX, int tgtY)
	{
		aimCoord.x = tgtX;
		aimCoord.y = tgtY;

		if (TileEntity(aimCoord.x, aimCoord.y, aimCoord.z) != nullptr) aimAcc = randomRangeFloat(0.5, 0.8);
		else aimAcc = 0;
		fakeAimAcc = aimAcc;
		aimStack = 0;

		if (aimCoord.x < PlayerX()) PlayerPtr->setDirection(4);
		else if (aimCoord.x > PlayerX()) PlayerPtr->setDirection(0);

		std::vector<Point2> lineCoord;
		makeLine(lineCoord, aimCoord.x - PlayerX(), aimCoord.y - PlayerY());
		aimTrailRev.clear();
		for (int i = 0; i < lineCoord.size(); i++)
		{
			if (lineCoord[i].x != 0 || lineCoord[i].y != 0)
			{
				aimTrailRev.push_back({ PlayerX() + lineCoord[i].x,PlayerY() + lineCoord[i].y,PlayerZ() });
			}
		}
	}
	void aimAddAcc()
	{
		if (aimAcc != 0)
		{
			aimAcc += randomRangeFloat(0.03, 0.07);
			if (aimAcc > 0.999) aimAcc = 0.999;
			PlayerPtr->flash = { 255, 255, 255, 255 };
			aimStack++;
		}
	}

	void clickUpGUI()
	{
		if (getStateInput() == false) { return; }

		if (checkCursor(&tabSmallBox))
		{
			aimAddAcc();
		}
		else if (checkCursor(&tab))
		{
			if ((getMouseX() - tab.x) + (getMouseY() - tab.y) < 178)
			{
				executeTabShot();
			}
			else // 마우스 커서가 직선 위에 있거나(경계 포함) 아래에 있는 경우
			{
				close(aniFlag::null);
			}

			
		}
	}
	void clickMotionGUI(int dx, int dy) {}
	void clickDownGUI() 
	{
		if (checkCursor(&letterbox) || checkCursor(&quickSlotRegion))
		{
			close(aniFlag::null);
		}
		else if (checkCursor(&tab) == false)
		{
			changeAimTarget(getAbsMouseGrid().x, getAbsMouseGrid().y);
		}
		
	}
	void clickRightGUI() {}
	void clickHoldGUI() {}
	void mouseWheel() {}
	void gamepadBtnDown() 
	{
		switch (event.gbutton.button)
		{
		case SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER:
			aimAddAcc();
			break;
		case SDL_GAMEPAD_BUTTON_EAST:
			close(aniFlag::null);
			break;

		}
	}
	void gamepadBtnMotion() {}
	void gamepadBtnUp() {}
	void step() 
	{
		tabType = tabFlag::aim;

		{
			if (PlayerPtr->flash.a > 0)
			{
				if (PlayerPtr->flash.a > 17) { PlayerPtr->flash.a -= 17; }
				else PlayerPtr->flash.a = 0;
			}
		}

		if (fabs(aimAcc - fakeAimAcc) > 0.002)
		{
			if (aimAcc > fakeAimAcc)
			{
				fakeAimAcc += 0.004;
			}
			else if (aimAcc < fakeAimAcc)
			{
				fakeAimAcc -= 0.004;
			}
		}



		if (option::inputMethod == input::gamepad)
		{
			if (delayR2 <= 0 && SDL_GetGamepadAxis(controller, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER) > 1000)
			{
				prt(L"탭이 실행되었다.\n");
				executeTabShot();
				delayR2 = 20;
			}
			else delayR2--;


			if (dpadDelay <= 0)
			{
				dpadDelay = 6;
				int dir = -1;
				bool dpadUpPressed = SDL_GetGamepadButton(controller, SDL_GAMEPAD_BUTTON_DPAD_UP);
				bool dpadDownPressed = SDL_GetGamepadButton(controller, SDL_GAMEPAD_BUTTON_DPAD_DOWN);
				bool dpadLeftPressed = SDL_GetGamepadButton(controller, SDL_GAMEPAD_BUTTON_DPAD_LEFT);
				bool dpadRightPressed = SDL_GetGamepadButton(controller, SDL_GAMEPAD_BUTTON_DPAD_RIGHT);

				if (dpadUpPressed && dpadLeftPressed) dir = 3;
				else if (dpadUpPressed && dpadRightPressed) dir = 1;
				else if (dpadDownPressed && dpadLeftPressed) dir = 5;
				else if (dpadDownPressed && dpadRightPressed) dir = 7;
				else if (dpadUpPressed) dir = 2;
				else if (dpadDownPressed) dir = 6;
				else if (dpadLeftPressed) dir = 4;
				else if (dpadRightPressed) dir = 0;

				if (dir != -1)
				{
					int dx, dy;
					dir2Coord(dir, dx, dy);
					changeAimTarget(aimCoord.x + dx, aimCoord.y + dy);
				}
			}
			else dpadDelay--;
		}
	}


	void executeTabShot()
	{
		Entity* victimEntity = TileEntity(aimCoord.x, aimCoord.y, aimCoord.z);
		int targetX = aimCoord.x;
		int targetY = aimCoord.y;
		int targetZ = aimCoord.z;
		int weaponRange = 10;

		PlayerPtr->aimWeaponLeft();

		if (victimEntity == nullptr)
		{
			return;
		}
		//맨손 사거리 이탈
		else if (PlayerPtr->getAimWeaponIndex() == -1)
		{
			if (1 < myMax(abs(PlayerX() - targetX), abs(PlayerY() - targetY)))
			{
				return;
			}

			PlayerPtr->startAtk(targetX, targetY, targetZ);
			turnWait(1.0);
			PlayerPtr->initAimStack();

		}
		//무기 사거리 이탈
		else
		{
			if (targetAtkType == atkType::throwing) //던지기
			{
				if (weaponRange < myMax(abs(PlayerX() - targetX), abs(PlayerY() - targetY)) ) { return; }

				//자기 자신에게 던지는 경우도 고려해야 되나?
				std::unique_ptr<ItemPocket> drop = std::make_unique<ItemPocket>(storageType::null);
				updateLog(replaceStr(L"You throw the (%item).", L"(%item)", PlayerPtr->getEquipPtr()->itemInfo[PlayerPtr->getAimWeaponIndex()].name));
				PlayerPtr->getEquipPtr()->transferItem(drop.get(), PlayerPtr->getAimWeaponIndex(), 1);
				PlayerPtr->throwing(std::move(drop), targetX, targetY);
				PlayerPtr->updateStatus();


				PlayerPtr->startAtk(targetX, targetY, targetZ, aniFlag::throwing);
				turnWait(1.0);
				PlayerPtr->initAimStack();
				PlayerPtr->setNextAtkType(targetAtkType);
			}
			else if (targetAtkType == atkType::shot) //사격
			{
				ItemData& tmpAimWeapon = PlayerPtr->getEquipPtr()->itemInfo[PlayerPtr->getAimWeaponIndex()];
				if (getBulletNumber(tmpAimWeapon) > 0) //사격인데 총알이 없을 경우
				{
					if (weaponRange < myMax(abs(PlayerX() - targetX), abs(PlayerY() - targetY))) { return; }
				}
				else return;

				//직탄식 총
				if (itemDex[tmpAimWeapon.pocketOnlyItem[0]].checkFlag(itemFlag::AMMO))
				{
					tmpAimWeapon.pocketPtr->popTopBullet();
				}
				//탄창식 총
				else if (itemDex[tmpAimWeapon.pocketOnlyItem[0]].checkFlag(itemFlag::MAGAZINE))
				{
					ItemData& magazineData = tmpAimWeapon.pocketPtr.get()->itemInfo[0];
					magazineData.pocketPtr->popTopBullet();
				}

				auto pEquip = PlayerPtr->getEquipPtr();
				if (pEquip->itemInfo[0].checkFlag(itemFlag::BOW)) PlayerPtr->setSpriteIndex(charSprIndex::WALK);

				PlayerPtr->startAtk(targetX, targetY, targetZ, aniFlag::shotSingle);
				turnWait(1.0);
				PlayerPtr->initAimStack();
				aimAcc = 0.6;
				PlayerPtr->setNextAtkType(targetAtkType);
			}
			else//근접공격
			{
				if (weaponRange < myMax(abs(PlayerX() - targetX), abs(PlayerY() - targetY))) { return; }

				PlayerPtr->startAtk(targetX, targetY, targetZ);
				turnWait(1.0);
				PlayerPtr->initAimStack();
				PlayerPtr->setNextAtkType(targetAtkType);
			}
		}
	}
};



