#include <SDL3/SDL.h>


export module CoordSelectCraft;

import std;
import util;
import GUI;
import Player;
import World;
import Loot;
import checkCursor;
import constVar;
import globalVar;
import wrapVar;
import drawSprite;
import textureVar;
import drawText;
import drawWindow;
import log;
import ItemPocket;

//반환형 : wstring L"x좌표,y좌표,회전으로 변경된 itemCode", ex) 3,5,167
export class CoordSelectCraft : public GUI
{
private:
	inline static CoordSelectCraft* ptr = nullptr;
	std::wstring telepathyStr = sysStr[175];
	int index = -1;
	std::wstring parameter = L"";
	std::vector<Point2> selectableCoord;

	bool advance = false; //좌표를 선택하고 확인 버튼을 한번 더 눌러야 진행되는 옵션
	int advanceIconIndex = -1;

	bool targetSelect = false;
	int targetGridX = 0;
	int targetGridY = 0;
	int rotatedItemCode = 0;

	SDL_Rect selectBox = { cameraW/2 - 117, cameraH/2 + 72, 234, 108 };
	SDL_Rect confirmBtn = { selectBox.x + 12, selectBox.y + 34, 64, 64 };
	SDL_Rect rotateBtn = { selectBox.x + 12 + 73 * 1, selectBox.y + 34 , 64, 64 };
	SDL_Rect cancelBtn = { selectBox.x + 12 + 73 * 2, selectBox.y + 34, 64, 64 };

public:

	CoordSelectCraft(int tgtItemCode, std::wstring inputTelepathyStr, std::vector<Point2> inputSelectableCoord) : GUI(true)
	{
		coAnswer.clear();
		selectableCoord = inputSelectableCoord;
		telepathyStr = inputTelepathyStr;
		rotatedItemCode = tgtItemCode;
		prt(L"CoordSelectCraft : 생성자가 호출되었습니다..\n");
		ptr = this;
	}

	~CoordSelectCraft()
	{
		prt(L"CoordSelectCraft : 소멸자가 호출되었습니다..\n");
		ptr = nullptr;
		UIType = act::null;
	}
	static CoordSelectCraft* ins() { return ptr; }
	void changeXY(int inputX, int inputY, bool center) {}
	void drawGUI()
	{
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
			drawTextCenter(telepathyStr, cameraW / 2, 109 + yOffset);
		}

		if (targetSelect == false)
		{


			if (checkCursor(&letterbox) == false && checkCursor(&tab) == false)
			{
				int revX, revY, revGridX, revGridY;
				if (option::inputMethod == input::touch)
				{
					revX = event.tfinger.x * cameraW - (cameraW / 2);
					revY = event.tfinger.y * cameraH - (cameraH / 2);
				}
				else
				{
					revX = getMouseX() - (cameraW / 2);
					revY = getMouseY() - (cameraH / 2);
				}
				revX += sgn(revX) * (8 * zoomScale);
				revGridX = revX / (16 * zoomScale);
				revY += sgn(revY) * (8 * zoomScale);
				revGridY = revY / (16 * zoomScale);



				setZoom(zoomScale);
				//drawSpriteCenter
				//(
				//	spr::whiteMarker,
				//	0,
				//	(cameraW / 2) + (int)(16.0 * zoomScale * revGridX),
				//	(cameraH / 2) + (int)(16.0 * zoomScale * revGridY)
				//);

				if (itemDex[rotatedItemCode].checkFlag(itemFlag::PROP_BIG)) revGridY -= 1;



				if (itemDex[rotatedItemCode].checkFlag(itemFlag::PROP))
				{
					if (rangeSet.find(getAbsMouseGrid()) != rangeSet.end())
					{
						SDL_SetTextureColorMod(spr::propset->getTexture(), 255, 255, 255);
						SDL_SetTextureAlphaMod(spr::propset->getTexture(), 100);
						
						if (revGridX > 0) PlayerPtr->setDirection(0);
						else if (revGridX < 0) PlayerPtr->setDirection(4);
					}
					else
					{
						SDL_SetTextureColorMod(spr::propset->getTexture(), 255, 0, 0);
						SDL_SetTextureAlphaMod(spr::propset->getTexture(), 100);
					}

					drawSpriteCenter
					(
						spr::propset,
						itemDex[rotatedItemCode].propSprIndex,
						(cameraW / 2) + (int)(16.0 * zoomScale * revGridX),
						(cameraH / 2) + (int)(16.0 * zoomScale * revGridY)
					);

					SDL_SetTextureColorMod(spr::propset->getTexture(), 255, 255, 255);
					SDL_SetTextureAlphaMod(spr::propset->getTexture(), 255);
				}
				else if (itemDex[rotatedItemCode].checkFlag(itemFlag::WALL))
				{
					if (rangeSet.find(getAbsMouseGrid()) != rangeSet.end())
					{
						SDL_SetTextureColorMod(spr::tileset->getTexture(), 255, 255, 255);
						SDL_SetTextureAlphaMod(spr::tileset->getTexture(), 100);

						if (revGridX > 0) PlayerPtr->setDirection(0);
						else if (revGridX < 0) PlayerPtr->setDirection(4);
					}
					else
					{
						SDL_SetTextureColorMod(spr::tileset->getTexture(), 255, 0, 0);
						SDL_SetTextureAlphaMod(spr::tileset->getTexture(), 100);
					}

					drawSpriteCenter
					(
						spr::tileset,
						itemDex[rotatedItemCode].tileSprIndex + 1,
						(cameraW / 2) + (int)(16.0 * zoomScale * revGridX),
						(cameraH / 2) + (int)(16.0 * zoomScale * revGridY)
					);

					SDL_SetTextureColorMod(spr::tileset->getTexture(), 255, 255, 255);
					SDL_SetTextureAlphaMod(spr::tileset->getTexture(), 255);
				}
				else if (itemDex[rotatedItemCode].checkFlag(itemFlag::FLOOR))
				{
					if (rangeSet.find(getAbsMouseGrid()) != rangeSet.end())
					{
						SDL_SetTextureColorMod(spr::tileset->getTexture(), 255, 255, 255);
						SDL_SetTextureAlphaMod(spr::tileset->getTexture(), 100);

						if (revGridX > 0) PlayerPtr->setDirection(0);
						else if (revGridX < 0) PlayerPtr->setDirection(4);
					}
					else
					{
						SDL_SetTextureColorMod(spr::tileset->getTexture(), 255, 0, 0);
						SDL_SetTextureAlphaMod(spr::tileset->getTexture(), 100);
					}

					drawSpriteCenter
					(
						spr::tileset,
						itemDex[rotatedItemCode].tileSprIndex,
						(cameraW / 2) + (int)(16.0 * zoomScale * revGridX),
						(cameraH / 2) + (int)(16.0 * zoomScale * revGridY)
					);

					SDL_SetTextureColorMod(spr::tileset->getTexture(), 255, 255, 255);
					SDL_SetTextureAlphaMod(spr::tileset->getTexture(), 255);
				}


				setZoom(1.0);




			}


		}
		else//타겟 선택이 완료된 후
		{
			setZoom(zoomScale);
			SDL_SetTextureColorMod(spr::propset->getTexture(), 0, 255, 0);
			SDL_SetTextureAlphaMod(spr::propset->getTexture(), 120);
			drawSpriteCenter
			(
				spr::propset,
				itemDex[rotatedItemCode].propSprIndex,
				(cameraW / 2),
				(cameraH / 2) + itemDex[rotatedItemCode].checkFlag(itemFlag::PROP_BIG)*zoomScale * (-16 * 1 + 8)
			);
			SDL_SetTextureColorMod(spr::propset->getTexture(), 255, 255, 255);
			SDL_SetTextureAlphaMod(spr::propset->getTexture(), 255);
			setZoom(1.0);

			//drawEdgeWindow(selectBox.x, selectBox.y, selectBox.w, selectBox.h, 16, dir16::dir2);

			drawSpriteCenter(spr::coordCraftBox, 0, selectBox.x + selectBox.w / 2, selectBox.y + selectBox.h / 2 - 14);

			setFontSize(18);
			drawText(itemDex[rotatedItemCode].name +L" " + itemDex[rotatedItemCode].dir, selectBox.x + 12, selectBox.y + 6);

			auto drawBtn = [](int iconIndex, SDL_Rect targetBtn)
				{
					SDL_Color btnColor = { 0x00, 0x00, 0x00 };
					SDL_Color outlineColor = { 0x4A, 0x4A, 0x4A };
					if (checkCursor(&targetBtn))
					{
						if (click == false) { btnColor = lowCol::blue; }
						else { btnColor = lowCol::deepBlue; }
						outlineColor = { 0xa6, 0xa6, 0xa6 };
					}

					drawFillRect(targetBtn, btnColor);
					drawRect(targetBtn, outlineColor);

					drawSpriteCenter(spr::icon48, iconIndex, targetBtn.x + targetBtn.w / 2, targetBtn.y + targetBtn.h / 2);

					int markerIndex = 0;
					if (timer::timer600 % 30 < 5) { markerIndex = 0; }
					else if (timer::timer600 % 30 < 10) { markerIndex = 1; }
					else if (timer::timer600 % 30 < 15) { markerIndex = 2; }
					else if (timer::timer600 % 30 < 20) { markerIndex = 1; }
					else { markerIndex = 0; }

					if (checkCursor(&targetBtn)) drawSpriteCenter(spr::coordCraftMarker, markerIndex, targetBtn.x + targetBtn.w / 2, targetBtn.y + targetBtn.h / 2);
				};


			if(checkCursor(&confirmBtn)==false) drawBtn(186, confirmBtn);
			else drawBtn(189, confirmBtn);
			

			if (itemDex[rotatedItemCode].dirChangeItemCode != 0)
			{
				drawBtn(187, rotateBtn);
			}
			else
			{
				SDL_Color btnColor = { 0x00, 0x00, 0x00 };
				SDL_Color outlineColor = { 0x4A, 0x4A, 0x4A };

				drawFillRect(rotateBtn, btnColor);
				drawRect(rotateBtn, outlineColor);

				drawSpriteCenter(spr::icon16, 187, rotateBtn.x + rotateBtn.w / 2, rotateBtn.y + rotateBtn.h / 2);
				drawFillRect(rotateBtn, btnColor, 150);
			}

			drawBtn(188, cancelBtn);
		}
	}
	void clickUpGUI()
	{
		if (checkCursor(&tab) == true)
		{
			coAnswer.clear();
			cameraFix = true;
			delete this;
		}
		else
		{
			if (checkCursor(&letterbox) == false && checkCursor(&tab) == false && checkCursor(&letterboxPopUpButton) == false)
			{
				if (targetSelect == false)//타겟 선택 전
				{
					int revX, revY, revGridX, revGridY;
					if (option::inputMethod == input::touch)
					{
						revX = event.tfinger.x * cameraW - (cameraW / 2);
						revY = event.tfinger.y * cameraH - (cameraH / 2);
					}
					else
					{
						revX = getMouseX() - (cameraW / 2);
						revY = getMouseY() - (cameraH / 2);
					}
					revX += sgn(revX) * (8 * zoomScale);
					revGridX = revX / (16 * zoomScale);
					revY += sgn(revY) * (8 * zoomScale);
					revGridY = revY / (16 * zoomScale);

					//상대좌표를 절대좌표로 변환
					int throwingX = PlayerX() + revGridX;
					int throwingY = PlayerY() + revGridY;
					int throwingZ = PlayerZ();
					std::wstring xStr = std::to_wstring(throwingX);
					std::wstring yStr = std::to_wstring(throwingY);
					std::wstring zStr = std::to_wstring(throwingZ);

					prt(L"절대좌표 (%d,%d) 타일을 터치했다.\n", wtoi(xStr.c_str()), wtoi(yStr.c_str()));


					if (selectableCoord.size() > 0)//선택좌표가 있는 경우일 때
					{
						for (int i = 0; i < selectableCoord.size(); i++)
						{
							if (selectableCoord[i].x == throwingX && selectableCoord[i].y == throwingY)
							{
								if (itemDex[rotatedItemCode].dirChangeItemCode == 0)
								{
									coAnswer = std::to_wstring(throwingX) + L"," + std::to_wstring(throwingY) + L"," + std::to_wstring(rotatedItemCode);
									cameraFix = true;
									delete this;
									return;
								}
								targetSelect = true;
								rangeSet.clear();
								targetGridX = throwingX;
								targetGridY = throwingY;
								cameraFix = false;
								cameraX = 16 * targetGridX + 8;
								cameraY = 16 * targetGridY + 8;
								break;
							}
							lowCol::deepBlue;

							if (i == selectableCoord.size() - 1)
							{
								prt(L"해당 좌표는 선택할 수 없다.\n");
							}
						}
					}
					else//선택좌표가 없으면 그냥 시야가 흰색이면 선택 OK
					{

						if (TileFov(throwingX, throwingY, throwingZ) == fovFlag::white)
						{
							if (itemDex[rotatedItemCode].dirChangeItemCode == 0)
							{
								coAnswer = std::to_wstring(throwingX) + L"," + std::to_wstring(throwingY) + L"," + std::to_wstring(rotatedItemCode);
								cameraFix = true;
								delete this;
								return;
							}
							targetSelect = true;
							targetGridX = throwingX;
							targetGridY = throwingY;
							cameraFix = false;
							cameraX = 16 * targetGridX + 8;
							cameraY = 16 * targetGridY + 8;
							if (targetGridX > PlayerX()) PlayerPtr->setDirection(0);
							else if (targetGridY < PlayerX()) PlayerPtr->setDirection(4);
						}
						else
						{
							prt(L"해당 좌표는 시야에 보이지 않는다.\n");
						}
					}
				}
				else
				{
					if (checkCursor(&confirmBtn))
					{
						coAnswer = std::to_wstring(targetGridX) + L"," + std::to_wstring(targetGridY) + L"," + std::to_wstring(rotatedItemCode);
						cameraFix = true;
						delete this;
					}
					else if (checkCursor(&rotateBtn))
					{
						if (itemDex[rotatedItemCode].dirChangeItemCode != 0)
						{
							rotatedItemCode = itemDex[rotatedItemCode].dirChangeItemCode;
						}
					}
					else if (checkCursor(&cancelBtn))
					{
						coAnswer.clear();
						cameraFix = true;
						delete this;
					}
				}
			}
		}
	}
	void step()
	{
		tabType = tabFlag::back;
	}
};