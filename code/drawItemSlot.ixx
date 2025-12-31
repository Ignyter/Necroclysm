#include <SDL3/SDL.h>

export module drawItemSlot;

import std;
import util;
import ItemData;
import ItemPocket;
import globalVar;
import drawText;
import constVar;
import wrapVar;
import checkCursor;
import drawSprite;
import textureVar;


export enum class cursorFlag
{
	none,
	click,
	hover,
	gray,
};

//아이템이 들어있는 사각형을 그림, 질량과 부피는 포함하지않음
export void drawItemRect(cursorFlag inputCursor, int x, int y, ItemData& inputItem)
{
	setFont(fontType::mainFont);
	int fontSize = 18;
	int yCorrection = 0;
	bool split = false;
	SDL_Rect itemBox = { x, y, 253, 32 };
	SDL_Color stadiumColor = { 0,0,0 };

	switch (inputCursor)
	{
	case cursorFlag::click:
		stadiumColor = lowCol::deepBlue;
		break;
	case cursorFlag::hover:
		stadiumColor = lowCol::blue;
		break;
	}

	drawStadium(itemBox.x, itemBox.y, itemBox.w, itemBox.h, stadiumColor, 183, 5);

	SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, 0xff);
	std::wstring mainName = L"";

	//아이템 이름
	mainName += inputItem.name;

	//후열 탄창이랑 탄환수 텍스트
	if (inputItem.checkFlag(itemFlag::GUN) || inputItem.checkFlag(itemFlag::MAGAZINE))
	{
		ItemPocket* pocketPtr = inputItem.pocketPtr.get();
		if (pocketPtr->itemInfo.size() > 0)
		{
			//리볼버
			if (pocketPtr->itemInfo[0].checkFlag(itemFlag::AMMO))
			{
				int bulletNumber = 0;
				ItemPocket* gunInside = pocketPtr;
				for (int i = 0; i < gunInside->itemInfo.size(); i++)
				{
					bulletNumber += gunInside->itemInfo[i].number;
				}

				mainName += L" (";
				mainName += std::to_wstring(bulletNumber);
				mainName += L"/";
				mainName += std::to_wstring(inputItem.pocketMaxNumber);
				mainName += L")";
			}
			else if (pocketPtr->itemInfo[0].checkFlag(itemFlag::MAGAZINE))//탄창
			{
				int bulletNumber = 0;
				ItemPocket* magazineInside = pocketPtr->itemInfo[0].pocketPtr.get();
				for (int i = 0; i < magazineInside->itemInfo.size(); i++)
				{
					bulletNumber += magazineInside->itemInfo[i].number;
				}
				mainName += L" (";
				mainName += std::to_wstring(bulletNumber);
				mainName += L"/";
				mainName += std::to_wstring(pocketPtr->itemInfo[0].pocketMaxNumber);
				mainName += L")";
			}
		}
		else
		{
			//리볼버 종류 또는 탄창
			if (itemDex[inputItem.pocketOnlyItem[0]].checkFlag(itemFlag::AMMO))
			{
				mainName += L" (";
				mainName += L"0";
				mainName += L"/";
				mainName += std::to_wstring(inputItem.pocketMaxNumber);
				mainName += L")";
			}
		}
	}

	if (inputItem.itemCode == itemRefCode::battery || inputItem.itemCode == itemRefCode::batteryPack)
	{
		mainName += L" (" + decimalCutter(100.0*(inputItem.powerStorage / static_cast<double>(inputItem.powerStorageMax)), 0) + L"%)";
	}
	else if (inputItem.pocketPtr != nullptr)
	{
		if (inputItem.pocketPtr->itemInfo.size() == 1)
		{
			ItemData& batteryData = inputItem.pocketPtr->itemInfo[0];
			if (batteryData.itemCode == itemRefCode::battery
				|| batteryData.itemCode == itemRefCode::batteryPack)
			{
				mainName += L" (" + decimalCutter(100.0 * (batteryData.powerStorage / static_cast<double>(batteryData.powerStorageMax)), 0) + L"%)";
			}
		}
	}


	const int widthLimit = 188;
	ItemPocket* pocketPtr = inputItem.pocketPtr.get();

	auto drawVolumeGauge = [&](int currentVol, int maxVol)
		{
			drawRect(itemBox.x + 242, itemBox.y + 3, 6, 27, col::white);
			int gaugeHeight = 23;
			int currentGaugeHeight = 0;

			if (maxVol > 0 && currentVol > 0)
			{
				currentGaugeHeight = (currentVol * gaugeHeight) / maxVol;
				if (currentGaugeHeight > gaugeHeight) currentGaugeHeight = gaugeHeight;
				if (currentGaugeHeight == 0) currentGaugeHeight = 1;
			}

			if (currentGaugeHeight > 0)
			{
				for (int i = 0; i < currentGaugeHeight; i++) {
					drawLine(itemBox.x + 244, itemBox.y + 5 + (gaugeHeight - currentGaugeHeight) + i,
						itemBox.x + 244, itemBox.y + 5 + (gaugeHeight - currentGaugeHeight) + i, col::white);
				}
			}
		};

	auto renderTextsWithSubInfo = [&](const std::wstring& mainText, const std::wstring& subText) {
		setFontSize(14);
		yCorrection = -5;
		if (queryTextWidth(mainText) > widthLimit)
		{
			setFontSize(12);
			drawText(mainText, itemBox.x + 50, itemBox.y + itemBox.h / 2 - 11 + yCorrection + 5);
		}
		else  drawText(mainText, itemBox.x + 50, itemBox.y + itemBox.h / 2 - 11 + yCorrection + 2);
		setFontSize(11);
		drawText(subText, itemBox.x + 50, itemBox.y + itemBox.h / 2 + 1);
		};

	if (inputItem.checkFlag(itemFlag::CONTAINER_LIQ) && pocketPtr->itemInfo.size() > 0)
	{
		std::wstring insideStr = L"(";
		insideStr += pocketPtr->itemInfo[0].name + L" ";
		int maxVol = inputItem.pocketMaxVolume;
		int currentVol = pocketPtr->getPocketVolume();
		insideStr += std::to_wstring(currentVol);
		insideStr += L"/";
		insideStr += std::to_wstring(maxVol);
		insideStr += L"mL)";

		renderTextsWithSubInfo(mainName, insideStr);
		drawVolumeGauge(currentVol, maxVol);
	}
	else if (inputItem.pocketMaxVolume > 0 && pocketPtr->itemInfo.size() > 0)
	{
		std::wstring insideStr = L"(";
		if (pocketPtr->itemInfo.size() == 1) insideStr += pocketPtr->itemInfo[0].name + L")";
		else if (pocketPtr->itemInfo.size() > 1) insideStr += std::to_wstring(pocketPtr->itemInfo.size()) + L" items)";
		int maxVol = inputItem.pocketMaxVolume;
		int currentVol = pocketPtr->getPocketVolume();

		renderTextsWithSubInfo(mainName, insideStr);
		drawVolumeGauge(currentVol, maxVol);
	}
	else
	{
		setFontSize(18); // 기본 18px
		if (queryTextWidth(mainName) > widthLimit)
		{
			setFontSize(17); // 1단계: 16px
			if (queryTextWidth(mainName) > widthLimit)
			{
				setFontSize(16); // 2단계: 12px
				if (queryTextWidth(mainName) > widthLimit)
				{
					setFontSize(15); // 3단계: 10px (최소)
					if (queryTextWidth(mainName) > widthLimit)
					{
						setFontSize(14); // 3단계: 10px (최소)
						yCorrection += 2;
					}
				}
			}
		}
		drawText(mainName, itemBox.x + 46, itemBox.y + itemBox.h / 2 - 12 + yCorrection);
	}

	if (inputItem.checkFlag(itemFlag::GRAYFILTER)) { drawStadium(itemBox.x, itemBox.y, itemBox.w, itemBox.h, stadiumColor, 183, 5); }

	//아이템 아이콘 그리기
	setZoom(2.0);
	if (inputItem.checkFlag(itemFlag::GRAYFILTER))
	{
		SDL_SetTextureBlendMode(spr::itemset->getTexture(), SDL_BLENDMODE_BLEND);
		SDL_SetTextureColorMod(spr::itemset->getTexture(), 128, 128, 128);
		drawSpriteCenter(spr::itemset, getItemSprIndex(inputItem), itemBox.x + 18, itemBox.y + itemBox.h / 2);
		SDL_SetTextureColorMod(spr::itemset->getTexture(), 255, 255, 255);
	}
	else
	{
		drawSpriteCenter(spr::itemset, getItemSprIndex(inputItem), itemBox.x + 18, itemBox.y + itemBox.h / 2);
	}

	//장비 중인 아이템이나 갯수가 1 이하인 아이템은 갯수 표시하지 않음
	if (inputItem.equipState == equipHandFlag::none && inputItem.number > 1)
	{
		setFontSize(13);
		std::wstring numberStr = L"x" + std::to_wstring(inputItem.number);
		drawTextOutline(numberStr, itemBox.x + 250 - queryTextWidth(numberStr), itemBox.y - 3,col::white);
	}

	setZoom(1.0);
}

//질량 부피 표시와 체크박스까지 포함한 확장형 아이템 박스를 그려냄
//질량 부피 표시와 체크박스까지 포함한 확장형 아이템 박스를 그려냄
export void drawItemRectExtend(bool cursor, int x, int y, ItemData& inputItem, int quantity, bool hasBox1, bool whiteCursor)
{
	//세로 38 가로폭 376의 아이템 슬롯을 그립니다.
	const int gapWidth = 7;

	///////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////     ▼BOX1(셀렉트박스) 그리기    ///////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////

	SDL_Rect box1 = { x, y, 43, 32 };
	SDL_Color statusColor;
	SDL_Color statusStrColor;
	std::wstring statusStr = L"-";
	setFont(fontType::mainFont);
	setFontSize(14);

	if (inputItem.equipState == equipHandFlag::normal)
	{
		statusColor = col::black;
		statusStr = L"E";
		statusStrColor = lowCol::green;
	}
	else if (inputItem.equipState == equipHandFlag::left)
	{
		statusColor = col::black;
		statusStr = sysStr[49];//왼손
		statusStrColor = lowCol::yellow;
		setFontSize(12);
	}
	else if (inputItem.equipState == equipHandFlag::right)
	{
		statusColor = col::black;
		statusStr = sysStr[50];
		statusStrColor = lowCol::yellow;
		setFontSize(12);
	}
	else if (inputItem.equipState == equipHandFlag::both)
	{
		statusColor = col::black;
		statusStr = sysStr[51];
		statusStrColor = lowCol::yellow;
	}
	else if (inputItem.lootSelect == inputItem.number) //전체선택
	{
		statusColor = lowCol::deepBlue;
		statusStr = L"+";
		statusStrColor = col::white;
	}
	else if (inputItem.lootSelect == 0)
	{
		statusColor = { 0,0,0 };
		statusStr = L"-";
		statusStrColor = col::gray;
	}
	else
	{
		statusColor = lowCol::purple;
		statusStr = std::to_wstring(inputItem.lootSelect);
		statusStrColor = col::white;
	}

	if (checkCursor(&box1) && itemListColorLock == false)
	{
		if (click == false)
		{
			statusColor = lowCol::blue;
			statusStrColor = col::white;
		}
		else
		{
			statusColor = lowCol::deepBlue;
			statusStrColor = col::white;
		}
	}

	if (hasBox1 == true)
	{
		drawStadium(box1.x, box1.y, box1.w, box1.h, statusColor, 183, 5);
		drawTextCenter(statusStr, box1.x + box1.w / 2, box1.y + box1.h / 2, statusStrColor);
	}
	else
	{
		box1.w = 0;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////     ▼BOX2(메인박스) 그리기    ///////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////

	SDL_Rect box2 = { box1.x + box1.w + gapWidth, box1.y, 253, box1.h };
	if (cursor == true) drawItemRect(cursorFlag::hover, box2.x, box2.y, inputItem);
	else
	{
		SDL_Rect clickRect = { box2.x ,y, 376 - (box1.w + gapWidth), box2.h };
		//박스 3가 존재하지 않을 경우
		if (quantity == -1) { clickRect.w = box2.w; }

		if (checkCursor(&clickRect) && itemListColorLock == false)
		{
			if (click == false) drawItemRect(cursorFlag::hover, box2.x, box2.y, inputItem);
			else drawItemRect(cursorFlag::click, box2.x, box2.y, inputItem);
		}
		else
		{
			drawItemRect(cursorFlag::none, box2.x, box2.y, inputItem);
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////     ▼BOX3(부가정보) 그리기    ///////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////

	if (quantity == 0)
	{
		SDL_Rect box3 = { box2.x + box2.w + gapWidth, box1.y, 65, box1.h };

		if (cursor == true) drawStadium(box3.x, box3.y, box3.w, box3.h, lowCol::blue, 183, 5);
		else
		{
			SDL_Rect clickRect = { box2.x ,y, 376 - (box1.w + gapWidth), box2.h };
			//박스 3가 존재하지 않을 경우
			if (quantity == -1) { clickRect.w = box2.w; }

			if (checkCursor(&clickRect) && itemListColorLock == false)
			{
				if (click == false)  drawStadium(box3.x, box3.y, box3.w, box3.h, lowCol::blue, 183, 5);
				else  drawStadium(box3.x, box3.y, box3.w, box3.h, lowCol::deepBlue, 183, 5);
			}
			else
			{
				drawStadium(box3.x, box3.y, box3.w, box3.h, col::black, 183, 5);
			}
		}

		SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, 0xff);

		setFont(fontType::pixel);
		setFontSize(11);
		setFont(fontType::mainFont);

		drawSpriteCenter(spr::icon13, 39, box3.x + 12, box2.y + 8);
		if (inputItem.weight >= 1000)
		{
			std::wstring kgStr = decimalCutter(inputItem.weight / 1000.0, 2);
			drawText(kgStr + L" kg", box3.x + 9 + 11, box2.y + 2);
		}
		else
		{
			std::wstring kgStr = std::to_wstring(inputItem.weight);
			drawText(kgStr + L" g", box3.x + 9 + 11, box2.y + 2);
		}

		drawSpriteCenter(spr::icon13, 40, box3.x + 12, box2.y + 23);
		if (getVolume(inputItem) >= 1000)
		{
			std::wstring volStr = decimalCutter(getVolume(inputItem) / 1000.0, 2);
			drawText(volStr + L" L", box3.x + 9 + 11, box2.y + 16);
		}
		else
		{
			std::wstring volStr = std::to_wstring(getVolume(inputItem));
			drawText(volStr + L" mL", box3.x + 9 + 11, box2.y + 16);
		}

		setFont(fontType::mainFont);
	}

	if (whiteCursor == true)
	{
		int cursorIndex = 0;
		{
			if (timer::timer600 % 30 < 5) { cursorIndex = 0; }
			else if (timer::timer600 % 30 < 10) { cursorIndex = 1; }
			else if (timer::timer600 % 30 < 15) { cursorIndex = 2; }
			else if (timer::timer600 % 30 < 20) { cursorIndex = 3; }
			else { cursorIndex = 0; }
		}
		if (cursor == true)
		{
			if (quantity == -1 && hasBox1 == false)
			{
				drawSprite(spr::itemCursorShort, cursorIndex, box2.x - 16, box2.y - 16);
			}
			else
			{
				drawSprite(spr::itemCursorLong, cursorIndex, box2.x - 16, box2.y - 16);
			}
		}
	}
}

//심플 아이템렉트를 그립니다. 텍스트를 입력하는 타입.
export void drawSimpleItemRect(cursorFlag inputCursor, int x, int y, int iconIndex, std::wstring text, bool gray)
{
	int fontSize = 12;
	int yCorrection = 0;
	const int widthLimit = 170;
	bool split = false;
	SDL_Rect itemBox = { x, y, 210, 18 };

	int btnColorSprIndex = 0;
	switch (inputCursor)
	{
	case cursorFlag::click:
		btnColorSprIndex = 2;
		break;
	case cursorFlag::hover:
		btnColorSprIndex = 1;
		break;
	}
	drawSprite(spr::itemSlotBtn, btnColorSprIndex, itemBox.x, itemBox.y);

	SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, 0xff);
	setFontSize(16);
	std::wstring mainName = text;

	setFontSize(fontSize);
	drawText(mainName, itemBox.x + 34, itemBox.y + 1 + yCorrection);

	if (gray)
	{
		SDL_SetTextureBlendMode(spr::itemSlotBtn->getTexture(), SDL_BLENDMODE_BLEND);
		SDL_SetTextureColorMod(spr::itemSlotBtn->getTexture(), 128, 128, 128);
		drawSpriteCenter(spr::itemSlotBtn, 0, itemBox.x, itemBox.y);
		SDL_SetTextureColorMod(spr::itemSlotBtn->getTexture(), 255, 255, 255);
	}

	//아이템 아이콘 그리기
	setZoom(1.5);
	if (gray)
	{
		SDL_SetTextureBlendMode(spr::itemset->getTexture(), SDL_BLENDMODE_BLEND);
		SDL_SetTextureColorMod(spr::itemset->getTexture(), 128, 128, 128);
		drawSpriteCenter(spr::itemset, iconIndex, itemBox.x + 15, itemBox.y + 9);
		SDL_SetTextureColorMod(spr::itemset->getTexture(), 255, 255, 255);
	}
	else
	{
		drawSpriteCenter(spr::itemset, iconIndex, itemBox.x + 15, itemBox.y + 9);
	}
	setZoom(1.0);
}

//심플 아이템렉트를 그립니다. 아이템 정보를 입력하는 타입. 무게나 부피 표시 없음
export void drawSimpleItemRect(cursorFlag inputCursor, int x, int y, ItemData& inputItem)
{

	std::wstring mainName = L"";

	//장비 중인 아이템이나 갯수가 1 이하인 아이템은 갯수 표시하지 않음
	if (inputItem.equipState == equipHandFlag::none && inputItem.number > 1) { mainName += std::to_wstring(inputItem.number) + L" "; }

	//아이템 이름
	mainName += inputItem.name;

	//후열 탄창이랑 탄환수 텍스트
	if (inputItem.checkFlag(itemFlag::GUN) || inputItem.checkFlag(itemFlag::MAGAZINE))
	{
        ItemPocket* pocketPtr = inputItem.pocketPtr.get();

		if (pocketPtr->itemInfo.size() > 0)
		{
			//리볼버
			if (pocketPtr->itemInfo[0].checkFlag(itemFlag::AMMO))
			{
				int bulletNumber = 0;
				ItemPocket* gunInside = pocketPtr;
				for (int i = 0; i < gunInside->itemInfo.size(); i++)
				{
					bulletNumber += gunInside->itemInfo[i].number;
				}

				mainName += L" (";
				mainName += std::to_wstring(bulletNumber);
				mainName += L"/";
				mainName += std::to_wstring(inputItem.pocketMaxNumber);
				mainName += L")";
			}
			else if (pocketPtr->itemInfo[0].checkFlag(itemFlag::MAGAZINE))//탄창
			{
				int bulletNumber = 0;
				ItemPocket* magazineInside = (pocketPtr->itemInfo[0].pocketPtr).get();
				for (int i = 0; i < magazineInside->itemInfo.size(); i++)
				{
					bulletNumber += magazineInside->itemInfo[i].number;
				}
				mainName += L" (";
				mainName += std::to_wstring(bulletNumber);
				mainName += L"/";
				mainName += std::to_wstring(pocketPtr->itemInfo[0].pocketMaxNumber);
				mainName += L")";
			}
		}
		else
		{
			//리볼버 종류 또는 탄창
			if (itemDex[inputItem.pocketOnlyItem[0]].checkFlag(itemFlag::AMMO))
			{
				mainName += L" (";
				mainName += L"0";
				mainName += L"/";
				mainName += std::to_wstring(inputItem.pocketMaxNumber);
				mainName += L")";
			}
		}
	}

	drawSimpleItemRect(inputCursor, x, y, getItemSprIndex(inputItem), col2Str(col::white) + mainName, inputItem.checkFlag(itemFlag::GRAYFILTER));

}

export void drawSimpleItemRectAdd(cursorFlag inputCursor, int x, int y)
{
	switch (inputCursor)
	{
	default:
		drawSprite(spr::itemSlotBtn, 9, x, y);
		break;
	case cursorFlag::hover:
		drawSprite(spr::itemSlotBtn, 10, x, y);
		break;
	case cursorFlag::click:
		drawSprite(spr::itemSlotBtn, 11, x, y);
		break;
	}
	drawSpriteCenter(spr::icon16, 40, x + spr::itemSlotBtn->getW() / 2, y + spr::itemSlotBtn->getH() / 2);
}



