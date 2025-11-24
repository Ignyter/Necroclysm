#include <SDL3/SDL.h>

#define CORO(func) delete coFunc; coFunc = new Corouter(func); (*coFunc).run();

export module Craft_input_mouse;

import Craft;
import globalVar;
import checkCursor;
import log;
import ItemData;

void Craft::clickUpGUI()
{
	if (getStateInput() == false) { return; }

	exInput = false;
	if (checkCursor(&tab))
	{
		executeTab();

	}
	else if (checkCursor(&tooltipCraftBtn) && craftCursor >= 0)//툴팁 조합하기
	{
		CORO(executeCraft());
	}
	else if (checkCursor(&tooltipBookmarkBtn) && craftCursor >= 0)//툴팁 즐겨찾기
	{
		CORO(executeBookmark());
	}
	else if (checkCursor(&bookmarkCategory))//즐겨찾기 카테고리 클릭업
	{
		craftCursor = -1;
		if (selectCategory != -2)
		{
			craftScroll = 0;
			selectCategory = -2;
			selectSubcategory = 0;
			int matchCount = recipePtr->searchFlag(itemFlag::BOOKMARK1);
			filterUpdate(matchCount);

		}
		else //재차 눌렀을 경우 전체 분류로 변경
		{
			selectCategory = -1;
		}
	}
	else if (checkCursor(&searchTextRect))
	{
		if (exInput == false)
		{
			exInput = true;
			SDL_StartTextInput(window);  // 추가
		}
		else
		{
			exInput = false;
			SDL_StopTextInput(window);  // 추가
		}
	}
	else if (checkCursor(&searchBtnRect))
	{
		craftCursor = -1;
		int matchCount = recipePtr->searchTxt(exInputText);
		filterUpdate(matchCount);
	}
	else
	{
		//카테고리 8버튼 클릭업
		for (int i = 0; i < 8; i++)
		{
			if (checkCursor(&craftCategory[i]))
			{
				craftCursor = -1;
				if (selectCategory != i)
				{
					craftScroll = 0;
					selectCategory = i;
					selectSubcategory = 0;

					itemCategory targetCategory;
					switch (selectCategory)
					{
					case 0:
						targetCategory = itemCategory::equipment;
						break;
					case 1:
						targetCategory = itemCategory::foods;
						break;
					case 2:
						targetCategory = itemCategory::tools;
						break;
					case 3:
						targetCategory = itemCategory::tech;
						break;
					case 4:
						targetCategory = itemCategory::consumables;
						break;
					case 5:
						targetCategory = itemCategory::vehicles;
						break;
					case 6:
						targetCategory = itemCategory::structures;
						break;
					case 7:
						targetCategory = itemCategory::materials;
						break;
					}
					int matchCount = recipePtr->searchCategory(targetCategory);
					filterUpdate(matchCount);
				}
				else
				{
					selectCategory = -1;
					selectSubcategory = 0;
					for (int i = 0; i < recipePtr->itemInfo.size(); i++) recipePtr->itemInfo[i].eraseFlag(itemFlag::BLACKFILTER);
					numNoneBlackFilter = recipePtr->itemInfo.size();
					recipePtr->sortByUnicode();
				}
				break;
			}
		}

		//서브카테고리 입력
		{
			int maxSubcategorySize;
			switch (selectCategory)
			{
			case -2://즐겨찾기
				maxSubcategorySize = 6;
				break;
			case -1://전체
				maxSubcategorySize = 1;
				break;
			case 0://장비
				maxSubcategorySize = 6;
				break;
			case 1://음식
				maxSubcategorySize = 6;
				break;
			case 2://도구
				maxSubcategorySize = 5;
				break;
			case 3://기술
				maxSubcategorySize = 3;
				break;
			case 4://소모품
				maxSubcategorySize = 5;
				break;
			case 5://차량
				maxSubcategorySize = 5;
				break;
			case 6://건축물
				maxSubcategorySize = 5;
				break;
			case 7://재료
				maxSubcategorySize = 6;
				break;
			}
			for (int i = 0; i < maxSubcategorySize; i++)
			{
				if (checkCursor(&subcategoryBox[i]))
				{
					craftCursor = -1;
					if (selectSubcategory != i)
					{
						craftScroll = 0;
						selectSubcategory = i;
						int matchCount;
						itemFlag targetFlag;
						itemSubcategory targetSubcategory;
						switch (selectCategory)
						{
						case -2:
							if (selectSubcategory == 0) { targetFlag = itemFlag::BOOKMARK1; }
							else if (selectSubcategory == 1) { targetFlag = itemFlag::BOOKMARK2; }
							else if (selectSubcategory == 2) { targetFlag = itemFlag::BOOKMARK3; }
							else if (selectSubcategory == 3) { targetFlag = itemFlag::BOOKMARK4; }
							else if (selectSubcategory == 4) { targetFlag = itemFlag::BOOKMARK5; }
							else if (selectSubcategory == 5) { targetFlag = itemFlag::BOOKMARK6; }
							matchCount = recipePtr->searchFlag(targetFlag);
							break;
						case 0://장비
							if (selectSubcategory == 0)
							{
								// All - 전체 장비 카테고리 표시
								itemCategory targetCategory = itemCategory::equipment;
								matchCount = recipePtr->searchCategory(targetCategory);
							}
							else
							{
								if (selectSubcategory == 1) { targetSubcategory = itemSubcategory::equipment_melee; }
								else if (selectSubcategory == 2) { targetSubcategory = itemSubcategory::equipment_ranged; }
								else if (selectSubcategory == 3) { targetSubcategory = itemSubcategory::equipment_firearms; }
								else if (selectSubcategory == 4) { targetSubcategory = itemSubcategory::equipment_throwing; }
								else if (selectSubcategory == 5) { targetSubcategory = itemSubcategory::equipment_clothing; }
								matchCount = recipePtr->searchSubcategory(targetSubcategory);
							}
							break;
						case 1://음식
							if (selectSubcategory == 0)
							{
								itemCategory targetCategory = itemCategory::foods;
								matchCount = recipePtr->searchCategory(targetCategory);
							}
							else
							{
								if (selectSubcategory == 1) { targetSubcategory = itemSubcategory::foods_cooked; }
								else if (selectSubcategory == 2) { targetSubcategory = itemSubcategory::foods_processed; }
								else if (selectSubcategory == 3) { targetSubcategory = itemSubcategory::foods_preserved; }
								else if (selectSubcategory == 4) { targetSubcategory = itemSubcategory::foods_drinks; }
								else if (selectSubcategory == 5) { targetSubcategory = itemSubcategory::foods_ingredients; }
								matchCount = recipePtr->searchSubcategory(targetSubcategory);
							}
							break;
						case 2://도구
							if (selectSubcategory == 0)
							{
								itemCategory targetCategory = itemCategory::tools;
								matchCount = recipePtr->searchCategory(targetCategory);
							}
							else
							{
								if (selectSubcategory == 1) { targetSubcategory = itemSubcategory::tools_hand; }
								else if (selectSubcategory == 2) { targetSubcategory = itemSubcategory::tools_power; }
								else if (selectSubcategory == 3) { targetSubcategory = itemSubcategory::tools_containers; }
								else if (selectSubcategory == 4) { targetSubcategory = itemSubcategory::tools_etc; }
								matchCount = recipePtr->searchSubcategory(targetSubcategory);
							}
							break;
						case 3://기술
							if (selectSubcategory == 0)
							{
								itemCategory targetCategory = itemCategory::tech;
								matchCount = recipePtr->searchCategory(targetCategory);
							}
							else
							{
								if (selectSubcategory == 1) { targetSubcategory = itemSubcategory::tech_bionics; }
								else if (selectSubcategory == 2) { targetSubcategory = itemSubcategory::tech_powerArmor; }
								matchCount = recipePtr->searchSubcategory(targetSubcategory);
							}
							break;
						case 4://소모품
							if (selectSubcategory == 0)
							{
								itemCategory targetCategory = itemCategory::consumables;
								matchCount = recipePtr->searchCategory(targetCategory);
							}
							else
							{
								if (selectSubcategory == 1) { targetSubcategory = itemSubcategory::consumable_medicine; }
								else if (selectSubcategory == 2) { targetSubcategory = itemSubcategory::consumable_ammo; }
								else if (selectSubcategory == 3) { targetSubcategory = itemSubcategory::consumable_fuel; }
								else if (selectSubcategory == 4) { targetSubcategory = itemSubcategory::consumable_etc; }
								matchCount = recipePtr->searchSubcategory(targetSubcategory);
							}
							break;
						case 5://차량
							if (selectSubcategory == 0)
							{
								itemCategory targetCategory = itemCategory::vehicles;
								matchCount = recipePtr->searchCategory(targetCategory);
							}
							else
							{
								if (selectSubcategory == 1) { targetSubcategory = itemSubcategory::vehicle_frames; }
								else if (selectSubcategory == 2) { targetSubcategory = itemSubcategory::vehicle_power; }
								else if (selectSubcategory == 3) { targetSubcategory = itemSubcategory::vehicle_exteriors; }
								else if (selectSubcategory == 4) { targetSubcategory = itemSubcategory::vehicle_parts; }
								matchCount = recipePtr->searchSubcategory(targetSubcategory);
							}
							break;
						case 6://건축물
							if (selectSubcategory == 0)
							{
								itemCategory targetCategory = itemCategory::structures;
								matchCount = recipePtr->searchCategory(targetCategory);
							}
							else
							{
								if (selectSubcategory == 1) { targetSubcategory = itemSubcategory::structure_walls; }
								else if (selectSubcategory == 2) { targetSubcategory = itemSubcategory::structure_floors; }
								else if (selectSubcategory == 3) { targetSubcategory = itemSubcategory::structure_ceilings; }
								else if (selectSubcategory == 4) { targetSubcategory = itemSubcategory::structure_props; }
								matchCount = recipePtr->searchSubcategory(targetSubcategory);
							}
							break;
						case 7://재료
							if (selectSubcategory == 0)
							{
								itemCategory targetCategory = itemCategory::materials;
								matchCount = recipePtr->searchCategory(targetCategory);
							}
							else
							{
								if (selectSubcategory == 1) { targetSubcategory = itemSubcategory::material_metals; }
								else if (selectSubcategory == 2) { targetSubcategory = itemSubcategory::material_organic; }
								else if (selectSubcategory == 3) { targetSubcategory = itemSubcategory::material_components; }
								else if (selectSubcategory == 4) { targetSubcategory = itemSubcategory::material_chemicals; }
								else if (selectSubcategory == 5) { targetSubcategory = itemSubcategory::material_etc; }
								matchCount = recipePtr->searchSubcategory(targetSubcategory);
							}
							break;
						}
						filterUpdate(matchCount);
					}
					break;
				}
			}
		}

		//아이템박스 클릭
		for (int i = 0; i < 24; i++)
		{
			if (checkCursor(&itemBox[i]))
			{
				if (craftCursor != i + CRAFT_MAX_COLUMN * craftScroll && i + CRAFT_MAX_COLUMN * craftScroll < numNoneBlackFilter)
				{
					craftCursor = i + CRAFT_MAX_COLUMN * craftScroll;
				}
				else
				{
					craftCursor = -1;
				}
			}
		}
	}
}
void Craft::clickMotionGUI(int dx, int dy)
{
	if (deactColorChange == false)
	{
		pointingCursor = -1;
		for (int i = 0; i < 24; i++)
		{
			if (checkCursor(&itemBox[i]))
			{
				if (i + CRAFT_MAX_COLUMN * craftScroll < recipePtr->itemInfo.size())
				{
					if (recipePtr->itemInfo[i + CRAFT_MAX_COLUMN * craftScroll].checkFlag(itemFlag::BLACKFILTER) == false) pointingCursor = i;
				}
			}
		}
	}

	if (checkCursor(&craftBase))
	{
		if (click == true)
		{
			int scrollAccelConst = 20; // 가속상수, 작아질수록 스크롤 속도가 빨라짐
			craftScroll = initCraftScroll + dy / scrollAccelConst;
			if (std::abs(dy / scrollAccelConst) >= 1)
			{
				deactClickUp = true;
				itemListColorLock = true;
			}
		}
	}
}
void Craft::clickDownGUI()
{
	initCraftScroll = craftScroll;
}

void Craft::mouseWheel()
{
	if (checkCursor(&craftBase))
	{
		int maxScroll = (numNoneBlackFilter - 1) / CRAFT_MAX_COLUMN - (CRAFT_MAX_ROW - 1);
		if (maxScroll < 0) maxScroll = 0;

		if (event.wheel.y > 0 && craftScroll > 0) craftScroll -= 1;
		else if (event.wheel.y < 0 && craftScroll < maxScroll) craftScroll += 1;
	}
}