import ItemPocket;

#include <SDL3/SDL.h>

import std;
import util;
import World;
import globalVar;
import constVar;
import Coord;
import log;
import ItemData;
import actFuncSet;


ItemPocket::ItemPocket(storageType inputType) { type = inputType; }

ItemPocket::~ItemPocket() { prt(L"ItemPocket : 소멸자가 호출되었습니다..\n"); }

storageType ItemPocket::getType() { return type; }

//@brief 아이템의 number에 관계없이 해당 index의 아이템을 제거함
void ItemPocket::eraseItemInfo(int index)
{
	auto it = itemInfo.begin();
	it += index;
	itemInfo.erase(it);
}

//@brief 입력한 숫자만큼 아이템의 수를 제거합니다. 만약 0이 되면 목록에서 제거합니다. 아이템의 수보다 큰 값을 입력 시 오류가 발생, 동시에 Select가 더 클 경우 그 값을 맞춤
void ItemPocket::subtractItemIndex(int index, int number)
{
	if (number <= 0) return;
	errorBox(itemInfo[index].number < number, L"The number of items to remove is greater than the number of items.");
	itemInfo[index].number -= number;
	if (itemInfo[index].lootSelect > itemInfo[index].number) { itemInfo[index].lootSelect = itemInfo[index].number; }//셀렉트 값 조정
	if (itemInfo[index].number == 0) { eraseItemInfo(index); }
}

//@brief 입력한 아이템 코드의 아이템을 찾아서 number개 포켓에서 제거합니다. subtractItemIndex를 활용하는 재귀함수.
void ItemPocket::subtractItemCode(int code, int number)
{
	if (number <= 0) return;
	int totalRemovedCount = 0;
	for (int i = itemInfo.size() - 1; i >= 0; --i)
	{
		if (itemInfo[i].itemCode == code)
		{
			int amountToRemove = myMin(number - totalRemovedCount, itemInfo[i].number);

			if (amountToRemove > 0)
			{
				subtractItemIndex(i, amountToRemove);
				totalRemovedCount += amountToRemove;
			}

			if (totalRemovedCount == number) return;
		}

		errorBox(totalRemovedCount > number, L"Error occurs at method subtractItemCode(ItemPocket.ixx), the number of Item has been subtracted over its own number.");
	}
}


//@brief 입력한 storagePtr로 아이템을 number개 옮김(송신하는 Ptr은 이 메소드를 실행하는 인스턴스), 수신++, 송신--,  실제 아이템의 수보다 많이 넣으면 오류 발생
//@param storagePtr 아이템을 받을 storage 포인터
//@param index 보낼 아이템의 인덱스(송신 객체의 index번째 아이템임)
//@return 수신받은 ItemPocket의 index
int ItemPocket::transferItem(ItemPocket* storagePtr, int index, int number)
{

	errorBox(storagePtr == nullptr, L"Destination storage pointer is null in transferItem.");
	errorBox(number <= 0, L"Number to transfer must be positive in transferItem.");
	errorBox(index < 0 || index >= itemInfo.size(), L"Source index out of bounds in transferItem.");
	errorBox(itemInfo[index].number < number, L"item number that have to transfer is not enough in transferItem function(ItemPocket.ixx)");
	errorBox(storagePtr->type == storageType::equip && number >= 2, L"2개 이상의 장비가 동시에 장착되었다.");


	bool transferWholeStack = (itemInfo[index].number == number);
	itemInfo[index].lootSelect = 0;
	//일치하는 아이템이 있을 경우 숫자만 더하고 종료함
	for (int i = 0; i < storagePtr->itemInfo.size(); ++i)
	{
		if (itemInfo[index].itemOverlay(storagePtr->itemInfo[i]))//완전히 일치하는 아이템이 있을 경우
		{
			storagePtr->itemInfo[i].number += number;

			if (transferWholeStack) eraseItemInfo(index);
			else subtractItemIndex(index, number);

			return i;
		}
	}


	//일치하는 아이템을 찾지 못했을 경우 그 아이템을 위해 새로운 행을 추가함
	int destinationIndex = -1;
	if (transferWholeStack)
	{
		if (storagePtr->type != storageType::equip) itemInfo[index].equipState = equipHandFlag::none;
		storagePtr->itemInfo.push_back(std::move(itemInfo[index]));
		destinationIndex = storagePtr->itemInfo.size() - 1;
		eraseItemInfo(index);
	}
	else
	{
		ItemData newItem = itemInfo[index].cloneForTransfer(number);
		newItem.codeID = genItemID();
		storagePtr->itemInfo.push_back(std::move(newItem));
		destinationIndex = storagePtr->itemInfo.size() - 1;
		subtractItemIndex(index, number);
	}

	return destinationIndex;
}


//@brief Dex에서 이 함수를 사용한 ItemPocket에 해당 index의 템을 number개만큼 추가함
int ItemPocket::addItemFromDex(int index, int number)
{

	errorBox(index < 0 || index >= itemDex.size(), L"Item index out of bounds for itemDex in addItemFromDex.");
	errorBox(number <= 0, L"Number to add must be positive in addItemFromDex.");

	ItemData tgtItem = std::move(cloneFromItemDex(itemDex[index], number));

	if (tgtItem.pocketMaxVolume == 0 && tgtItem.pocketMaxNumber == 0)
	{
		for (int i = 0; i < itemInfo.size(); ++i)
		{
			if (tgtItem.itemOverlay(itemInfo[i]))
			{
				itemInfo[i].number += number;
				return i;
			}
		}

		itemInfo.push_back(std::move(tgtItem));
		return (itemInfo.size() - 1);
	}
	else
	{
		itemInfo.push_back(std::move(tgtItem));
		return (itemInfo.size() - 1);
	}
}


int ItemPocket::addItemFromDex(int index) { return addItemFromDex(index, 1); }

void ItemPocket::addItemFromDex(std::vector<Point2> inputVec)
{
	for (int i = 0; i < inputVec.size(); i++)
	{
		addItemFromDex(inputVec[i].x, inputVec[i].y);
	}
}


void ItemPocket::addRecipe(int inputItemCode)
{
	errorBox(getType() != storageType::recipe, L"The addRecipe function was executed while the storageType was not set to 'recipe'(ItemPocket.ixx).");
	if (itemDex[inputItemCode].checkFlag(itemFlag::NOT_RECIPE)) return; //NOT_RECIPE 플래그가 있을 경우 레시피목록에 추가하지 않음

	if (itemInfo.size() == 0) { addItemFromDex(inputItemCode, 1); }
	else
	{
		for (int i = 0; i < itemInfo.size(); i++)
		{
			//이미 레시피에 있는 아이템이면 추가하지 않음
			if (itemInfo[i].itemCode == inputItemCode) break;

			if (i == itemInfo.size() - 1) addItemFromDex(inputItemCode, 1);
		}
	}
}

void ItemPocket::swap(int index1, int index2)
{
	if (index1 == index2) return;
	errorBox(index1 >= itemInfo.size() || index1 < 0, L"index1 is an invalid value in swap.");
	errorBox(index2 >= itemInfo.size() || index2 < 0, L"index2 is an invalid value in swap.");
	std::swap(itemInfo[index1], itemInfo[index2]);
}

void ItemPocket::sortByUnicode(int startIndex, int endIndex)
{
	std::sort(itemInfo.begin() + startIndex, itemInfo.begin() + endIndex + 1,
		[](ItemData& a, ItemData& b)
		{
			std::wstring str1 = a.name;
			std::wstring str2 = b.name;
			for (int i = 0; i < myMin(str1.size(), str2.size()); i++)
			{
				if (str1[i] != str2[i]) { return str1[i] < str2[i]; }
			}
			return str1.size() < str2.size();
		}
	);
}
void ItemPocket::sortByUnicode() { sortByUnicode(0, itemInfo.size() - 1); }

void ItemPocket::sortEquip()
{
	int stackCursor = 0;

	//양손
	for (int i = itemInfo.size() - 1; i >= 0; i--)
	{
		if (itemInfo[i].equipState == equipHandFlag::both)
		{
			swap(stackCursor, i);
			stackCursor++;
			break;
		}
	}

	//왼손
	for (int i = itemInfo.size() - 1; i >= 0; i--)
	{
		if (itemInfo[i].equipState == equipHandFlag::left)
		{
			swap(stackCursor, i);
			stackCursor++;
			break;
		}
	}

	//오른손
	for (int i = itemInfo.size() - 1; i >= 0; i--)
	{
		if (itemInfo[i].equipState == equipHandFlag::right)
		{
			swap(stackCursor, i);
			stackCursor++;
			break;
		}
	}
}

void ItemPocket::sortWeightDescend(int startIndex, int endIndex)
{
	std::sort(itemInfo.begin() + startIndex, itemInfo.begin() + endIndex + 1,
		[](ItemData& a, ItemData& b)
		{
			return (a.weight > b.weight);
		}
	);
}
void ItemPocket::sortWeightDescend() { sortWeightDescend(0, itemInfo.size() - 1); }

void ItemPocket::sortWeightAscend(int startIndex, int endIndex)
{
	std::sort(itemInfo.begin() + startIndex, itemInfo.begin() + endIndex + 1,
		[](ItemData& a, ItemData& b)
		{
			return (a.weight < b.weight);
		}
	);
}
void ItemPocket::sortWeightAscend() { sortWeightAscend(0, itemInfo.size() - 1); }


////////////////////////////////////////////////////////////////////////////////

int ItemPocket::searchTxt(std::wstring inputStr, int startIndex, int endIndex)
{
	std::wstring keyword = inputStr.substr(0, inputStr.find(L"-") - 1);
	std::sort(itemInfo.begin() + startIndex, itemInfo.begin() + endIndex + 1,
		[keyword](ItemData& a, ItemData& b)
		{
			bool sameKeyword1 = (a.name.find(keyword.c_str()) != std::wstring::npos);
			bool sameKeyword2 = (b.name.find(keyword.c_str()) != std::wstring::npos);
			return sameKeyword1 > sameKeyword2;
		}
	);

	int matchCount = 0;
	for (int i = startIndex; i <= endIndex; i++)
	{
		if (itemInfo[i].name.find(keyword.c_str()) != std::wstring::npos) matchCount++;
	}
	return matchCount;
}
int ItemPocket::searchTxt(std::wstring inputStr) { return searchTxt(inputStr, 0, itemInfo.size() - 1); }

int ItemPocket::searchFlag(itemFlag inputFlag, int startIndex, int endIndex)
{
	std::sort(itemInfo.begin() + startIndex, itemInfo.begin() + endIndex + 1,
		[inputFlag](ItemData& a, ItemData& b)
		{
			bool sameKeyword1 = a.checkFlag(inputFlag);
			bool sameKeyword2 = b.checkFlag(inputFlag);
			return (sameKeyword1 > sameKeyword2);
		}
	);

	int matchCount = 0;
	for (int i = startIndex; i <= endIndex; i++)
	{
		if (itemInfo[i].checkFlag(inputFlag)) matchCount++;
	}
	return matchCount;
}
int ItemPocket::searchFlag(itemFlag inputFlag) { return searchFlag(inputFlag, 0, itemInfo.size() - 1); }

int ItemPocket::searchCategory(itemCategory input, int startIndex, int endIndex)
{
	std::sort(itemInfo.begin() + startIndex, itemInfo.begin() + endIndex + 1,
		[input](ItemData& a, ItemData& b)
		{
			bool sameKeyword1 = a.category == input;
			bool sameKeyword2 = b.category == input;
			return (sameKeyword1 > sameKeyword2);
		}
	);


	int matchCount = 0;
	for (int i = startIndex; i <= endIndex; i++) if (itemInfo[i].category == input) matchCount++;
	return matchCount;
}
int ItemPocket::searchCategory(itemCategory input) { return searchCategory(input, 0, itemInfo.size() - 1); }

int ItemPocket::searchSubcategory(itemSubcategory input, int startIndex, int endIndex)
{
	std::sort(itemInfo.begin() + startIndex, itemInfo.begin() + endIndex + 1,
		[input](ItemData& a, ItemData& b)
		{
			bool sameKeyword1 = a.subcategory == input;
			bool sameKeyword2 = b.subcategory == input;
			return (sameKeyword1 > sameKeyword2);
		}
	);

	int matchCount = 0;
	for (int i = startIndex; i <= endIndex; i++) if (itemInfo[i].subcategory == input) matchCount++;
	return matchCount;
}
int ItemPocket::searchSubcategory(itemSubcategory input) { return searchSubcategory(input, 0, itemInfo.size() - 1); }

//@brief 현재 이 포켓의 하위포켓을 모두 체크해 가지고있는 아이템의 숫자 반환
int ItemPocket::numberItem(int inputCode, int currentDepth)
{
	constexpr int MAX_DEPTH = 50;
	errorBox(currentDepth > MAX_DEPTH, L"Maximum recursion depth exceeded in numberItem. Possible pocket cycle?");

	int count = 0;
	for (int i = 0; i < itemInfo.size(); i++)
	{
		if (itemInfo[i].itemCode == inputCode) count += itemInfo[i].number;

		if (itemInfo[i].pocketPtr != nullptr)
		{
			int subCount = itemInfo[i].pocketPtr->numberItem(inputCode, currentDepth + 1);
			count += subCount;
		}
	}
	return count;
}

//현재 이 포켓의 하위포켓을 모두 체크하여 입력한 도구기술이 존재하는지 반환
bool ItemPocket::checkToolQuality(int input, int currentDepth) // 깊이 인자 추가
{
	constexpr int MAX_DEPTH = 50;
	errorBox(currentDepth > MAX_DEPTH, L"Maximum recursion depth exceeded in checkToolQuality. Possible pocket cycle?");

	for (int i = 0; i < itemInfo.size(); ++i)
	{
		for (int j = 0; j < itemInfo[i].toolQuality.size(); ++j)
		{
			if (itemInfo[i].toolQuality[j] == input) return true;
		}

		if (itemInfo[i].pocketPtr != nullptr)
		{
			if (itemInfo[i].pocketPtr->checkToolQuality(input, currentDepth + 1)) return true;
		}
	}
	return false;
}

int ItemPocket::getPocketVolume()
{
	int totalVolume = 0;
	for (int i = 0; i < itemInfo.size(); i++)
	{
		totalVolume += itemInfo[i].originalVolume * itemInfo[i].number;
	}
	return totalVolume;
}

int ItemPocket::getPocketWeight()
{
	int totalWeight = 0;
	for (int i = 0; i < itemInfo.size(); i++)
	{
		totalWeight += itemInfo[i].weight * itemInfo[i].number;
	}
	return totalWeight;
}

int ItemPocket::getPocketNumber()
{
	int totalNumber = 0;
	for (int i = 0; i < itemInfo.size(); i++)
	{
		totalNumber += itemInfo[i].number;
	}
	return totalNumber;
}

//입력한 포켓의 내부 아이템 수를 반환
int ItemPocket::countPocketItemNumber()
{
	int number = 0;
	for (int i = 0; i < itemInfo.size(); i++)
	{
		number += itemInfo[i].number;
	}
	return number;
};

//상단 총알의 데이터를 삭제
void ItemPocket::popTopBullet()
{
	errorBox(itemInfo.size() == 0, L"function popTopBullet has executed with no bullet in ItemPocket.ixx");
	if (itemInfo[0].number == 1) { itemInfo.erase(itemInfo.begin()); }
	else { itemInfo[0].number--; }
};

//현재 포켓의 모든 아이템 상태를 업데이트(현재는 전원 ON/OFF만 처리함)
void ItemPocket::updateItems()
{
	for (ItemData& item : itemInfo)
	{
		if (item.checkFlag(itemFlag::TOGGLE_ON))
		{
			if(item.pocketPtr != nullptr 
				&& item.pocketPtr->itemInfo.size()>0
				&& item.pocketPtr->itemInfo[0].powerStorage >= item.gndUsePower)
			{
				item.pocketPtr->itemInfo[0].powerStorage -= item.gndUsePower;
			}
			else
			{
				actFunc::toggle(item);
			}
		}
	}

}


//@brief 현재 이 총에 장전된 모든 총알을 벡터 형태로 반환
ItemPocket* getBulletPocket(ItemData& inputGun)
{
	ItemPocket* pocket = nullptr;

	if (inputGun.pocketOnlyItem.empty()) return nullptr;

	const int acceptedItemCode = inputGun.pocketOnlyItem[0];

	if (itemDex[acceptedItemCode].checkFlag(itemFlag::AMMO)) pocket = inputGun.pocketPtr.get();
	else if (itemDex[acceptedItemCode].checkFlag(itemFlag::MAGAZINE))
	{
		if (inputGun.pocketPtr)
		{
			ItemPocket* gunInternalPocket = inputGun.pocketPtr.get();
			if (!gunInternalPocket->itemInfo.empty())
			{
				ItemData& magazineItem = gunInternalPocket->itemInfo[0];
				pocket = magazineItem.pocketPtr.get();
			}
		}
	}
	return pocket;
}


//이 총에 장전된 모든 총알의 갯수 반환, 탄창일 때도 할것
int getBulletNumber(ItemData& inputGun)
{
	ItemPocket* bulletPocket = getBulletPocket(inputGun);
	if (bulletPocket != nullptr)
	{
		int bulletNumber = 0;
		for (int i = 0; i < bulletPocket->itemInfo.size(); i++)
		{
			bulletNumber += bulletPocket->itemInfo[i].number;
		}
		return bulletNumber;
	}
	else { return 0; }
};




