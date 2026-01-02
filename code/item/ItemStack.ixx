#include <SDL3/SDL.h>

export module ItemStack;

import std;
import util;
import constVar;
import textureVar;
import ItemPocket;
import Ani;
import Sprite;
import Coord;

export class ItemStack : public Ani, public Coord
{
private:
	Sprite* sprite;
	std::unique_ptr<ItemPocket> storage;
	int sprIndex = 0;
	int targetSprIndex = 0;//던지기 이벤트일 때 타겟의 인덱스
public:
	ItemStack(Point3 inputCoor);
	ItemStack(Point3 inputCoor, std::vector<std::pair<int, int>> inputItems);
	~ItemStack();
	Sprite* getSprite(); 
	void setSprite(Sprite* inputSprite);
	ItemPocket* getPocket();
	void checkEmpty();
	void pullStackLights();
	void pullStackLights(Point3 tgtCoor);
	bool runAnimation(bool shutdown);
	void updateItems();
};