import util;
import Entity;
import globalVar;
import wrapVar;
import constVar;
import textureVar;
import turnWait;
import World;
import Sticker;
import Flame;
import Player;
import Prop;

import Bullet;
import Aim;
import TileData;
import ItemData;
import ItemStack;
import Particle;
import globalTime;

bool Entity::runAnimation(bool shutdown)
{
	//if (isPlayer) prt(L"Player의 runAnimation이 실행되었다.\n");
	//else prt(L"Entity %p의 runAnimation이 실행되었다.\n",this);

	if (getAniType() == aniFlag::move)//만약 플레이어 인스턴스의 좌표와 목적좌표가 다를 경우
	{
		//8프레임-4(스피드2)
		//6프레임-3(스피드3)
		//4프레임-2(스피드4)

		// 1 / 60초마다 runAnimation이 실행됨

		addTimer();

		if (getTimer() == 1)
		{
			footChanged = false;
		}

		if (getFakeX() > 0)
		{
			addFakeX(-entityInfo.gridMoveSpd);
			if (getFakeX() < 0) setFakeX(0);
		}
		else if (getFakeX() < 0)
		{
			addFakeX(+entityInfo.gridMoveSpd);
			if (getFakeX() > 0) setFakeX(0);
		}

		if (getFakeY() > 0)
		{
			addFakeY(-entityInfo.gridMoveSpd);
			if (getFakeY() < 0) setFakeY(0);
		}
		else if (getFakeY() < 0)
		{
			addFakeY(+entityInfo.gridMoveSpd);
			if (getFakeY() > 0) setFakeY(0);
		}


		if (entityInfo.isPlayer)
		{
			cameraFix = false;
			cameraX = getX() + getIntegerFakeX();
			cameraY = getY() + getIntegerFakeY();
		}

		if (std::abs(getIntegerFakeX()) <= 8.0 || std::abs(getIntegerFakeY()) <= 8.0)
		{
			if (footChanged == false)
			{
				if (getLeftFoot() == true)
				{
					setSpriteIndex(1);
					setLeftFoot(false);
				}
				else
				{
					setSpriteIndex(2);
					setLeftFoot(true);
				}
				footChanged = true;
			}
		}

		if (std::abs(getIntegerFakeX()) == 0.0 && std::abs(getIntegerFakeY()) == 0.0)
		{
			setSpriteIndex(0);
			resetTimer();
			setAniType(aniFlag::null);
			setFakeX(0);
			setFakeY(0);

			
			endMove();
			if(turnCycle == turn::playerAnime) turnWait(1.0);
			if (entityInfo.isPlayer) cameraFix = true;
			return true;
		}
	}
	else if (getAniType() == aniFlag::atk)
	{
		addTimer();

		char dx;
		char dy;

		static bool dualAtk = false;
		static bool leftAtk = false;
		static bool rightAtk = false;
		static bool twoHandedAtk = false;
		static bool unarmedAtk = false;
		static equipHandFlag prevDualAtk = equipHandFlag::right;
		

		if (getTimer() == 1)
		{
			if (entityInfo.isPlayer)
			{
				dualAtk = false;
				leftAtk = false;
				rightAtk = false;
				twoHandedAtk = false;
				unarmedAtk = false;

				bool findLeft = false;
				bool findRight = false;
				bool findTwoHanded = false;

				auto& equip = PlayerPtr->getEquipPtr()->itemInfo;
				for (int i = 0; i < equip.size(); i++)
				{
					if (equip[i].equipState == equipHandFlag::left && !equip[i].checkFlag(itemFlag::SHIELD)) findLeft = true;
					else if (equip[i].equipState == equipHandFlag::right && !equip[i].checkFlag(itemFlag::SHIELD)) findRight = true;
					else if (equip[i].equipState == equipHandFlag::both) findTwoHanded = true;
				}

				if (findLeft == true && findRight == true) dualAtk = true;
				else if (findLeft == true) leftAtk = true;
				else if (findRight == true) rightAtk = true;
				else if (findTwoHanded == true) twoHandedAtk = true;
				else unarmedAtk = true;
			}
		}


		switch (entityInfo.direction)
		{
		case 0: dx = 1; dy = 0; break;
		case 1: dx = 1; dy = -1; break;
		case 2: dx = 0; dy = -1; break;
		case 3: dx = -1; dy = -1; break;
		case 4: dx = -1; dy = 0; break;
		case 5: dx = -1; dy = 1; break;
		case 6: dx = 0; dy = 1; break;
		case 7: dx = 1; dy = 1; break;
		}

		Entity* address = TileEntity(atkTarget.x, atkTarget.y, atkTarget.z);
		std::wstring stickerID = L"BASEATK" + std::to_wstring((unsigned __int64)this);

		if (shutdown == true)//사망으로 인한 강제종료
		{
			if (address != nullptr)
			{
				address->flash.a = 0;
			}
			aniUSet.erase(aniUSet.find(this));
			delete(((Sticker*)(StickerList.find(stickerID))->second));
			setFakeX(0);
			setFakeY(0);
			return true;
		}

		switch (getTimer())
		{
		case 1:
			if (entityInfo.isPlayer)
			{
				if (dualAtk)
				{
					if (prevDualAtk == equipHandFlag::right) PlayerPtr->setSpriteIndex(charSprIndex::RATK1);
					else  PlayerPtr->setSpriteIndex(charSprIndex::LATK1);
				}
				else if (leftAtk) PlayerPtr->setSpriteIndex(charSprIndex::LATK1);
				else if (rightAtk) PlayerPtr->setSpriteIndex(charSprIndex::RATK1);
				else if (twoHandedAtk) PlayerPtr->setSpriteIndex(charSprIndex::MINING1);
				else if (unarmedAtk) PlayerPtr->setSpriteIndex(charSprIndex::LATK1);
			}
			else
			{
				if(entityInfo.atkSpr1 != -1) setSpriteIndex(entityInfo.atkSpr1);
			}
			break;
		case 3:
			setFakeX(getFakeX() + 2.5 * dx);
			setFakeY(getFakeY() + 2.5 * dy);
			break;
		case 4:
			setFakeX(getFakeX() + 2.0 * dx);
			setFakeY(getFakeY() + 2.0 * dy);
			break;
		case 5:
			setFakeX(getFakeX() + 1.5 * dx);
			setFakeY(getFakeY() + 1.5 * dy);
			break;
		case 6:
			setFakeX(getFakeX() + 1.0 * dx);
			setFakeY(getFakeY() + 1.0 * dy);
			break;
		case 7:
			setFakeX(getFakeX() + 0.5 * dx);
			setFakeY(getFakeY() + 0.5 * dy);
			if (address != nullptr)
			{
				address->setFakeX(address->getIntegerFakeX() + 2 * dx);
				address->setFakeY(address->getIntegerFakeY() + 2 * dy);
			}
			attack(atkTarget.x, atkTarget.y);
			new Sticker(false, getX() + (16 * (atkTarget.x - getGridX())), getY() + (16 * (atkTarget.y - getGridY())), spr::effectCut1, 0, stickerID, true);
			if (entityInfo.isPlayer)
			{
				if (dualAtk)
				{
					if (prevDualAtk == equipHandFlag::right)
					{
						PlayerPtr->setSpriteIndex(charSprIndex::RATK2);
						prevDualAtk = equipHandFlag::left;
					}
					else
					{
						PlayerPtr->setSpriteIndex(charSprIndex::LATK2);
						prevDualAtk = equipHandFlag::right;
					}
				}
				else if (leftAtk) PlayerPtr->setSpriteIndex(charSprIndex::LATK2);
				else if (rightAtk) PlayerPtr->setSpriteIndex(charSprIndex::RATK2);
				else if (twoHandedAtk) PlayerPtr->setSpriteIndex(charSprIndex::MINING2);
				else if (unarmedAtk) PlayerPtr->setSpriteIndex(charSprIndex::LATK2);
			}
			else
			{
				if (entityInfo.atkSpr2 != -1) setSpriteIndex(entityInfo.atkSpr2);
			}
			break;
		case 9:
			setFakeX(getFakeX() - 0.5 * dx);
			setFakeY(getFakeY() - 0.5 * dy);
			if (address != nullptr)
			{
				address->setFakeX(address->getIntegerFakeX() - 1 * dx);
				address->setFakeY(address->getIntegerFakeY() - 1 * dy);
			}
			break;
		case 10:
			setFakeX(getFakeX() - 1.0 * dx);
			setFakeY(getFakeY() - 1.0 * dy);
			break;
		case 11:
			setFakeX(getFakeX() - 1.5 * dx);
			setFakeY(getFakeY() - 1.5 * dy);
			if (address != nullptr)
			{
				address->setFakeX(address->getIntegerFakeX() - 1 * dx);
				address->setFakeY(address->getIntegerFakeY() - 1 * dy);
			}
			((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(1);
			break;
		case 12:
			setFakeX(getFakeX() - 2.0 * dx);
			setFakeY(getFakeY() - 2.0 * dy);
			break;
		case 13:
			setFakeX(getFakeX() - 2.5 * dx);
			setFakeY(getFakeY() - 2.5 * dy);
			break;
		case 15:
			((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(2);
			break;
		case 17:
			delete(((Sticker*)(StickerList.find(stickerID))->second));
		case 19:
			setFakeX(0);
			setFakeY(0);
			resetTimer();
			setAniType(aniFlag::null);
			if (entityInfo.isPlayer)
			{
				PlayerPtr->setSpriteIndex(charSprIndex::WALK);
			}
			else setSpriteIndex(0);
			if (entityInfo.isPlayer == true) { turnWait(endAtk()); }
			else { endAtk(); }
			return true;
		}
	}
	else if (getAniType() == aniFlag::felling)
	{
		addTimer();

		char dx;
		char dy;

		switch (entityInfo.direction)
		{
		case 0: dx = 1; dy = 0; break;
		case 1: dx = 1; dy = -1; break;
		case 2: dx = 0; dy = -1; break;
		case 3: dx = -1; dy = -1; break;
		case 4: dx = -1; dy = 0; break;
		case 5: dx = -1; dy = 1; break;
		case 6: dx = 0; dy = 1; break;
		case 7: dx = 1; dy = 1; break;
		}

		Prop* address = TileProp(PlayerX()+dx, PlayerY()+dy, PlayerZ());
		std::wstring stickerID = L"BASEATK" + std::to_wstring((unsigned __int64)this);

		if (shutdown == true)//사망으로 인한 강제종료
		{
			aniUSet.erase(aniUSet.find(this));
			delete(((Sticker*)(StickerList.find(stickerID))->second));
			setFakeX(0);
			setFakeY(0);
			return true;
		}

		switch (getTimer())
		{
		case 1:
			PlayerPtr->setSpriteIndex(charSprIndex::MINING1);
			break;
		case 3:
			setFakeX(getFakeX() + 2.5 * dx);
			setFakeY(getFakeY() + 2.5 * dy);
			break;
		case 4:
			setFakeX(getFakeX() + 2.0 * dx);
			setFakeY(getFakeY() + 2.0 * dy);
			break;
		case 5:
			setFakeX(getFakeX() + 1.5 * dx);
			setFakeY(getFakeY() + 1.5 * dy);
			break;
		case 6:
			setFakeX(getFakeX() + 1.0 * dx);
			setFakeY(getFakeY() + 1.0 * dy);
			break;
		case 7:
			setFakeX(getFakeX() + 0.5 * dx);
			setFakeY(getFakeY() + 0.5 *dy);
			if (address != nullptr)
			{
				address->setFakeX(address->getIntegerFakeX() + 2 * dx);
				address->setFakeY(address->getIntegerFakeY() + 2 * dy);
			}
			PlayerPtr->setSpriteIndex(charSprIndex::MINING2);
			address->leadItem.propHP -= 180;
			address->displayHPBarCount = 100;
			address->alphaHPBar = 255;
			address->alphaFakeHPBar = 255;

			if (address->leadItem.propHP <= 0)
			{
				if (address->leadItem.checkFlag(itemFlag::STUMP) == false)
				{
					addAniUSetPlayer(address, aniFlag::treeFalling);
					address->displayHPBarCount = 50;
					address->leadItem.eraseFlag(itemFlag::PROP_BLOCKER);
					PlayerPtr->updateVision(PlayerPtr->entityInfo.eyeSight);
				}
				else
				{
					address->displayHPBarCount = 50;
				}
			}

			new Sticker(false, getX() + (16 * dx), getY() + (16 * dy), spr::effectCut1, 0, stickerID, true);
			break;
		case 9:
			setFakeX(getIntegerFakeX() - 0.5 * dx);
			setFakeY(getIntegerFakeY() - 0.5 * dy);
			if (address != nullptr)
			{
				address->setFakeX(address->getIntegerFakeX() - 1 * dx);
				address->setFakeY(address->getIntegerFakeY() - 1 * dy);
			}
			break;
		case 10:
			setFakeX(getFakeX() - 1.0 * (float)dx);
			setFakeY(getIntegerFakeY() - 1.0 * (float)dy);
			break;
		case 11:
			setFakeX(getFakeX() - 1.5 * dx);
			setFakeY(getFakeY() - 1.5 * dy);
			if (address != nullptr)
			{
				address->setFakeX(address->getIntegerFakeX() - 1 * dx);
				address->setFakeY(address->getIntegerFakeY() - 1 * dy);
			}
			((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(1);
			break;
		case 12:
			setFakeX(getFakeX() - 2.0 * (float)dx);
			setFakeY(getFakeY() - 2.0 * (float)dy);
			break;
		case 13:
			setFakeX(getFakeX() - 2.5 * dx);
			setFakeY(getFakeY() - 2.5 * dy);
			break;
		case 15:
			((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(2);
			break;
		case 17:
			delete(((Sticker*)(StickerList.find(stickerID))->second));
		case 19:
			setFakeX(0);
			setFakeY(0);
			resetTimer();
			setAniType(aniFlag::null);
			PlayerPtr->setSpriteIndex(charSprIndex::WALK_2H);
			if (entityInfo.isPlayer == true) { turnWait(1.0); }
			else {  }
			return true;
		}
	}
	else if (getAniType() == aniFlag::miningWall)
	{
		addTimer();

		char dx;
		char dy;

		switch (entityInfo.direction)
		{
		case 0: dx = 1; dy = 0; break;
		case 1: dx = 1; dy = -1; break;
		case 2: dx = 0; dy = -1; break;
		case 3: dx = -1; dy = -1; break;
		case 4: dx = -1; dy = 0; break;
		case 5: dx = -1; dy = 1; break;
		case 6: dx = 0; dy = 1; break;
		case 7: dx = 1; dy = 1; break;
		}

		std::wstring stickerID = L"BASEATK" + std::to_wstring((unsigned __int64)this);

		if (shutdown == true)//사망으로 인한 강제종료
		{
			aniUSet.erase(aniUSet.find(this));
			delete(((Sticker*)(StickerList.find(stickerID))->second));
			setFakeX(0);
			setFakeY(0);
			return true;
		}

		TileData& tile = World::ins()->getTile(PlayerX() + dx, PlayerY() + dy, PlayerZ());

		switch (getTimer())
		{
		case 1:
			PlayerPtr->setSpriteIndex(charSprIndex::MINING1);
			break;
		case 3:
			setFakeX(getFakeX() + 2.5 * dx);
			setFakeY(getFakeY() + 2.5 * dy);
			break;
		case 4:
			setFakeX(getFakeX() + 2.0 * dx);
			setFakeY(getFakeY() + 2.0 * dy);
			break;
		case 5:
			setFakeX(getFakeX() + 1.5 * dx);
			setFakeY(getFakeY() + 1.5 * dy);
			break;
		case 6:
			setFakeX(getFakeX() + 1.0 * dx);
			setFakeY(getFakeY() + 1.0 * dy);
			break;
		case 7:
			setFakeX(getFakeX() + 0.5 * dx);
			setFakeY(getFakeY() + 0.5 * dy);
			PlayerPtr->setSpriteIndex(charSprIndex::MINING2);
			tile.wallHP -= 40;
			tile.displayHPBarCount = 100;
			tile.alphaHPBar = 255;
			tile.alphaFakeHPBar = 255;

			if (tile.wallHP <= 0)
			{
				tile.displayHPBarCount = 50;


				if (TileWall(PlayerX() + dx, PlayerY() + dy, PlayerZ()) == itemRefCode::dirtWall)
				{
					if(randomRange(0,100)<=25) createItemStack({ PlayerX() + dx, PlayerY() + dy, PlayerZ() }, { {396,1} });
					for (int i = 0; i < 8; i++)
					{
						new Particle(getX() + 16 * dx + randomRange(-3, 3), getY() + 16 * dy + 4 + randomRange(-3, 3), randomRange(24, 31), randomRangeFloat(-1.2, 1.2), randomRangeFloat(-2.6, -3.2), 0.18, randomRange(25, 35));
					}
				}
				else if (TileWall(PlayerX() + dx, PlayerY() + dy, PlayerZ()) == itemRefCode::stoneWall)
				{
					if (randomRange(0, 100) <= 25) createItemStack({ PlayerX() + dx, PlayerY() + dy, PlayerZ() }, { {398,1} });
					for (int i = 0; i < 8; i++)
					{
						new Particle(getX() + 16 * dx + randomRange(-3, 3), getY() + 16 * dy + 4 + randomRange(-3, 3), randomRange(8, 15), randomRangeFloat(-1.2, 1.2), randomRangeFloat(-2.6, -3.2), 0.18, randomRange(25, 35));
					}
				}
				else if (TileWall(PlayerX() + dx, PlayerY() + dy, PlayerZ()) == itemRefCode::glassWall)
				{
					for (int i = 0; i < 8; i++)
					{
						new Particle(getX() + 16 * dx + randomRange(-3, 3), getY() + 16 * dy + 4 + randomRange(-3, 3), randomRange(32, 39), randomRangeFloat(-1.2, 1.2), randomRangeFloat(-2.6, -3.2), 0.18, randomRange(25, 35));
					}
				}
				else if (TileWall(PlayerX() + dx, PlayerY() + dy, PlayerZ()) == itemRefCode::wireFence)
				{
					for (int i = 0; i < 8; i++)
					{
						new Particle(getX() + 16 * dx + randomRange(-3, 3), getY() + 16 * dy + 4 + randomRange(-3, 3), randomRange(40, 47), randomRangeFloat(-1.2, 1.2), randomRangeFloat(-2.6, -3.2), 0.18, randomRange(25, 35));
					}
				}
				else
				{
					for (int i = 0; i < 8; i++)
					{
						new Particle(getX() + 16 * dx + randomRange(-3, 3), getY() + 16 * dy + 4 + randomRange(-3, 3), randomRange(8, 15), randomRangeFloat(-1.2, 1.2), randomRangeFloat(-2.6, -3.2), 0.18, randomRange(25, 35));
					}
				}

				ItemStack* itemPtr = TileItemStack(PlayerX() + dx, PlayerY() + dy, PlayerZ());
				if(itemPtr != nullptr) addAniUSetPlayer(itemPtr, aniFlag::drop);

				DestroyWall(PlayerX() + dx, PlayerY() + dy, PlayerZ());
				PlayerPtr->updateVision(PlayerPtr->entityInfo.eyeSight);
			}
			new Sticker(false, getX() + (16 * dx), getY() + (16 * dy), spr::effectCut1, 0, stickerID, true);
			break;
		case 9:
			setFakeX(getIntegerFakeX() - 0.5 * dx);
			setFakeY(getIntegerFakeY() - 0.5 * dy);
			break;
		case 10:
			setFakeX(getFakeX() - 1.0 * (float)dx);
			setFakeY(getIntegerFakeY() - 1.0 * (float)dy);
			break;
		case 11:
			setFakeX(getFakeX() - 1.5 * dx);
			setFakeY(getFakeY() - 1.5 * dy);
			((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(1);
			break;
		case 12:
			setFakeX(getFakeX() - 2.0 * (float)dx);
			setFakeY(getFakeY() - 2.0 * (float)dy);
			break;
		case 13:
			setFakeX(getFakeX() - 2.5 * dx);
			setFakeY(getFakeY() - 2.5 * dy);
			break;
		case 15:
			((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(2);
			break;
		case 17:
			delete(((Sticker*)(StickerList.find(stickerID))->second));
		case 19:
			setFakeX(0);
			setFakeY(0);
			resetTimer();
			setAniType(aniFlag::null);
			PlayerPtr->setSpriteIndex(charSprIndex::WALK_2H);
			if (entityInfo.isPlayer == true) { turnWait(1.0); }
			else {}
			return true;
		}
	}
	else if (getAniType() == aniFlag::throwing)
	{
		//거리에 따라 적이 피격하는데에 걸리는 시간을 길게 만들 것
		addTimer();

		char dx;
		char dy;

		switch (entityInfo.direction)
		{
		case 0: dx = 1; dy = 0; break;
		case 1: dx = 1; dy = -1; break;
		case 2: dx = 0; dy = -1; break;
		case 3: dx = -1; dy = -1; break;
		case 4: dx = -1; dy = 0; break;
		case 5: dx = -1; dy = 1; break;
		case 6: dx = 0; dy = 1; break;
		case 7: dx = 1; dy = 1; break;
		}

		Entity* address = TileEntity(atkTarget.x, atkTarget.y, atkTarget.z);
		std::wstring stickerID = L"BASEATK" + std::to_wstring((unsigned __int64)this);

		if (shutdown == true)//사망으로 인한 강제종료
		{
			if (address != nullptr)
			{
				address->flash.a = 0;
			}
			aniUSet.erase(aniUSet.find(this));
			delete(((Sticker*)(StickerList.find(stickerID))->second));
			setFakeX(0);
			setFakeY(0);
			return true;
		}

		switch (getTimer())
		{
		case 2:
			setFakeX(getIntegerFakeX() + 3 * dx);
			setFakeY(getIntegerFakeY() + 3 * dy);
			break;
		case 3:
			setFakeX(getIntegerFakeX() + 2 * dx);
			setFakeY(getIntegerFakeY() + 2 * dy);
			break;
		case 4:
			setFakeX(getIntegerFakeX() + 1 * dx);
			setFakeY(getIntegerFakeY() + 1 * dy);
			if (address != nullptr)
			{
				address->setFakeX(address->getIntegerFakeX() + 2 * dx);
				address->setFakeY(address->getIntegerFakeY() + 2 * dy);
			}

			attack(atkTarget.x, atkTarget.y);
			new Sticker(false, getX() + (16 * (atkTarget.x - getGridX())), getY() + (16 * (atkTarget.y - getGridY())), spr::effectCut1, 0, stickerID, true);
			break;
		case 5:
			setFakeX(getIntegerFakeX() - 1 * dx);
			setFakeY(getIntegerFakeY() - 1 * dy);
			if (address != nullptr)
			{
				address->setFakeX(address->getIntegerFakeX() - 1 * dx);
				address->setFakeY(address->getIntegerFakeY() - 1 * dy);
			}
			break;
		case 6:
			setFakeX(getIntegerFakeX() - 2 * dx);
			setFakeY(getIntegerFakeY() - 2 * dy);
			if (address != nullptr)
			{
				address->setFakeX(address->getIntegerFakeX() - 1 * dx);
				address->setFakeY(address->getIntegerFakeY() - 1 * dy);
			}
			((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(1);
			break;
		case 7:
			setFakeX(getIntegerFakeX() - 3 * dx);
			setFakeY(getIntegerFakeY() - 3 * dy);
			break;
		case 8:
			((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(2);
			break;
		case 10:
			delete(((Sticker*)(StickerList.find(stickerID))->second));
		case 20:
			setFakeX(0);
			setFakeY(0);
			resetTimer();
			setAniType(aniFlag::null);
			if (entityInfo.isPlayer == true) { turnWait(endAtk()); }
			else { endAtk(); }
			return true;
		}
	}
	else if (getAniType() == aniFlag::shotSingle)
	{
		addTimer();
		static Bullet* bulletPtr = nullptr;

		int dx, dy;
		dir2Coord(entityInfo.direction, dx, dy);

		Entity* ePtr = TileEntity(atkTarget.x, atkTarget.y, atkTarget.z);
		std::wstring stickerID = L"BASEATK" + std::to_wstring((unsigned __int64)this);

		if (shutdown == true)//사망으로 인한 강제종료
		{
			if (ePtr != nullptr) ePtr->flash.a = 0;
			aniUSet.erase(aniUSet.find(this));
			delete(((Sticker*)(StickerList.find(stickerID))->second));
			setFakeX(0);
			setFakeY(0);
			return true;
		}

		float spd = 7.0;
		float delX = 16.0 * (atkTarget.x - getGridX());
		float delY = 16.0 * (atkTarget.y - getGridY());
		float dist = std::sqrt(std::pow(delX, 2) + std::pow(delY, 2));
		float cosVal = delX / dist;
		float sinVal = delY / dist;
		float xSpd = spd * cosVal;
		float ySpd = spd * sinVal;
		static int hitTimer = -1;
		if (getTimer() == 1)
		{
			bulletPtr = new Bullet(getX(), getY());
			bulletPtr->sprite = spr::bulletset;
			bulletPtr->sprIndex = 0 + del2Dir(delX, delY);
		}

		if (bulletPtr != nullptr)
		{
			bulletPtr->addFakeX(xSpd);
			bulletPtr->addFakeY(ySpd);


			std::wprintf(L"bullet fake X:%f, fake Y:%f, delX :%f, delY: %f\n", bulletPtr->getFakeX(), bulletPtr->getFakeY(), delX, delY);
			if (std::fabs(bulletPtr->getFakeX()) >= std::fabs(delX) && std::fabs(bulletPtr->getFakeY()) >= std::fabs(delY))
			{
				delete bulletPtr;
				bulletPtr = nullptr;
				hitTimer = getTimer();
			}
		}


		switch (getTimer())
		{
		case 2:

			setFakeX(getIntegerFakeX() - 2 * dx);
			setFakeY(getIntegerFakeY() - 2 * dy);
			break;
		case 3:
			setFakeX(getIntegerFakeX() - 1 * dx);
			setFakeY(getIntegerFakeY() - 1 * dy);
			break;
		case 5:
			setFakeX(getIntegerFakeX() + 1 * dx);
			setFakeY(getIntegerFakeY() + 1 * dy);
			break;
		case 6:
			setFakeX(getIntegerFakeX() + 2 * dx);
			setFakeY(getIntegerFakeY() + 2 * dy);
			break;
		}

		if (bulletPtr == nullptr)
		{
			if (getTimer() == hitTimer)
			{
				if (ePtr != nullptr)
				{
					ePtr->setFakeX(ePtr->getIntegerFakeX() + 2 * dx);
					ePtr->setFakeY(ePtr->getIntegerFakeY() + 2 * dy);
				}
				attack(atkTarget.x, atkTarget.y);
				new Sticker(false, getX() + (16 * (atkTarget.x - getGridX())), getY() + (16 * (atkTarget.y - getGridY())), spr::effectCut1, 0, stickerID, true);
			}
			else if (getTimer() == hitTimer + 1)
			{
				if (ePtr != nullptr)
				{
					ePtr->setFakeX(ePtr->getIntegerFakeX() - 1 * dx);
					ePtr->setFakeY(ePtr->getIntegerFakeY() - 1 * dy);
				}
			}
			else if (getTimer() == hitTimer + 2)
			{
				if (ePtr != nullptr)
				{
					ePtr->setFakeX(ePtr->getIntegerFakeX() - 1 * dx);
					ePtr->setFakeY(ePtr->getIntegerFakeY() - 1 * dy);
				}
				((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(1);
			}
			else if (getTimer() == hitTimer + 4)
			{
				((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(2);
			}
			else if (getTimer() == hitTimer + 6)
			{
				delete(((Sticker*)(StickerList.find(stickerID))->second));
			}
			else if (getTimer() == hitTimer + 8)
			{
				setFakeX(0);
				setFakeY(0);
				resetTimer();
				setAniType(aniFlag::null);

				if (Aim::ins() != nullptr)
				{
					Aim::ins()->close(aniFlag::null);
				}

				if (entityInfo.isPlayer == true) { turnWait(endAtk()); }
				else { endAtk(); }
				return true;
			}

		}
	}
	else if (getAniType() == aniFlag::fireStorm)
	{
		addTimer();

		std::wstring stickerID = L"FIRESTORM";

		switch (getTimer())
		{
		case 1:
		{
			new Sticker(false, getX() + (16 * (getSkillTarget().x - getGridX())), getY() + (16 * (getSkillTarget().y - getGridY())), spr::fireStorm, 0, stickerID, true);
			createFlame({ getSkillTarget().x,getSkillTarget().y,getGridZ() }, flameFlag::BIG);
			break;
		}
		case 5:
		{
			((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(1);
			
			createFlame({ getSkillTarget().x + 1, getSkillTarget().y, getGridZ() }, flameFlag::NORMAL);
			createFlame({ getSkillTarget().x - 1, getSkillTarget().y, getGridZ() }, flameFlag::NORMAL);
			createFlame({ getSkillTarget().x, getSkillTarget().y + 1, getGridZ() }, flameFlag::NORMAL);
			createFlame({ getSkillTarget().x, getSkillTarget().y - 1, getGridZ() }, flameFlag::NORMAL);
			break;
		}
		case 9:
		{
			((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(2);

			createFlame({ getSkillTarget().x, getSkillTarget().y - 2, getGridZ() }, flameFlag::SMALL);
			createFlame({ getSkillTarget().x - 1, getSkillTarget().y - 1, getGridZ() }, flameFlag::SMALL);
			createFlame({ getSkillTarget().x - 2, getSkillTarget().y, getGridZ() }, flameFlag::SMALL);
			createFlame({ getSkillTarget().x - 1, getSkillTarget().y + 1, getGridZ() }, flameFlag::SMALL);
			createFlame({ getSkillTarget().x, getSkillTarget().y + 2, getGridZ() }, flameFlag::SMALL);
			createFlame({ getSkillTarget().x + 1, getSkillTarget().y + 1, getGridZ() }, flameFlag::SMALL);
			createFlame({ getSkillTarget().x + 2, getSkillTarget().y, getGridZ() }, flameFlag::SMALL);
			createFlame({ getSkillTarget().x + 1, getSkillTarget().y - 1, getGridZ() }, flameFlag::SMALL);
			break;
		}
		case 13:
			((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(3);
			break;
		case 17:
			((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(4);
			break;
		case 21:
			((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(5);
			break;
		case 25:
			((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(6);
			break;
		case 29:
			((Sticker*)(StickerList.find(stickerID))->second)->setSpriteIndex(7);
			break;
		case 33:
			delete(((Sticker*)(StickerList.find(stickerID))->second));
			resetTimer();
			setAniType(aniFlag::null);
			return true;
			break;
		}
	}
	else if (getAniType() == aniFlag::entityThrow)
	{
		addTimer();

		static int arriveTimer = 0;

		Point3 dstGrid = { throwCoord.x,throwCoord.y,throwCoord.z };
		static Point3 prevCoor;

		Sticker* sPtr = nullptr;
		std::wstring stickerID = L"THROW" + std::to_wstring((unsigned __int64)this);
		if (getTimer() == 1)
		{
			int gX = getGridX();
			int gY = getGridY();
			int gZ = getGridZ();
			int spriteIndex = throwingItemPocket->itemInfo.size() > 0 ? getItemSprIndex(throwingItemPocket->itemInfo.back()) : 0;
			Sticker* sPtr = new Sticker(false, getX(), getY(), spr::itemset, spriteIndex, stickerID, true);
			arriveTimer = 0;
			prevCoor = { getGridX(),getGridY(),getGridZ() };
		}

		float spd = 4.0;
		int relX = 16 * (dstGrid.x - getGridX());
		int relY = 16 * (dstGrid.y - getGridY());
		float dist = std::sqrt(std::pow(relX, 2) + std::pow(relY, 2));
		float xSpd, ySpd;

		if (StickerList.find(stickerID) != StickerList.end()) sPtr = ((Sticker*)(StickerList.find(stickerID))->second);
		else sPtr = nullptr;

		if (sPtr != nullptr && arriveTimer == 0 && (relX != 0 || relY != 0))
		{
			float cosVal = relX / dist;
			float sinVal = relY / dist;
			xSpd = spd * cosVal;
			ySpd = spd * sinVal;
			sPtr->addFakeX(xSpd);
			sPtr->addFakeY(ySpd);

			Point3 cGrid = sPtr->getClosestGridWithFake();
			if (cGrid != prevCoor)
			{
				prevCoor = cGrid;
				if (throwingItemPocket->itemInfo.size() > 0 && throwingItemPocket->itemInfo[0].lightPtr != nullptr)
				{
					throwingItemPocket->itemInfo[0].lightPtr.get()->moveLight(cGrid.x, cGrid.y, getGridZ());
					PlayerPtr->updateVision();
				}
			}
		}

		if (arriveTimer != 0 || sPtr == nullptr || (relX == 0 && relY == 0) || (std::abs(sPtr->getFakeX()) >= std::abs(relX) && std::abs(sPtr->getFakeY()) >= std::abs(relY)))
		{
			if (arriveTimer == 0) arriveTimer = getTimer();

			ItemStack* targetStack;

			bool throwToProp = false;
			Prop* propPtr = TileProp(dstGrid.x, dstGrid.y, dstGrid.z);
			int totalVolume = 0;
			for (int i = 0; i < throwingItemPocket->itemInfo.size(); i++)
			{
				totalVolume += getVolume(throwingItemPocket->itemInfo[i]) * throwingItemPocket->itemInfo[i].number;
			}
			throwToProp = propPtr != nullptr
				&& propPtr->leadItem.pocketPtr != nullptr
				&& propPtr->leadItem.pocketPtr->getPocketVolume() + totalVolume < propPtr->leadItem.pocketMaxVolume;

			Vehicle* vPtr = TileVehicle(dstGrid.x, dstGrid.y, dstGrid.z);
			bool throwToVehicle = false;
			ItemPocket* throwTargetPocket = nullptr;
			if (vPtr != nullptr)
			{
				ItemPocket* vParts = vPtr->partInfo[{ dstGrid.x, dstGrid.y }].get();
				for (int i = vParts->itemInfo.size() - 1; i >= 0; i--)
				{
					if (vParts->itemInfo[i].pocketMaxVolume > vParts->getPocketVolume() + totalVolume)
					{
						throwToVehicle = true;
						throwTargetPocket = vParts->itemInfo[i].pocketPtr.get();
						break;
					}
				}
			}

			if (throwToVehicle)
			{
				static int arriveFakeX = 0;
				static int arriveFakeY = 0;
				if (getTimer() == arriveTimer)
				{
					arriveFakeX = sPtr->getFakeX();
					arriveFakeY = sPtr->getFakeY();
					sPtr->setFakeY(arriveFakeY - 4);
				}
				else if (getTimer() == arriveTimer + 1) sPtr->setFakeY(arriveFakeY - 5);
				else if (getTimer() == arriveTimer + 3) sPtr->setFakeY(arriveFakeY - 6);
				else if (getTimer() == arriveTimer + 6) sPtr->setFakeY(arriveFakeY - 7);
				else if (getTimer() == arriveTimer + 9) sPtr->setFakeY(arriveFakeY - 6);
				else if (getTimer() == arriveTimer + 11) sPtr->setFakeY(arriveFakeY - 5);
				else if (getTimer() == arriveTimer + 12) sPtr->setFakeY(arriveFakeY - 4);
				else if (getTimer() >= arriveTimer + 15)
				{
					sPtr->setFakeY(arriveFakeY);
					while (throwingItemPocket->itemInfo.size() > 0)
					{
						int itemCount = throwingItemPocket->itemInfo[0].number;
						throwingItemPocket->transferItem(throwTargetPocket, 0, itemCount);
					}

					delete sPtr;
					resetTimer();
					setAniType(aniFlag::null);
					if (entityInfo.isPlayer == true) { turnWait(1.0); }
					return true;
				}
			}
			else if (throwToProp)
			{
				if (getTimer() == arriveTimer)
				{
					propPtr->setFakeY(-4);
					delete sPtr;
				}
				else if (getTimer() == arriveTimer + 1) propPtr->setFakeY(-5);
				else if (getTimer() == arriveTimer + 3) propPtr->setFakeY(-6);
				else if (getTimer() == arriveTimer + 6) propPtr->setFakeY(-7);
				else if (getTimer() == arriveTimer + 9) propPtr->setFakeY(-6);
				else if (getTimer() == arriveTimer + 11) propPtr->setFakeY(-5);
				else if (getTimer() == arriveTimer + 12) propPtr->setFakeY(-4);
				else if (getTimer() >= arriveTimer + 15)
				{
					propPtr->setFakeY(0);
					while (throwingItemPocket->itemInfo.size() > 0)
					{
						int itemCount = throwingItemPocket->itemInfo[0].number;
						throwingItemPocket->transferItem(propPtr->leadItem.pocketPtr.get(), 0, itemCount);
					}

					resetTimer();
					setAniType(aniFlag::null);
					if (entityInfo.isPlayer == true) { turnWait(1.0); }
					return true;
				}
			}
			else if (TileItemStack(dstGrid.x, dstGrid.y, dstGrid.z) == nullptr) //그 자리에 템 없는 경우
			{
				//기존 스택이 없으면 새로 만들고 그 ptr을 전달

				createItemStack(dstGrid);
				targetStack = TileItemStack(dstGrid);
				while (throwingItemPocket->itemInfo.size() > 0)
				{
					int itemCount = throwingItemPocket->itemInfo[0].number;
					throwingItemPocket->transferItem(targetStack->getPocket(), 0, itemCount);
				}
				addAniUSetPlayer(targetStack, aniFlag::drop);

				delete sPtr;
				resetTimer();
				setAniType(aniFlag::null);
				if (entityInfo.isPlayer == true) { turnWait(1.0); }
				return true;
			}
			else //이미 그 자리에 아이템이 있는 경우
			{
				//기존 스택이 있으면 그 스택을 그대로 전달
				targetStack = TileItemStack(dstGrid);
				while (throwingItemPocket->itemInfo.size() > 0)
				{
					int itemCount = throwingItemPocket->itemInfo[0].number;
					throwingItemPocket->transferItem(targetStack->getPocket(), 0, itemCount);
				}
				addAniUSetPlayer(targetStack, aniFlag::drop);

				delete sPtr;
				resetTimer();
				setAniType(aniFlag::null);

				if (entityInfo.isPlayer == true) { turnWait(1.0); }
				return true;
			}
		}
		}
	else if (getAniType() == aniFlag::faint)
	{
		addTimer();
		int pX = getX();
		int pY = getY();

		switch (getTimer())
		{
		case 1:
			PlayerPtr->setSpriteIndex(charSprIndex::CRAWL);
			PlayerPtr->entityInfo.isEyesClose = true;
			setFakeY(-4);
			break;
		case 2:
			setFakeY(-5);
			break;
		case 4:
			setFakeY(-6);
			break;
		case 7:
			setFakeY(-7);
			break;
		case 10:
			setFakeY(-6);
			break;
		case 12:
			setFakeY(-5);
			break;
		case 13:
			setFakeY(-4);
			break;
		case 16:
			setFakeY(0);
			resetTimer();
			setAniType(aniFlag::null);
			return true;
		}
	}
	else if (getAniType() == aniFlag::roll)	
	{

		addTimer();


		if (getTimer() == 1) entityInfo.walkMode = walkFlag::crouch;
		


		if (getFakeX() > 0)
		{
			addFakeX(-entityInfo.gridMoveSpd);
			if (getFakeX() < 0) setFakeX(0);
		}
		else if (getFakeX() < 0)
		{
			addFakeX(+entityInfo.gridMoveSpd);
			if (getFakeX() > 0) setFakeX(0);
		}

		if (getFakeY() > 0)
		{
			addFakeY(-entityInfo.gridMoveSpd);
			if (getFakeY() < 0) setFakeY(0);
		}
		else if (getFakeY() < 0)
		{
			addFakeY(+entityInfo.gridMoveSpd);
			if (getFakeY() > 0) setFakeY(0);
		}


		if (entityInfo.isPlayer)
		{
			cameraFix = false;
			cameraX = getX() + getIntegerFakeX();
			cameraY = getY() + getIntegerFakeY();
		}


		if (PlayerPtr->entityInfo.sprFlip == false)
		{
			if (getTimer() == 1) entityInfo.sprAngle = 90.0f;
			if (std::abs(getIntegerFakeX()) <= 5.0 && std::abs(getIntegerFakeY()) <= 5.0) entityInfo.sprAngle = 270.0f;
			else if (std::abs(getIntegerFakeX()) <= 10.0 && std::abs(getIntegerFakeY()) <= 10.0) entityInfo.sprAngle = 180.0f;
		}
		else
		{
			if (getTimer() == 1) entityInfo.sprAngle = -90.0f;
			if (std::abs(getIntegerFakeX()) <= 5.0 && std::abs(getIntegerFakeY()) <= 5.0) entityInfo.sprAngle = -270.0f;
			else if (std::abs(getIntegerFakeX()) <= 10.0 && std::abs(getIntegerFakeY()) <= 10.0) entityInfo.sprAngle = -180.0f;
		}

		if (std::abs(getIntegerFakeX()) == 0.0 && std::abs(getIntegerFakeY()) == 0.0)
		{
			setSpriteIndex(0);
			resetTimer();
			setAniType(aniFlag::null);
			setFakeX(0);
			setFakeY(0);

			turnWait(1.0);
			entityInfo.sprAngle = 0.0f;
			endMove();
			if (entityInfo.isPlayer)
			{
				cameraFix = true;
				changePlayerWalkMode(walkFlag::walk);
			}
			return true;
		}
	}
	else if (getAniType() == aniFlag::leap)
	{
		addTimer();
		static int xDist = 0;
		static int yDist = 0;
		static float xSpd = 0;
		static float ySpd = 0;
		static float totalDistance = 0;
		static float jumpHeight = 20.0f; // 점프 최대 높이
		static int startX = 0;
		static int startY = 0;

		if (getTimer() == 1)
		{
			

			/////////////////////////////////////////////////////////////////////////////////////////////////////////////
			entityInfo.walkMode = walkFlag::crouch;
			int skillX = getSkillTarget().x;
			int skillY = getSkillTarget().y;

			int fakeX = getFakeX();
			int fakeY = getFakeY();

			startX = getGridX() + getFakeX() / 16;
			startY = getGridY() + getFakeY() / 16;

			xDist = 16 * (getSkillTarget().x - startX);
			yDist = 16 * (getSkillTarget().y - startY);
			std::wprintf(L"xDist는 %d, yDist는 %d\n", xDist, yDist);
			xSpd = xDist / 20.0f;
			ySpd = yDist / 20.0f;

			// 총 이동 거리 계산
			totalDistance = std::sqrt(xDist * xDist + yDist * yDist);
			entityInfo.jumpOffsetY = 0.0f; // 점프 오프셋 초기화
		}

		// 현재 남은 거리 계산
		float currentDistance = std::sqrt(getFakeX() * getFakeX() + getFakeY() * getFakeY());
		float progress = 1.0f - (currentDistance / totalDistance);

		// 포물선 점프 높이 계산
		entityInfo.jumpOffsetY = -4.0f * jumpHeight * progress * (1.0f - progress);

		// 스프라이트 변경
		if (progress < 0.3f)
		{
			setSpriteIndex(charSprIndex::DASH);
		}
		else if (progress > 0.7f)
		{
			setSpriteIndex(charSprIndex::LAND);
		}
		else
		{
			setSpriteIndex(charSprIndex::HOVER);
		}

		addFakeX(xSpd);
		if (xSpd > 0 && getFakeX() > 0) setFakeX(0);
		if (xSpd < 0 && getFakeX() < 0) setFakeX(0);

		addFakeY(ySpd);
		if (ySpd > 0 && getFakeY() > 0) setFakeY(0);
		if (ySpd < 0 && getFakeY() < 0) setFakeY(0);

		if (entityInfo.isPlayer)
		{
			cameraFix = false;
			cameraX = getX() + getIntegerFakeX();
			cameraY = getY() + getIntegerFakeY();
		}

		if (std::abs(getIntegerFakeX()) == 0.0 && std::abs(getIntegerFakeY()) == 0.0)
		{
			setSpriteIndex(0);
			resetTimer();
			setAniType(aniFlag::null);
			setFakeX(0);
			setFakeY(0);
			entityInfo.jumpOffsetY = 0.0f; // 점프 오프셋 초기화
			turnWait(1.0);
			endMove();
			if (entityInfo.isPlayer)
			{
				changePlayerWalkMode(walkFlag::walk);
				cameraFix = true;
				cameraX = getX();
				cameraY = getY();
			}
			return true;
		}
	}
	else if (getAniType() == aniFlag::propTurnOnOff)
	{
		return hitAnimation(shutdown, [this]()
			{
				int dx, dy;
				dir2Coord(entityInfo.direction, dx, dy);
				Prop* address = TileProp(PlayerX() + dx, PlayerY() + dy, PlayerZ());
				if (address != nullptr)
				{
					if (address->leadItem.checkFlag(itemFlag::PROP_POWER_OFF))
					{
						address->leadItem.eraseFlag(itemFlag::PROP_POWER_OFF);
						address->leadItem.addFlag(itemFlag::PROP_POWER_ON);
					}
					else if (address->leadItem.checkFlag(itemFlag::PROP_POWER_ON))
					{
						address->leadItem.eraseFlag(itemFlag::PROP_POWER_ON);
						address->leadItem.addFlag(itemFlag::PROP_POWER_OFF);
					}
				}
			});
	}
	else if (getAniType() == aniFlag::changePropDelay)
	{
		return hitAnimation(shutdown, [this]()
			{
				int dx, dy;
				dir2Coord(entityInfo.direction, dx, dy);
				Prop* address = TileProp(PlayerX() + dx, PlayerY() + dy, PlayerZ());
				if (address != nullptr)
				{
					address->delayMaxStack += 1;
					if (address->delayMaxStack >= 10) address->delayMaxStack = 0;
					address->delayStartTurn = 0.0;
				}
			});
	}

	return false;
}


//플레이어의 entityInfo.direction 방향으로 한대 치는 모션, 람다함수를 입력해 타격 시점에 원하는 함수 실행 가능 
bool Entity::hitAnimation(bool shutdown, const std::function<void()> inputFunc)
{
	addTimer();

	int dx, dy;

	static int prevSprIndex;
	bool twoHanded = false;
	if (getEquipPtr()->itemInfo[0].checkFlag(itemFlag::TWOHANDED)) twoHanded = true;
	dir2Coord(entityInfo.direction, dx, dy);

	if (shutdown == true)//사망으로 인한 강제종료
	{
		aniUSet.erase(aniUSet.find(this));
		setFakeX(0);
		setFakeY(0);
		return true;
	}

	switch (getTimer())
	{
	case 1:
		prevSprIndex = PlayerPtr->getSpriteIndex();
		if (twoHanded) PlayerPtr->setSpriteIndex(charSprIndex::MINING1);
		else PlayerPtr->setSpriteIndex(charSprIndex::RATK1);
		break;
	case 3:
		setFakeX(getFakeX() + 2.5 * dx);
		setFakeY(getFakeY() + 2.5 * dy);
		break;
	case 4:
		setFakeX(getFakeX() + 2.0 * dx);
		setFakeY(getFakeY() + 2.0 * dy);
		break;
	case 5:
		setFakeX(getFakeX() + 1.5 * dx);
		setFakeY(getFakeY() + 1.5 * dy);
		break;
	case 6:
		setFakeX(getFakeX() + 1.0 * dx);
		setFakeY(getFakeY() + 1.0 * dy);
		break;
	case 7:
		setFakeX(getFakeX() + 0.5 * dx);
		setFakeY(getFakeY() + 0.5 * dy);

		if (twoHanded) PlayerPtr->setSpriteIndex(charSprIndex::MINING2);
		else PlayerPtr->setSpriteIndex(charSprIndex::RATK2);

		inputFunc();
		break;
	case 9:
		setFakeX(getIntegerFakeX() - 0.5 * dx);
		setFakeY(getIntegerFakeY() - 0.5 * dy);
		break;
	case 10:
		setFakeX(getFakeX() - 1.0 * (float)dx);
		setFakeY(getIntegerFakeY() - 1.0 * (float)dy);
		break;
	case 11:
		setFakeX(getFakeX() - 1.5 * dx);
		setFakeY(getFakeY() - 1.5 * dy);
		break;
	case 12:
		setFakeX(getFakeX() - 2.0 * (float)dx);
		setFakeY(getFakeY() - 2.0 * (float)dy);
		break;
	case 13:
		setFakeX(getFakeX() - 2.5 * dx);
		setFakeY(getFakeY() - 2.5 * dy);
		break;
	case 15:
		break;
	case 17:
	case 19:
		setFakeX(0);
		setFakeY(0);
		resetTimer();
		setAniType(aniFlag::null);
		PlayerPtr->setSpriteIndex(prevSprIndex);
		if (entityInfo.isPlayer == true) { turnWait(1.0); }
		else {}
		return true;
	}

	return false;
}