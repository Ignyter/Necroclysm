import HUD;

import std;
import util;
import globalVar;
import wrapVar;
import Vehicle;
import log;
import Dialogue;
import Loot;

void HUD::tileTouch(int touchX, int touchY) //일반 타일 터치
{


	if (ctrlVeh == nullptr && currentUsingSkill ==-1)//차량 조종 중이 아닐 경우
	{
		//화면에 있는 아이템 터치
		if (touchX == PlayerX() && touchY == PlayerY()) //자신 위치 터치
		{
			if (TileItemStack(touchX, touchY, PlayerZ()) != nullptr)
			{
				prt(L"루팅창 오픈 함수 실행\n");
				ItemStack* targetStack = TileItemStack(PlayerX(), PlayerY(), PlayerZ());
				new Loot(targetStack);
				click = false;
			}
			else if (TileVehicle(touchX, touchY, PlayerZ()) != nullptr)
			{
				Vehicle* belowVehicle = TileVehicle(touchX, touchY, PlayerZ());
				bool findController = false;
				prt(L"below prop의 사이즈는 %d이다.\n", belowVehicle->partInfo[{touchX, touchY}]->itemInfo.size());
				for (int i = 0; i < belowVehicle->partInfo[{touchX, touchY}]->itemInfo.size(); i++)
				{
					if (belowVehicle->partInfo[{touchX, touchY}]->itemInfo[i].itemCode == 99)//차량 조종장치
					{
						if (ctrlVeh == nullptr)
						{
							ctrlVeh = belowVehicle;
							barAct = actSet::vehicle;
							typeHUD = vehFlag::car;
							PlayerPtr->updateVision();
							PlayerPtr->updateMinimap();
						}
						else
						{
							ctrlVeh = nullptr;
							barAct = actSet::null;
							typeHUD = vehFlag::none;
							PlayerPtr->updateVision();
							PlayerPtr->updateMinimap();
						}
					}
					else if (belowVehicle->partInfo[{touchX, touchY}]->itemInfo[i].itemCode == 311)//헬기 조종장치
					{
						if (ctrlVeh == nullptr)
						{
							ctrlVeh = belowVehicle;
							barAct = actSet::helicopter;
							typeHUD = vehFlag::heli;
						}
						else
						{
							ctrlVeh = nullptr;
							barAct = actSet::null;
							typeHUD = vehFlag::none;;
						}
					}
					else if (belowVehicle->partInfo[{touchX, touchY}]->itemInfo[i].itemCode == 313)//열차 조종장치
					{
						if (ctrlVeh == nullptr)
						{
							ctrlVeh = belowVehicle;
							barAct = actSet::train;
							typeHUD = vehFlag::minecart;
						}
						else
						{
							ctrlVeh = nullptr;
							barAct = actSet::null;
							typeHUD = vehFlag::none;;
						}
					}
					else if (belowVehicle->partInfo[{touchX, touchY}]->itemInfo[i].itemCode == itemRefCode::minecartController) //열차 조종장치
					{
						if (ctrlVeh == nullptr)
						{
							ctrlVeh = belowVehicle;
							barAct = actSet::train;
							typeHUD = vehFlag::minecart;
						}
						else
						{
							ctrlVeh = nullptr;
							barAct = actSet::null;
							typeHUD = vehFlag::none;;
						}
					}
				}
			}
			else
			{
				if (TileProp(touchX, touchY, PlayerZ()) != nullptr)
				{
					Prop* tgtProp = TileProp(touchX, touchY, PlayerZ());
					int tgtItemCode = tgtProp->leadItem.itemCode;
					if (tgtProp->leadItem.checkFlag(itemFlag::UPSTAIR))
					{
						if (TileFloor(PlayerX(), PlayerY(), PlayerZ() + 1) == 0)
						{
							updateLog(L"There is no floor above these stairs.");
						}
						else
						{
							updateLog(L"You go up the stairs.");

							EntityPtrMove({ PlayerX(), PlayerY(), PlayerZ() }, { PlayerX(), PlayerY(), PlayerZ() + 1 });

							PlayerPtr->updateVision(PlayerPtr->entityInfo.eyeSight);
							PlayerPtr->updateMinimap();

							Prop* upProp = TileProp(touchX, touchY, PlayerZ());
							if (upProp == nullptr)
							{
								createProp({ PlayerX(), PlayerY(), PlayerZ() }, 299);//하강계단
							}
						}
					}
					else if (tgtProp->leadItem.checkFlag(itemFlag::DOWNSTAIR))
					{
						if (TileWall(PlayerX(), PlayerY(), PlayerZ() + 1) != 0)
						{
							updateLog(L"The stairs going down are blocked by a wall.");
						}
						else
						{
							updateLog(L"You go down the stairs.");

							EntityPtrMove({ PlayerX(), PlayerY(), PlayerZ() }, { PlayerX(), PlayerY(), PlayerZ() - 1 });

							PlayerPtr->updateVision(PlayerPtr->entityInfo.eyeSight);
							PlayerPtr->updateMinimap();

							Prop* downProp = TileProp(touchX, touchY, PlayerZ());
							if (downProp == nullptr)
							{
								createProp({ PlayerX(), PlayerY(), PlayerZ() }, 298);//상승계단
							}
						}
					}
				}
			}
		}
		else if ((std::abs(touchX - PlayerX()) <= 1 && std::abs(touchY - PlayerY()) <= 1) && isWalkable({ touchX, touchY, PlayerZ() }) == false)//1칸 이내
		{
			if (TileWall(touchX, touchY, PlayerZ()) != 0)
			{
				auto ePtr = PlayerPtr->getEquipPtr();
				for (int i = 0; i < ePtr->itemInfo.size(); i++)
				{
					if (ePtr->itemInfo[i].itemCode == itemRefCode::pickaxe)
					{
						if (ePtr->itemInfo[i].equipState == equipHandFlag::both)
						{
							PlayerPtr->setDirection(coord2Dir(touchX - PlayerX(), touchY - PlayerY()));
							addAniUSetPlayer(PlayerPtr, aniFlag::miningWall);
							break;
						}
					}
				}
			}
			else if (TileVehicle(touchX, touchY, PlayerZ()) != nullptr)
			{
				ItemPocket* tgtPocket = TileVehicle(touchX, touchY, PlayerZ())->partInfo[{touchX, touchY }].get();
				for (int i = 0; i < tgtPocket->itemInfo.size(); i++)
				{
					if (tgtPocket->itemInfo[i].checkFlag(itemFlag::VPART_DOOR_CLOSE))
					{
						tgtPocket->itemInfo[i].eraseFlag(itemFlag::VPART_DOOR_CLOSE);
						tgtPocket->itemInfo[i].addFlag(itemFlag::VPART_DOOR_OPEN);

						tgtPocket->itemInfo[i].eraseFlag(itemFlag::VPART_NOT_WALKABLE);

						if (tgtPocket->itemInfo[i].checkFlag(itemFlag::PROP_GAS_OBSTACLE_ON))
						{
							tgtPocket->itemInfo[i].eraseFlag(itemFlag::PROP_GAS_OBSTACLE_ON);
							tgtPocket->itemInfo[i].addFlag(itemFlag::PROP_GAS_OBSTACLE_OFF);
						}

						tgtPocket->itemInfo[i].propSprIndex += 16;
						PlayerPtr->updateVision(PlayerPtr->entityInfo.eyeSight);
					}
				}
			}
			else if (TileProp(touchX, touchY, PlayerZ()) != nullptr)
			{
				Prop* tgtProp = TileProp(touchX, touchY, PlayerZ());
				int tgtItemCode = tgtProp->leadItem.itemCode;
				if (tgtProp->leadItem.checkFlag(itemFlag::DOOR_CLOSE))
				{
					if (tgtProp->leadItem.checkFlag(itemFlag::PROP_WALKABLE) == false)
					{
						tgtProp->leadItem.eraseFlag(itemFlag::DOOR_CLOSE);
						tgtProp->leadItem.addFlag(itemFlag::DOOR_OPEN);

						tgtProp->leadItem.addFlag(itemFlag::PROP_WALKABLE);
						tgtProp->leadItem.eraseFlag(itemFlag::PROP_BLOCKER);
						tgtProp->leadItem.extraSprIndexSingle++;

						if (tgtProp->leadItem.checkFlag(itemFlag::PROP_GAS_OBSTACLE_ON))
						{
							tgtProp->leadItem.eraseFlag(itemFlag::PROP_GAS_OBSTACLE_ON);
							tgtProp->leadItem.addFlag(itemFlag::PROP_GAS_OBSTACLE_OFF);
						}

						PlayerPtr->updateVision(PlayerPtr->entityInfo.eyeSight);
					}
				}
				else if (tgtProp->leadItem.checkFlag(itemFlag::UPSTAIR))
				{
					if (TileFloor(PlayerX(), PlayerY(), PlayerZ() + 1) == 0)
					{
						updateLog(L"There is no floor above these stairs.");
					}
					else
					{
						updateLog(L"You go up the stairs.");
                        PlayerPtr->setGrid(PlayerX(), PlayerY(), PlayerZ() + 1);
					}
				}
				else if (tgtProp->leadItem.checkFlag(itemFlag::DOWNSTAIR))
				{
				}
				else if (tgtProp->leadItem.checkFlag(itemFlag::TREE))
				{
					PlayerPtr->setDirection(coord2Dir(touchX - PlayerX(), touchY - PlayerY()));
					addAniUSetPlayer(PlayerPtr, aniFlag::felling);
				}
				else if (tgtProp->leadItem.itemCode == itemRefCode::leverRL || tgtProp->leadItem.itemCode == itemRefCode::leverUD)
				{
					PlayerPtr->setDirection(coord2Dir(touchX - PlayerX(), touchY - PlayerY()));
					addAniUSetPlayer(PlayerPtr, aniFlag::propTurnOnOff);
                }
				else if (tgtProp->leadItem.itemCode == itemRefCode::tactSwitchRL || tgtProp->leadItem.itemCode == itemRefCode::tactSwitchUD)
				{
					if (tgtProp->leadItem.checkFlag(itemFlag::PROP_POWER_OFF))
					{
						PlayerPtr->setDirection(coord2Dir(touchX - PlayerX(), touchY - PlayerY()));
						addAniUSetPlayer(PlayerPtr, aniFlag::propTurnOnOff);
					}
				}
			}
			else if (TileEntity(touchX, touchY, PlayerZ()) != nullptr && TileEntity(touchX, touchY, PlayerZ())->entityInfo.relation == relationFlag::friendly)
			{
				new Dialogue();

			}
			else
			{
				PlayerPtr->startMove(coord2Dir(touchX - PlayerX(), touchY - PlayerY()));
			}
		}
		else
		{
			PlayerPtr->setAStarDst(touchX, touchY);
		}
	}
	else//차량을 조종하는 상태일 경우
	{
		if (touchX == PlayerX() && touchY == PlayerY())
		{
			ctrlVeh = nullptr;
			barAct = actSet::null;
		}
	}
}