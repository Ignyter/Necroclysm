import Prop;

import std;
import util;
import globalVar;
import constVar;
import wrapVar;
import globalTime;


constexpr double SYSTEM_VOLTAGE = 24.0;
constexpr double TIME_PER_TURN = 60.0;
constexpr double EPSILON = 0.000001;

/*
* 
* relayR,U,L,D : Ʈ�������Ϳ� ������ ���⼺ ����
* 
* 
* <Ʈ��������>
* transistorR : ����Ʈ���� ����, ��ܰ� �ϴ��� ���ζ���
* transistorU : ����Ʈ���� ���, ������ ������ ���ζ���
* transistorL : ����Ʈ���� ����, ��ܰ� �ϴ��� ���ζ���
* transistorD : ����Ʈ���� �ϴ�, ������ ������ ���ζ���
* 
* <��������Ʈ>
* andGateR : ������� ����(R), vcc �Է��� �׻� ���, ������ �ϴ��� �Է���
* andGateL : ������� ����(L), vcc �Է��� �׻� ���, ������ �ϴ��� �Է���
* andGate�� �ܼ��� �Ҹ������� 2��, �� ���� �ִ� 1�� �޴µ� ���� ������ 2 �̻��̸� �۵���
* 
* orGateR & orGateL : andGate�� ����, �ٸ� ON/OFF ������ ACTIVE �÷��׷� ��
* xorGateR & xorGateL : andGate�� ����, �ٸ� ON/OFF ������ ACTIVE �÷��׷� ��
* notGateR : ������� ����(R), vcc �Է��� �׻� ���, ������ �Է���(GND)
* notGateL : ������� ����(L), vcc �Է��� �׻� ���, ������ �Է���(GND)
* 
* srLatchR : ������� ����(R), vcc �Է��� �׻� ���, ������ S(set)�� �ϴ��� R(reset)��
* srLatchL : ������� ����(L), vcc �Է��� �׻� ���, ������ S(set)�� �ϴ��� R(reset)��
* 
* leverRL : �¿츸 ����
* leverUD : ���ϸ� ����
* 
* Ʈ�������Ϳ� ��������Ʈ�� �����̿� �ٸ��� ���� ���� �ÿ� ��� ȸ�ΰ� ����Ǿ�� ��
* Ʈ�������Ϳ� ��������Ʈ�� ����Ʈ <-> ���� ���ΰ� ���İ� �� �ǰ� �� ������ ��
* ����Ʈ->���ζ������� ������ �ȵǰ� �Ϸ��� BFS Ž�� �������� skipBFSSet�� �߰����ָ� ��
* ���ζ���->����Ʈ�� ������ �ȵǰ� �Ϸ��� isConnected �Լ����� üũ���ָ� ��
* 
* <�Ŀ���ũ>
* �Ŀ���ũ�� ��°� ����(������)�� �� Ÿ�Ͽ� ���ÿ� �ִ� Ư���� ������.
* powerBankR : ������ ���, �������� �����Է�(����)
* powerBankL : �������� ���, ������ �����Է�(����)
* 
* <�׶���>
* gndUsePower�� ������(�����¿�, z�� ����) �Է���. gndUsePower>0�̸� �ݵ�� ���⼺ ������ �������� ������ ����
* �ݴ�� ���⼺ ������ �����ϸ� �翬�� gndUsePower�� 0�� ����
*/

bool Prop::hasGround()
{
    //일반 접지
    if (leadItem.gndUsePower > 0) return true;
    //방향성 접지
    if (leadItem.gndUsePowerRight > 0) return true;
    if (leadItem.gndUsePowerUp > 0) return true;
    if (leadItem.gndUsePowerLeft > 0) return true;
    if (leadItem.gndUsePowerDown > 0) return true;

    //통과형 접지(sink)도 GND로 인식
    if (leadItem.gndSink > 0) return true;
    if (leadItem.gndSinkRight > 0) return true;
    if (leadItem.gndSinkUp > 0) return true;
    if (leadItem.gndSinkLeft > 0) return true;
    if (leadItem.gndSinkDown > 0) return true;

    return false;
}

bool Prop::hasSink()
{
    //통과형 접지(sink) 체크
    if (leadItem.gndSink > 0) return true;
    if (leadItem.gndSinkRight > 0) return true;
    if (leadItem.gndSinkUp > 0) return true;
    if (leadItem.gndSinkLeft > 0) return true;
    if (leadItem.gndSinkDown > 0) return true;

    return false;
}

double Prop::getTotalChargeFlux()
{
    return (chargeFlux[dir16::right] + chargeFlux[dir16::up] + chargeFlux[dir16::left] + chargeFlux[dir16::down] + chargeFlux[dir16::above] + chargeFlux[dir16::below]);
}

bool Prop::isChargeFlowing()
{
    return chargeFlux[dir16::right] != 0
        || chargeFlux[dir16::up] != 0
        || chargeFlux[dir16::left] != 0
        || chargeFlux[dir16::down] != 0
        || chargeFlux[dir16::above] != 0
        || chargeFlux[dir16::below] != 0;
}

void Prop::updateCircuitNetwork()
{
    if(debug::printCircuitLog) std::wprintf(L"------------------------- ȸ�θ� ������Ʈ ���� ------------------------\n");
    int cursorX = getGridX();
    int cursorY = getGridY();
    int cursorZ = getGridZ();

    if (cursorX == 1 && cursorY == -14)
    {
        int a = 3;
    }

    std::queue<Point3> frontierQueue;
    std::unordered_set<Point3, Point3::Hash> visitedSet;
    std::vector<Prop*> voltagePropVec;

    std::unordered_set<Point3, Point3::Hash> skipBFSSet;

    int circuitMaxEnergy = 0;
    int circuitTotalLoad = 0;
    bool hasGround = false;

    //==============================================================================
    // 1. ȸ�� ���� Ž��(BFS)
    //==============================================================================

    if(saveFrontierQueue.size()>0 && saveVisitedSet.size()>0)
    {
        if (debug::printCircuitLog) std::wprintf(L"------------------------- ���� ȸ�θ� Ž�� ��� �ҷ����� ------------------------\n");
        frontierQueue = saveFrontierQueue;
        visitedSet = saveVisitedSet;
    }
    else frontierQueue.push({ cursorX, cursorY, cursorZ });
    
    //BFS�� �������� �������� ����(CROSS ���̺�������), ���ϳ� ���п����� �׻� ������ ��
    while (!frontierQueue.empty())
    {
        
        Point3 current = frontierQueue.front();
        frontierQueue.pop();
        crossStates.clear();

        Prop* currentProp = TileProp(current.x, current.y, current.z);
        ItemData& currentItem = currentProp->leadItem;

        if (visitedSet.find(current) != visitedSet.end()) continue;
        visitedSet.insert(current);

        if (debug::printCircuitLog) std::wprintf(L"[BFS Ž��] %ls (%d,%d,%d) \n", currentProp->leadItem.name.c_str(), current.x, current.y, current.z);
        if (currentProp == nullptr)
        {
            std::wprintf(L"[���] BFS�� nullptr ���ӿ� ������ (%d,%d,%d)\n", current.x, current.y, current.z);
            continue;
        }

        if (current.x == -3 && current.y == -13)
            int a = 3;


        if (currentProp && (currentProp->leadItem.checkFlag(itemFlag::CIRCUIT) || currentProp->leadItem.checkFlag(itemFlag::CABLE)))
        {
            
            currentProp->runUsed = true;
            if (currentProp->leadItem.itemCode == itemRefCode::powerBankR || currentProp->leadItem.itemCode == itemRefCode::powerBankL) currentProp->runUsed = false;

            currentProp->totalLossCharge = 0;

            if (currentProp->leadItem.checkFlag(itemFlag::VOLTAGE_SOURCE))
            {
                if (currentProp->leadItem.checkFlag(itemFlag::PROP_POWER_ON) &&
                    currentProp->leadItem.checkFlag(itemFlag::PROP_POWER_OFF) == false)
                {
                    circuitMaxEnergy += currentProp->leadItem.electricMaxPower;
                    voltagePropVec.push_back(currentProp);
                }
                else if (currentProp->leadItem.itemCode == itemRefCode::powerBankR || currentProp->leadItem.itemCode == itemRefCode::powerBankL)
                {
                    circuitMaxEnergy += std::min(static_cast<double>(currentProp->leadItem.electricMaxPower), currentProp->leadItem.powerStorage);
                    voltagePropVec.push_back(currentProp);

                }

            }
            if (currentProp->leadItem.itemCode == itemRefCode::chargingPort)//��¡��Ʈ�� ���...
            {
                currentProp->leadItem.gndUsePower = 1;
                ItemStack* hereStack = TileItemStack(current.x, current.y, current.z);  // �� ����
                if (hereStack != nullptr)
                {
                    std::vector<ItemData>& hereItems = hereStack->getPocket()->itemInfo;  // �� ���� (�ߺ� ȣ�� ����)
                    for (ItemData& item : hereItems)
                    {
                        if (item.itemCode == itemRefCode::battery || item.itemCode == itemRefCode::batteryPack)
                        {
                            if (item.powerStorage < item.powerStorageMax)
                            {
                                currentProp->leadItem.gndUsePower += item.powerStorageMax - std::floor(item.powerStorage);
                            }
                        }
                    }
                }
            }

            //전방향 소비전력이 있을 경우 loadSet에 추가
            //추가 isConnect 6방향 체크로 방향성 접지 loadSet 추가 파워뱅크와 동일(수정함 봐)
            if (currentProp->leadItem.gndUsePower > 0)
            {
                circuitTotalLoad += currentProp->leadItem.gndUsePower;
                hasGround = true;
            }

            //통과형 sink도 load에 포함 (전력 소모가 있으므로)
            if (currentProp->leadItem.gndSink > 0)
            {
                circuitTotalLoad += static_cast<int>(currentProp->leadItem.gndSink);
                hasGround = true;
            }

            //��Ʈ ����ġ�� ��� ���� �� ���� �ÿ� ����
            if (currentProp->leadItem.itemCode == itemRefCode::tactSwitchRL || currentProp->leadItem.itemCode == itemRefCode::tactSwitchUD)
            {
                if (currentProp->leadItem.checkFlag(itemFlag::PROP_POWER_ON))
                {
                    if (currentProp->leadItem.checkFlag(itemFlag::PROP_NEXT_TURN_POWER_OFF) == false)
                    {
                        currentProp->leadItem.addFlag(itemFlag::PROP_NEXT_TURN_POWER_OFF);
                    }
                    else
                    {
                        currentProp->leadItem.eraseFlag(itemFlag::PROP_NEXT_TURN_POWER_OFF);
                        currentProp->leadItem.eraseFlag(itemFlag::PROP_POWER_ON);
                        currentProp->leadItem.addFlag(itemFlag::PROP_POWER_OFF);
                    }
                }
            }
            else if(currentProp->leadItem.itemCode == itemRefCode::pressureSwitchRL || currentProp->leadItem.itemCode == itemRefCode::pressureSwitchUD)
            {
                int totalWeight = 0;
                
                //������ ����
                ItemStack* tgtItemStack = TileItemStack(current.x, current.y, current.z);
                if (tgtItemStack != nullptr)
                {
                    auto& tileItemInfo = tgtItemStack->getPocket()->itemInfo;
                    for (int i = 0; i < tileItemInfo.size(); i++)
                    {
                        totalWeight += tileItemInfo[i].weight * tileItemInfo[i].number;
                    }
                }

                //��ƼƼ ����
                Entity* ePtr = TileEntity(current.x, current.y, current.z);
                if(ePtr != nullptr) totalWeight += ePtr->entityInfo.weight;
                
                
                if (totalWeight >= 5000.0 && currentProp->leadItem.checkFlag(itemFlag::PROP_POWER_OFF))
                {
                    currentProp->leadItem.eraseFlag(itemFlag::PROP_POWER_OFF);
                    currentProp->leadItem.addFlag(itemFlag::PROP_POWER_ON);
                }
                else if (totalWeight < 5000.0 && currentProp->leadItem.checkFlag(itemFlag::PROP_POWER_ON))
                {
                    currentProp->leadItem.eraseFlag(itemFlag::PROP_POWER_ON);
                    currentProp->leadItem.addFlag(itemFlag::PROP_POWER_OFF);
                }
            }


            const dir16 directions[] = { dir16::right, dir16::up, dir16::left, dir16::down, dir16::above, dir16::below };

            if (skipBFSSet.find(current) == skipBFSSet.end())
            {
                for (int i = 0; i < 6; ++i)
                {
                    int dx, dy, dz;
                    dirToXYZ(directions[i], dx, dy, dz);
                    Point3 nextCoord = { current.x + dx, current.y + dy, current.z + dz };
                    Prop* nextProp = TileProp(nextCoord.x, nextCoord.y, nextCoord.z);
                    ItemData& nextItem = nextProp->leadItem;

                    
                    if (nextProp && nextProp->leadItem.checkFlag(itemFlag::CROSSED_CABLE))
                    {
                        if (crossStates[current] == crossFlag::horizontal && (directions[i] == dir16::up || directions[i] == dir16::down))
                        {
                            crossStates.erase(current);
                            continue;
                        }
                        if (crossStates[current] == crossFlag::vertical && (directions[i] == dir16::right || directions[i] == dir16::left))
                        {
                            crossStates.erase(current);
                            continue;
                        }
                    }

                    if (isConnected(current, directions[i]))
                    {
                        if (nextProp != nullptr && nextProp->hasGround())
                        {
                            //���̽����� ���ζ������� BFS�� �߰��ϴ� ���� ����
                            if (nextProp->leadItem.itemCode == itemRefCode::transistorL && directions[i] == dir16::right) skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::transistorU && directions[i] == dir16::down) skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::transistorR && directions[i] == dir16::left) skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::transistorD && directions[i] == dir16::up) skipBFSSet.insert(nextCoord);

                            if (nextProp->leadItem.itemCode == itemRefCode::relayL && directions[i] == dir16::right) skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::relayU && directions[i] == dir16::down) skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::relayR && directions[i] == dir16::left) skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::relayD && directions[i] == dir16::up) skipBFSSet.insert(nextCoord);

                            if(nextProp->leadItem.itemCode == itemRefCode::andGateR && (directions[i] == dir16::right || directions[i] == dir16::up)) 
                                skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::andGateL && (directions[i] == dir16::left || directions[i] == dir16::up)) 
                                skipBFSSet.insert(nextCoord);

                            if (nextProp->leadItem.itemCode == itemRefCode::orGateR && (directions[i] == dir16::right || directions[i] == dir16::up))
                                skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::orGateL && (directions[i] == dir16::left || directions[i] == dir16::up))
                                skipBFSSet.insert(nextCoord);

                            if (nextProp->leadItem.itemCode == itemRefCode::xorGateR && (directions[i] == dir16::right || directions[i] == dir16::up))
                                skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::xorGateL && (directions[i] == dir16::left || directions[i] == dir16::up))
                                skipBFSSet.insert(nextCoord);

                            if (nextProp->leadItem.itemCode == itemRefCode::notGateR && (directions[i] == dir16::right))
                                skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::notGateL && (directions[i] == dir16::left))
                                skipBFSSet.insert(nextCoord);

                            if (nextProp->leadItem.itemCode == itemRefCode::srLatchR && (directions[i] == dir16::right || directions[i] == dir16::up))
                                skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::srLatchL && (directions[i] == dir16::left || directions[i] == dir16::up))
                                skipBFSSet.insert(nextCoord);


                            if (nextProp->leadItem.itemCode == itemRefCode::powerBankR && (directions[i] == dir16::right))
                                skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::powerBankL && (directions[i] == dir16::left))
                                skipBFSSet.insert(nextCoord);



                            Point3 rightCoord = { current.x + 1, current.y, current.z };
                            Point3 upCoord = { current.x, current.y - 1, current.z };
                            Point3 leftCoord = { current.x - 1, current.y, current.z };
                            Point3 downCoord = { current.x, current.y + 1, current.z };

                            //�Ŀ���ũ �����ӵ� ����
                            if (nextItem.itemCode == itemRefCode::powerBankR || nextItem.itemCode == itemRefCode::powerBankL)
                            {
                                double ratio = (nextItem.powerStorage) / static_cast<double>(nextItem.powerStorageMax);
                                errorBox(ratio > 1 || ratio < 0, L"Ratio is out of range");

                                if (nextItem.itemCode == itemRefCode::powerBankR)
                                {
                                    nextItem.gndUsePowerLeft = itemDex[nextItem.itemCode].gndUsePowerLeft;
                                    nextItem.gndUsePowerLeft *= std::log(1 + 20 * (1.02 - ratio)) / std::log(1 + 20 * 1.02);
                                    nextItem.gndUsePowerLeft = myMin(nextItem.gndUsePowerLeft, nextItem.powerStorageMax - nextItem.powerStorage);
                                    if (nextItem.gndUsePowerLeft < 0) nextItem.gndUsePowerLeft = 0;
                                }
                                else if (nextItem.itemCode == itemRefCode::powerBankL)
                                {
                                    nextItem.gndUsePowerRight = itemDex[nextItem.itemCode].gndUsePowerRight;
                                    nextItem.gndUsePowerRight *= std::log(1 + 20 * (1.02 - ratio)) / std::log(1 + 20 * 1.02);
                                    nextItem.gndUsePowerRight = myMin(nextItem.gndUsePowerRight, nextItem.powerStorageMax - nextItem.powerStorage);
                                    if (nextItem.gndUsePowerRight < 0) nextItem.gndUsePowerRight = 0;
                                }


                            }

                            //방향성 종단형 GND
                            if (directions[i] == dir16::right && nextProp->leadItem.gndUsePowerLeft > 0)
                            {
                                circuitTotalLoad += nextProp->leadItem.gndUsePowerLeft;
                                hasGround = true;
                            }
                            else if (directions[i] == dir16::up && nextProp->leadItem.gndUsePowerDown > 0)
                            {
                                circuitTotalLoad += nextProp->leadItem.gndUsePowerDown;
                                hasGround = true;
                            }
                            else if (directions[i] == dir16::left && nextProp->leadItem.gndUsePowerRight > 0)
                            {
                                circuitTotalLoad += nextProp->leadItem.gndUsePowerRight;
                                hasGround = true;
                            }
                            else if (directions[i] == dir16::down && nextProp->leadItem.gndUsePowerUp > 0)
                            {
                                circuitTotalLoad += nextProp->leadItem.gndUsePowerUp;
                                hasGround = true;
                            }

                            //방향성 통과형 sink
                            if (directions[i] == dir16::right && nextProp->leadItem.gndSinkLeft > 0)
                            {
                                circuitTotalLoad += static_cast<int>(nextProp->leadItem.gndSinkLeft);
                                hasGround = true;
                            }
                            else if (directions[i] == dir16::up && nextProp->leadItem.gndSinkDown > 0)
                            {
                                circuitTotalLoad += static_cast<int>(nextProp->leadItem.gndSinkDown);
                                hasGround = true;
                            }
                            else if (directions[i] == dir16::left && nextProp->leadItem.gndSinkRight > 0)
                            {
                                circuitTotalLoad += static_cast<int>(nextProp->leadItem.gndSinkRight);
                                hasGround = true;
                            }
                            else if (directions[i] == dir16::down && nextProp->leadItem.gndSinkUp > 0)
                            {
                                circuitTotalLoad += static_cast<int>(nextProp->leadItem.gndSinkUp);
                                hasGround = true;
                            }
                        }

                        
                        if (nextProp->leadItem.checkFlag(itemFlag::CROSSED_CABLE))
                        {
                            if (directions[i] == dir16::down || directions[i] == dir16::up) crossStates[nextCoord] = crossFlag::vertical;
                            else if (directions[i] == dir16::down || directions[i] == dir16::up) crossStates[nextCoord] = crossFlag::horizontal;
                        }
                        frontierQueue.push(nextCoord);
                        
                    }
                }
            }
            else skipBFSSet.erase(current);
                
        }
    }

    //==============================================================================
    // 2. �ִ� ���� ����
    //==============================================================================
    for (auto coord : visitedSet)
    {
        Prop* propPtr = TileProp(coord.x, coord.y, coord.z);
        if (propPtr != nullptr)
        {
            propPtr->nodeMaxCharge = circuitMaxEnergy;

            //(����ȸ�ο�) �׻� ���� ���� �� ����
            propPtr->nodeCharge = circuitMaxEnergy;

        }
    }

    //==============================================================================
    // 3. ���п� ���� ����
    //==============================================================================
    double totalPushedCharge = 0;

    randomVectorShuffle(voltagePropVec);

    int totalAvailablePower = 0;
    for (Prop* voltProp : voltagePropVec)
    {
        if (voltProp->leadItem.checkFlag(itemFlag::PROP_POWER_ON) &&
            voltProp->leadItem.checkFlag(itemFlag::PROP_POWER_OFF) == false)
        {
            totalAvailablePower += voltProp->leadItem.electricMaxPower;
        }
    }


    for (Prop* voltProp : voltagePropVec)
    {
        constexpr double LOSS_COMPENSATION_FACTOR = 1.2;

        voltProp->nodeCharge = voltProp->nodeMaxCharge;
        int x = voltProp->getGridX();
        int y = voltProp->getGridY();
        int z = voltProp->getGridZ();
        double voltRatio = (double)voltProp->leadItem.electricMaxPower / (double)totalAvailablePower;
        double voltOutputPower = myMin(std::ceil(circuitTotalLoad * voltRatio), voltProp->leadItem.electricMaxPower);
        voltProp->prevPushedCharge = 0;
        voltOutputPower *= LOSS_COMPENSATION_FACTOR;  // ���׼ս� ���� ���� (�⺻�� 120%)

        if (debug::printCircuitLog) std::wprintf(L"========================�����п� %p : �о�� ���ۡ�========================\n", voltProp);
        if (voltProp->leadItem.checkFlag(itemFlag::PROP_POWER_ON) || voltProp->leadItem.checkFlag(itemFlag::PROP_POWER_OFF) == false)
        {
            double finalVoltOutput = voltOutputPower;
            if (voltProp->leadItem.itemCode == itemRefCode::powerBankR || voltProp->leadItem.itemCode == itemRefCode::powerBankL)
                finalVoltOutput = std::min(voltOutputPower, voltProp->leadItem.powerStorage);


            if (voltProp->leadItem.checkFlag(itemFlag::VOLTAGE_OUTPUT_RIGHT) && isConnected({ x,y,z }, dir16::right))
                voltProp->prevPushedCharge += pushCharge(voltProp, dir16::right, finalVoltOutput, {}, 0);
            else if (voltProp->leadItem.checkFlag(itemFlag::VOLTAGE_OUTPUT_UP) && isConnected({ x,y,z }, dir16::up))
                voltProp->prevPushedCharge += pushCharge(voltProp, dir16::up, finalVoltOutput, {}, 0);
            else if (voltProp->leadItem.checkFlag(itemFlag::VOLTAGE_OUTPUT_LEFT) && isConnected({ x,y,z }, dir16::left))
                voltProp->prevPushedCharge += pushCharge(voltProp, dir16::left, finalVoltOutput, {}, 0);
            else if (voltProp->leadItem.checkFlag(itemFlag::VOLTAGE_OUTPUT_DOWN) && isConnected({ x,y,z }, dir16::down))
                voltProp->prevPushedCharge += pushCharge(voltProp, dir16::down, finalVoltOutput, {}, 0);

            totalPushedCharge += voltProp->prevPushedCharge;
            voltProp->nodeCharge = voltProp->nodeMaxCharge;
        }
    }
}

bool Prop::isConnected(Point3 currentCoord, dir16 dir)
{
    Prop* currentProp = TileProp(currentCoord.x, currentCoord.y, currentCoord.z);

    Point3 delCoord = { 0,0,0 };
    itemFlag hostFlag, guestFlag;
    switch (dir)
    {
    case dir16::right:
        delCoord = { +1,0,0 };
        hostFlag = itemFlag::CABLE_CNCT_RIGHT;
        guestFlag = itemFlag::CABLE_CNCT_LEFT;
        break;
    case dir16::up:
        delCoord = { 0,-1,0 };
        hostFlag = itemFlag::CABLE_CNCT_UP;
        guestFlag = itemFlag::CABLE_CNCT_DOWN;
        break;
    case dir16::left:
        delCoord = { -1,0,0 };
        hostFlag = itemFlag::CABLE_CNCT_LEFT;
        guestFlag = itemFlag::CABLE_CNCT_RIGHT;
        break;
    case dir16::down:
        delCoord = { 0,+1,0 };
        hostFlag = itemFlag::CABLE_CNCT_DOWN;
        guestFlag = itemFlag::CABLE_CNCT_UP;
        break;
    case dir16::above:
        delCoord = { 0,0,+1 };
        hostFlag = itemFlag::CABLE_Z_ASCEND;
        guestFlag = itemFlag::CABLE_Z_DESCEND;
        break;
    case dir16::below:
        delCoord = { 0,0,-1 };
        hostFlag = itemFlag::CABLE_Z_DESCEND;
        guestFlag = itemFlag::CABLE_Z_ASCEND;
        break;
    default:
        errorBox(L"[Error] isConnected lambda function received invalid direction argument.\n");
        break;
    }
    Prop* targetProp = TileProp(currentCoord.x + delCoord.x, currentCoord.y + delCoord.y, currentCoord.z + delCoord.z);

    if (targetProp == nullptr) return false;

    if(currentProp->leadItem.itemCode == itemRefCode::tactSwitchRL 
        || currentProp->leadItem.itemCode == itemRefCode::tactSwitchUD
        || currentProp->leadItem.itemCode == itemRefCode::leverRL
        || currentProp->leadItem.itemCode == itemRefCode::leverUD
        || currentProp->leadItem.itemCode == itemRefCode::pressureSwitchRL
        || currentProp->leadItem.itemCode == itemRefCode::pressureSwitchUD
        )
    {
        if (currentProp->leadItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }

    ItemData& tgtItem = targetProp->leadItem;

    if ((dir == dir16::right || dir == dir16::left) && tgtItem.itemCode == itemRefCode::leverRL)
    {
        if (tgtItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }
    else if ((dir == dir16::up || dir == dir16::down) && tgtItem.itemCode == itemRefCode::leverUD)
    {
        if (tgtItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }
    else if ((dir == dir16::right || dir == dir16::left) && tgtItem.itemCode == itemRefCode::tactSwitchRL)
    {
        if (tgtItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }
    else if ((dir == dir16::up || dir == dir16::down) && tgtItem.itemCode == itemRefCode::tactSwitchUD)
    {
        if (tgtItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }
    else if ((dir == dir16::right || dir == dir16::left) && tgtItem.itemCode == itemRefCode::pressureSwitchRL)
    {
        if (tgtItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }
    else if ((dir == dir16::up || dir == dir16::down) && tgtItem.itemCode == itemRefCode::pressureSwitchUD)
    {
        if (tgtItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }


    if ((dir == dir16::right || dir == dir16::left) && (tgtItem.itemCode == itemRefCode::relayU || tgtItem.itemCode == itemRefCode::relayD))
    {
        if (tgtItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }
    else if ((dir == dir16::up || dir == dir16::down) && (tgtItem.itemCode == itemRefCode::relayR || tgtItem.itemCode == itemRefCode::relayL))
    {
        if (tgtItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }

    if ((dir == dir16::right || dir == dir16::left) && (tgtItem.itemCode == itemRefCode::transistorU || tgtItem.itemCode == itemRefCode::transistorD))
    {
        if (tgtItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }
    else if ((dir == dir16::up || dir == dir16::down) && (tgtItem.itemCode == itemRefCode::transistorR || tgtItem.itemCode == itemRefCode::transistorL))
    {
        if (tgtItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }

    if (dir == dir16::down && (tgtItem.itemCode == itemRefCode::andGateR ||tgtItem.itemCode == itemRefCode::andGateL))
    {
        if (tgtItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }
    //��������Ʈ ��� ���̿��� �����н�
    else if (dir == dir16::left && tgtItem.itemCode == itemRefCode::andGateR) return false;
    else if (dir == dir16::right && tgtItem.itemCode == itemRefCode::andGateL)return false;

    if (dir == dir16::down && (tgtItem.itemCode == itemRefCode::orGateR || tgtItem.itemCode == itemRefCode::orGateL))
    {
        if (tgtItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }
    else if (dir == dir16::left && tgtItem.itemCode == itemRefCode::orGateR) return false;
    else if (dir == dir16::right && tgtItem.itemCode == itemRefCode::orGateL) return false;

    if (dir == dir16::down && (tgtItem.itemCode == itemRefCode::xorGateR || tgtItem.itemCode == itemRefCode::xorGateL))
    {
        if (tgtItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }
    else if (dir == dir16::left && tgtItem.itemCode == itemRefCode::xorGateR) return false;
    else if (dir == dir16::right && tgtItem.itemCode == itemRefCode::xorGateL) return false;


    if (dir == dir16::down && (tgtItem.itemCode == itemRefCode::notGateR || tgtItem.itemCode == itemRefCode::notGateL))
    {
        if (tgtItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }
    if (dir == dir16::left && tgtItem.itemCode == itemRefCode::notGateR) return false;
    else if (dir == dir16::right && tgtItem.itemCode == itemRefCode::notGateL) return false;


    if (dir == dir16::down && (tgtItem.itemCode == itemRefCode::srLatchR || tgtItem.itemCode == itemRefCode::srLatchL))
    {
        if (tgtItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }
    else if (dir == dir16::left && tgtItem.itemCode == itemRefCode::srLatchR) return false;
    else if (dir == dir16::right && tgtItem.itemCode == itemRefCode::srLatchL) return false;

    if (dir == dir16::left && tgtItem.itemCode == itemRefCode::delayR) return false;
    else if (dir == dir16::right && tgtItem.itemCode == itemRefCode::delayL) return false;

    if (dir == dir16::right && tgtItem.itemCode == itemRefCode::diodeL) return false;
    else if (dir == dir16::up && tgtItem.itemCode == itemRefCode::diodeD)return false;
    else if (dir == dir16::left && tgtItem.itemCode == itemRefCode::diodeR)return false;
    else if (dir == dir16::down && tgtItem.itemCode == itemRefCode::diodeU)return false;

    if (dir == dir16::left && tgtItem.itemCode == itemRefCode::powerBankR) return false;
    else if (dir == dir16::right && tgtItem.itemCode == itemRefCode::powerBankL) return false;


    ItemData& crtItem = currentProp->leadItem;

    //(Ʈ��������) ���ζ��ο��� ���̽� ���� ����
    if (crtItem.itemCode == itemRefCode::transistorL && dir == dir16::left) return false;
    else if(crtItem.itemCode == itemRefCode::transistorU && dir == dir16::up) return false;
    else if(crtItem.itemCode == itemRefCode::transistorR && dir == dir16::right) return false;
    else if (crtItem.itemCode == itemRefCode::transistorD && dir == dir16::down) return false;

    if (crtItem.itemCode == itemRefCode::relayL && dir == dir16::left) return false;
    else if (crtItem.itemCode == itemRefCode::relayU && dir == dir16::up) return false;
    else if (crtItem.itemCode == itemRefCode::relayR && dir == dir16::right) return false;
    else if (crtItem.itemCode == itemRefCode::relayD && dir == dir16::down) return false;

    //(��������Ʈ) ���ζ��ο��� �Է���1,2 ���� ����
    if (crtItem.itemCode == itemRefCode::andGateR && (dir == dir16::left || dir == dir16::down)) return false;
    else if (crtItem.itemCode == itemRefCode::andGateL && (dir == dir16::right || dir == dir16::down)) return false;

    if (crtItem.itemCode == itemRefCode::orGateR && (dir == dir16::left || dir == dir16::down)) return false;
    else if (crtItem.itemCode == itemRefCode::orGateL && (dir == dir16::right || dir == dir16::down)) return false;

    if (crtItem.itemCode == itemRefCode::xorGateR && (dir == dir16::left || dir == dir16::down)) return false;
    else if (crtItem.itemCode == itemRefCode::xorGateL && (dir == dir16::right || dir == dir16::down)) return false;

    //(NOT����Ʈ) ���ζ��ο��� �Է��� ���� ����
    if (crtItem.itemCode == itemRefCode::notGateR && dir == dir16::left) return false;
    else if (crtItem.itemCode == itemRefCode::notGateL && dir == dir16::right) return false;

    if (crtItem.itemCode == itemRefCode::srLatchR && (dir == dir16::left || dir == dir16::down)) return false;
    else if (crtItem.itemCode == itemRefCode::srLatchL && (dir == dir16::right || dir == dir16::down)) return false;

    if (crtItem.itemCode == itemRefCode::delayR && dir == dir16::right && crtItem.checkFlag(itemFlag::PROP_POWER_OFF))
    {
        return false;
    }
    if (crtItem.itemCode == itemRefCode::delayL && dir == dir16::left && crtItem.checkFlag(itemFlag::PROP_POWER_OFF))
    {
        return false;
    }

    //(�Ŀ���ũ) ���ζ���(����)���� �Էº� ����
    if (crtItem.itemCode == itemRefCode::powerBankR && dir == dir16::left) return false;
    else if (crtItem.itemCode == itemRefCode::powerBankL && dir == dir16::right) return false;

    if (dir == dir16::above || dir == dir16::below)
    {
        bool currentCondition = currentProp->leadItem.checkFlag(itemFlag::CABLE) && currentProp->leadItem.checkFlag(hostFlag);
        bool targetCondition = targetProp->leadItem.checkFlag(itemFlag::CABLE) && targetProp->leadItem.checkFlag(guestFlag);

        if (currentCondition && targetCondition) return true;
        else return false;
    }
    else if (dir == dir16::right || dir == dir16::up || dir == dir16::left || dir == dir16::down)
    {
        bool currentCondition = (currentProp->leadItem.checkFlag(itemFlag::CABLE) || currentProp->leadItem.checkFlag(hostFlag));
        if (crtItem.checkFlag(itemFlag::CROSSED_CABLE))
        {
            if (crossStates.find(currentCoord) != crossStates.end())
            {
                if (crossStates[currentCoord] == crossFlag::horizontal && (dir == dir16::up || dir == dir16::down)) currentCondition = false;
                else if (crossStates[currentCoord] == crossFlag::vertical && (dir == dir16::right || dir == dir16::left)) currentCondition = false;
            }
        }

        bool targetCondition = (targetProp->leadItem.checkFlag(itemFlag::CABLE) || targetProp->leadItem.checkFlag(guestFlag));

        if (currentCondition && targetCondition) return true;
        else return false;
    }
    else errorBox(L"[Error] isConnected lambda function received invalid direction argument.\n");
}

bool Prop::isGround(Point3 current, dir16 dir)
{
    Prop* currentProp = TileProp(current.x, current.y, current.z);
    itemFlag groundFlag;


    if (current.x == 0 && current.y == -14)
        int a = 3;

    Point3 rightCoord = { current.x + 1, current.y, current.z };
    Point3 upCoord = { current.x, current.y - 1, current.z };
    Point3 leftCoord = { current.x - 1, current.y, current.z };
    Point3 downCoord = { current.x, current.y + 1, current.z };
    Prop* nextProp = nullptr;
    if (dir == dir16::right) nextProp = TileProp(rightCoord);
    else if (dir == dir16::up) nextProp = TileProp(upCoord);
    else if (dir == dir16::left) nextProp = TileProp(leftCoord);
    else if (dir == dir16::down) nextProp = TileProp(downCoord);

    if (nextProp == nullptr) return false;
    else
    {
        if (nextProp->hasGround() == false) return false;

        if (isConnected(current, dir))
        {
            //종단형 GND 체크
            if (nextProp->leadItem.gndUsePower > 0) return true;
            else if (dir == dir16::right && nextProp->leadItem.gndUsePowerLeft > 0) return true;
            else if (dir == dir16::up && nextProp->leadItem.gndUsePowerDown > 0) return true;
            else if (dir == dir16::left && nextProp->leadItem.gndUsePowerRight > 0) return true;
            else if (dir == dir16::down && nextProp->leadItem.gndUsePowerUp > 0) return true;

            //통과형 Sink 체크 (GND의 일종)
            if (nextProp->leadItem.gndSink > 0) return true;
            else if (dir == dir16::right && nextProp->leadItem.gndSinkLeft > 0) return true;
            else if (dir == dir16::up && nextProp->leadItem.gndSinkDown > 0) return true;
            else if (dir == dir16::left && nextProp->leadItem.gndSinkRight > 0) return true;
            else if (dir == dir16::down && nextProp->leadItem.gndSinkUp > 0) return true;
        }
    }
    return false;
}

bool Prop::isSink(Point3 current, dir16 dir)
{
    //지정된 방향의 다음 노드가 통과형 sink인지 체크
    Point3 rightCoord = { current.x + 1, current.y, current.z };
    Point3 upCoord = { current.x, current.y - 1, current.z };
    Point3 leftCoord = { current.x - 1, current.y, current.z };
    Point3 downCoord = { current.x, current.y + 1, current.z };
    Prop* nextProp = nullptr;
    if (dir == dir16::right) nextProp = TileProp(rightCoord);
    else if (dir == dir16::up) nextProp = TileProp(upCoord);
    else if (dir == dir16::left) nextProp = TileProp(leftCoord);
    else if (dir == dir16::down) nextProp = TileProp(downCoord);

    if (nextProp == nullptr) return false;

    if (nextProp->hasSink() == false) return false;

    if (isConnected(current, dir))
    {
        //전방향 sink
        if (nextProp->leadItem.gndSink > 0) return true;
        //방향성 sink
        else if (dir == dir16::right && nextProp->leadItem.gndSinkLeft > 0) return true;
        else if (dir == dir16::up && nextProp->leadItem.gndSinkDown > 0) return true;
        else if (dir == dir16::left && nextProp->leadItem.gndSinkRight > 0) return true;
        else if (dir == dir16::down && nextProp->leadItem.gndSinkUp > 0) return true;
    }
    return false;
}

bool Prop::isConnected(Prop* currentProp, dir16 dir)
{
    return isConnected({ currentProp->getGridX(),currentProp->getGridY(),currentProp->getGridZ() }, dir);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double Prop::pushCharge(Prop* donorProp, dir16 txDir, double txChargeAmount, std::unordered_set<Prop*> pathVisited, int depth)
{
    errorBox(donorProp == nullptr, L"[Error] pushCharge: null donor\n");
    int dx, dy, dz;
    dirToXYZ(txDir, dx, dy, dz);
    Point3 nextCoord = { donorProp->getGridX() + dx, donorProp->getGridY() + dy, donorProp->getGridZ() + dz };
    Prop* nextProp = TileProp(nextCoord);
    errorBox(nextProp == nullptr, L"[Error] pushCharge: no acceptor found\n");

    txChargeAmount = std::min(donorProp->nodeCharge, txChargeAmount);

    errorBox(txChargeAmount > donorProp->nodeCharge + EPSILON, L"[Error] pushCharge: insufficient electron\n");
    errorBox(!isConnected({ donorProp->getGridX(), donorProp->getGridY(), donorProp->getGridZ() }, txDir),
        L"[Error] pushCharge: not connected\n");

    if (pathVisited.find(donorProp) != pathVisited.end()) return 0;
    pathVisited.insert(donorProp);
    if (pathVisited.find(nextProp) != pathVisited.end()) return 0;

    // �鿩���� ����
    std::wstring indent(depth * 2, L' ');  // depth���� 2ĭ��

    if (debug::printCircuitLog) std::wprintf(L"%s[PUSH] (%d,%d) �� (%d,%d) �õ�: %.2f\n",
        indent.c_str(),
        donorProp->getGridX(), donorProp->getGridY(),
        nextProp->getGridX(), nextProp->getGridY(),
        txChargeAmount);

    Point3 current = { donorProp->getGridX(), donorProp->getGridY(), donorProp->getGridZ() };
    if (isGround(current,txDir))
    {
        //통과형 sink인지 체크 - sink는 전력 소모 후 나머지를 계속 전파
        bool isSinkNode = isSink(current, txDir);

        double remainEnergy = 0;
        double sinkRemainEnergy = 0;

        //종단형 GND 남은 용량 계산
        if (txDir == dir16::right && nextProp->leadItem.gndUsePowerLeft > 0)
            remainEnergy = nextProp->leadItem.gndUsePowerLeft - nextProp->chargeFlux[dir16::left];
        else if (txDir == dir16::up && nextProp->leadItem.gndUsePowerDown > 0)
            remainEnergy = nextProp->leadItem.gndUsePowerDown - nextProp->chargeFlux[dir16::down];
        else if (txDir == dir16::left && nextProp->leadItem.gndUsePowerRight > 0)
            remainEnergy = nextProp->leadItem.gndUsePowerRight - nextProp->chargeFlux[dir16::right];
        else if (txDir == dir16::down && nextProp->leadItem.gndUsePowerUp > 0)
            remainEnergy = nextProp->leadItem.gndUsePowerUp - nextProp->chargeFlux[dir16::up];
        else if (nextProp->leadItem.gndUsePower > 0)
            remainEnergy = nextProp->leadItem.gndUsePower - nextProp->getTotalChargeFlux();

        //통과형 sink 남은 용량 계산
        if (txDir == dir16::right && nextProp->leadItem.gndSinkLeft > 0)
            sinkRemainEnergy = nextProp->leadItem.gndSinkLeft - nextProp->chargeFluxSink[dir16::left];
        else if (txDir == dir16::up && nextProp->leadItem.gndSinkDown > 0)
            sinkRemainEnergy = nextProp->leadItem.gndSinkDown - nextProp->chargeFluxSink[dir16::down];
        else if (txDir == dir16::left && nextProp->leadItem.gndSinkRight > 0)
            sinkRemainEnergy = nextProp->leadItem.gndSinkRight - nextProp->chargeFluxSink[dir16::right];
        else if (txDir == dir16::down && nextProp->leadItem.gndSinkUp > 0)
            sinkRemainEnergy = nextProp->leadItem.gndSinkUp - nextProp->chargeFluxSink[dir16::up];
        else if (nextProp->leadItem.gndSink > 0)
            sinkRemainEnergy = nextProp->leadItem.gndSink - nextProp->getTotalSinkFlux();

        //종단형 GND만 있는 경우 (기존 동작)
        if (remainEnergy > EPSILON && sinkRemainEnergy <= EPSILON)
        {
            double consumeEnergy = std::min(txChargeAmount, remainEnergy);
            transferCharge(donorProp, nextProp, consumeEnergy, indent, txDir, true);
            return consumeEnergy;
        }
        //통과형 sink만 있는 경우
        else if (sinkRemainEnergy > EPSILON && remainEnergy <= EPSILON)
        {
            double sinkConsumeEnergy = std::min(txChargeAmount, sinkRemainEnergy);

            //sink로 전력 소모 (chargeFluxSink에 기록)
            transferCharge(donorProp, nextProp, sinkConsumeEnergy, indent, txDir, true);
            //sink flux 기록
            nextProp->chargeFluxSink[reverse(txDir)] += sinkConsumeEnergy;

            //남은 전력으로 다른 방향 전파 계속
            double afterSinkCharge = txChargeAmount - sinkConsumeEnergy;
            double propagatedCharge = 0;

            if (afterSinkCharge > EPSILON)
            {
                //nextProp를 pathVisited에 추가하지 않음 - 계속 전파를 위해
                std::vector<dir16> possibleDirs;
                Point3 nextPropCoord = { nextProp->getGridX(), nextProp->getGridY(), nextProp->getGridZ() };

                if (nextProp->leadItem.checkFlag(itemFlag::CROSSED_CABLE))
                {
                    if (txDir == dir16::right || txDir == dir16::left) crossStates[nextPropCoord] = crossFlag::horizontal;
                    else if (txDir == dir16::up || txDir == dir16::down) crossStates[nextPropCoord] = crossFlag::vertical;
                }

                for (auto dir : { dir16::right, dir16::up, dir16::left, dir16::down, dir16::above, dir16::below })
                {
                    if (dir == reverse(txDir)) continue;
                    if (isConnected(nextPropCoord, dir))
                    {
                        possibleDirs.push_back(dir);
                    }
                }

                if (possibleDirs.empty() == false)
                {
                    pathVisited.insert(donorProp);
                    propagatedCharge = divideCharge(nextProp, afterSinkCharge, possibleDirs, pathVisited, depth + 1);
                }
            }

            return sinkConsumeEnergy + propagatedCharge;
        }
        //둘 다 있는 경우 - 종단형 GND 우선, 나머지는 sink 통과 후 전파
        else if (remainEnergy > EPSILON && sinkRemainEnergy > EPSILON)
        {
            //먼저 종단형 GND로 소모
            double gndConsumeEnergy = std::min(txChargeAmount, remainEnergy);
            transferCharge(donorProp, nextProp, gndConsumeEnergy, indent, txDir, true);

            double afterGndCharge = txChargeAmount - gndConsumeEnergy;
            double totalConsumed = gndConsumeEnergy;

            //남은 전력이 있으면 sink 통과 후 전파
            if (afterGndCharge > EPSILON)
            {
                double sinkConsumeEnergy = std::min(afterGndCharge, sinkRemainEnergy);
                transferCharge(donorProp, nextProp, sinkConsumeEnergy, indent, txDir, true);
                nextProp->chargeFluxSink[reverse(txDir)] += sinkConsumeEnergy;
                totalConsumed += sinkConsumeEnergy;

                double afterSinkCharge = afterGndCharge - sinkConsumeEnergy;
                if (afterSinkCharge > EPSILON)
                {
                    std::vector<dir16> possibleDirs;
                    Point3 nextPropCoord = { nextProp->getGridX(), nextProp->getGridY(), nextProp->getGridZ() };

                    for (auto dir : { dir16::right, dir16::up, dir16::left, dir16::down, dir16::above, dir16::below })
                    {
                        if (dir == reverse(txDir)) continue;
                        if (isConnected(nextPropCoord, dir))
                        {
                            possibleDirs.push_back(dir);
                        }
                    }

                    if (possibleDirs.empty() == false)
                    {
                        pathVisited.insert(donorProp);
                        totalConsumed += divideCharge(nextProp, afterSinkCharge, possibleDirs, pathVisited, depth + 1);
                    }
                }
            }

            return totalConsumed;
        }
        else return 0;
    }

    double pushedCharge = std::min(txChargeAmount, nextProp->nodeCharge);

    double dividedCharge = 0;
    if (pushedCharge > EPSILON)
    {
        std::vector<dir16> possibleDirs;

        if (nextProp->leadItem.checkFlag(itemFlag::CROSSED_CABLE))
        {
            if (txDir == dir16::right || txDir == dir16::left) crossStates[nextCoord] = crossFlag::horizontal;
            else if (txDir == dir16::up || txDir == dir16::down) crossStates[nextCoord] = crossFlag::vertical;
        }

        for (auto dir : { dir16::right, dir16::up, dir16::left, dir16::down, dir16::above, dir16::below })
        {
            if (dir == reverse(txDir)) continue;
            if (isConnected({ nextProp->getGridX(), nextProp->getGridY(), nextProp->getGridZ() }, dir))
            {
                possibleDirs.push_back(dir);
            }
        }

        if (possibleDirs.empty() == false)
        {
            dividedCharge = divideCharge(nextProp, pushedCharge, possibleDirs, pathVisited, depth + 1);
        }
    }

    double finalTxCharge = std::min(txChargeAmount, nextProp->nodeMaxCharge - nextProp->nodeCharge);
    transferCharge(donorProp,nextProp,finalTxCharge,indent,txDir,false);
    return finalTxCharge;
}


double Prop::divideCharge(Prop* propPtr, double inputCharge, std::vector<dir16> possibleDirs, std::unordered_set<Prop*> pathVisited, int depth)
{
    double totalPushedCharge = 0;
    double remainingCharge = inputCharge;
    std::vector<dir16> dirsToRemove;
    std::vector<dir16> gndDirs;
    std::vector<dir16> nonGndDirs;
    dirsToRemove.reserve(6);
    gndDirs.reserve(6);
    nonGndDirs.reserve(6);

    while (remainingCharge > EPSILON && !possibleDirs.empty())
    {
        dirsToRemove.clear();
        gndDirs.clear();
        nonGndDirs.clear();
        double gndPushedCharge = 0;
        double loopPushedCharge = 0;

        //���� �켱 ���
        for (auto dir : possibleDirs)
        {
            if (isGround({ propPtr->getGridX(), propPtr->getGridY(), propPtr->getGridZ() }, dir))
            {
                gndDirs.push_back(dir);
            }
            else nonGndDirs.push_back(dir);
        }

        if (gndDirs.size() > 0)
        {
            double gndSplitCharge = remainingCharge / gndDirs.size();
            for (auto dir : gndDirs)
            {
                auto newPathVisited = pathVisited;
                double branchPushedCharge = pushCharge(propPtr, dir, gndSplitCharge, newPathVisited, depth);
                gndPushedCharge += branchPushedCharge;
                totalPushedCharge += branchPushedCharge;
                if (branchPushedCharge < EPSILON) dirsToRemove.push_back(dir);
            }

            possibleDirs.erase
            (
                std::remove_if
                (
                    possibleDirs.begin(), 
                    possibleDirs.end(),
                    [&dirsToRemove](dir16 d) { return std::find(dirsToRemove.begin(), dirsToRemove.end(), d) != dirsToRemove.end(); }
                ),
                 possibleDirs.end()
            );

            remainingCharge -= gndPushedCharge;
        }

        dirsToRemove.clear();
        if (possibleDirs.empty()) break;

        if (nonGndDirs.size() > 0)
        {
            double splitCharge = remainingCharge / nonGndDirs.size();
            for (auto dir : nonGndDirs)
            {
                auto newPathVisited = pathVisited;
                double branchPushedCharge = pushCharge(propPtr, dir, splitCharge, newPathVisited, depth);
                loopPushedCharge += branchPushedCharge;
                totalPushedCharge += branchPushedCharge;
                if (branchPushedCharge < EPSILON) dirsToRemove.push_back(dir);
            }

            for (auto dir : dirsToRemove) possibleDirs.erase(std::remove(possibleDirs.begin(), possibleDirs.end(), dir), possibleDirs.end());
            remainingCharge -= loopPushedCharge;
        }

        if (loopPushedCharge < EPSILON && gndPushedCharge < EPSILON) break;
    }

    return totalPushedCharge;
}

void Prop::transferCharge(Prop* thisProp, Prop* nextProp, double txChargeAmount, const std::wstring& indent, dir16 txDir, bool isGroundTransfer = false)
{
    if (txChargeAmount < EPSILON)
    {
        if (debug::printCircuitLog)
        {
            std::wprintf(L"%s[���� ��ŵ] (%d,%d) �� (%d,%d) ��:%.8f (EPSILON �̸�)\n",
                indent.c_str(),
                thisProp->getGridX(), thisProp->getGridY(),
                nextProp->getGridX(), nextProp->getGridY(),
                txChargeAmount);
        }
        return;
    }

    double current = txChargeAmount / (SYSTEM_VOLTAGE * TIME_PER_TURN);
    double electricLoss = current * current * thisProp->leadItem.electricResistance * TIME_PER_TURN;
    double requiredFromDonor = txChargeAmount + electricLoss;
    thisProp->totalLossCharge += electricLoss;

    if (requiredFromDonor > thisProp->nodeCharge + EPSILON)
    {
        double availableRatio = thisProp->nodeCharge / requiredFromDonor;
        requiredFromDonor = thisProp->nodeCharge;
        txChargeAmount *= availableRatio;
        electricLoss = requiredFromDonor - txChargeAmount;
    }

    thisProp->nodeCharge -= requiredFromDonor;
    thisProp->chargeFlux[txDir] -= txChargeAmount;
    if (thisProp->leadItem.itemCode == itemRefCode::powerBankR || thisProp->leadItem.itemCode == itemRefCode::powerBankL)
        thisProp->leadItem.powerStorage -= requiredFromDonor;

    if(isGroundTransfer == false) nextProp->nodeCharge += txChargeAmount;
    nextProp->chargeFlux[reverse(txDir)] += txChargeAmount;

    if (debug::printCircuitLog)
    {
        if (isGroundTransfer)
        {
            std::wprintf(L"%s[���� GND] (%d,%d)[%.2f��%.2f] �� (%d,%d) ����:%.2f �ս�:%.2f ����:%.2f/%d\n",
                indent.c_str(),
                thisProp->getGridX(), thisProp->getGridY(),
                thisProp->nodeCharge + requiredFromDonor, thisProp->nodeCharge,
                nextProp->getGridX(), nextProp->getGridY(),
                txChargeAmount, electricLoss,
                nextProp->getTotalChargeFlux(), nextProp->leadItem.gndUsePower);
        }
        else
        {
            std::wprintf(L"%s[����] (%d,%d)[%.2f��%.2f] �� (%d,%d)[%.2f/%d] ����:%.2f �ս�:%.2f\n",
                indent.c_str(),
                thisProp->getGridX(), thisProp->getGridY(),
                thisProp->nodeCharge + requiredFromDonor, thisProp->nodeCharge,
                nextProp->getGridX(), nextProp->getGridY(),
                nextProp->nodeCharge, nextProp->nodeMaxCharge,
                txChargeAmount, electricLoss);
        }
    }
}

void Prop::initChargeBFS(std::queue<Point3> startPointSet)
{
    std::queue<Point3> frontierQueue = startPointSet;
    std::unordered_set<Point3, Point3::Hash> visitedSet;

    while (!frontierQueue.empty())
    {
        Point3 current = frontierQueue.front();
        frontierQueue.pop();

        if (visitedSet.find(current) != visitedSet.end()) continue;
        visitedSet.insert(current);

        Prop* thisProp = TileProp(current.x, current.y, current.z);
        ItemData& currentItem = thisProp->leadItem;
        if (thisProp)
        {
            thisProp->nodeCharge = thisProp->nodeMaxCharge;
        }

        const dir16 directions[] = { dir16::right, dir16::up, dir16::left, dir16::down, dir16::above, dir16::below };
        for (int i = 0; i < 6; ++i)
        {
            if (isConnected(current, directions[i]))
            {
                int dx, dy, dz;
                dirToXYZ(directions[i], dx, dy, dz);
                Point3 nextCoord = { current.x + dx, current.y + dy, current.z + dz };
                Prop* nextProp = TileProp(nextCoord.x, nextCoord.y, nextCoord.z);
                if (nextProp != nullptr)
                {
                    //다음 BFS에 해당 프롭에서는 추가하지 말 GND와 방향성 GND 상태요구가 만족할 것이 없는 경우
                    ItemData& nextItem = nextProp->leadItem;
                    thisProp->chargeFlux[directions[i]] = 0;
                    nextProp->chargeFlux[reverse(directions[i])] = 0;

                    //통과형 sink flux도 초기화
                    thisProp->chargeFluxSink[directions[i]] = 0;
                    nextProp->chargeFluxSink[reverse(directions[i])] = 0;
                }
                frontierQueue.push(nextCoord);
            }
        }
    }
}


void Prop::loadAct()
{

    int iCode = leadItem.itemCode;

    //��� ����� ����� �� ���Ͽ� ���޵� ���Ϸ��� usePower �̻����� �������� �Ǵ��Ͽ� ���� ������ �����ų� ����
    //�� ��������Ʈ���� ���޵� ���Ϸ��� �ƴ϶� ������ �������� ó��
    if (iCode == itemRefCode::transistorR
        || iCode == itemRefCode::transistorU
        || iCode == itemRefCode::transistorL
        || iCode == itemRefCode::transistorD)
    {
        bool baseInput = false;

        if (iCode == itemRefCode::transistorR && chargeFlux[dir16::right] >= 1.0) baseInput = true;
        else if (iCode == itemRefCode::transistorU && chargeFlux[dir16::up] >= 1.0) baseInput = true;
        else if (iCode == itemRefCode::transistorL && chargeFlux[dir16::left] >= 1.0) baseInput = true;
        else if (iCode == itemRefCode::transistorD && chargeFlux[dir16::down] >= 1.0) baseInput = true;

        if (baseInput)
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_OFF))
            {
                propTurnOn();
            }
        }
        else
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
            {
                propTurnOff();
            }
        }
    }
    else if (iCode == itemRefCode::relayR
        || iCode == itemRefCode::relayU
        || iCode == itemRefCode::relayL
        || iCode == itemRefCode::relayD)
    {
        bool baseInput = false;

        if (iCode == itemRefCode::relayR && chargeFlux[dir16::right] >= 1.0) baseInput = true;
        else if (iCode == itemRefCode::relayU && chargeFlux[dir16::up] >= 1.0) baseInput = true;
        else if (iCode == itemRefCode::relayL && chargeFlux[dir16::left] >= 1.0) baseInput = true;
        else if (iCode == itemRefCode::relayD && chargeFlux[dir16::down] >= 1.0) baseInput = true;

        if (baseInput)
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_OFF))
            {
                propTurnOn();
            }
        }
        else
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
            {
                propTurnOff();
            }
        }
    }
    else if (iCode == itemRefCode::andGateR || iCode == itemRefCode::andGateL)
    {
        bool firstInput, secondInput;

        if (iCode == itemRefCode::andGateR)
        {
            firstInput = chargeFlux[dir16::left] >= 1.0;
            secondInput = chargeFlux[dir16::down] >= 1.0;
        }
        else
        {
            firstInput = chargeFlux[dir16::right] >= 1.0;
            secondInput = chargeFlux[dir16::down] >= 1.0;
        }

        if (firstInput && secondInput)
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_OFF))
            {
                propTurnOn();
            }
        }
        else
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
            {
                propTurnOff();
            }
        }
    }
    else if (iCode == itemRefCode::orGateR || iCode == itemRefCode::orGateL)
    {
        bool firstInput, secondInput;

        if (iCode == itemRefCode::orGateR)
        {
            firstInput = chargeFlux[dir16::left] >= 1.0;
            secondInput = chargeFlux[dir16::down] >= 1.0;
        }
        else
        {
            firstInput = chargeFlux[dir16::right] >= 1.0;
            secondInput = chargeFlux[dir16::down] >= 1.0;
        }

        if (firstInput || secondInput)
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_OFF))
            {
                propTurnOn();
            }
        }
        else
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
            {
                propTurnOff();
            }
        }
    }
    else if (iCode == itemRefCode::xorGateR || iCode == itemRefCode::xorGateL)
    {
        bool firstInput, secondInput;

        if (iCode == itemRefCode::xorGateR)
        {
            firstInput = chargeFlux[dir16::left] >= 1.0;
            secondInput = chargeFlux[dir16::down] >= 1.0;
        }
        else
        {
            firstInput = chargeFlux[dir16::right] >= 1.0;
            secondInput = chargeFlux[dir16::down] >= 1.0;
        }

        if (firstInput != secondInput)
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_OFF))
            {
                propTurnOn();
            }
        }
        else
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
            {
                propTurnOff();
            }
        }
    }
    else if (iCode == itemRefCode::notGateR || iCode == itemRefCode::notGateL)
    {
        bool inputActive;
        if (iCode == itemRefCode::notGateR) inputActive = chargeFlux[dir16::left] >= 1.0;
        else inputActive = chargeFlux[dir16::right] >= 1.0;

        if (inputActive == false)
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_OFF))
            {
                propTurnOn();
            }
        }
        else
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
            {
                propTurnOff();
            }
        }
    }
    else if (iCode == itemRefCode::srLatchR || iCode == itemRefCode::srLatchL)
    {
        bool setInput, resetInput;

        if (iCode == itemRefCode::srLatchR)
        {
            setInput = chargeFlux[dir16::left] >= 1.0;
            resetInput = chargeFlux[dir16::down] >= 1.0;
        }
        else
        {
            setInput = chargeFlux[dir16::right] >= 1.0;
            resetInput = chargeFlux[dir16::down] >= 1.0;
        }

        if (setInput && resetInput) // �������´� ����
        {
            if (randomRange(0, 1) == 0)
            {
                if (leadItem.checkFlag(itemFlag::PROP_POWER_OFF))
                    propTurnOn();
            }
            else
            {
                if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
                    propTurnOff();
            }
        }
        else if (setInput) // Set
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_OFF))
                propTurnOn();
        }
        else if (resetInput) // Reset
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
                propTurnOff();
        }
    }
    else if (iCode == itemRefCode::delayR || iCode == itemRefCode::delayL)
    {
        reserveDelayInit.erase(this);

        bool inputActive;
        if (iCode == itemRefCode::delayR) inputActive = chargeFlux[dir16::left] >= 1.0;
        else inputActive = chargeFlux[dir16::right] >= 1.0;

        if (inputActive == true)
        {
            if (delayStartTurn == 0.0) delayStartTurn = getElapsedTurn();

            if (leadItem.checkFlag(itemFlag::PROP_POWER_OFF))
            {
                if (getElapsedTurn() - delayStartTurn >= static_cast<double>(delayMaxStack) - EPSILON) propTurnOn();
            }
        }
        else
        {
            reserveDelayInit.insert(this);
            if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
            {
                propTurnOff();
            }
        }
    }
    else if (iCode == itemRefCode::powerBankR || iCode == itemRefCode::powerBankL) //�Ŀ���ũ ������ ��� ���Ͽ� �̴��ص� �۵�
    {
        ItemData& loadItem = leadItem;
        loadItem.powerStorage += getInletCharge();

        constexpr double CHARGE_EPSILON = 0.5;
        if (loadItem.powerStorage >= loadItem.powerStorageMax - CHARGE_EPSILON)
        {
            loadItem.powerStorage = loadItem.powerStorageMax;
        }
    }
    else if (iCode == itemRefCode::chargingPort)
    {
        ItemStack* hereStack = TileItemStack(getGridX(), getGridY(), getGridZ());
        if (hereStack != nullptr)
        {
            std::vector<ItemData>& items = hereStack->getPocket()->itemInfo;
            double inletCharge = getInletCharge();
            if (inletCharge > 0)
            {
                // ���� ������ ������ �ε��� ����
                std::vector<int> chargeableIndices;
                for (int i = 0; i < items.size(); i++)
                {
                    if (items[i].itemCode == itemRefCode::battery || items[i].itemCode == itemRefCode::batteryPack)
                    {
                        if (items[i].powerStorage < items[i].powerStorageMax)
                        {
                            chargeableIndices.push_back(i);
                        }
                    }
                }

                // �յ� �й�
                if (chargeableIndices.size() > 0)
                {
                    double chargePerItem = inletCharge / chargeableIndices.size();

                    for (int idx : chargeableIndices)
                    {
                        ItemData& item = items[idx];
                        double remaining = item.powerStorageMax - item.powerStorage;
                        double actualCharge = std::min(chargePerItem, remaining);

                        item.powerStorage += actualCharge;

                        constexpr double CHARGE_EPSILON = 0.5;
                        if (item.powerStorage >= item.powerStorageMax - CHARGE_EPSILON)
                        {
                            item.powerStorage = item.powerStorageMax;
                        }
                    }
                }
            }
        }
        }
    else //일반적인 부하들은 그라운드에서의 usePower 이상이면 작동함 아니면 끔
    {
        //종단형 GND 체크
        bool gndPowerSufficient = false;
        if (leadItem.gndUsePower > 0)
        {
            gndPowerSufficient = getTotalChargeFlux() >= static_cast<double>(leadItem.gndUsePower);
        }

        //통과형 sink 체크
        bool sinkPowerSufficient = false;
        if (leadItem.gndSink > 0)
        {
            sinkPowerSufficient = getTotalSinkFlux() >= leadItem.gndSink;
        }

        //방향성 sink 체크
        if (leadItem.gndSinkRight > 0 && chargeFluxSink[dir16::right] >= leadItem.gndSinkRight)
            sinkPowerSufficient = true;
        if (leadItem.gndSinkUp > 0 && chargeFluxSink[dir16::up] >= leadItem.gndSinkUp)
            sinkPowerSufficient = true;
        if (leadItem.gndSinkLeft > 0 && chargeFluxSink[dir16::left] >= leadItem.gndSinkLeft)
            sinkPowerSufficient = true;
        if (leadItem.gndSinkDown > 0 && chargeFluxSink[dir16::down] >= leadItem.gndSinkDown)
            sinkPowerSufficient = true;

        //둘 중 하나라도 충족되면 작동
        if (gndPowerSufficient || sinkPowerSufficient)
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_OFF))
            {
                propTurnOn();
            }
        }
        else
        {
            if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
            {
                propTurnOff();
            }
        }
    }
}

