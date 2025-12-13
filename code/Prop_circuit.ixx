import Prop;

import std;
import util;
import globalVar;
import constVar;
import wrapVar;


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
* leverRL : �¿츸 ����
* leverUD : ���ϸ� ����
* 
* Ʈ�������Ϳ� ��������Ʈ�� �����̿� �ٸ��� ���� ���� �ÿ� ��� ȸ�ΰ� ����Ǿ�� ��
* Ʈ�������Ϳ� ��������Ʈ�� ����Ʈ <-> ���� ���ΰ� ���İ� �� �ǰ� �� ������ ��
* ����Ʈ->���ζ������� ������ �ȵǰ� �Ϸ��� BFS Ž�� �������� skipBFSSet�� �߰����ָ� ��
* ���ζ���->����Ʈ�� ������ �ȵǰ� �Ϸ��� isConnected �Լ����� üũ���ָ� ��
*/


std::unordered_set<Prop*> Prop::updateCircuitNetwork()
{
    if(debug::printCircuitLog) std::wprintf(L"------------------------- ȸ�θ� ������Ʈ ���� ------------------------\n");
    int cursorX = getGridX();
    int cursorY = getGridY();
    int cursorZ = getGridZ();

    std::queue<Point3> frontierQueue;
    std::unordered_set<Point3, Point3::Hash> visitedSet;
    std::vector<Prop*> voltagePropVec;

    std::unordered_set<Point3, Point3::Hash> skipBFSSet;
    std::unordered_set<Prop*> loadSet; //���ϰ� �������� ���ڱ���

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
    
    while (!frontierQueue.empty())
    {

        Point3 current = frontierQueue.front();
        frontierQueue.pop();

        if (visitedSet.find(current) != visitedSet.end()) continue;
        visitedSet.insert(current);

        Prop* currentProp = TileProp(current.x, current.y, current.z);
        if (debug::printCircuitLog) std::wprintf(L"[BFS Ž��] %ls (%d,%d,%d) \n", currentProp->leadItem.name.c_str(), current.x, current.y, current.z);

        if (currentProp && (currentProp->leadItem.checkFlag(itemFlag::CIRCUIT) || currentProp->leadItem.checkFlag(itemFlag::CABLE)))
        {
            currentProp->runUsed = true;

            if (currentProp->leadItem.checkFlag(itemFlag::VOLTAGE_SOURCE))
            {
                if (currentProp->leadItem.checkFlag(itemFlag::PROP_POWER_ON) &&
                    currentProp->leadItem.checkFlag(itemFlag::PROP_POWER_OFF) == false)
                {
                    circuitMaxEnergy += currentProp->leadItem.electricMaxPower;
                    voltagePropVec.push_back(currentProp);
                }
            }

            if (currentProp->leadItem.electricUsePower > 0)
            {
                ItemData& currentLeadItem = currentProp->leadItem;
                int iCode = currentLeadItem.itemCode;

                if (iCode == itemRefCode::transistorR
                    || iCode == itemRefCode::transistorU
                    || iCode == itemRefCode::transistorL
                    || iCode == itemRefCode::transistorD
                    || iCode == itemRefCode::andGateR 
                    || iCode == itemRefCode::andGateL
                    || iCode == itemRefCode::orGateR
                    || iCode == itemRefCode::orGateL
                    || iCode == itemRefCode::xorGateR
                    || iCode == itemRefCode::xorGateL
                    || iCode == itemRefCode::notGateR
                    || iCode == itemRefCode::notGateL
                    )
                {
                    Point3 currentCoord = { current.x, current.y, current.z };
                    Point3 rightCoord = { current.x + 1, current.y, current.z };
                    Point3 upCoord = { current.x, current.y - 1, current.z };
                    Point3 leftCoord = { current.x - 1, current.y, current.z };
                    Point3 downCoord = { current.x, current.y + 1, current.z };

                    if (currentLeadItem.checkFlag(itemFlag::VOLTAGE_GND_RIGHT))
                    {
                        Prop* rightProp = TileProp(rightCoord.x, rightCoord.y, rightCoord.z);
                        // NOT 게이트는 입력핀 방향에 회로 컴포넌트가 존재하기만 하면 loadSet에 추가
                        // (레버가 OFF여서 visitedSet에 없더라도 입력 상태를 체크해야 함)
                        if(visitedSet.find(rightCoord) != visitedSet.end() ||
                           (iCode == itemRefCode::notGateL && rightProp != nullptr &&
                            (rightProp->leadItem.checkFlag(itemFlag::CIRCUIT) || rightProp->leadItem.checkFlag(itemFlag::CABLE))))
                        {
                            loadSet.insert(currentProp);
                        }
                    }
                    if (currentLeadItem.checkFlag(itemFlag::VOLTAGE_GND_UP))
                    {
                        if(visitedSet.find(upCoord) != visitedSet.end())
                        {
                            loadSet.insert(currentProp);
                        }
                    }
                    if (currentLeadItem.checkFlag(itemFlag::VOLTAGE_GND_LEFT))
                    {
                        Prop* leftProp = TileProp(leftCoord.x, leftCoord.y, leftCoord.z);
                        // NOT 게이트는 입력핀 방향에 회로 컴포넌트가 존재하기만 하면 loadSet에 추가
                        if(visitedSet.find(leftCoord) != visitedSet.end() ||
                           (iCode == itemRefCode::notGateR && leftProp != nullptr &&
                            (leftProp->leadItem.checkFlag(itemFlag::CIRCUIT) || leftProp->leadItem.checkFlag(itemFlag::CABLE))))
                        {
                            loadSet.insert(currentProp);
                        }
                    }
                    if (currentLeadItem.checkFlag(itemFlag::VOLTAGE_GND_DOWN))
                    {
                        if(visitedSet.find(downCoord) != visitedSet.end())
                        {
                            loadSet.insert(currentProp);
                        }
                    }
                }
                else loadSet.insert(currentProp);
                
            }

            //��Ʈ����ġ�� ��� ���� �� ���� �ÿ� ����
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


            const dir16 directions[] = { dir16::right, dir16::up, dir16::left, dir16::down, dir16::ascend, dir16::descend };
            const itemFlag groundFlags[][2] = {
                { itemFlag::VOLTAGE_GND_LEFT, itemFlag::VOLTAGE_GND_ALL },
                { itemFlag::VOLTAGE_GND_DOWN, itemFlag::VOLTAGE_GND_ALL },
                { itemFlag::VOLTAGE_GND_RIGHT, itemFlag::VOLTAGE_GND_ALL },
                { itemFlag::VOLTAGE_GND_UP, itemFlag::VOLTAGE_GND_ALL },
                { itemFlag::VOLTAGE_GND_ALL, itemFlag::VOLTAGE_GND_ALL },
                { itemFlag::VOLTAGE_GND_ALL, itemFlag::VOLTAGE_GND_ALL }
            };


            if (skipBFSSet.find(current) == skipBFSSet.end())
            {
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
                        }

                        frontierQueue.push(nextCoord);

                        if (nextProp &&
                            (nextProp->leadItem.checkFlag(groundFlags[i][0]) ||
                                nextProp->leadItem.checkFlag(groundFlags[i][1])))
                        {
                            hasGround = true;
                        }
                    }
                }
            }
            else skipBFSSet.erase(current);
                
        }
    }

    for(auto propPtr : loadSet)
    {
        circuitTotalLoad += propPtr->leadItem.electricUsePower;
    }

    // ��尡 2�� �̸��̸� ������� ����
    if (visitedSet.size() < 2)
    {
        runUsed = true;
        return loadSet;
    }


    //std::wprintf(L"\n����������������������������������������������������������������������\n");
    //std::wprintf(L"�� ȸ�� �м� �Ϸ�                  ��\n");
    //std::wprintf(L"����������������������������������������������������������������������\n");
    //std::wprintf(L"�� ��� ��: %3d                    ��\n", visitedSet.size());
    //std::wprintf(L"�� �ִ� ����: %3d kJ               ��\n", circuitMaxEnergy);
    //std::wprintf(L"�� ���� ����: %s                   ��\n", hasGround ? L"Yes" : L"No ");
    //std::wprintf(L"����������������������������������������������������������������������\n\n");

    //==============================================================================
    // 2. �ִ� ���� ����
    //==============================================================================
    for (auto coord : visitedSet)
    {
        Prop* propPtr = TileProp(coord.x, coord.y, coord.z);
        if (propPtr != nullptr)
        {
            
            propPtr->nodeMaxCharge = circuitMaxEnergy;
        
            //����ȸ�δ� �׻� ���� ���� �� ����
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
            if (voltProp->leadItem.checkFlag(itemFlag::VOLTAGE_OUTPUT_RIGHT) && isConnected({ x,y,z }, dir16::right))
                voltProp->prevPushedCharge += pushCharge(voltProp, dir16::right, voltOutputPower, {}, 0);
            else if (voltProp->leadItem.checkFlag(itemFlag::VOLTAGE_OUTPUT_UP) && isConnected({ x,y,z }, dir16::up))
                voltProp->prevPushedCharge += pushCharge(voltProp, dir16::up, voltOutputPower, {}, 0);
            else if (voltProp->leadItem.checkFlag(itemFlag::VOLTAGE_OUTPUT_LEFT) && isConnected({ x,y,z }, dir16::left))
                voltProp->prevPushedCharge += pushCharge(voltProp, dir16::left, voltOutputPower, {}, 0);
            else if (voltProp->leadItem.checkFlag(itemFlag::VOLTAGE_OUTPUT_DOWN) && isConnected({ x,y,z }, dir16::down))
                voltProp->prevPushedCharge += pushCharge(voltProp, dir16::down, voltOutputPower, {}, 0);

            totalPushedCharge += voltProp->prevPushedCharge;
            voltProp->nodeCharge = voltProp->nodeMaxCharge;
        }
    }

    return loadSet;
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
    case dir16::ascend:
        delCoord = { 0,0,+1 };
        hostFlag = itemFlag::CABLE_Z_ASCEND;
        guestFlag = itemFlag::CABLE_Z_DESCEND;
        break;
    case dir16::descend:
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


    if (dir == dir16::ascend || dir == dir16::descend)
    {
        if (currentProp->leadItem.checkFlag(itemFlag::CABLE) && currentProp->leadItem.checkFlag(hostFlag) &&
            targetProp->leadItem.checkFlag(itemFlag::CABLE) && targetProp->leadItem.checkFlag(guestFlag))
        {
            return true;
        }
        else return false;
    }
    else if (dir == dir16::right || dir == dir16::up || dir == dir16::left || dir == dir16::down)
    {
        if ((currentProp->leadItem.checkFlag(itemFlag::CABLE) || currentProp->leadItem.checkFlag(hostFlag)) &&
            (targetProp->leadItem.checkFlag(itemFlag::CABLE) || targetProp->leadItem.checkFlag(guestFlag)))
            return true;
        else return false;
    }
    else errorBox(L"[Error] isConnected lambda function received invalid direction argument.\n");
}

bool Prop::isGround(Point3 currentCoord, dir16 dir)
{
    Prop* currentProp = TileProp(currentCoord.x, currentCoord.y, currentCoord.z);
    Point3 delCoord = { 0,0,0 };
    itemFlag groundFlag;
    switch (dir)
    {
    case dir16::right:
        delCoord = { +1,0,0 };
        groundFlag = itemFlag::VOLTAGE_GND_LEFT;
        break;
    case dir16::up:
        delCoord = { 0,-1,0 };
        groundFlag = itemFlag::VOLTAGE_GND_DOWN;
        break;
    case dir16::left:
        delCoord = { -1,0,0 };
        groundFlag = itemFlag::VOLTAGE_GND_RIGHT;
        break;
    case dir16::down:
        delCoord = { 0,+1,0 };
        groundFlag = itemFlag::VOLTAGE_GND_UP;
        break;
    case dir16::ascend:
    case dir16::descend:
        groundFlag = itemFlag::VOLTAGE_GND_ALL;
        break;
    default:
        errorBox(L"[Error] isGround lambda function received invalid direction argument.\n");
        break;
    }
    Prop* targetProp = TileProp(currentCoord.x + delCoord.x, currentCoord.y + delCoord.y, currentCoord.z + delCoord.z);
    if (targetProp == nullptr) return false;
    if (targetProp->leadItem.checkFlag(groundFlag) || targetProp->leadItem.checkFlag(itemFlag::VOLTAGE_GND_ALL))
    {
        //�ߺ��ΰ�? ����ȭ�� ���ؼ� ��� �� ����
        if (isConnected(currentCoord, dir)) return true;
    }
    else return false;
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
    Prop* acceptorProp = TileProp(donorProp->getGridX() + dx, donorProp->getGridY() + dy, donorProp->getGridZ() + dz);
    errorBox(acceptorProp == nullptr, L"[Error] pushCharge: no acceptor found\n");

    txChargeAmount = std::min(donorProp->nodeCharge, txChargeAmount);

    errorBox(txChargeAmount > donorProp->nodeCharge + EPSILON, L"[Error] pushCharge: insufficient electron\n");
    errorBox(!isConnected({ donorProp->getGridX(), donorProp->getGridY(), donorProp->getGridZ() }, txDir),
        L"[Error] pushCharge: not connected\n");

    if (pathVisited.find(donorProp) != pathVisited.end()) return 0;
    pathVisited.insert(donorProp);
    if (pathVisited.find(acceptorProp) != pathVisited.end()) return 0;

    // �鿩���� ����
    std::wstring indent(depth * 2, L' ');  // depth���� 2ĭ��

    if (debug::printCircuitLog) std::wprintf(L"%s[PUSH] (%d,%d) �� (%d,%d) �õ�: %.2f\n",
        indent.c_str(),
        donorProp->getGridX(), donorProp->getGridY(),
        acceptorProp->getGridX(), acceptorProp->getGridY(),
        txChargeAmount);

    bool isGrounded = false;
    if (acceptorProp->leadItem.checkFlag(itemFlag::VOLTAGE_GND_ALL)) isGrounded = true;
    else if (txDir == dir16::right && acceptorProp->leadItem.checkFlag(itemFlag::VOLTAGE_GND_LEFT))
    {
        isGrounded = true;
    }
    else if (txDir == dir16::up && acceptorProp->leadItem.checkFlag(itemFlag::VOLTAGE_GND_DOWN))
    {
        isGrounded = true;
    }
    else if (txDir == dir16::left && acceptorProp->leadItem.checkFlag(itemFlag::VOLTAGE_GND_RIGHT))
    {
        isGrounded = true;
    }
    else if (txDir == dir16::down && acceptorProp->leadItem.checkFlag(itemFlag::VOLTAGE_GND_UP))
    {
        isGrounded = true;
    }

    if (isGrounded)
    {
        double remainEnergy = acceptorProp->leadItem.electricUsePower - acceptorProp->groundCharge;
        int iCode = acceptorProp->leadItem.itemCode;

        if (iCode == itemRefCode::andGateR 
            || iCode == itemRefCode::andGateL
            || iCode == itemRefCode::orGateR
            || iCode == itemRefCode::orGateL
            || iCode == itemRefCode::xorGateR
            || iCode == itemRefCode::xorGateL
            || iCode == itemRefCode::notGateR
            || iCode == itemRefCode::notGateL
            )
        {
            if (txDir == dir16::right && acceptorProp->leadItem.checkFlag(itemFlag::GND_ACTIVE_LEFT) == false)
            {
                remainEnergy = 1.0;
                acceptorProp->leadItem.addFlag(itemFlag::GND_ACTIVE_LEFT);
            }
            else if (txDir == dir16::down && acceptorProp->leadItem.checkFlag(itemFlag::GND_ACTIVE_UP) == false)
            {
                remainEnergy = 1.0;
                acceptorProp->leadItem.addFlag(itemFlag::GND_ACTIVE_UP);
            }
            else if (txDir == dir16::left && acceptorProp->leadItem.checkFlag(itemFlag::GND_ACTIVE_RIGHT) == false)
            {
                remainEnergy = 1.0;
                acceptorProp->leadItem.addFlag(itemFlag::GND_ACTIVE_RIGHT);
            }
            else if (txDir == dir16::up && acceptorProp->leadItem.checkFlag(itemFlag::GND_ACTIVE_DOWN) == false)
            {
                remainEnergy = 1.0;
                acceptorProp->leadItem.addFlag(itemFlag::GND_ACTIVE_DOWN);
            }
            else remainEnergy = 0.0;  
        }

        if (remainEnergy > EPSILON)
        {
            double consumeEnergy = std::min(txChargeAmount, remainEnergy);
            transferCharge(donorProp, acceptorProp, consumeEnergy, indent, true);
            return consumeEnergy;
        }
        else return 0;
    }

    double pushedCharge = std::min(txChargeAmount, acceptorProp->nodeCharge);

    double dividedCharge = 0;
    if (pushedCharge > EPSILON)
    {
        std::vector<dir16> possibleDirs;
        for (auto dir : { dir16::right, dir16::up, dir16::left, dir16::down, dir16::ascend, dir16::descend })
        {
            if (dir == reverse(txDir)) continue;
            if (isConnected({ acceptorProp->getGridX(), acceptorProp->getGridY(), acceptorProp->getGridZ() }, dir))
            {
                possibleDirs.push_back(dir);
            }
        }

        if (possibleDirs.empty() == false)
        {
            dividedCharge = divideCharge(acceptorProp, pushedCharge, possibleDirs, pathVisited, depth + 1);
        }
    }

    double finalTxCharge = std::min(txChargeAmount, acceptorProp->nodeMaxCharge - acceptorProp->nodeCharge);
    transferCharge(donorProp,acceptorProp,finalTxCharge,indent,false);
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

void Prop::transferCharge(Prop* donorProp, Prop* acceptorProp, double txChargeAmount, const std::wstring& indent, bool isGroundTransfer = false)
{
    if (txChargeAmount < EPSILON)
    {
        if (debug::printCircuitLog)
        {
            std::wprintf(L"%s[���� ��ŵ] (%d,%d) �� (%d,%d) ��:%.8f (EPSILON �̸�)\n",
                indent.c_str(),
                donorProp->getGridX(), donorProp->getGridY(),
                acceptorProp->getGridX(), acceptorProp->getGridY(),
                txChargeAmount);
        }
        return;
    }

    double current = txChargeAmount / (SYSTEM_VOLTAGE * TIME_PER_TURN);
    double electricLoss = current * current * acceptorProp->leadItem.electricResistance * TIME_PER_TURN;
    double requiredFromDonor = txChargeAmount + electricLoss;

    if (requiredFromDonor > donorProp->nodeCharge + EPSILON)
    {
        double availableRatio = donorProp->nodeCharge / requiredFromDonor;
        requiredFromDonor = donorProp->nodeCharge;
        txChargeAmount *= availableRatio;
        electricLoss = requiredFromDonor - txChargeAmount;
    }

    donorProp->nodeCharge -= requiredFromDonor;
    donorProp->nodeOutputCharge += requiredFromDonor;

    if (isGroundTransfer) acceptorProp->groundCharge += txChargeAmount;
    else acceptorProp->nodeCharge += txChargeAmount;

    acceptorProp->nodeInputCharge += txChargeAmount;

    if (debug::printCircuitLog)
    {
        if (isGroundTransfer)
        {
            std::wprintf(L"%s[���� GND] (%d,%d)[%.2f��%.2f] �� (%d,%d) ����:%.2f �ս�:%.2f ����:%.2f/%d\n",
                indent.c_str(),
                donorProp->getGridX(), donorProp->getGridY(),
                donorProp->nodeCharge + requiredFromDonor, donorProp->nodeCharge,
                acceptorProp->getGridX(), acceptorProp->getGridY(),
                txChargeAmount, electricLoss,
                acceptorProp->groundCharge, acceptorProp->leadItem.electricUsePower);
        }
        else
        {
            std::wprintf(L"%s[����] (%d,%d)[%.2f��%.2f] �� (%d,%d)[%.2f/%d] ����:%.2f �ս�:%.2f\n",
                indent.c_str(),
                donorProp->getGridX(), donorProp->getGridY(),
                donorProp->nodeCharge + requiredFromDonor, donorProp->nodeCharge,
                acceptorProp->getGridX(), acceptorProp->getGridY(),
                acceptorProp->nodeCharge, acceptorProp->nodeMaxCharge,
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

        Prop* currentProp = TileProp(current.x, current.y, current.z);
        if (currentProp)
        {
            currentProp->nodeCharge = currentProp->nodeMaxCharge;
            currentProp->groundCharge = 0;
            currentProp->nodeInputCharge = 0;
            currentProp->nodeOutputCharge = 0;
        }

        const dir16 directions[] = { dir16::right, dir16::up, dir16::left, dir16::down, dir16::ascend, dir16::descend };
        for (int i = 0; i < 6; ++i)
        {
            if (isConnected(current, directions[i]))
            {
                int dx, dy, dz;
                dirToXYZ(directions[i], dx, dy, dz);
                Point3 nextCoord = { current.x + dx, current.y + dy, current.z + dz };
                frontierQueue.push(nextCoord);
            }
        }
    }
}

