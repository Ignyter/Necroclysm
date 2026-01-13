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
* relayR,U,L,D : 트랜지스터와 동일한 방향성 가짐
* 
* 
* <트랜지스터>
* transistorR : 게이트핀이 우측, 상단과 하단이 메인라인
* transistorU : 게이트핀이 상단, 좌측과 우측이 메인라인
* transistorL : 게이트핀이 좌측, 상단과 하단이 메인라인
* transistorD : 게이트핀이 하단, 좌측과 우측이 메인라인
* 
* <논리게이트>
* andGateR : 출력핀이 우측(R), vcc 입력은 항상 상단, 좌측과 하단은 입력핀
* andGateL : 출력핀이 좌측(L), vcc 입력은 항상 상단, 우측과 하단은 입력핀
* andGate는 단순히 소모전력이 2고, 각 핀은 최대 1을 받는데 받은 전력이 2 이상이면 작동함
* 
* orGateR & orGateL : andGate와 동일, 다만 ON/OFF 판정을 ACTIVE 플래그로 함
* xorGateR & xorGateL : andGate와 동일, 다만 ON/OFF 판정을 ACTIVE 플래그로 함
* notGateR : 출력핀이 우측(R), vcc 입력은 항상 상단, 좌측은 입력핀(GND)
* notGateL : 출력핀이 좌측(L), vcc 입력은 항상 상단, 우측은 입력핀(GND)
* 
* srLatchR : 출력핀이 우측(R), vcc 입력은 항상 상단, 좌측은 S(set)핀 하단은 R(reset)핀
* srLatchL : 출력핀이 좌측(L), vcc 입력은 항상 상단, 우측은 S(set)핀 하단은 R(reset)핀
* 
* leverRL : 좌우만 연결
* leverUD : 상하만 연결
* 
* 트랜지스터와 논리게이트는 릴레이와 다르게 상태 변경 시에 모든 회로가 재계산되어야 함
* 트랜지스터와 논리게이트는 게이트 <-> 메인 라인간 전파가 안 되게 잘 수정할 것
* 게이트->메인라인으로 누설이 안되게 하려면 BFS 탐색 과정에서 skipBFSSet에 추가해주면 됨
* 메인라인->게이트로 누설이 안되게 하려면 isConnected 함수에서 체크해주면 됨
* 
* <파워뱅크>
* 파워뱅크는 출력과 접지(충전부)가 한 타일에 동시에 있는 특이한 구조임.
* powerBankR : 왼쪽이 출력, 오른쪽이 충전입력(접지)
* powerBankL : 오른쪽이 출력, 왼쪽이 충전입력(접지)
* 
* <그라운드>
* gndUsePower는 전방향(상하좌우, z축 제외) 입력임. gndUsePower>0이면 반드시 지향성 접지가 존재하지 않음에 유의
* 반대로 지향성 접지가 존재하면 당연히 gndUsePower는 0과 같음
*/


/*
* <BFS 작동 로직>
* 1. BFS 탐색으로 연결된 모든 회로망을 탐색
* 2. 회로망 내의 모든 전압원의 가용 전력 계산
* 3. 회로망 내의 모든 부하의 총 부하량 계산 (지향성 부하 포함)
* 4. 각각의 전압원이 필요한만큼 전하를 밀어냄 (저항손실을 고려해 약간 더 많이)
* 
*/

const wchar_t* dirToArrow(dir16 dir)
{
    switch (dir) {
    case dir16::right: return L"→";
    case dir16::up:    return L"↑";
    case dir16::left:  return L"←";
    case dir16::down:  return L"↓";
    case dir16::above: return L"↗";
    case dir16::below: return L"↘";
    default: return L"?";
    }
}

bool Prop::hasGround()
{
    //일반 접지
    if (leadItem.gndUsePower > 0) return true;
    //지향성 접지
    if (leadItem.gndUsePowerRight > 0) return true;
    if (leadItem.gndUsePowerUp > 0) return true;
    if (leadItem.gndUsePowerLeft > 0) return true;
    if (leadItem.gndUsePowerDown > 0) return true;

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
    if(debug::printCircuitLog) std::wprintf(L"------------------------- 회로망 업데이트 시작 ------------------------\n");
    int cursorX = getGridX();
    int cursorY = getGridY();
    int cursorZ = getGridZ();

    std::queue<Point3> frontierQueue;
    std::unordered_set<Point3, Point3::Hash> visitedSet;
    std::vector<Prop*> voltagePropVec;

    std::unordered_set<Point3, Point3::Hash> skipBFSSet;

    int circuitMaxEnergy = 0;
    int circuitTotalLoad = 0;

    //==============================================================================
    // 1. 회로 최초 탐색(BFS)
    //==============================================================================

    if(saveFrontierQueue.size()>0 && saveVisitedSet.size()>0)
    {
        if (debug::printCircuitLog) std::wprintf(L"------------------------- 이전 회로망 탐색 결과 불러오기 ------------------------\n");
        frontierQueue = saveFrontierQueue;
        visitedSet = saveVisitedSet;
    }
    else frontierQueue.push({ cursorX, cursorY, cursorZ });
    
    //BFS는 전선에서 시작하지 않음(CROSS 케이블때문에), 부하나 전압원에서 항상 시작할 것
    while (!frontierQueue.empty())
    {
        
        Point3 current = frontierQueue.front();
        frontierQueue.pop();
        crossStates.clear();

        Prop* currentProp = TileProp(current.x, current.y, current.z);

        if (visitedSet.find(current) != visitedSet.end()) continue;
        visitedSet.insert(current);

        if (debug::printCircuitLog) std::wprintf(L"[BFS 탐색] %ls (%d,%d,%d) \n", currentProp->leadItem.name.c_str(), current.x, current.y, current.z);
        if (currentProp == nullptr)
        {
            std::wprintf(L"[경고] BFS가 nullptr 프롭에 도달함 (%d,%d,%d)\n", current.x, current.y, current.z);
            continue;
        }

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
            if (currentProp->leadItem.itemCode == itemRefCode::chargingPort)//차징포트일 경우...
            {
                currentProp->leadItem.gndUsePower = 1;
                ItemStack* hereStack = TileItemStack(current.x, current.y, current.z);  // ← 수정
                if (hereStack != nullptr)
                {
                    std::vector<ItemData>& hereItems = hereStack->getPocket()->itemInfo;  // ← 수정 (중복 호출 제거)
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

            //현재 프롭이 소비전력이 있을 경우 loadSet에 추가
            //뒤의 isConnect 6방향 체크에 지향성 부하 loadSet 추가 메커니즘이 있음(주의할 것)
            if (currentProp->leadItem.gndUsePower > 0)
            {
                circuitTotalLoad += currentProp->leadItem.gndUsePower;
            }

            //택트 스위치일 경우 다음 턴 시작 시에 종료
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
                
                //아이템 무게
                ItemStack* tgtItemStack = TileItemStack(current.x, current.y, current.z);
                if (tgtItemStack != nullptr)
                {
                    auto& tileItemInfo = tgtItemStack->getPocket()->itemInfo;
                    for (int i = 0; i < tileItemInfo.size(); i++)
                    {
                        totalWeight += tileItemInfo[i].weight * tileItemInfo[i].number;
                    }
                }

                //엔티티 무게
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
                            //베이스에서 메인라인으로 BFS를 추가하는 것을 막음
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

                            //파워뱅크 충전속도 제한
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

                            if (directions[i] == dir16::right && nextProp->leadItem.gndUsePowerLeft > 0)
                            {
                                circuitTotalLoad += nextProp->leadItem.gndUsePowerLeft;
                            }
                            else if (directions[i] == dir16::up && nextProp->leadItem.gndUsePowerDown > 0)
                            {
                                circuitTotalLoad += nextProp->leadItem.gndUsePowerDown;
                            }
                            else if (directions[i] == dir16::left && nextProp->leadItem.gndUsePowerRight > 0)
                            {
                                circuitTotalLoad += nextProp->leadItem.gndUsePowerRight;
                            }
                            else if (directions[i] == dir16::down && nextProp->leadItem.gndUsePowerUp > 0)
                            {
                                circuitTotalLoad += nextProp->leadItem.gndUsePowerUp;
                            }
                        }

                        
                        if (nextProp->leadItem.checkFlag(itemFlag::CROSSED_CABLE))
                        {
                            if (directions[i] == dir16::down || directions[i] == dir16::up) crossStates[nextCoord] = crossFlag::vertical;
                            else if (directions[i] == dir16::right || directions[i] == dir16::left) crossStates[nextCoord] = crossFlag::horizontal;
                        }
                        frontierQueue.push(nextCoord);
                        
                    }
                }
            }
            else skipBFSSet.erase(current);
                
        }
    }

    //==============================================================================
    // 2. 최대 전력 설정
    //==============================================================================
    for (auto coord : visitedSet)
    {
        Prop* propPtr = TileProp(coord.x, coord.y, coord.z);
        if (propPtr != nullptr)
        {
            propPtr->nodeMaxCharge = circuitMaxEnergy;

            //(전자회로용) 항상 전자 가득 찬 상태
            propPtr->nodeCharge = circuitMaxEnergy;

        }
    }

    //==============================================================================
    // 3. 전압원 전송 시작
    //==============================================================================

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
        voltOutputPower *= LOSS_COMPENSATION_FACTOR;  // 저항손실 보존 변수 (기본값 120%)

        if (debug::printCircuitLog) std::wprintf(L"========================▼전압원 %p : 밀어내기 시작▼========================\n", voltProp);
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

            voltProp->nodeCharge = voltProp->nodeMaxCharge;
        }
    }

    if (debug::printCircuitLog)
    {
        std::wprintf(L"======================== 회로망 요약 ========================\n");
        std::wprintf(L"노드: %zu개, 전압원: %zu개, 총부하: %d, 총전력: %d\n",
            visitedSet.size(), voltagePropVec.size(), circuitTotalLoad, circuitMaxEnergy);
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
    //논리게이트 출력 다이오드 바이패스
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

    //(트랜지스터) 메인라인에서 베이스 방향 절연
    if (crtItem.itemCode == itemRefCode::transistorL && dir == dir16::left) return false;
    else if(crtItem.itemCode == itemRefCode::transistorU && dir == dir16::up) return false;
    else if(crtItem.itemCode == itemRefCode::transistorR && dir == dir16::right) return false;
    else if (crtItem.itemCode == itemRefCode::transistorD && dir == dir16::down) return false;

    if (crtItem.itemCode == itemRefCode::relayL && dir == dir16::left) return false;
    else if (crtItem.itemCode == itemRefCode::relayU && dir == dir16::up) return false;
    else if (crtItem.itemCode == itemRefCode::relayR && dir == dir16::right) return false;
    else if (crtItem.itemCode == itemRefCode::relayD && dir == dir16::down) return false;

    //(논리게이트) 메인라인에서 입력핀1,2 방향 절연
    if (crtItem.itemCode == itemRefCode::andGateR && (dir == dir16::left || dir == dir16::down)) return false;
    else if (crtItem.itemCode == itemRefCode::andGateL && (dir == dir16::right || dir == dir16::down)) return false;

    if (crtItem.itemCode == itemRefCode::orGateR && (dir == dir16::left || dir == dir16::down)) return false;
    else if (crtItem.itemCode == itemRefCode::orGateL && (dir == dir16::right || dir == dir16::down)) return false;

    if (crtItem.itemCode == itemRefCode::xorGateR && (dir == dir16::left || dir == dir16::down)) return false;
    else if (crtItem.itemCode == itemRefCode::xorGateL && (dir == dir16::right || dir == dir16::down)) return false;

    //(NOT게이트) 메인라인에서 입력핀 방향 절연
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

    //(파워뱅크) 메인라인(현재)에서 입력부 차단
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
    errorBox(dir == dir16::above || dir == dir16::below, L"[Error] isGround: invalid direction\n");

    int dx, dy, dz;
    dirToXYZ(dir, dx, dy, dz);
    Prop* nextProp = TileProp(current.x + dx, current.y + dy, current.z + dz);

    if (nextProp == nullptr || nextProp->hasGround() == false)
        return false;

    if (isConnected(current, dir))
    {
        if (nextProp->leadItem.gndUsePower > 0) return true;
        if (dir == dir16::right && nextProp->leadItem.gndUsePowerLeft > 0) return true;
        if (dir == dir16::up && nextProp->leadItem.gndUsePowerDown > 0) return true;
        if (dir == dir16::left && nextProp->leadItem.gndUsePowerRight > 0) return true;
        if (dir == dir16::down && nextProp->leadItem.gndUsePowerUp > 0) return true;
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

    std::wstring indent(depth * 2, L' ');

    if (pathVisited.find(donorProp) != pathVisited.end())
    {
        if (debug::printCircuitLog)
            std::wprintf(L"%s[PUSH-SKIP] (%d,%d) 이미 방문됨\n", indent.c_str(), donorProp->getGridX(), donorProp->getGridY());
        return 0;
    }
    pathVisited.insert(donorProp);
    if (pathVisited.find(nextProp) != pathVisited.end()) return 0;



    if (debug::printCircuitLog) std::wprintf(L"%s[PUSH] (%d,%d) → (%d,%d) [%ls] 시도: %.2f\n",
        indent.c_str(),
        donorProp->getGridX(), donorProp->getGridY(),
        nextProp->getGridX(), nextProp->getGridY(),
        dirToArrow(txDir), 
        txChargeAmount);

    Point3 current = { donorProp->getGridX(), donorProp->getGridY(), donorProp->getGridZ() };
    if (isGround(current,txDir))
    {
        double remainEnergy;

        if (txDir == dir16::right && nextProp->leadItem.gndUsePowerLeft > 0)
            remainEnergy = nextProp->leadItem.gndUsePowerLeft - nextProp->chargeFlux[dir16::left];
        else if (txDir == dir16::up && nextProp->leadItem.gndUsePowerDown > 0)
            remainEnergy = nextProp->leadItem.gndUsePowerDown - nextProp->chargeFlux[dir16::down];
        else if (txDir == dir16::left && nextProp->leadItem.gndUsePowerRight > 0)
            remainEnergy = nextProp->leadItem.gndUsePowerRight - nextProp->chargeFlux[dir16::right];
        else if (txDir == dir16::down && nextProp->leadItem.gndUsePowerUp > 0)
            remainEnergy = nextProp->leadItem.gndUsePowerUp - nextProp->chargeFlux[dir16::up];
        else
            remainEnergy = nextProp->leadItem.gndUsePower - nextProp->getTotalChargeFlux();

        if (remainEnergy > EPSILON)
        {
            double consumeEnergy = std::min(txChargeAmount, remainEnergy);
            transferCharge(donorProp, nextProp, consumeEnergy, indent, txDir,true);
            return consumeEnergy;
        }
        else return 0;
    }

    double pushedCharge = std::min(txChargeAmount, nextProp->nodeCharge);

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
            if (possibleDirs.size() == 1)
            {
                auto newPathVisited = pathVisited;
                pushCharge(nextProp, possibleDirs[0], pushedCharge, newPathVisited, depth + 1);
            }
            else if (possibleDirs.size() > 1)
            {
                divideCharge(nextProp, pushedCharge, possibleDirs, pathVisited, depth + 1);
            }
        }
    }

    double finalTxCharge = std::min(txChargeAmount, nextProp->nodeMaxCharge - nextProp->nodeCharge);
    transferCharge(donorProp,nextProp,finalTxCharge,indent,txDir,false);
    return finalTxCharge;
}


void Prop::divideCharge(Prop* propPtr, double inputCharge, std::vector<dir16> possibleDirs, std::unordered_set<Prop*> pathVisited, int depth)
{
    std::wstring indent(depth * 2, L' ');  // 인덴트 생성

    if (debug::printCircuitLog)
    {
        std::wprintf(L"%s[DIVIDE] (%d,%d) 분배시작: %.2f → %zu방향\n",
            indent.c_str(),
            propPtr->getGridX(), propPtr->getGridY(),
            inputCharge, possibleDirs.size());
    }

    double remainingCharge = inputCharge;
    std::vector<dir16> dirsToRemove;
    std::vector<dir16> gndDirs;
    std::vector<dir16> nonGndDirs;
    dirsToRemove.reserve(6);
    gndDirs.reserve(6);
    nonGndDirs.reserve(6);

    int loopCount = 0;
    while (remainingCharge > EPSILON && !possibleDirs.empty())
    {
        loopCount++;
        dirsToRemove.clear();
        gndDirs.clear();
        nonGndDirs.clear();
        double gndPushedCharge = 0;
        double loopPushedCharge = 0;

        //접지 우선 배분
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

            if (debug::printCircuitLog)
            {
                std::wprintf(L"%s  [DIV-GND] 접지 %zu방향, 각 %.2f씩\n",
                    indent.c_str(), gndDirs.size(), gndSplitCharge);
            }

            for (auto dir : gndDirs)
            {
                auto newPathVisited = pathVisited;
                double branchPushedCharge = pushCharge(propPtr, dir, gndSplitCharge, newPathVisited, depth);
                gndPushedCharge += branchPushedCharge;
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

            if (debug::printCircuitLog)
            {
                std::wprintf(L"%s  [DIV-LOOP] 일반 %zu방향, 각 %.2f씩\n",
                    indent.c_str(), nonGndDirs.size(), splitCharge);
            }

            for (auto dir : nonGndDirs)
            {
                auto newPathVisited = pathVisited;
                double branchPushedCharge = pushCharge(propPtr, dir, splitCharge, newPathVisited, depth);
                loopPushedCharge += branchPushedCharge;
                if (branchPushedCharge < EPSILON) dirsToRemove.push_back(dir);
            }

            for (auto dir : dirsToRemove) possibleDirs.erase(std::remove(possibleDirs.begin(), possibleDirs.end(), dir), possibleDirs.end());
            remainingCharge -= loopPushedCharge;
        }

        if (debug::printCircuitLog && (gndPushedCharge > EPSILON || loopPushedCharge > EPSILON))
        {
            std::wprintf(L"%s  [DIV-RESULT] 루프%d: GND=%.2f, 일반=%.2f, 잔여=%.2f\n",
                indent.c_str(), loopCount, gndPushedCharge, loopPushedCharge, remainingCharge);
        }

        if (loopPushedCharge < EPSILON && gndPushedCharge < EPSILON) break;
    }

    if (debug::printCircuitLog)
    {
        std::wprintf(L"%s[DIVIDE-END] (%d,%d) 총 %d회 반복, 미분배=%.2f\n",
            indent.c_str(),
            propPtr->getGridX(), propPtr->getGridY(),
            loopCount, remainingCharge);
    }
}


void Prop::transferCharge(Prop* thisProp, Prop* nextProp, double txChargeAmount, const std::wstring& indent, dir16 txDir, bool isGroundTransfer = false)
{
    if (txChargeAmount < EPSILON)
    {
        if (debug::printCircuitLog)
        {
            std::wprintf(L"%s[전송 스킵] (%d,%d) → (%d,%d) 양:%.8f (EPSILON 미만)\n",
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
            std::wprintf(L"\x1b[33m%s[전송 GND] (%d,%d)[%.2f→%.2f] → (%d,%d) 전송:%.2f 손실:%.2f 부하:%.2f/%d\x1b[0m\n",
                indent.c_str(),
                thisProp->getGridX(), thisProp->getGridY(),
                thisProp->nodeCharge + requiredFromDonor, thisProp->nodeCharge,
                nextProp->getGridX(), nextProp->getGridY(),
                txChargeAmount, electricLoss,
                nextProp->getTotalChargeFlux(), nextProp->leadItem.gndUsePower);
        }
        else
        {
            std::wprintf(L"%s[전송] (%d,%d)[%.2f→%.2f] → (%d,%d)[%.2f/%d] 전송:%.2f 손실:%.2f\n",
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
                    //만약 BFS로 해당 방향으로 추가했을 때 GND가 지향성 GND 전력요구를 가지고 있을 경우
                    ItemData& nextItem = nextProp->leadItem;
                    thisProp->chargeFlux[directions[i]] = 0;
                    nextProp->chargeFlux[reverse(directions[i])] = 0;
                }
                frontierQueue.push(nextCoord);
            }
        }
    }
}


void Prop::loadAct()
{

    int iCode = leadItem.itemCode;

    //모든 계산이 종료된 후 부하에 공급된 전하량이 usePower 이상인지 이하인지 판단하여 부하 프롭이 켜지거나 꺼짐
    //단 논리게이트들은 공급된 전하량이 아니라 별도의 로직으로 처리
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

        if (setInput && resetInput) // 금지상태는 랜덤
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
    else if (iCode == itemRefCode::powerBankR || iCode == itemRefCode::powerBankL) //파워뱅크 충전의 경우 부하에 미달해도 작동
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
                // 충전 가능한 아이템 인덱스 수집
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

                // 균등 분배
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
    else //일반적인 부하들은 그라운드차지가 usePower 이상이면 켜지고 아니면 꺼짐
    {
        if (getTotalChargeFlux() >= static_cast<double>(leadItem.gndUsePower))
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

