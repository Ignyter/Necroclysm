import Prop;

import std;
import util;
import globalVar;
import constVar;
import wrapVar;

constexpr bool DEBUG_CIRCUIT_LOG = true;
constexpr double SYSTEM_VOLTAGE = 24.0;
constexpr double TIME_PER_TURN = 60.0;
constexpr double EPSILON = 0.000001;




void Prop::updateCircuitNetwork()
{
    int cursorX = getGridX();
    int cursorY = getGridY();
    int cursorZ = getGridZ();

    std::queue<Point3> frontierQueue;
    std::unordered_set<Point3, Point3::Hash> visitedSet;
    std::vector<Point3> visitedVec;//디버그용 나중에 지울 것
    std::vector<Prop*> voltagePropVec;
    std::vector<Prop*> loadVec; //부하가 가해지는 전자기기들

    std::unordered_set<Point3, Point3::Hash> skipBFSSet;

    int circuitMaxEnergy = 0;
    int circuitTotalLoad = 0;
    bool hasGround = false;


    //==============================================================================
    // 1. 회로 최초 탐색(BFS)
    //==============================================================================
    frontierQueue.push({ cursorX, cursorY, cursorZ });
    while (!frontierQueue.empty())
    {
        Point3 current = frontierQueue.front();
        frontierQueue.pop();

        if (visitedSet.find(current) != visitedSet.end()) continue;
        visitedSet.insert(current);
        visitedVec.push_back(current);


        Prop* currentProp = TileProp(current.x, current.y, current.z);

        if (currentProp && (currentProp->leadItem.checkFlag(itemFlag::CIRCUIT) || currentProp->leadItem.checkFlag(itemFlag::CABLE)))
        {
            currentProp->runUsed = true;

            currentProp->nodeInputElectron = 0;
            currentProp->nodeOutputElectron = 0;

            if (currentProp->leadItem.checkFlag(itemFlag::VOLTAGE_SOURCE))
            {
                if (currentProp->leadItem.checkFlag(itemFlag::PROP_POWER_ON) &&
                    currentProp->leadItem.checkFlag(itemFlag::PROP_POWER_OFF) == false)
                {
                    circuitMaxEnergy += currentProp->leadItem.electricMaxPower;
                    voltagePropVec.push_back(currentProp);
                }
            }

            if (currentProp->leadItem.electricUsePower > 0) loadVec.push_back(currentProp);



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
                            if (nextProp->leadItem.itemCode == itemRefCode::transistorL && directions[i] == dir16::right) skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::transistorU && directions[i] == dir16::down) skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::transistorR && directions[i] == dir16::left) skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::transistorD && directions[i] == dir16::up) skipBFSSet.insert(nextCoord);

                            if (nextProp->leadItem.itemCode == itemRefCode::relayL && directions[i] == dir16::right) skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::relayU && directions[i] == dir16::down) skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::relayR && directions[i] == dir16::left) skipBFSSet.insert(nextCoord);
                            else if (nextProp->leadItem.itemCode == itemRefCode::relayD && directions[i] == dir16::up) skipBFSSet.insert(nextCoord);
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

    for (int i = 0; i < loadVec.size(); i++)
    {
        circuitTotalLoad += loadVec[i]->leadItem.electricUsePower;
    }

    // 노드가 2개 미만이면 출력하지 않음
    if (visitedSet.size() < 2)
    {
        runUsed = true;
        return;
    }


    //std::wprintf(L"\n┌─────────────────────────────────┐\n");
    //std::wprintf(L"│ 회로 분석 완료                  │\n");
    //std::wprintf(L"├─────────────────────────────────┤\n");
    //std::wprintf(L"│ 노드 수: %3d                    │\n", visitedSet.size());
    //std::wprintf(L"│ 최대 전력: %3d kJ               │\n", circuitMaxEnergy);
    //std::wprintf(L"│ 접지 존재: %s                   │\n", hasGround ? L"Yes" : L"No ");
    //std::wprintf(L"└─────────────────────────────────┘\n\n");

    //==============================================================================
    // 2. 최대 전력 설정
    //==============================================================================
    for (auto coord : visitedSet)
    {
        Prop* propPtr = TileProp(coord.x, coord.y, coord.z);
        if (propPtr != nullptr)
        {
            
            propPtr->nodeMaxElectron = circuitMaxEnergy;
        
            //전자회로는 항상 전자 가득 찬 상태
            propPtr->nodeElectron = circuitMaxEnergy;
        }
    }

    //==============================================================================
    // 3. 전압원 전송 시작
    //==============================================================================
    double totalPushedElectron = 0;

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

        voltProp->nodeElectron = voltProp->nodeMaxElectron;
        int x = voltProp->getGridX();
        int y = voltProp->getGridY();
        int z = voltProp->getGridZ();
        double voltRatio = (double)voltProp->leadItem.electricMaxPower / (double)totalAvailablePower;
        double voltOutputPower = myMin(std::ceil(circuitTotalLoad * voltRatio), voltProp->leadItem.electricMaxPower);
        voltProp->prevPushedElectron = 0;
        voltOutputPower *= LOSS_COMPENSATION_FACTOR;  // 저항손실 보존 변수 (기본값 110%)

        if (DEBUG_CIRCUIT_LOG) std::wprintf(L"========================▼전압원 %p : 밀어내기 시작▼========================\n", voltProp);
        if (voltProp->leadItem.checkFlag(itemFlag::PROP_POWER_ON) || voltProp->leadItem.checkFlag(itemFlag::PROP_POWER_OFF) == false)
        {
            if (voltProp->leadItem.checkFlag(itemFlag::VOLTAGE_OUTPUT_RIGHT) && isConnected({ x,y,z }, dir16::right))
                voltProp->prevPushedElectron += pushElectron(voltProp, dir16::right, voltOutputPower, {}, 0);
            else if (voltProp->leadItem.checkFlag(itemFlag::VOLTAGE_OUTPUT_UP) && isConnected({ x,y,z }, dir16::up))
                voltProp->prevPushedElectron += pushElectron(voltProp, dir16::up, voltOutputPower, {}, 0);
            else if (voltProp->leadItem.checkFlag(itemFlag::VOLTAGE_OUTPUT_LEFT) && isConnected({ x,y,z }, dir16::left))
                voltProp->prevPushedElectron += pushElectron(voltProp, dir16::left, voltOutputPower, {}, 0);
            else if (voltProp->leadItem.checkFlag(itemFlag::VOLTAGE_OUTPUT_DOWN) && isConnected({ x,y,z }, dir16::down))
                voltProp->prevPushedElectron += pushElectron(voltProp, dir16::down, voltOutputPower, {}, 0);

            totalPushedElectron += voltProp->prevPushedElectron;
            voltProp->nodeElectron = voltProp->nodeMaxElectron;
        }
    }


    //==============================================================================
    // 5. 최종 상태 출력
    //==============================================================================
    //std::wprintf(L"\n┌────────────────────────────────────┐\n");
    //std::wprintf(L"│ 최종 회로 상태                     │\n");
    //std::wprintf(L"├────────────────────────────────────┤\n");

    //for (auto coord : visitedVec)
    //{
    //    Prop* propPtr = TileProp(coord.x, coord.y, coord.z);
    //    if (propPtr != nullptr)
    //    {
    //        std::wstring nodeType = L"Wire";
    //        if (propPtr->leadItem.checkFlag(itemFlag::VOLTAGE_SOURCE)) nodeType = L"Gen ";
    //        else if (propPtr->leadItem.checkFlag(itemFlag::VOLTAGE_GND_ALL) ||
    //            propPtr->leadItem.checkFlag(itemFlag::VOLTAGE_GND_UP) ||
    //            propPtr->leadItem.checkFlag(itemFlag::VOLTAGE_GND_DOWN) ||
    //            propPtr->leadItem.checkFlag(itemFlag::VOLTAGE_GND_LEFT) ||
    //            propPtr->leadItem.checkFlag(itemFlag::VOLTAGE_GND_RIGHT))
    //            nodeType = L"GND ";

    //        wchar_t status = L'○';
    //        if (propPtr->nodeElectron == 0) status = L'○';
    //        else if (propPtr->nodeElectron == propPtr->nodeMaxElectron) status = L'●';
    //        else status = L'◐';

    //        std::wprintf(L"│ %s (%3d,%3d): %3d/%3d %c     │\n",
    //            nodeType.c_str(),
    //            coord.x, coord.y,
    //            propPtr->nodeElectron, propPtr->nodeMaxElectron,
    //            status);
    //    }
    //}

    //std::wprintf(L"└────────────────────────────────────┘\n\n");
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

    if ((dir == dir16::right || dir == dir16::left) && targetProp->leadItem.itemCode == itemRefCode::leverRL)
    {
        if (targetProp->leadItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }
    else if ((dir == dir16::up || dir == dir16::down) && targetProp->leadItem.itemCode == itemRefCode::leverUD)
    {
        if (targetProp->leadItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }



    if ((dir == dir16::right || dir == dir16::left) && (targetProp->leadItem.itemCode == itemRefCode::transistorU || targetProp->leadItem.itemCode == itemRefCode::transistorD))
    {
        if (targetProp->leadItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }
    else if ((dir == dir16::up || dir == dir16::down) && (targetProp->leadItem.itemCode == itemRefCode::transistorR || targetProp->leadItem.itemCode == itemRefCode::transistorL))
    {
        if (targetProp->leadItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }

    if ((dir == dir16::right || dir == dir16::left) && (targetProp->leadItem.itemCode == itemRefCode::relayU || targetProp->leadItem.itemCode == itemRefCode::relayD))
    {
        if (targetProp->leadItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }
    else if ((dir == dir16::up || dir == dir16::down) && (targetProp->leadItem.itemCode == itemRefCode::relayR || targetProp->leadItem.itemCode == itemRefCode::relayL))
    {
        if (targetProp->leadItem.checkFlag(itemFlag::PROP_POWER_OFF)) return false;
    }

    //메인라인에서 베이스로 나가는 방향 절연
    if (currentProp->leadItem.itemCode == itemRefCode::transistorL && dir == dir16::left) return false;
    else if(currentProp->leadItem.itemCode == itemRefCode::transistorU && dir == dir16::up) return false;
    else if(currentProp->leadItem.itemCode == itemRefCode::transistorR && dir == dir16::right) return false;
    else if (currentProp->leadItem.itemCode == itemRefCode::transistorD && dir == dir16::down) return false;

    if (currentProp->leadItem.itemCode == itemRefCode::relayL && dir == dir16::left) return false;
    else if (currentProp->leadItem.itemCode == itemRefCode::relayU && dir == dir16::up) return false;
    else if (currentProp->leadItem.itemCode == itemRefCode::relayR && dir == dir16::right) return false;
    else if (currentProp->leadItem.itemCode == itemRefCode::relayD && dir == dir16::down) return false;

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
        //중복인가? 최적화를 위해선 없어도 될 수도
        if (isConnected(currentCoord, dir)) return true;
    }
    else return false;
}

bool Prop::isConnected(Prop* currentProp, dir16 dir)
{
    return isConnected({ currentProp->getGridX(),currentProp->getGridY(),currentProp->getGridZ() }, dir);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double Prop::pushElectron(Prop* donorProp, dir16 txDir, double txElectronAmount, std::unordered_set<Prop*> pathVisited, int depth)
{
    errorBox(donorProp == nullptr, L"[Error] pushElectron: null donor\n");
    int dx, dy, dz;
    dirToXYZ(txDir, dx, dy, dz);
    Prop* acceptorProp = TileProp(donorProp->getGridX() + dx, donorProp->getGridY() + dy, donorProp->getGridZ() + dz);
    errorBox(acceptorProp == nullptr, L"[Error] pushElectron: no acceptor found\n");

    txElectronAmount = std::min(donorProp->nodeElectron, txElectronAmount);

    errorBox(txElectronAmount > donorProp->nodeElectron + EPSILON, L"[Error] pushElectron: insufficient electron\n");
    errorBox(!isConnected({ donorProp->getGridX(), donorProp->getGridY(), donorProp->getGridZ() }, txDir),
        L"[Error] pushElectron: not connected\n");

    if (pathVisited.find(donorProp) != pathVisited.end()) return 0;
    pathVisited.insert(donorProp);
    if (pathVisited.find(acceptorProp) != pathVisited.end()) return 0;

    // 들여쓰기 생성
    std::wstring indent(depth * 2, L' ');  // depth마다 2칸씩
    std::wprintf(L"%s[PUSH] (%d,%d) → (%d,%d) 시도: %.2f\n",
        indent.c_str(),
        donorProp->getGridX(), donorProp->getGridY(),
        acceptorProp->getGridX(), acceptorProp->getGridY(),
        txElectronAmount);

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
        double remainEnergy = acceptorProp->leadItem.electricUsePower - acceptorProp->groundChargeEnergy;

        if (remainEnergy > EPSILON)
        {
            double consumeEnergy = std::min(txElectronAmount, remainEnergy);
            transferElectron(donorProp, acceptorProp, consumeEnergy, indent, true);
            return consumeEnergy;
        }
        else return 0;
    }

    double pushedElectron = std::min(txElectronAmount, acceptorProp->nodeElectron);

    double dividedElectron = 0;
    if (pushedElectron > EPSILON)
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
            dividedElectron = divideElectron(acceptorProp, pushedElectron, possibleDirs, pathVisited, depth + 1);
        }
    }

    double finalTxElectron = std::min(txElectronAmount, acceptorProp->nodeMaxElectron - acceptorProp->nodeElectron);
    transferElectron(donorProp,acceptorProp,finalTxElectron,indent,false);
    return finalTxElectron;
}


double Prop::divideElectron(Prop* propPtr, double inputElectron, std::vector<dir16> possibleDirs, std::unordered_set<Prop*> pathVisited, int depth)
{
    double totalPushedElectron = 0;
    double remainingElectron = inputElectron;
    std::vector<dir16> dirsToRemove;
    std::vector<dir16> gndDirs;
    std::vector<dir16> nonGndDirs;
    dirsToRemove.reserve(6);
    gndDirs.reserve(6);
    nonGndDirs.reserve(6);

    while (remainingElectron > EPSILON && !possibleDirs.empty())
    {
        dirsToRemove.clear();
        gndDirs.clear();
        nonGndDirs.clear();
        double gndPushedElectron = 0;
        double loopPushedElectron = 0;

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
            double gndSplitElectron = remainingElectron / gndDirs.size();
            for (auto dir : gndDirs)
            {
                auto newPathVisited = pathVisited;
                double branchPushedElectron = pushElectron(propPtr, dir, gndSplitElectron, newPathVisited, depth);
                gndPushedElectron += branchPushedElectron;
                totalPushedElectron += branchPushedElectron;
                if (branchPushedElectron < EPSILON) dirsToRemove.push_back(dir);
            }

            for (auto dir : dirsToRemove) possibleDirs.erase(std::remove(possibleDirs.begin(), possibleDirs.end(), dir), possibleDirs.end());
            remainingElectron -= gndPushedElectron;
        }

        dirsToRemove.clear();
        if (possibleDirs.empty()) break;

        if (nonGndDirs.size() > 0)
        {
            double splitElectron = remainingElectron / nonGndDirs.size();
            for (auto dir : nonGndDirs)
            {
                auto newPathVisited = pathVisited;
                double branchPushedElectron = pushElectron(propPtr, dir, splitElectron, newPathVisited, depth);
                loopPushedElectron += branchPushedElectron;
                totalPushedElectron += branchPushedElectron;
                if (branchPushedElectron < EPSILON) dirsToRemove.push_back(dir);
            }

            for (auto dir : dirsToRemove) possibleDirs.erase(std::remove(possibleDirs.begin(), possibleDirs.end(), dir), possibleDirs.end());
            remainingElectron -= loopPushedElectron;
        }

        if (loopPushedElectron < EPSILON && gndPushedElectron < EPSILON) break;
    }

    return totalPushedElectron;
}

void Prop::transferElectron(Prop* donorProp, Prop* acceptorProp, double txElectronAmount, const std::wstring& indent, bool isGroundTransfer = false)
{
    if (txElectronAmount < EPSILON)
    {
        if (DEBUG_CIRCUIT_LOG)
        {
            std::wprintf(L"%s[전송 스킵] (%d,%d) → (%d,%d) 양:%.8f (EPSILON 미만)\n",
                indent.c_str(),
                donorProp->getGridX(), donorProp->getGridY(),
                acceptorProp->getGridX(), acceptorProp->getGridY(),
                txElectronAmount);
        }
        return;
    }

    double current = txElectronAmount / (SYSTEM_VOLTAGE * TIME_PER_TURN);
    double electricLoss = current * current * acceptorProp->leadItem.electricResistance * TIME_PER_TURN;
    double requiredFromDonor = txElectronAmount + electricLoss;

    if (requiredFromDonor > donorProp->nodeElectron + EPSILON)
    {
        double availableRatio = donorProp->nodeElectron / requiredFromDonor;
        requiredFromDonor = donorProp->nodeElectron;
        txElectronAmount *= availableRatio;
        electricLoss = requiredFromDonor - txElectronAmount;
    }

    donorProp->nodeElectron -= requiredFromDonor;
    donorProp->nodeOutputElectron += requiredFromDonor;

    if (isGroundTransfer) acceptorProp->groundChargeEnergy += txElectronAmount;
    else acceptorProp->nodeElectron += txElectronAmount;

    acceptorProp->nodeInputElectron += txElectronAmount;

    if (DEBUG_CIRCUIT_LOG)
    {
        if (isGroundTransfer)
        {
            std::wprintf(L"%s[전송 GND] (%d,%d)[%.2f→%.2f] → (%d,%d) 전송:%.2f 손실:%.2f 부하:%.2f/%d\n",
                indent.c_str(),
                donorProp->getGridX(), donorProp->getGridY(),
                donorProp->nodeElectron + requiredFromDonor, donorProp->nodeElectron,
                acceptorProp->getGridX(), acceptorProp->getGridY(),
                txElectronAmount, electricLoss,
                acceptorProp->groundChargeEnergy, acceptorProp->leadItem.electricUsePower);
        }
        else
        {
            std::wprintf(L"%s[전송] (%d,%d)[%.2f→%.2f] → (%d,%d)[%.2f/%d] 전송:%.2f 손실:%.2f\n",
                indent.c_str(),
                donorProp->getGridX(), donorProp->getGridY(),
                donorProp->nodeElectron + requiredFromDonor, donorProp->nodeElectron,
                acceptorProp->getGridX(), acceptorProp->getGridY(),
                acceptorProp->nodeElectron, acceptorProp->nodeMaxElectron,
                txElectronAmount, electricLoss);
        }
    }
}

