export module Prop;

import std;
import constVar;
import util;
import ItemData;
import Ani;
import AI;
import Light;
import Coord;
import Drawable;

export CircuitAxis axisFromDir(dir16 dir);

export class Prop : public Ani, public AI, public Coord, public Drawable
{
private:

public:

    ItemData leadItem;
    int displayHPBarCount = 0; //양수 200으로 설정시 점점 떨어지다가 1이 되면 alpha를 대신 줄임. alpha마저 모두 줄면 0으로
    int alphaHPBar = 0;
    int alphaFakeHPBar = 0;
    float treeAngle = 0.0; //벌목 때 나무들이 가지는 앵글, 0이 아닐 경우 활성화됨

    bool runUsed = false; //runProp
    bool runUsedH = false;
    bool runUsedV = false;

    int nodeMaxCharge = 0;
    int nodeMaxChargeH = 0;
    int nodeMaxChargeV = 0;
    double nodeCharge = 0;
    double nodeChargeH = 0;
    double nodeChargeV = 0;
    double totalLossCharge = 0; //이번 턴에 저항으로 손실된 모든 에너지값

    std::unordered_map<dir16, double> chargeFlux = { {dir16::right,0},{dir16::up,0},{dir16::left,0},{dir16::down,0},{dir16::above,0},{dir16::below,0} };
    std::unordered_map<dir16, double> chargeFluxH = { {dir16::right,0},{dir16::up,0},{dir16::left,0},{dir16::down,0},{dir16::above,0},{dir16::below,0} };
    std::unordered_map<dir16, double> chargeFluxV = { {dir16::right,0},{dir16::up,0},{dir16::left,0},{dir16::down,0},{dir16::above,0},{dir16::below,0} };

    double prevPushedCharge = 0;
    double prevVoltOutputRatio = 1.0; //전압원에서의 이전 출력

    int gndVisitCount = -1;

    int delayMaxStack = 3;

    double delayStartTurn = 0;

    bool reserveDelayStackAdd = false;
    bool reserveDelayStackInit = false;


    Prop(Point3 inputCoor, int leadItemCode);

    ~Prop();

    void setGrid(int inputGridX, int inputGridY, int inputGridZ) override;

    void updateSprIndex();

    bool runAI();

    bool runAnimation(bool shutdown);

    void drawSelf() override;

    bool usesAxisSplit() const { return leadItem.checkFlag(itemFlag::CROSSED_CABLE); }

    void resetRunUsed();
    bool isAxisProcessed(CircuitAxis axis) const;
    bool isFullyProcessed() const;
    void markAxisProcessed(CircuitAxis axis);

    std::unordered_map<dir16, double>& getChargeFluxMap(CircuitAxis axis);
    const std::unordered_map<dir16, double>& getChargeFluxMap(CircuitAxis axis) const;
    double getChargeFlux(dir16 dir) const;
    void setChargeFlux(dir16 dir, double value);

    double& nodeChargeForAxis(CircuitAxis axis);
    int& nodeMaxChargeForAxis(CircuitAxis axis);

    double getTotalChargeFlux(CircuitAxis axis = CircuitAxis::omni);

    bool isChargeFlowing(CircuitAxis axis = CircuitAxis::omni);

    void initChargeFlux();

    double getInletCharge(CircuitAxis axis = CircuitAxis::omni);
    
    double getOutletCharge(CircuitAxis axis = CircuitAxis::omni);

    std::unordered_set<Prop*> updateCircuitNetwork(CircuitAxis startAxis = CircuitAxis::omni);

    bool isConnected(Point3 currentCoord, dir16 dir);

    bool isConnected(Prop* currentProp, dir16 dir);

    bool isGround(Point3 currentCoord, dir16 dir, CircuitAxis axis);

    void transferCharge(Prop* donorProp, Prop* acceptorProp, double txChargeAmount, const std::wstring& indent, dir16 txDir, bool isGroundTransfer, CircuitAxis axis);

    double pushCharge(Prop* donorProp, dir16 txDir, double txChargeAmount, std::unordered_set<Prop*> pathVisited, int depth, CircuitAxis axis);

    double divideCharge(Prop* propPtr, double inputCharge, std::vector<dir16> possibleDirs, std::unordered_set<Prop*> pathVisited, int depth, CircuitAxis axis);

    void propTurnOn();

    void propTurnOff();

    void initChargeBFS(std::queue<CircuitKey> startPointSet);

};
