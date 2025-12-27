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

export enum class crossFlag
{
    horizontal,
    vertical,
    omni,
};

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

    int nodeMaxCharge = 0;
    double nodeCharge = 0;

    double totalLossCharge = 0; //이번 턴에 저항으로 손실된 모든 에너지값

    std::unordered_map<dir16, double> chargeFlux = { {dir16::right,0},{dir16::up,0},{dir16::left,0},{dir16::down,0},{dir16::above,0},{dir16::below,0} };

    double prevPushedCharge = 0;

    int delayMaxStack = 3;
    double delayStartTurn = 0;

    std::unordered_map<Point3, crossFlag, Point3::Hash> crossStates;

    Prop(Point3 inputCoor, int leadItemCode);

    ~Prop();

    void setGrid(int inputGridX, int inputGridY, int inputGridZ) override;

    void updateSprIndex();

    bool runAI();

    bool runAnimation(bool shutdown);

    void drawSelf() override;

    double getTotalChargeFlux();

    bool isChargeFlowing();

    void initChargeFlux()
    {
        chargeFlux[dir16::right] = 0;
        chargeFlux[dir16::up] = 0;
        chargeFlux[dir16::left] = 0;
        chargeFlux[dir16::down] = 0;
        chargeFlux[dir16::above] = 0;
        chargeFlux[dir16::below] = 0;
    }

    double getInletCharge()
    {
        double totalInlet = 0;
        if (chargeFlux[dir16::right] > 0) totalInlet += chargeFlux[dir16::right];
        if (chargeFlux[dir16::up] > 0) totalInlet += chargeFlux[dir16::up];
        if (chargeFlux[dir16::left] > 0) totalInlet += chargeFlux[dir16::left];
        if (chargeFlux[dir16::down] > 0) totalInlet += chargeFlux[dir16::down];
        if (chargeFlux[dir16::above] > 0) totalInlet += chargeFlux[dir16::above];
        if (chargeFlux[dir16::below] > 0) totalInlet += chargeFlux[dir16::below];

        return totalInlet;
    }
    
    double getOutletCharge()
    {
        double totalOutlet = 0;
        if (chargeFlux[dir16::right] < 0) totalOutlet -= chargeFlux[dir16::right];
        if (chargeFlux[dir16::up] < 0) totalOutlet -= chargeFlux[dir16::up];
        if (chargeFlux[dir16::left] < 0) totalOutlet -= chargeFlux[dir16::left];
        if (chargeFlux[dir16::down] < 0) totalOutlet -= chargeFlux[dir16::down];
        if (chargeFlux[dir16::above] < 0) totalOutlet -= chargeFlux[dir16::above];
        if (chargeFlux[dir16::below] < 0) totalOutlet -= chargeFlux[dir16::below];

        return totalOutlet;
    }

    void updateCircuitNetwork();

    bool isConnected(Point3 currentCoord, dir16 dir);

    bool isConnected(Prop* currentProp, dir16 dir);

    bool isGround(Point3 currentCoord, dir16 dir);

    void transferCharge(Prop* donorProp, Prop* acceptorProp, double txChargeAmount, const std::wstring& indent, dir16 txDir, bool isGroundTransfer);

    double pushCharge(Prop* donorProp, dir16 txDir, double txChargeAmount, std::unordered_set<Prop*> pathVisited, int depth);

    double divideCharge(Prop* propPtr, double inputCharge, std::vector<dir16> possibleDirs, std::unordered_set<Prop*> pathVisited, int depth);

    void propTurnOn();

    void propTurnOff();

    void initChargeBFS(std::queue<Point3> startPointSet);

};
