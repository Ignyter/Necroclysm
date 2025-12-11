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
    double nodeInputCharge = 0;
    double nodeOutputCharge = 0;
    double groundCharge = 0; //전자기기 사용되기 전에 저장된 에너지(접지 방향)

    double prevPushedCharge = 0;
    double prevVoltOutputRatio = 1.0; //전압원에서의 이전 출력



    Prop(Point3 inputCoor, int leadItemCode);

    ~Prop();

    void setGrid(int inputGridX, int inputGridY, int inputGridZ) override;

    void updateSprIndex();

    bool runAI();

    bool runAnimation(bool shutdown);

    void drawSelf() override;

    void updateCircuitNetwork();

    bool isConnected(Point3 currentCoord, dir16 dir);

    bool isConnected(Prop* currentProp, dir16 dir);

    bool isGround(Point3 currentCoord, dir16 dir);

    void runPropFunc();

    void transferCharge(Prop* donorProp, Prop* acceptorProp, double txChargeAmount, const std::wstring& indent, bool isGroundTransfer);

    double pushCharge(Prop* donorProp, dir16 txDir, double txChargeAmount, std::unordered_set<Prop*> pathVisited, int depth);

    double divideCharge(Prop* propPtr, double inputCharge, std::vector<dir16> possibleDirs, std::unordered_set<Prop*> pathVisited, int depth);

    void propTurnOn();

    void propTurnOff();
};
