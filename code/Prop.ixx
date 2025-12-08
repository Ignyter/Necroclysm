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

    int nodeMaxElectron = 0;
    double nodeElectron = 0;
    double nodeInputElectron = 0;
    double nodeOutputElectron = 0;
    double groundChargeEnergy = 0; //전자기기 사용되기 전에 저장된 에너지(접지 방향)

    double prevPushedElectron = 0;
    double prevVoltOutputRatio = 1.0; //전압원에서의 이전 출력

    Prop(Point3 inputCoor, int leadItemCode);

    ~Prop();

    void setGrid(int inputGridX, int inputGridY, int inputGridZ) override;

    void updateSprIndex();

    bool runAI();

    bool runAnimation(bool shutdown);

    void drawSelf() override;

    void updateCircuitNetwork();

    bool isConnected(Point3 currentCoord, dir16 dir, bool forBFS = false);

    bool isConnected(Prop* currentProp, dir16 dir, bool forBFS = false);

    bool isGround(Point3 currentCoord, dir16 dir);

    void runPropFunc();

    void transferElectron(Prop* donorProp, Prop* acceptorProp, double txElectronAmount, const std::wstring& indent, bool isGroundTransfer);

    double pushElectron(Prop* donorProp, dir16 txDir, double txElectronAmount, std::unordered_set<Prop*> pathVisited, int depth);

    double divideElectron(Prop* propPtr, double inputElectron, std::vector<dir16> possibleDirs, std::unordered_set<Prop*> pathVisited, int depth);

    void propTurnOn();

    void propTurnOff();
};
