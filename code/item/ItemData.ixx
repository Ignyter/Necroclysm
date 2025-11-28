export module ItemData;

import std;
import util;
import constVar;
import Light;
import ItemDataBase;
export import ItemPocket;

//__int8 : -128~127
//__int16 : -32768 ~32767
//__int32 : –2,147,483,648 ~2,147,483,647


export struct ItemData : public ItemDataBase
{
    ///////////////////비데이터베이스 변수////////////////////
    unsigned __int32 pocketVolume = 0;
    unsigned __int32 number = 1;
    unsigned __int32 lootSelect = 0;
    unsigned __int32 reactSelect = 0;

    std::unique_ptr<ItemPocket> pocketPtr;
    equipHandFlag equipState = equipHandFlag::none;
    unsigned __int8 modeCurrent = 0;

    unsigned __int16 unfinishItemCode = 0;
    unsigned __int8 unfinishPercent = 100;

    unsigned __int8 extraSprIndexSingle = 0; //벽의 연결로 추가되는 sprIndex(일단은 차량만 구현)
    unsigned __int8 extraSprIndex16 = 0; //16단위로 움직이는 추가 sprIndex

    unsigned __int16 dirChangeItemCode = 0;
    dir16 propDir16 = dir16::dir0;

    std::unique_ptr<Light> lightPtr;
    __int16 propHP = 100;
    float propFakeHP = 100.0f;

    bool checkFlag(itemFlag inputFlag) const;

    void addFlag(itemFlag inputFlag);

    void eraseFlag(itemFlag inputFlag);

    ItemData cloneForTransfer(int transferNumber) const;

    bool itemOverlay(const ItemData& tgtItem) const;

    ItemData();
    virtual ~ItemData();
    ItemData(ItemData&&) noexcept;
    ItemData& operator=(ItemData&&) noexcept = default;
    ItemData(const ItemData&) = delete;
    ItemData& operator=(const ItemData&) = delete;

    bool isPocketOnlyItem(int inputCode);
};

export ItemData cloneFromItemDex(ItemData& inputData, int transferNumber);