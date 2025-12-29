export module statusEffect;

export enum statusEffectFlag
{
    none = -1,
    confused = 0,
    bleeding = 1,
    hungry = 2,
    dehydrated = 3,
    blind = 4,
    tired = 5,
    exhausted = 6,

    run = 7,
    crouch = 8,
    crawl = 9,
};

export class statusEffect
{
public:
    statusEffectFlag effectType;
    float duration;
};