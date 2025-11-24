module;

#include <SDL3/SDL.h>

export module textureVar;

import std;
import Sprite;

export namespace texture
{
	SDL_Texture* minimap = nullptr;
	SDL_Texture* worldmap = nullptr;
	SDL_Texture* navimap = nullptr;
	SDL_Texture* mainGaugeWhiteShadow = nullptr;
	SDL_Texture* hpGaugeWhiteShadow = nullptr;
}

export namespace spr
{
	std::unordered_map<std::wstring, Sprite*> spriteMapper;

	Sprite* charsetHero = nullptr;
	Sprite* zombie1 = nullptr;
	Sprite* arabianHorse = nullptr;
	Sprite* defaultMonster = nullptr;
	Sprite* equipIcon = nullptr;
	Sprite* effectBash1 = nullptr;
	Sprite* effectCut1 = nullptr;
	Sprite* effectCut2 = nullptr;
	Sprite* effectPierce1 = nullptr;
	Sprite* effectBite1 = nullptr;
	Sprite* effectClaw1 = nullptr;
	Sprite* icon13 = nullptr;
	Sprite* icon48 = nullptr;
	Sprite* batteryGauge = nullptr;
	Sprite* ecliptic = nullptr;
	Sprite* weather = nullptr;
	Sprite* weatherCloud = nullptr;
	Sprite* itemset = nullptr;
	Sprite* windowArrow = nullptr;
	Sprite* whiteMarker = nullptr;
	Sprite* yellowMarker = nullptr;
	Sprite* equipRoll = nullptr;
	Sprite* lootRoll = nullptr;
	Sprite* detailRoll = nullptr;
	Sprite* pocketRoll = nullptr;
	Sprite* itemCursorLong = nullptr;
	Sprite* itemCursorShort = nullptr;
	Sprite* lootBagArrow = nullptr;
	Sprite* icon16 = nullptr;
	Sprite* guideBtn = nullptr;
	Sprite* buttons = nullptr;
	Sprite* buttonsPressed = nullptr;
	Sprite* menuPopUp = nullptr;
	Sprite* letterboxBtnMarker = nullptr;
	Sprite* proficIcon = nullptr;
	Sprite* proficIconGold = nullptr;
	Sprite* bionicSkeleton = nullptr;
	Sprite* bionicSlotGauge = nullptr;
	Sprite* mutationIcon = nullptr;
	Sprite* staminaGauge = nullptr;
	Sprite* segment = nullptr;
	Sprite* tileset = nullptr;
	Sprite* wallset = nullptr;
	Sprite* propset = nullptr;
	Sprite* icon32 = nullptr;
	Sprite* ring24 = nullptr;
	Sprite* bloodM = nullptr;
	Sprite* aimMarker = nullptr;
	Sprite* aimMarkerTmp = nullptr;
	Sprite* aimAtkTypeMarker = nullptr;
	Sprite* partsSlotGauge = nullptr;
	Sprite* tab = nullptr;
	Sprite* aimLRChange = nullptr;
	Sprite* epsilonFont = nullptr;
	Sprite* loadingAnime = nullptr;
	Sprite* buildCursor = nullptr;
	Sprite* msgChoiceBtn = nullptr;
	Sprite* itemSlotBtn = nullptr;
	Sprite* itemSlotPocketArrow = nullptr;
	Sprite* lstSelectBox = nullptr;
	Sprite* skinYellow = nullptr;
	Sprite* eyesBlue = nullptr;
	Sprite* eyesRed = nullptr;
	Sprite* eyesClosed = nullptr;
	Sprite* eyesHalf = nullptr;
	Sprite* beardMustacheBlack = nullptr;
	Sprite* hairCommaBlack = nullptr;
	Sprite* hairBob1Black = nullptr;
	Sprite* hairPonytailBlack = nullptr;
	Sprite* hairMiddlePart = nullptr;
	Sprite* shadow = nullptr;
	Sprite* vehicleHUD = nullptr;
	Sprite* dashboard = nullptr;
	Sprite* vehicleHUDParts = nullptr;
	Sprite* vehicleHUDSteeringWheel = nullptr;
	Sprite* vehicleActCursor = nullptr;
	Sprite* dirMarker = nullptr;
	Sprite* craftEdge = nullptr;
	Sprite* craftSelectWindow = nullptr;
	Sprite* windowArrow2 = nullptr;
	Sprite* mapHereMarker = nullptr;
	Sprite* screenRain = nullptr;
	Sprite* screenSnow = nullptr;
	Sprite* symbolSunny = nullptr;
	Sprite* symbolCloudy = nullptr;
	Sprite* symbolMoon = nullptr;
	Sprite* symbolRain = nullptr;
	Sprite* symbolStorm = nullptr;
	Sprite* symbolSnow = nullptr;
	Sprite* tailPedalL = nullptr;
	Sprite* tailPedalR = nullptr;
	Sprite* trainBrake = nullptr;
	Sprite* mainRotor = nullptr;
	Sprite* singleQuickSlot = nullptr;
	Sprite* topQuickSlotBtn = nullptr;
	Sprite* skillSet = nullptr;
	Sprite* fireStorm = nullptr;
	Sprite* flameSet = nullptr;
	Sprite* gasEffect1 = nullptr;
	Sprite* steamEffect1 = nullptr;
	Sprite* trail = nullptr;
	Sprite* statusEffectRect = nullptr;
	Sprite* statusIcon = nullptr;
	Sprite* minimapEdge = nullptr;
	Sprite* bulletset = nullptr;
	Sprite* aimMarkerWhite = nullptr;
	Sprite* hornCoverRed = nullptr;
	Sprite* mapMagnifyIcon = nullptr;
	Sprite* staminaGaugeCircle = nullptr;
	Sprite* speechBubble = nullptr;
	Sprite* particle = nullptr;
	Sprite* footprint = nullptr;
	Sprite* craftItemRect = nullptr;
	Sprite* msgBox = nullptr;
	Sprite* skillRect = nullptr;
	Sprite* gridMarker = nullptr;
	Sprite* tabBoxAim = nullptr;

	Sprite* youDied = nullptr;
	Sprite* gameOverOptionRect = nullptr;
	Sprite* gameOverOptionMarker = nullptr;

	Sprite* statusEffectGaugeCircle = nullptr;

	Sprite* thoughtBubble = nullptr;
	Sprite* waveFoam = nullptr;
	
	Sprite* seaFoam = nullptr;
	Sprite* waterFoam = nullptr;

	Sprite* itemBackgroundRect = nullptr;

	Sprite* newWindowArrow = nullptr;

	Sprite* icon28 = nullptr;

	Sprite* downRightLetterbox = nullptr;
	Sprite* mainGauge = nullptr;

	Sprite* bodyShape = nullptr;
	Sprite* hpGauge = nullptr;

	Sprite* icon24 = nullptr;

	Sprite* gamepadInstruction = nullptr;
	Sprite* gamepadButtons = nullptr;
	Sprite* keyboardButtons = nullptr;
}
