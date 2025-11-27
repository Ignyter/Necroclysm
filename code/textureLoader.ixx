#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>

export module textureLoader;

import std;
import globalVar;
import constVar;
import textureVar;
import Sprite;

export void textureLoader()
{
	//load texture
	texture::minimap = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, MINIMAP_DIAMETER, MINIMAP_DIAMETER);
	SDL_SetTextureScaleMode(texture::minimap, SDL_SCALEMODE_NEAREST);
	texture::navimap = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, NAVIMAP_WIDTH, NAVIMAP_HEIGHT);
	SDL_SetTextureScaleMode(texture::navimap, SDL_SCALEMODE_NEAREST);
	texture::worldmap = IMG_LoadTexture(renderer, "image/worldmap.png");
	SDL_SetTextureScaleMode(texture::worldmap, SDL_SCALEMODE_NEAREST);
	texture::mainGaugeWhiteShadow = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, 170, 16);
	SDL_SetTextureScaleMode(texture::mainGaugeWhiteShadow, SDL_SCALEMODE_NEAREST);

	texture::hpGaugeWhiteShadow = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, 95, 13);
	SDL_SetTextureScaleMode(texture::hpGaugeWhiteShadow, SDL_SCALEMODE_NEAREST);

	spr::charsetHero = new Sprite(renderer, "image/charset/baseCharset.png", 48, 48);//new Sprite(renderer, "image/charset/baseCharset.png");
	spr::defaultMonster = new Sprite(renderer, "image/charset/zombie1.png", 48, 48);
	spr::effectBash1 = new Sprite(renderer, "image/effect/effectBash1.png", 48, 48);
	spr::effectCut1 = new Sprite(renderer, "image/effect/effectCut1.png", 48, 48);
	spr::effectCut2 = new Sprite(renderer, "image/effect/effectCut2.png", 48, 48);
	spr::effectPierce1 = new Sprite(renderer, "image/effect/effectPierce1.png", 48, 48);
	spr::effectBite1 = new Sprite(renderer, "image/effect/effectBite1.png", 48, 48);
	spr::effectClaw1 = new Sprite(renderer, "image/effect/effectClaw1.png", 48, 48);
	spr::icon13 = new Sprite(renderer, "image/UI/icon13.png", 13, 13);
	spr::icon48 = new Sprite(renderer, "image/UI/icon48.png", 48, 48);
	spr::batteryGauge = new Sprite(renderer, "image/UI/batteryGauge.png", 32, 46);
	spr::ecliptic = new Sprite(renderer, "image/UI/GUI/HUD/ecliptic.png", 76, 36);
	spr::weather = new Sprite(renderer, "image/UI/weather.png", 48, 48);
	spr::weatherCloud = new Sprite(renderer, "image/UI/weatherCloud.png", 48, 48);
	spr::itemset = new Sprite(renderer, "image/item/itemset.png", 48, 48);
	spr::windowArrow = new Sprite(renderer, "image/UI/windowArrow.png", 16, 16);
	spr::whiteMarker = new Sprite(renderer, "image/UI/whiteMarker.png", 16, 16);
	spr::yellowMarker = new Sprite(renderer, "image/UI/yellowMarker.png", 16, 16);
	spr::itemCursorLong = new Sprite(renderer, "image/UI/itemCursorLong.png", 360, 64);
	spr::itemCursorShort = new Sprite(renderer, "image/UI/itemCursorShort.png", 304, 64);
	spr::lootBagArrow = new Sprite(renderer, "image/UI/lootBagArrow.png", 32, 32);
	spr::icon16 = new Sprite(renderer, "image/UI/icon16.png", 16, 16);
	spr::guideBtn = new Sprite(renderer, "image/UI/guideBtn.png", 87, 33);
	spr::buttons = new Sprite(renderer, "image/UI/buttons.png", 48, 48);
	spr::buttonsPressed = new Sprite(renderer, "image/UI/buttonsPressed.png", 48, 48);
	spr::menuPopUp = new Sprite(renderer, "image/UI/menuPopUp.png", 35, 36);
	spr::letterboxBtnMarker = new Sprite(renderer, "image/UI/barButtonMarker.png", 108, 108);
	spr::proficIcon = new Sprite(renderer, "image/UI/proficIcon.png", 16, 16);
	spr::proficIconGold = new Sprite(renderer, "image/UI/proficIconGold.png", 16, 16);
	spr::bionicSkeleton = new Sprite(renderer, "image/UI/bionicSkeleton.png", 36, 72);
	spr::bionicSlotGauge = new Sprite(renderer, "image/UI/bionicSlotGauge.png", 64, 10);
	spr::mutationIcon = new Sprite(renderer, "image/UI/mutationIcon.png", 16, 16);
	spr::staminaGauge = new Sprite(renderer, "image/UI/staminaGauge.png", 48, 48);
	spr::segment = new Sprite(renderer, "image/UI/segment.png", 12, 16);
	spr::tileset = new Sprite(renderer, "image/tileset/tileset.png", 16, 16);
	spr::propset = new Sprite(renderer, "image/tileset/propset.png", 48, 48);
	spr::icon32 = new Sprite(renderer, "image/UI/icon32.png", 32, 32);
	spr::ring24 = new Sprite(renderer, "image/UI/ring24.png", 24, 24);
	spr::bloodM = new Sprite(renderer, "image/effect/bloodM1.png", 48, 48);
	spr::aimMarker = new Sprite(renderer, "image/UI/aimMarker.png", 48, 48);
	spr::aimMarkerTmp = new Sprite(renderer, "image/UI/aimMarkerTmp.png", 48, 48);

	spr::aimAtkTypeMarker = new Sprite(renderer, "image/UI/aimAtkTypeMarker.png", 64, 64);
	spr::tab = new Sprite(renderer, "image/UI/GUI/HUD/tab.png", 180, 180);
	spr::aimLRChange = new Sprite(renderer, "image/UI/aimLRChange.png", 18, 18);
	spr::epsilonFont = new Sprite(renderer, "image/epsilonFont.png", 4, 6);
	spr::loadingAnime = new Sprite(renderer, "image/UI/loadingAnime.png", 32, 32);
	spr::buildCursor = new Sprite(renderer, "image/UI/buildCursor.png", 48, 48);
	spr::msgChoiceBtn = new Sprite(renderer, "image/UI/msgChoiceBtn.png", 134, 68);

	spr::itemSlotBtn = new Sprite(renderer, "image/UI/item/itemSlotBtn.png", 210, 18);
	spr::itemSlotPocketArrow = new Sprite(renderer, "image/UI/item/itemSlotBtn.png", 8, 40);
	spr::lstSelectBox = new Sprite(renderer, "image/UI/GUI/lstSelectBox.png", 360, 44);// (240, 29)->(360,44)

	spr::skinYellow = new Sprite(renderer, "image/charset/body/skinYellow.png", 48, 48);
	spr::eyesBlue = new Sprite(renderer, "image/charset/body/eyesBlue.png", 48, 48);
	spr::eyesRed = new Sprite(renderer, "image/charset/body/eyesRed.png", 48, 48);
	spr::eyesHalf = new Sprite(renderer, "image/charset/body/eyesHalf.png", 48, 48);
	spr::eyesClosed = new Sprite(renderer, "image/charset/body/eyesClosed.png", 48, 48);
	spr::beardMustacheBlack = new Sprite(renderer, "image/charset/body/beardMustacheBlack.png", 48, 48);

	spr::hairCommaBlack = new Sprite(renderer, "image/charset/body/hairCommaBlack.png", 48, 48);
	spr::hairBob1Black = new Sprite(renderer, "image/charset/body/hairBob1Black.png", 48, 48);
	spr::hairPonytailBlack = new Sprite(renderer, "image/charset/body/hairPonytailBlack.png", 48, 48);
	spr::hairMiddlePart = new Sprite(renderer, "image/charset/body/hairMiddlePart.png", 48, 48);
	spr::hornCoverRed = new Sprite(renderer, "image/charset/body/hornCoverRed.png", 48, 48);

	spr::shadow = new Sprite(renderer, "image/charset/shadow.png", 48, 48);

	spr::vehicleHUD = new Sprite(renderer, "image/UI/GUI/Vehicle/vehicleHUD.png", 900, 580);
	spr::dashboard = new Sprite(renderer, "image/UI/GUI/Vehicle/dashboard.png", 720, 580);
	spr::vehicleHUDParts = new Sprite(renderer, "image/UI/GUI/Vehicle/vehicleHUDParts.png", 64, 64);
	spr::vehicleHUDSteeringWheel = new Sprite(renderer, "image/UI/GUI/Vehicle/vehicleHUDSteeringWheel.png", 256, 256);
	spr::vehicleActCursor = new Sprite(renderer, "image/UI/GUI/Vehicle/vehicleActCursor.png", 72, 72);

	spr::dirMarker = new Sprite(renderer, "image/UI/dirMarker.png", 48, 48);
	spr::windowArrow2 = new Sprite(renderer, "image/UI/GUI/windowArrow2.png", 48, 48);
	spr::mapHereMarker = new Sprite(renderer, "image/UI/GUI/Map/mapHereMarker.png", 23, 23);

	spr::screenRain = new Sprite(renderer, "image/weather/screenRain.png", 720, 720);
	spr::screenSnow = new Sprite(renderer, "image/weather/screenSnow.png", 720, 720);

	spr::symbolSunny = new Sprite(renderer, "image/UI/GUI/HUD/symbolSunny.png", 48, 48);
	spr::symbolCloudy = new Sprite(renderer, "image/UI/GUI/HUD/symbolCloudy.png", 48, 48);
	spr::symbolMoon = new Sprite(renderer, "image/UI/GUI/HUD/symbolMoon.png", 48, 48);
	spr::symbolRain = new Sprite(renderer, "image/UI/GUI/HUD/symbolRain.png", 48, 48);
	spr::symbolStorm = new Sprite(renderer, "image/UI/GUI/HUD/symbolStorm.png", 48, 48);
	spr::symbolSnow = new Sprite(renderer, "image/UI/GUI/HUD/symbolSnow.png", 48, 48);

	spr::tailPedalL = new Sprite(renderer, "image/UI/GUI/HUD/tailPedalL.png", 64, 64);
	spr::tailPedalR = new Sprite(renderer, "image/UI/GUI/HUD/tailPedalR.png", 64, 64);
	spr::trainBrake = new Sprite(renderer, "image/UI/GUI/HUD/trainBrake.png", 80, 80);

	spr::mainRotor = new Sprite(renderer, "image/mainRotor3.png", 128, 128);

	for (const auto& entry : std::filesystem::directory_iterator("image/charset/equip"))
	{
		if (entry.is_regular_file() && entry.path().extension() == ".png")
		{
			spr::spriteMapper[entry.path().stem()] = new Sprite(renderer, entry.path().string(), 48, 48);
		}
	}

	for (const auto& entry : std::filesystem::directory_iterator("image/charset"))
	{
		if (entry.is_regular_file() && entry.path().extension() == ".png")
		{
			spr::spriteMapper[entry.path().stem()] = new Sprite(renderer, entry.path().string(), 48, 48);
		}
	}

	spr::singleQuickSlot = new Sprite(renderer, "image/UI/GUI/HUD/singleQuickSlot.png", 180, 38);
	spr::topQuickSlotBtn = new Sprite(renderer, "image/UI/GUI/HUD/topQuickSlotBtn.png", 43, 38);

	spr::skillSet = new Sprite(renderer, "image/UI/skillSet.png", 16, 16);

	spr::fireStorm = new Sprite(renderer, "image/effect/skill/fireStorm.png", 80, 96);

	spr::flameSet = new Sprite(renderer, "image/tileset/flameSet.png", 48, 48);

	spr::gasEffect1 = new Sprite(renderer, "image/gas/gasEffect1.png", 48, 48);
	spr::steamEffect1 = new Sprite(renderer, "image/gas/steamEffect1.png", 48, 48);

	spr::trail = new Sprite(renderer, "image/trail.png", 16, 16);

	spr::statusEffectRect = new Sprite(renderer, "image/UI/GUI/HUD//statusEffectRect.png", 130, 34);
	spr::statusIcon = new Sprite(renderer, "image/UI/statusIcon.png", 16, 16);

	spr::minimapEdge = new Sprite(renderer, "image/UI/GUI/HUD//minimapEdge.png", 244, 244);

	spr::bulletset = new Sprite(renderer, "image/item/bulletset.png", 48, 48);
	spr::aimMarkerWhite = new Sprite(renderer, "image/UI/aimMarkerWhite.png", 48, 48);

	spr::mapMagnifyIcon = new Sprite(renderer, "image/UI/GUI/Map/mapMagnifyIcon.png", 128, 40);
	spr::staminaGaugeCircle = new Sprite(renderer, "image/UI/GUI/HUD/staminaGaugeCircle.png", 48, 48);
	spr::speechBubble = new Sprite(renderer, "image/UI/GUI/speechBubble.png", 320, 137);
	spr::particle = new Sprite(renderer, "image/particle.png", 16, 16);
	spr::footprint = new Sprite(renderer, "image/footprint.png", 16, 16);
	spr::craftItemRect = new Sprite(renderer, "image/UI/craftItemRect.png", 58, 58);
	spr::msgBox = new Sprite(renderer, "image/UI/msgBox.png", 300, 236);

	spr::skillRect = new Sprite(renderer, "image/UI/GUI/Skill/skillRect.png", 250, 34);

	spr::gridMarker = new Sprite(renderer, "image/UI/gridMarker.png", 16, 16);

	spr::tabBoxAim = new Sprite(renderer, "image/UI/tabBoxAim.png", 122, 122);

	spr::youDied = new Sprite(renderer, "image/UI/youDied.png", 1440, 184);

	spr::gameOverOptionRect = new Sprite(renderer, "image/UI/gameOverOptionRect.png", 220, 160);
	spr::gameOverOptionMarker = new Sprite(renderer, "image/UI/gameOverOptionMarker.png", 156, 48);

	spr::statusEffectGaugeCircle = new Sprite(renderer, "image/UI/GUI/HUD/gauge28.png", 28, 28);
	spr::thoughtBubble = new Sprite(renderer, "image/thoughtBubble.png", 16, 16);
	spr::waveFoam = new Sprite(renderer, "image/waveFoam.png", 16, 16);
	spr::seaFoam = new Sprite(renderer, "image/seaFoam.png", 16, 16);
	spr::waterFoam = new Sprite(renderer, "image/waterFoam.png", 16, 16);

    spr::itemBackgroundRect = new Sprite(renderer, "image/UI/GUI/itemBackgroundRect.png", 68, 68);

    spr::newWindowArrow = new Sprite(renderer, "image/UI/GUI/newWindowArrow.png", 30, 52);

    spr::icon28 = new Sprite(renderer, "image/UI/icon28.png", 28, 28);
    spr::downRightLetterbox = new Sprite(renderer, "image/UI/GUI/HUD/downRightLetterbox.png", 782, 176);
	spr::mainGauge = new Sprite(renderer, "image/UI/GUI/HUD/mainGauge.png", 170, 16);

    spr::bodyShape = new Sprite(renderer, "image/UI/GUI/HUD/bodyShape.png", 100, 256);
    spr::hpGauge = new Sprite(renderer, "image/UI/GUI/HUD/hpGauge.png", 95, 13);

    spr::icon24 = new Sprite(renderer, "image/UI/icon24.png", 24, 24);

    spr::gamepadInstruction = new Sprite(renderer, "image/UI/gamepadInstruction.png", 176, 112);
	spr::gamepadButtons = new Sprite(renderer, "image/UI/gamepadButtons.png", 48, 48);
    spr::keyboardButtons = new Sprite(renderer, "image/UI/keyboardButtons.png", 48, 48);

	spr::icon80 = new Sprite(renderer, "image/UI/icon80.png", 80, 80);

    spr::gearStick = new Sprite(renderer, "image/UI/GUI/Vehicle/gearStick.png", 80, 144);
}