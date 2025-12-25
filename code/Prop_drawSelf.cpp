import Prop;

#include <SDL3/SDL.h>

import util;
import globalVar;
import constVar;
import textureVar;
import wrapVar;
import drawSprite;
import World;
import globalTime;

import drawText;


void Prop::drawSelf()
{
    int tileSize = 16 * zoomScale;
    int bigShift = 16 * (leadItem.checkFlag(itemFlag::PROP_BIG));
    SDL_Rect dst;
    dst.x = cameraW / 2 + zoomScale * ((16 * getGridX() + 8) - cameraX) - ((16 * zoomScale) / 2);
    dst.y = cameraH / 2 + zoomScale * ((16 * getGridY() + 8 - bigShift) - cameraY) - ((16 * zoomScale) / 2);
    dst.w = tileSize;
    dst.h = tileSize;

    int drawX = dst.x + dst.w / 2 + zoomScale * getIntegerFakeX();
    int drawY = dst.y + dst.h / 2 + zoomScale * getIntegerFakeY();


    setZoom(zoomScale);
    if (leadItem.checkFlag(itemFlag::TREE) && getGridX() == PlayerX() && getGridY() - 1 == PlayerY() && getGridZ() == PlayerZ() && !leadItem.checkFlag(itemFlag::STUMP))
    {
        SDL_SetTextureAlphaMod(spr::propset->getTexture(), 100); //텍스쳐 투명도 설정
    }
    else
    {
        SDL_SetTextureAlphaMod(spr::propset->getTexture(), 255); //텍스쳐 투명도 설정
    }

    SDL_SetTextureBlendMode(spr::propset->getTexture(), SDL_BLENDMODE_BLEND); //블렌드모드 설정
    int sprIndex = leadItem.propSprIndex + leadItem.extraSprIndexSingle + 16 * leadItem.extraSprIndex16;

    if (leadItem.checkFlag(itemFlag::TREE))//나무일 경우 그림자
    {
        drawSpriteCenter
        (
            spr::propset,
            sprIndex + 8,
            drawX,
            drawY
        );
    }




    if (leadItem.checkFlag(itemFlag::CABLE_BEHIND))
    {
        Prop* rightProp = TileProp(getGridX() + 1, getGridY(), getGridZ());
        Prop* topProp = TileProp(getGridX(), getGridY() - 1, getGridZ());
        Prop* lProp = TileProp(getGridX() - 1, getGridY(), getGridZ());
        Prop* dProp = TileProp(getGridX(), getGridY() + 1, getGridZ());

        bool isRightCable = rightProp != nullptr && (rightProp->leadItem.checkFlag(itemFlag::CABLE)||rightProp->leadItem.checkFlag(itemFlag::CABLE_CNCT_LEFT));
        bool isUpCable = topProp != nullptr && (topProp->leadItem.checkFlag(itemFlag::CABLE) || topProp->leadItem.checkFlag(itemFlag::CABLE_CNCT_DOWN));
        bool isLeftCable = lProp != nullptr && (lProp->leadItem.checkFlag(itemFlag::CABLE) || lProp->leadItem.checkFlag(itemFlag::CABLE_CNCT_RIGHT));
        bool isDownCable = dProp != nullptr && (dProp->leadItem.checkFlag(itemFlag::CABLE) || dProp->leadItem.checkFlag(itemFlag::CABLE_CNCT_UP));

        if (isRightCable || isUpCable || isLeftCable || isDownCable)
        {
            int cableSprIndex = 2720;

            if (isRightCable && !isUpCable && !isLeftCable && !isDownCable) cableSprIndex = 3036;
            else if (!isRightCable && isUpCable && !isLeftCable && !isDownCable) cableSprIndex = 3037;
            else if (!isRightCable && !isUpCable && isLeftCable && !isDownCable) cableSprIndex = 3034;
            else if (!isRightCable && !isUpCable && !isLeftCable && isDownCable) cableSprIndex = 3035;

            else if (isRightCable && isUpCable && !isLeftCable && !isDownCable) cableSprIndex = 3033;
            else if (isRightCable && !isUpCable && isLeftCable && !isDownCable) cableSprIndex = 3240;
            else if (isRightCable && !isUpCable && !isLeftCable && isDownCable) cableSprIndex = 3029;
            else if (!isRightCable && isUpCable && isLeftCable && !isDownCable) cableSprIndex = 3033;
            else if (!isRightCable && isUpCable && !isLeftCable && isDownCable) cableSprIndex = 3243;
            else if (!isRightCable && !isUpCable && isLeftCable && isDownCable) cableSprIndex = 3027;

            else if (isRightCable && isUpCable && isLeftCable && !isDownCable) cableSprIndex = 3032;
            else if (isRightCable && isUpCable && !isLeftCable && isDownCable) cableSprIndex = 3030;
            else if (isRightCable && !isUpCable && isLeftCable && isDownCable) cableSprIndex = 3028;
            else if (!isRightCable && isUpCable && isLeftCable && isDownCable) cableSprIndex = 3026;

            else if (isRightCable && isUpCable && isLeftCable && isDownCable) cableSprIndex = 3024;


            drawSpriteCenter
            (
                spr::propset,
                cableSprIndex,
                drawX,
                drawY
            );


            if (isChargeFlowing())
            {
                if (chargeFlux[dir16::right] > 0) drawSpriteCenter(spr::propset, 3041, drawX, drawY);;
                if (chargeFlux[dir16::up] > 0) drawSpriteCenter(spr::propset, 3042, drawX, drawY);;
                if (chargeFlux[dir16::left] > 0) drawSpriteCenter(spr::propset, 3043, drawX, drawY);;
                if (chargeFlux[dir16::down] > 0) drawSpriteCenter(spr::propset, 3044, drawX, drawY);;
            }
        }
    }

    if (leadItem.checkFlag(itemFlag::PLANT_SEASON_DEPENDENT) && !leadItem.checkFlag(itemFlag::STUMP))
    {
        if (World::ins()->getTile(getGridX(), getGridY(), PlayerZ()).hasSnow == true) sprIndex += 4;
        else
        {
            if (getSeason() == seasonFlag::summer) { sprIndex += 1; }
            else if (getSeason() == seasonFlag::autumn) { sprIndex += 2; }
            else if (getSeason() == seasonFlag::winter) { sprIndex += 3; }
        }
    }
    else if (leadItem.checkFlag(itemFlag::STUMP))
    {
        sprIndex += 7;
    }
    else if (leadItem.itemCode == itemRefCode::gasolineGeneratorR ||
        leadItem.itemCode == itemRefCode::gasolineGeneratorT ||
        leadItem.itemCode == itemRefCode::gasolineGeneratorL ||
        leadItem.itemCode == itemRefCode::gasolineGeneratorB)
    {
        if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
        {
            static Uint32 lastUpdateTime = 0;
            static int animFrame = 0;

            Uint32 currentTime = SDL_GetTicks();

            if (currentTime - lastUpdateTime >= 100)
            {
                animFrame = (animFrame + 1) % 3;
                lastUpdateTime = currentTime;
            }

            sprIndex += (2 + animFrame);
        }
    }


    if (leadItem.itemCode == itemRefCode::leverRL || leadItem.itemCode == itemRefCode::leverUD)
    {
        if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
        {
            if (isChargeFlowing()) sprIndex += 2;
            else sprIndex += 1;
        }
    }
    else if (leadItem.itemCode == itemRefCode::tactSwitchRL || leadItem.itemCode == itemRefCode::tactSwitchUD)
    {
        if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
        {
            if (isChargeFlowing()) sprIndex += 2;
            else sprIndex += 1;
        }
    }
    else if (leadItem.itemCode == itemRefCode::pressureSwitchRL || leadItem.itemCode == itemRefCode::pressureSwitchUD)
    {
        if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
        {
            if (isChargeFlowing()) sprIndex += 2;
            else sprIndex += 1;
        }
    }
    else if (leadItem.itemCode == itemRefCode::transistorR || leadItem.itemCode == itemRefCode::transistorU || leadItem.itemCode == itemRefCode::transistorL || leadItem.itemCode == itemRefCode::transistorD)
    {
        if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
        {
            if (leadItem.itemCode == itemRefCode::transistorU || leadItem.itemCode == itemRefCode::transistorD)
            {
                Prop* leftProp = TileProp(getGridX() - 1, getGridY(), getGridZ());
                Prop* rightProp = TileProp(getGridX() + 1, getGridY(), getGridZ());
                if (leftProp != nullptr && leftProp->isChargeFlowing()) sprIndex += 2;
                else if (rightProp != nullptr && rightProp->isChargeFlowing()) sprIndex += 2;
                else sprIndex += 1;
            }
            else if (leadItem.itemCode == itemRefCode::transistorR || leadItem.itemCode == itemRefCode::transistorL)
            {
                Prop* upProp = TileProp(getGridX(), getGridY() - 1, getGridZ());
                Prop* downProp = TileProp(getGridX(), getGridY() + 1, getGridZ());
                if (upProp != nullptr && upProp->isChargeFlowing()) sprIndex += 2;
                else if (downProp != nullptr && downProp->isChargeFlowing()) sprIndex += 2;
                else sprIndex += 1;
            }
        }
    }
    else if (leadItem.itemCode== itemRefCode::relayR || leadItem.itemCode == itemRefCode::relayU || leadItem.itemCode == itemRefCode::relayL || leadItem.itemCode == itemRefCode::relayD)
    {
        if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
        {
            if(leadItem.itemCode == itemRefCode::relayU || leadItem.itemCode == itemRefCode::relayD)
            {
                Prop* leftProp = TileProp(getGridX() - 1, getGridY(), getGridZ());
                Prop* rightProp = TileProp(getGridX() + 1, getGridY(), getGridZ());
                if (leftProp != nullptr && leftProp->isChargeFlowing()) sprIndex += 2;
                else if (rightProp != nullptr && rightProp->isChargeFlowing()) sprIndex += 2;
                else sprIndex += 1;
            }
            else if (leadItem.itemCode == itemRefCode::relayR || leadItem.itemCode == itemRefCode::relayL)
            {
                Prop* upProp = TileProp(getGridX(), getGridY() - 1, getGridZ());
                Prop* downProp = TileProp(getGridX(), getGridY() + 1, getGridZ());
                if (upProp != nullptr && upProp->isChargeFlowing()) sprIndex += 2;
                else if (downProp != nullptr && downProp->isChargeFlowing()) sprIndex += 2;
                else sprIndex += 1;
            }
        }
    }
    else if (leadItem.itemCode == itemRefCode::andGateR 
        || leadItem.itemCode == itemRefCode::andGateL
        || leadItem.itemCode == itemRefCode::orGateR
        || leadItem.itemCode == itemRefCode::orGateL
        || leadItem.itemCode == itemRefCode::xorGateR
        || leadItem.itemCode == itemRefCode::xorGateL
        || leadItem.itemCode == itemRefCode::notGateR
        || leadItem.itemCode == itemRefCode::notGateL
        || leadItem.itemCode == itemRefCode::srLatchR
        || leadItem.itemCode == itemRefCode::srLatchL
        )
    {
        if (leadItem.checkFlag(itemFlag::PROP_POWER_ON)) sprIndex += 1;
    }
    else if (leadItem.itemCode == itemRefCode::delayR || leadItem.itemCode == itemRefCode::delayL)
    {
        if (delayMaxStack == 0) sprIndex = 3089;
        else sprIndex = 3089 + delayMaxStack;

        if (leadItem.itemCode == itemRefCode::delayL) sprIndex += 16;
    }



    if (leadItem.checkFlag(itemFlag::CABLE) && leadItem.checkFlag(itemFlag::CROSSED_CABLE))
    {
        bool flowH = (chargeFlux[dir16::right] != 0) || (chargeFlux[dir16::left] != 0);
        bool flowV = (chargeFlux[dir16::up] != 0) || (chargeFlux[dir16::down] != 0);

        int baseIndex = (leadItem.itemCode == itemRefCode::silverCable) ? 3128 : 3124;
        int offset = 0;
        if (flowH && flowV) offset = 3;
        else if (flowH) offset = 1;
        else if (flowV) offset = 2;
        sprIndex = baseIndex + offset;
    }

    drawSpriteCenter
    (
        spr::propset,
        sprIndex,
        drawX,
        drawY
    );


    if (leadItem.itemCode == itemRefCode::powerBankR)
    {
        if (chargeFlux[dir16::left] > 0) drawSpriteCenter(spr::propset,3159,drawX,drawY);

        if (chargeFlux[dir16::right] < 0) drawSpriteCenter(spr::propset, 3160, drawX, drawY);
    }
    else if (leadItem.itemCode == itemRefCode::powerBankL)
    {
        if (chargeFlux[dir16::left] > 0) drawSpriteCenter(spr::propset, 3159 + 16, drawX, drawY);

        if (chargeFlux[dir16::right] < 0) drawSpriteCenter(spr::propset, 3160 + 16, drawX, drawY);
    }

    if (leadItem.checkFlag(itemFlag::PROP_POWER_ON))
    {
        if (leadItem.itemCode == itemRefCode::gasolineGeneratorR ||
            leadItem.itemCode == itemRefCode::gasolineGeneratorT ||
            leadItem.itemCode == itemRefCode::gasolineGeneratorL ||
            leadItem.itemCode == itemRefCode::gasolineGeneratorB)
        {
            int portSprIndex;
            if (leadItem.itemCode == itemRefCode::gasolineGeneratorR) portSprIndex = 3200;
            else if (leadItem.itemCode == itemRefCode::gasolineGeneratorT)  portSprIndex = 3201;
            else if (leadItem.itemCode == itemRefCode::gasolineGeneratorL)   portSprIndex = 3202;
            else if (leadItem.itemCode == itemRefCode::gasolineGeneratorB)   portSprIndex = 3203;

            float pulseSpeed = 0.003f; // 펄스 속도 (작을수록 느림)
            float minBrightness = 0.6f; // 최소 밝기 (0.0~1.0)
            float maxBrightness = 1.0f; // 최대 밝기

            float pulse = (sin(SDL_GetTicks() * pulseSpeed) + 1.0f) * 0.5f; // 0.0~1.0 사이값
            float colorAlpha = minBrightness + (maxBrightness - minBrightness) * pulse;

            SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
            SDL_SetTextureColorMod(spr::propset->getTexture(),
                (Uint8)(255.0f * colorAlpha),
                (Uint8)(255.0f * colorAlpha),
                (Uint8)(255.0f * colorAlpha));

            drawSpriteCenter
            (
                spr::propset,
                portSprIndex,
                drawX,
                drawY
            );

            SDL_SetTextureColorMod(spr::propset->getTexture(), 255, 255, 255);
        }
    }

    if (treeAngle != 0)
    {
        int extraSprIndex = 9;
        if (World::ins()->getTile(getGridX(), getGridY(), PlayerZ()).hasSnow == true) extraSprIndex += 4;
        else
        {
            if (getSeason() == seasonFlag::summer) { extraSprIndex += 1; }
            else if (getSeason() == seasonFlag::autumn) { extraSprIndex += 2; }
            else if (getSeason() == seasonFlag::winter) { extraSprIndex += 3; }
        }
        SDL_Point pt = { 24.0 * zoomScale,40.0 * zoomScale };
        drawSpriteCenterRotate
        (
            spr::propset,
            leadItem.propSprIndex + extraSprIndex,
            drawX,
            drawY,
            treeAngle,
            &pt
        );
    }

    if (leadItem.checkFlag(itemFlag::CABLE) && leadItem.checkFlag(itemFlag::CROSSED_CABLE) == false)
    {
        Prop* rProp = TileProp(getGridX() + 1, getGridY(), getGridZ());
        Prop* uProp = TileProp(getGridX(), getGridY() - 1, getGridZ());
        Prop* lProp = TileProp(getGridX() - 1, getGridY(), getGridZ());
        Prop* dProp = TileProp(getGridX(), getGridY() + 1, getGridZ());

        if (getGridX() == 11 && getGridY() == -15)
        {
            int a = 3;
        }

        bool rConnected = rProp != nullptr && (rProp->leadItem.checkFlag(itemFlag::CABLE) || rProp->leadItem.checkFlag(itemFlag::CABLE_CNCT_LEFT));
        bool uConnected = uProp != nullptr && (uProp->leadItem.checkFlag(itemFlag::CABLE) || uProp->leadItem.checkFlag(itemFlag::CABLE_CNCT_DOWN));
        bool lConnected = lProp != nullptr && (lProp->leadItem.checkFlag(itemFlag::CABLE) || lProp->leadItem.checkFlag(itemFlag::CABLE_CNCT_RIGHT));
        bool dConnected = dProp != nullptr && (dProp->leadItem.checkFlag(itemFlag::CABLE) || dProp->leadItem.checkFlag(itemFlag::CABLE_CNCT_UP));

        if (isChargeFlowing())
        {
            if (rConnected && lConnected && !uConnected && !dConnected) //─
            {
                drawSpriteCenter(spr::propset, 2949, drawX, drawY);
            }
            else if (!rConnected && !lConnected && uConnected && dConnected) //│
            {
                drawSpriteCenter(spr::propset, 2950, drawX, drawY);
            }
            else
            {
                drawSpriteCenter(spr::propset, 2944, drawX, drawY);

                if (rConnected)
                {
                    if (rProp->isChargeFlowing())
                        drawSpriteCenter(spr::propset, 2945, drawX, drawY);
                }
                
                if (uConnected)
                {
                    if (uProp->isChargeFlowing())
                        drawSpriteCenter(spr::propset, 2946, drawX, drawY);
                }

                if(lConnected)
                {
                    if (lProp->isChargeFlowing())
                        drawSpriteCenter(spr::propset, 2947, drawX, drawY);
                }

                if(dConnected)
                {
                    if (dProp->isChargeFlowing())
                        drawSpriteCenter(spr::propset, 2948, drawX, drawY);
                }
            }
        }


        setFlip((getGridZ() % 2 != 0) ? SDL_FLIP_HORIZONTAL : SDL_FLIP_NONE);

        if (leadItem.checkFlag(itemFlag::CABLE_Z_ASCEND) || leadItem.checkFlag(itemFlag::CABLE_Z_DESCEND))
        {
            bool downConnected = false;
            Prop* downProp = TileProp(getGridX(), getGridY() + 1, getGridZ());
            if (downProp != nullptr && (downProp->leadItem.checkFlag(itemFlag::CABLE) || downProp->leadItem.checkFlag(itemFlag::CABLE_CNCT_UP))) downConnected = true;

            if (leadItem.itemCode == itemRefCode::copperCable) drawSpriteCenter(spr::propset, 2993 + downConnected, drawX, drawY);
            if (leadItem.itemCode == itemRefCode::silverCable) drawSpriteCenter(spr::propset, 2993 + 16 + downConnected, drawX, drawY);

            if (isChargeFlowing()) drawSpriteCenter(spr::propset, 2944, drawX, drawY);
        }

        if (leadItem.checkFlag(itemFlag::CABLE_Z_ASCEND))
        {
            if (leadItem.itemCode == itemRefCode::copperCable)
            {
                drawSpriteCenter(spr::propset, 2995, drawX, drawY);//상단으로 이어진 구리 케이블
            }
            else if (leadItem.itemCode == itemRefCode::silverCable)
            {
                drawSpriteCenter(spr::propset, 2995 + 16, drawX, drawY);//상단으로 이어진 은 케이블
            }

            if (isChargeFlowing()) drawSpriteCenter(spr::propset, 2951, drawX, drawY);
        }

        if (leadItem.checkFlag(itemFlag::CABLE_Z_DESCEND))
        {
            if (leadItem.itemCode == itemRefCode::copperCable)
            {
                drawSpriteCenter(spr::propset, 2997, drawX, drawY);//하단으로 이어진 구리 케이블
            }
            else if (leadItem.itemCode == itemRefCode::silverCable)
            {
                drawSpriteCenter(spr::propset, 2997 + 16, drawX, drawY);//하단으로 이어진 은 케이블
            }

            if (isChargeFlowing()) drawSpriteCenter(spr::propset, 2952, drawX, drawY);
        }

        setFlip(SDL_FLIP_NONE);
    }

    //if (leadItem.checkFlag(itemFlag::CIRCUIT))
    //{

    //    setFontSize(9);
    //    renderTextOutlineCenter(std::to_wstring(nodeCharge) + L"/" + std::to_wstring(nodeMaxCharge),
    //        drawX,
    //        dst.y + dst.h / 2 - (int)(12 * zoomScale) + zoomScale * getIntegerFakeY(),
    //        col::white);
    //}

    if (displayHPBarCount > 0)//개체 HP 표기
    {
        int pivotX = dst.x + dst.w / 2 - (int)(8 * zoomScale);
        int pivotY = dst.y + dst.h / 2 + (int)(16 * zoomScale);
        if (leadItem.propFakeHP > leadItem.propHP) leadItem.propFakeHP -= ((float)leadItem.propMaxHP / 100.0);
        else if (leadItem.propFakeHP < leadItem.propHP) leadItem.propFakeHP = leadItem.propHP;
        if (leadItem.propFakeHP != leadItem.propHP)
        {
            if (alphaFakeHPBar > 20) alphaFakeHPBar -= 20;
            else
            {
                alphaFakeHPBar = 0;
                leadItem.propFakeHP = leadItem.propHP;
            }
        }
        else alphaFakeHPBar = 0;
        if (displayHPBarCount > 1) displayHPBarCount--;
        else if (displayHPBarCount == 1)
        {
            alphaHPBar -= 10;
            if (alphaHPBar <= 0)
            {
                alphaHPBar = 0;
                displayHPBarCount = 0;
            }
        }
        draw3pxGauge(
            pivotX,
            pivotY,
            zoomScale,
            (float)leadItem.propHP / (float)leadItem.propMaxHP,
            alphaHPBar,
            lowCol::green,
            (float)leadItem.propFakeHP / (float)leadItem.propMaxHP,
            alphaFakeHPBar
        );
    }

    SDL_SetTextureAlphaMod(spr::propset->getTexture(), 255); //텍스쳐 투명도 설정
    setZoom(1.0);
};