export module dir16;

import std;
import Vec3;
import errorBox;

export enum class dir16
{
    dir0,
    dir0_5,
    dir1,
    dir1_5,
    dir2,
    dir2_5,
    dir3,
    dir3_5,
    dir4,
    dir4_5,
    dir5,
    dir5_5,
    dir6,
    dir6_5,
    dir7,
    dir7_5,
    none,

    right = dir0,
	upRight = dir1,
	up = dir2,
	upLeft = dir3,
	left = dir4,
	downLeft = dir5,
	down = dir6,
	downRight = dir7,
    above,//+Z
    below,//-Z
};

export dir16 ACW(dir16 input)
{
    if (input == dir16::dir0) return dir16::dir0_5;
    else if (input == dir16::dir0_5) return dir16::dir1;
    else if (input == dir16::dir1) return dir16::dir1_5;
    else if (input == dir16::dir1_5) return dir16::dir2;
    else if (input == dir16::dir2) return dir16::dir2_5;
    else if (input == dir16::dir2_5) return dir16::dir3;
    else if (input == dir16::dir3) return dir16::dir3_5;
    else if (input == dir16::dir3_5) return dir16::dir4;
    else if (input == dir16::dir4) return dir16::dir4_5;
    else if (input == dir16::dir4_5) return dir16::dir5;
    else if (input == dir16::dir5) return dir16::dir5_5;
    else if (input == dir16::dir5_5) return dir16::dir6;
    else if (input == dir16::dir6) return dir16::dir6_5;
    else if (input == dir16::dir6_5) return dir16::dir7;
    else if (input == dir16::dir7) return dir16::dir7_5;
    else if (input == dir16::dir7_5) return dir16::dir0;
    else errorBox(L"[utility] ACW에서 알 수 없는 방향이 입력되었다.");
}

export dir16 ACW2(dir16 input)
{
    return ACW(ACW(input));
}

export dir16 CW(dir16 input)
{
    if (input == dir16::dir0) return dir16::dir7_5;
    else if (input == dir16::dir0_5) return dir16::dir0;
    else if (input == dir16::dir1) return dir16::dir0_5;
    else if (input == dir16::dir1_5) return dir16::dir1;
    else if (input == dir16::dir2) return dir16::dir1_5;
    else if (input == dir16::dir2_5) return dir16::dir2;
    else if (input == dir16::dir3) return dir16::dir2_5;
    else if (input == dir16::dir3_5) return dir16::dir3;
    else if (input == dir16::dir4) return dir16::dir3_5;
    else if (input == dir16::dir4_5) return dir16::dir4;
    else if (input == dir16::dir5) return dir16::dir4_5;
    else if (input == dir16::dir5_5) return dir16::dir5;
    else if (input == dir16::dir6) return dir16::dir5_5;
    else if (input == dir16::dir6_5) return dir16::dir6;
    else if (input == dir16::dir7) return dir16::dir6_5;
    else if (input == dir16::dir7_5) return dir16::dir7;
    else errorBox(L"[utility] CW에서 알 수 없는 방향이 입력되었다.");
}

export dir16 CW2(dir16 input)
{
    return CW(CW(input));
}

export dir16 reverse(dir16 input)
{
    if (input == dir16::dir0) return dir16::dir4;
    else if (input == dir16::dir0_5) return dir16::dir4_5;
    else if (input == dir16::dir1) return dir16::dir5;
    else if (input == dir16::dir1_5) return dir16::dir5_5;
    else if (input == dir16::dir2) return dir16::dir6;
    else if (input == dir16::dir2_5) return dir16::dir6_5;
    else if (input == dir16::dir3) return dir16::dir7;
    else if (input == dir16::dir3_5) return dir16::dir7_5;
    else if (input == dir16::dir4) return dir16::dir0;
    else if (input == dir16::dir4_5) return dir16::dir0_5;
    else if (input == dir16::dir5) return dir16::dir1;
    else if (input == dir16::dir5_5) return dir16::dir1_5;
    else if (input == dir16::dir6) return dir16::dir2;
    else if (input == dir16::dir6_5) return dir16::dir2_5;
    else if (input == dir16::dir7) return dir16::dir3;
    else if (input == dir16::dir7_5) return dir16::dir3_5;
    else if (input == dir16::above) return dir16::below;
    else if (input == dir16::below) return dir16::above;
    else errorBox(L"[utility] reverse에서 알 수 없는 방향이 입력되었다.");
}

export std::wstring dir16ToString(dir16 input)
{
    if (input == dir16::dir0) return L"0°";
    else if (input == dir16::dir0_5) return L"22.5°";
    else if (input == dir16::dir1) return L"45°";
    else if (input == dir16::dir1_5) return L"67.5°";
    else if (input == dir16::dir2) return L"90°";
    else if (input == dir16::dir2_5) return L"112.5°";
    else if (input == dir16::dir3) return L"135°";
    else if (input == dir16::dir3_5) return L"157.5°";
    else if (input == dir16::dir4) return L"180°";
    else if (input == dir16::dir4_5) return L"202.5°";
    else if (input == dir16::dir5) return L"225°";
    else if (input == dir16::dir5_5) return L"247.5°";
    else if (input == dir16::dir6) return L"270°";
    else if (input == dir16::dir6_5) return L"292.5°";
    else if (input == dir16::dir7) return L"315°";
    else if (input == dir16::dir7_5) return L"337.5°";
    else errorBox(L"[utility] dir16ToString에서 알 수 없는 방향이 입력되었다.");
}

export Vec3 dir16ToVec(dir16 input)
{
    if (input == dir16::dir0) return getDefaultVec(0);
    else if (input == dir16::dir0_5) return getDefaultVec(1);
    else if (input == dir16::dir1) return getDefaultVec(2);
    else if (input == dir16::dir1_5) return getDefaultVec(3);
    else if (input == dir16::dir2) return getDefaultVec(4);
    else if (input == dir16::dir2_5) return getDefaultVec(5);
    else if (input == dir16::dir3) return getDefaultVec(6);
    else if (input == dir16::dir3_5) return getDefaultVec(7);
    else if (input == dir16::dir4) return getDefaultVec(8);
    else if (input == dir16::dir4_5) return getDefaultVec(9);
    else if (input == dir16::dir5) return getDefaultVec(10);
    else if (input == dir16::dir5_5) return getDefaultVec(11);
    else if (input == dir16::dir6) return getDefaultVec(12);
    else if (input == dir16::dir6_5) return getDefaultVec(13);
    else if (input == dir16::dir7) return getDefaultVec(14);
    else if (input == dir16::dir7_5) return getDefaultVec(15);
    else errorBox(L"[utility] dir16ToVec에서 알 수 없는 방향이 입력되었다.");
}

export int dir16toInt16(dir16 input)
{
    if (input == dir16::dir0) return 0;
    else if (input == dir16::dir0_5) return 1;
    else if (input == dir16::dir1) return 2;
    else if (input == dir16::dir1_5) return 3;
    else if (input == dir16::dir2) return 4;
    else if (input == dir16::dir2_5) return 5;
    else if (input == dir16::dir3) return 6;
    else if (input == dir16::dir3_5) return 7;
    else if (input == dir16::dir4) return 8;
    else if (input == dir16::dir4_5) return 9;
    else if (input == dir16::dir5) return 10;
    else if (input == dir16::dir5_5) return 11;
    else if (input == dir16::dir6) return 12;
    else if (input == dir16::dir6_5) return 13;
    else if (input == dir16::dir7) return 14;
    else if (input == dir16::dir7_5) return 15;
    else errorBox(L"[utility] dir16toInt16에서 알 수 없는 방향이 입력되었다.");
}


export dir16 int8todir16(int inputDir)
{
    if (inputDir == 0) return dir16::dir0;
    else if (inputDir == 1) return dir16::dir1;
    else if (inputDir == 2) return dir16::dir2;
    else if (inputDir == 3) return dir16::dir3;
    else if (inputDir == 4) return dir16::dir4;
    else if (inputDir == 5) return dir16::dir5;
    else if (inputDir == 6) return dir16::dir6;
    else if (inputDir == 7) return dir16::dir7;
    else errorBox(L"[utility] int8todir16에서 0~7 범위가 아닌 dir이 입력되었다.");
}

export float dir16toAngle(dir16 input)
{
    switch (input)
    {
    case dir16::dir0:   return   0.f;
    case dir16::dir0_5: return  22.5f;
    case dir16::dir1:   return  45.f;
    case dir16::dir1_5: return  67.5f;
    case dir16::dir2:   return  90.f;
    case dir16::dir2_5: return 112.5f;
    case dir16::dir3:   return 135.f;
    case dir16::dir3_5: return 157.5f;
    case dir16::dir4:   return 180.f;
    case dir16::dir4_5: return 202.5f;
    case dir16::dir5:   return 225.f;
    case dir16::dir5_5: return 247.5f;
    case dir16::dir6:   return 270.f;
    case dir16::dir6_5: return 292.5f;
    case dir16::dir7:   return 315.f;
    case dir16::dir7_5: return 337.5f;
    default:            return   0.f;
    }
};

export void dirToXYZ(dir16 inputDir, int& dx, int& dy, int& dz)
{
    switch (inputDir)
    {
    case dir16::above: dx =  0; dy =  0; dz = 1; break;
    case dir16::below:dx =  0; dy =  0; dz = -1; break;

    case dir16::right:     dx = 1; dy = 0; dz = 0; break;
    case dir16::upRight:   dx = 1; dy = -1; dz = 0; break;
    case dir16::up:        dx = 0; dy = -1; dz = 0; break;
    case dir16::upLeft:    dx = -1; dy = -1; dz = 0; break;
    case dir16::left:     dx = -1; dy = 0; dz = 0; break;
    case dir16::downLeft:  dx = -1; dy = 1; dz = 0; break;
    case dir16::down:      dx = 0; dy = 1; dz = 0; break;
    case dir16::downRight: dx = 1; dy = 1; dz = 0; break;
    default:            dx =  0; dy =  0; dz = 0; break;
    }
}