export module decimalCutter;
import std;
//@brief 실수를 입력했을 때 해당 소수점까지 반올림하고 그 문자열을 반환하는 함수
//@param val 입력할 실수
//@param place 표시할 소수점 자릿수
//@return std::wstring
export std::wstring decimalCutter(float val, int place)
{
    std::wstring targetStr = std::to_wstring(val);
    bool countStart = false;
    int endPoint = 0;
    for (int i = 0; i < targetStr.size(); i++)
    {
        if (countStart == true)
        {
            place--;
            if (place == 0)
            {
                endPoint = i;
                break;
            }
        }
        if (targetStr[i] == 46)
        {
            countStart = true;
        }
    }

    std::wstring result = targetStr.substr(0, endPoint + 1);

    if (result.find(L'.') != std::wstring::npos)
    {
        while (!result.empty() && result.back() == L'0')
        {
            result.pop_back();
        }
        if (!result.empty() && result.back() == L'.')
        {
            result.pop_back();
        }
    }

    return result;
}