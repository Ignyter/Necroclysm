export module decimalCutter;
import std;
//@brief 실수를 입력했을 때 해당 소수점까지 반올림하고 그 문자열을 반환하는 함수
//@param val 입력할 실수
//@param place 표시할 소수점 자릿수
//@return std::wstring
export std::wstring decimalCutter(float val, int place)
{
    if (place <= 0)
    {
        return std::to_wstring(static_cast<int>(std::round(val)));
    }

    float multiplier = std::pow(10.0f, place);
    float rounded = std::round(val * multiplier) / multiplier;

    std::wostringstream oss;
    oss << std::fixed << std::setprecision(place) << rounded;
    std::wstring result = oss.str();

    // 뒤의 불필요한 0 제거
    while (!result.empty() && result.back() == L'0')
    {
        result.pop_back();
    }
    if (!result.empty() && result.back() == L'.')
    {
        result.pop_back();
    }

    return result;
}