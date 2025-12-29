export module gasData;

import std;

export class gasData
{
public:
    int gasCode;
    int gasVol;
    bool operator==(const gasData& other) const
    {
        return gasCode == other.gasCode && gasVol == other.gasVol;
    }
};

namespace std
{
    template<>
    struct hash<gasData>
    {
        std::size_t operator()(const gasData& g) const
        {
            return std::hash<int>()(g.gasCode);
        }
    };
}