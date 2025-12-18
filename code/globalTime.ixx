export module globalTime;
import std;
import constVar;
static int year = START_YEAR;
static int month = START_MONTH;
static int day = START_DAY;
static int hour = START_HOUR;
static int min = START_MINUTE;
static int sec = 0;

export int getYear() { return year; }
export int getMonth() { return month; }
export int getDay() { return day; }
export int getHour() { return hour; }
export int getMin() { return min; }
export int getSec() { return sec; }

export seasonFlag getSeason()
{
    if (getMonth() == 12 || getMonth() == 1 || getMonth() == 2) return seasonFlag::winter;
    else if (getMonth() == 3 || getMonth() == 4 || getMonth() == 5) return seasonFlag::spring;
    else if (getMonth() == 6 || getMonth() == 7 || getMonth() == 8) return seasonFlag::summer;
    else return seasonFlag::autumn;
}

int getDaysInMonth(int targetYear, int targetMonth)
{
    switch (targetMonth)
    {
    case 1: case 3: case 5: case 7: case 8: case 10: case 12:
        return 31;
    case 4: case 6: case 9: case 11:
        return 30;
    case 2:
        if (targetYear % 4 == 0 && (targetYear % 100 != 0 || targetYear % 400 == 0))
            return 29;
        else
            return 28;
    default:
        return 30;
    }
}

void addTimeMonth(int inputMonth)
{
    month += inputMonth;
    while (month > 12)
    {
        month -= 12;
        year++;
    }
}

void addTimeDay(int inputDay)
{
    day += inputDay;
    int endOfMonth = getDaysInMonth(year, month);

    while (day > endOfMonth)
    {
        day -= endOfMonth;
        addTimeMonth(1);
        endOfMonth = getDaysInMonth(year, month);
    }
}

void addTimeHour(int inputHour)
{
    hour += inputHour;
    while (hour >= 24)
    {
        hour -= 24;
        addTimeDay(1);
    }
}

void addTimeMinInt(int inputMin)
{
    min += inputMin;
    while (min >= 60)
    {
        min -= 60;
        addTimeHour(1);
    }
}

void addTimeSec(int inputSec)
{
    sec += inputSec;
    while (sec >= 60)
    {
        sec -= 60;
        addTimeMinInt(1);
    }
}

export void setDebugTime(int inputHour, int inputMin)
{
    hour = inputHour;
    min = inputMin;
}

export void setDebugDay(int inputMonth, int inputDay)
{
    month = inputMonth;
    day = inputDay;
}

export void addTimeTurn(float inputTurn)
{
    addTimeSec((int)(60.0 * (inputTurn - (int)inputTurn)));
    min += (int)(inputTurn);
    while (min >= 60)
    {
        min -= 60;
        addTimeHour(1);
    }
}

export int getElapsedDays()
{
    int totalDays = 0;

    for (int y = START_YEAR; y < year; y++)
    {
        if (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0))
            totalDays += 366;
        else
            totalDays += 365;
    }

    if (year == START_YEAR)
    {
        for (int m = START_MONTH; m < month; m++)
        {
            totalDays += getDaysInMonth(year, m);
        }
    }
    else
    {
        for (int m = START_MONTH; m <= 12; m++)
        {
            totalDays += getDaysInMonth(START_YEAR, m);
        }
        for (int m = 1; m < month; m++)
        {
            totalDays += getDaysInMonth(year, m);
        }
    }

    totalDays += (day - START_DAY);

    return totalDays;
}

// index 0 : new moon (신월)
// index 1 : waxing crescent (초승달)
// index 2 : first quarter (상현달)
// index 3 : waxing gibbous (상현망)
// index 4 : full moon (보름달)
// index 5 : waning gibbous (하현망)
// index 6 : third quarter (하현달)
// index 7 : waning crescent (그믐달)
export int calculateMoonPhase()
{
    const double LUNAR_CYCLE_DAYS = 29.53058867;
    double totalTime = getElapsedDays();
    totalTime += (getHour() * 60.0 + getMin()) / (24.0 * 60.0);
    double cycleProgress = fmod(totalTime, LUNAR_CYCLE_DAYS) / LUNAR_CYCLE_DAYS;
    int phaseIndex = (int)(cycleProgress * 8.0);
    return std::max(0, std::min(7, phaseIndex)); 
};

export double getElapsedTurn()
{
    // 1턴 = 1분
    double totalMin = 0.0;
    totalMin += (double)getElapsedDays() * 24.0 * 60.0;
    totalMin += (double)getHour() * 60.0;
    totalMin += (double)getMin();
    totalMin += (double)getSec() / 60.0; 
    return totalMin;
}