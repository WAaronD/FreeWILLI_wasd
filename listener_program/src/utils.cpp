#include "utils.h"

#include "pch.h"

/**
 * @brief Prints whether the program is running in Debug or Release mode.
 */
void printMode()
{
#ifdef DEBUG
    std::cout << "Running Debug Mode" << std::endl;
#else
    std::cout << "Running Release Mode" << std::endl;
#endif
}

/**
 * @brief Converts a `TimePoint` object to a formatted string representation.
 *
 * This function converts a `TimePoint` object into a string representation that
 * includes the date, time, and microseconds. The output is formatted as
 * "YYYY-MM-DD HH:MM:SS.mmmmmm".
 *
 * @param timePoint A `TimePoint` object representing the time to be converted.
 * @return A string representing the formatted date and time with microseconds.
 */
std::string convertTimePointToString(const TimePoint& timePoint)
{
    // Convert TimePoint to time_t to get calendar time
    std::time_t calendarTime = std::chrono::system_clock::to_time_t(timePoint);
    std::tm* utcTime = std::gmtime(&calendarTime);  // Convert to UTC (or use std::localtime
                                                    // for local time)

    // Format the calendar time (year, month, day, hour, minute, second)
    std::ostringstream formattedTimeStream;
    formattedTimeStream << std::put_time(utcTime, "%y%m%d_%H%M%S");

    // Extract the microseconds from the TimePoint
    auto durationSinceEpoch = timePoint.time_since_epoch();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(durationSinceEpoch) % 1000000;

    // Add the microseconds to the formatted time string
    formattedTimeStream << "_" << std::setw(6) << std::setfill('0') << microseconds.count();  // Zero-pad to 6 digits

    return formattedTimeStream.str();
}
