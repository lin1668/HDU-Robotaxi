#pragma once

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <mutex>

// Unified diagnostics for YFork, Station and final vehicle-control output.
// All writers share one stream so a single yfork.log contains the complete
// cause-and-effect chain for every frame.
inline FILE *&yforkDiagFile()
{
    static FILE *fp = nullptr;
    return fp;
}

inline std::mutex &yforkDiagMutex()
{
    static std::mutex mutex;
    return mutex;
}

inline unsigned long long &yforkDiagLineCounter()
{
    static unsigned long long counter = 0;
    return counter;
}

// Keep periodic state snapshots without flooding yfork.log. Trigger,
// transition and completion records pass through unchanged.
inline bool yforkDiagShouldWrite(const char *source)
{
    static unsigned int stationFrames = 0;
    static unsigned int controlFrames = 0;
    static unsigned int parkFrames = 0;

    if (!source)
        return true;
    if (std::strcmp(source, "STATION_FRAME") == 0)
        return (++stationFrames % 10) == 0;
    if (std::strcmp(source, "CTRL_FINAL") == 0)
        return (++controlFrames % 10) == 0;
    if (std::strcmp(source, "PARK_FRAME") == 0)
        return (++parkFrames % 10) == 0;
    return true;
}

inline void yforkDiagReset(const char *reason)
{
    std::lock_guard<std::mutex> lock(yforkDiagMutex());
    FILE *&fp = yforkDiagFile();
    if (fp)
        fclose(fp);
    fp = fopen("./yfork.log", "w");
    yforkDiagLineCounter() = 0;
    if (!fp)
        return;
    fprintf(fp, "[YFORK_DIAG] SESSION_START reason=%s\n", reason ? reason : "unknown");
    fflush(fp);
}

inline void yforkDiagVLog(const char *source, const char *fmt, va_list args)
{
    std::lock_guard<std::mutex> lock(yforkDiagMutex());
    if (!yforkDiagShouldWrite(source))
        return;
    FILE *&fp = yforkDiagFile();
    if (!fp)
        fp = fopen("./yfork.log", "a");
    if (!fp)
        return;

    const auto now = std::chrono::system_clock::now();
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now.time_since_epoch())
                            .count() % 1000;
    const time_t raw = std::chrono::system_clock::to_time_t(now);
    struct tm localTime;
#if defined(_WIN32)
    localtime_s(&localTime, &raw);
#else
    localtime_r(&raw, &localTime);
#endif

    fprintf(fp, "[%02d:%02d:%02d.%03lld] #%06llu [%s] ",
            localTime.tm_hour, localTime.tm_min, localTime.tm_sec,
            static_cast<long long>(millis), ++yforkDiagLineCounter(),
            source ? source : "DIAG");
    vfprintf(fp, fmt, args);
    fprintf(fp, "\n");
    fflush(fp);
}

inline void yforkDiagLog(const char *source, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    yforkDiagVLog(source, fmt, args);
    va_end(args);
}
