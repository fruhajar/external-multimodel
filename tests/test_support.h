#ifndef BALLISTICS_PM_TEST_SUPPORT_H
#define BALLISTICS_PM_TEST_SUPPORT_H

#include <cmath>
#include <cstdio>
#include <string>

namespace test {

inline int failures = 0;
inline int checks = 0;

inline void report(bool ok, const char* expr, const char* file, int line,
                   const std::string& detail) {
    ++checks;
    if (ok) {
        return;
    }
    ++failures;
    std::printf("FAIL %s:%d  %s%s%s\n", file, line, expr,
                detail.empty() ? "" : "\n     ", detail.c_str());
}

inline std::string values(double got, double want, double tol) {
    char buf[160];
    std::snprintf(buf, sizeof buf, "got %.9g want %.9g (tolerance %.3g, off by %.3g)",
                  got, want, tol, std::abs(got - want));
    return buf;
}

inline int summary(const char* name) {
    std::printf("%s: %d checks, %d failures\n", name, checks, failures);
    return failures == 0 ? 0 : 1;
}

} // namespace test

#define CHECK(expr) ::test::report((expr), #expr, __FILE__, __LINE__, "")

#define CHECK_MSG(expr, detail) ::test::report((expr), #expr, __FILE__, __LINE__, (detail))

#define CHECK_NEAR(got, want, tol)                                                      \
    ::test::report(std::abs((got) - (want)) <= (tol), #got " ~= " #want, __FILE__,      \
                   __LINE__, ::test::values((got), (want), (tol)))

#define CHECK_REL(got, want, frac)                                                      \
    ::test::report(std::abs((got) - (want)) <= std::abs(want) * (frac),                 \
                   #got " ~= " #want, __FILE__, __LINE__,                               \
                   ::test::values((got), (want), std::abs(want) * (frac)))

#endif
