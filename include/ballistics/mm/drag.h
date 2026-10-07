#ifndef BALLISTICS_MM_DRAG_H
#define BALLISTICS_MM_DRAG_H

#include <memory>
#include <vector>

namespace Ballistics::MM {

class DragModel {
public:
    virtual ~DragModel() = default;

    // Zero-yaw drag coefficient.
    virtual double at(double mach) const = 0;

    // Yaw drag: Cd grows with the square of the angle of attack. Models that track attitude use
    // this; the default ignores alpha, which is all a point mass can do.
    virtual double at(double mach, double /*alpha*/) const { return at(mach); }
};

enum class DragFamily { G1, G7, Shell };

// Cd of each reference shape at rest, so a round's form factor is its own Cd over this.
double referenceCd(DragFamily family);

// A standard family scaled by a form factor. Cubic through the interior, linear at the ends.
class StandardDrag final : public DragModel {
public:
    StandardDrag(DragFamily family, double formFactor);

    double at(double mach) const override;
    double at(double mach, double alpha) const override;

    DragFamily family() const { return m_family; }
    double formFactor() const { return m_formFactor; }

    // Quadratic yaw-drag coefficient, Cd = Cd0 + Cd_a2 * alpha^2.
    void setYawDrag(double cdAlphaSquared) { m_cdAlphaSquared = cdAlphaSquared; }

    static std::shared_ptr<const DragModel> forCd(DragFamily family, double cd);

private:
    DragFamily m_family;
    double m_formFactor;
    double m_cdAlphaSquared = 0.0;
};

// A measured Mach/Cd table.
class CustomDrag final : public DragModel {
public:
    CustomDrag(std::vector<double> mach, std::vector<double> cd, double formFactor = 1.0);

    double at(double mach) const override;
    double at(double mach, double alpha) const override;

    void setYawDrag(double cdAlphaSquared) { m_cdAlphaSquared = cdAlphaSquared; }
    bool empty() const { return m_mach.size() < 2; }

private:
    std::vector<double> m_mach, m_cd;
    double m_formFactor;
    double m_cdAlphaSquared = 0.0;
};

} // namespace Ballistics::MM

#endif
