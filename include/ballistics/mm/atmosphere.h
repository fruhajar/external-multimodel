#ifndef BALLISTICS_MM_ATMOSPHERE_H
#define BALLISTICS_MM_ATMOSPHERE_H

namespace Ballistics::MM {

class Atmosphere {
public:
    virtual ~Atmosphere() = default;

    virtual double density(double altitudeMsl) const = 0;
    virtual double speedOfSound(double altitudeMsl) const = 0;
    virtual double temperature(double altitudeMsl) const = 0;
};

// ISA below 25 km, dry air.
class Isa final : public Atmosphere {
public:
    double density(double altitudeMsl) const override;
    double speedOfSound(double altitudeMsl) const override;
    double temperature(double altitudeMsl) const override;

    static const Isa& standard();
};

// ISA shifted to a station reading, with humidity. Moist air is less dense than dry air at the
// same pressure and temperature, which is the second-order met correction after temperature.
class IsaStation final : public Atmosphere {
public:
    IsaStation(double temperatureK, double pressurePa, double stationAltitude,
               double relativeHumidity = 0.0);

    double density(double altitudeMsl) const override;
    double speedOfSound(double altitudeMsl) const override;
    double temperature(double altitudeMsl) const override;

    double seaLevelTemperature() const { return m_seaLevelTemperature; }
    double seaLevelPressure() const { return m_seaLevelPressure; }

private:
    double m_seaLevelTemperature;
    double m_seaLevelPressure;
    double m_vapourPressure;
};

} // namespace Ballistics::MM

#endif
