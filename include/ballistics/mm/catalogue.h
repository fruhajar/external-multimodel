#ifndef BALLISTICS_MM_CATALOGUE_H
#define BALLISTICS_MM_CATALOGUE_H

#include "projectile.h"

#include <map>
#include <string>
#include <vector>

namespace Ballistics::MM {

class Catalogue {
public:
    static Catalogue withBuiltins();

    const Projectile* find(const std::string& id) const;
    std::vector<std::string> ids() const;

    void add(Projectile p);
    bool remove(const std::string& id);
    std::size_t size() const { return m_byId.size(); }

private:
    std::map<std::string, Projectile> m_byId;
};

} // namespace Ballistics::MM

#endif
