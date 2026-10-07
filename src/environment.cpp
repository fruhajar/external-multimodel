#include "ballistics/mm/environment.h"

namespace Ballistics::MM {

Environment Environment::standard() {
    Environment e;
    e.atmosphere = &Isa::standard();
    e.wind = &WindField::calm();
    return e;
}

} // namespace Ballistics::MM
