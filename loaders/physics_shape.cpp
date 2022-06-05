#include "physics_shape.hpp"

#include <PxPhysicsAPI.h>


bool PhysicsShapeLoader::cached (const std::string& filename, std::uint32_t*)
{
    return false;
}

bool PhysicsShapeLoader::load (million::resources::Handle handle, const std::string& filename)
{
    return true;
}

void PhysicsShapeLoader::unload (million::resources::Handle handle)
{

}
