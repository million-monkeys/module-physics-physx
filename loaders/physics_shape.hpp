#pragma once

#include <million/engine.hpp>

class PhysicsShapeLoader : public million::api::resources::Loader {
public:
    virtual ~PhysicsShapeLoader() {}
    bool cached (const std::string& filename, std::uint32_t*) final;

    bool load (million::resources::Handle handle, const std::string& filename) final;
    void unload (million::resources::Handle handle) final;

    entt::hashed_string name () const { return "physics-shape"_hs; }
};
