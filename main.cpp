#include <million/monkeys.hpp>
#include <PxPhysicsAPI.h>
#include <components/core.hpp>
#include <entt/entity/view.hpp>

#include "loaders/physics_shape.hpp"

#include <magic_enum.hpp>

namespace filter {
    enum class Flags : physx::PxU32 {
        Enabled = 0b0001,
    };

    physx::PxU32& id (physx::PxFilterData& fd) {return fd.word0;}
    physx::PxU32& mask (physx::PxFilterData& fd) {return fd.word1;}
    physx::PxU32 flags (physx::PxFilterData& fd) {return fd.word2;}
    bool is_enabled (physx::PxFilterData& fd) {return flags(fd) & magic_enum::enum_integer(Flags::Enabled);}

    template <Flags... FilterFlags> void set_flags (physx::PxFilterData& fd) {
        fd.word2 = (fd.word2 & 0xfffffff0) | (magic_enum::enum_integer(FilterFlags) | ...);
    }

    void setup (physx::PxRigidActor* actor, physx::PxU32 filter_id, physx::PxU32 filter_mask)
    {
        physx::PxFilterData filter_data;
        id(filter_data) = filter_id; // set own ID
        mask(filter_data) = filter_mask;  // set ID mask to filter pairs that trigger a contact callback

        // Always enabled
        set_flags<filter::Flags::Enabled>(filter_data);

        const physx::PxU32 num_shapes = actor->getNbShapes();
        std::vector<physx::PxShape*> shapes(num_shapes);
        actor->getShapes(shapes.data(), num_shapes);
        for (auto shape : shapes) {
            shape->setSimulationFilterData(filter_data);
        }
    }
    void setup (physx::PxShape* shape, physx::PxU32 filter_id, physx::PxU32 filter_mask)
    {
        physx::PxFilterData filter_data;
        id(filter_data) = filter_id; // set own ID
        mask(filter_data) = filter_mask;  // set ID mask to filter pairs that trigger a contact callback

        // Always enabled
        set_flags<filter::Flags::Enabled>(filter_data);

        // Add filter data to shape
        shape->setSimulationFilterData(filter_data);
    }
}

physx::PxFilterFlags contactReportFilterShader(
    physx::PxFilterObjectAttributes attributes_0, physx::PxFilterData filter_data_0,
	physx::PxFilterObjectAttributes attributes_1, physx::PxFilterData filter_data_1,
	physx::PxPairFlags& pair_flags,
    const void* constant_block, physx::PxU32 constant_block_size)
{
    PX_UNUSED(constant_block);
    PX_UNUSED(constant_block_size);

    // let triggers through
    if(physx::PxFilterObjectIsTrigger(attributes_0) || physx::PxFilterObjectIsTrigger(attributes_1)) {
        pair_flags = physx::PxPairFlag::eTRIGGER_DEFAULT;
        return physx::PxFilterFlag::eDEFAULT;
    }

    // Disabled bodies don't collide
    if (!filter::is_enabled(filter_data_0) || !filter::is_enabled(filter_data_1)) {
        return physx::PxFilterFlag::eKILL;
    }
    // generate contacts for all that were not filtered above
    pair_flags = physx::PxPairFlag::eCONTACT_DEFAULT;

    // trigger the contact callback for pairs (A,B) where
    // the filtermask of A contains the ID of B and vice versa.
    if(filter::is_enabled(filter_data_0) &&
        (filter::mask(filter_data_0) & filter::id(filter_data_1)) && (filter::mask(filter_data_1) & filter::id(filter_data_0))) {
        pair_flags |= physx::PxPairFlag::eNOTIFY_TOUCH_FOUND;
    }

    return physx::PxFilterFlag::eDEFAULT;

	// pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT
	// 		  | physx::PxPairFlag::eDETECT_CCD_CONTACT
	// 		  | physx::PxPairFlag::eNOTIFY_TOUCH_CCD
	// 		  |	physx::PxPairFlag::eNOTIFY_TOUCH_FOUND
	// 		  | physx::PxPairFlag::eNOTIFY_CONTACT_POINTS
	// 		  | physx::PxPairFlag::eCONTACT_EVENT_POSE;
	// return physx::PxFilterFlag::eDEFAULT;
}


struct EntityInfo {
    entt::entity entity;
};


class MM_MODULE_NAME(PhysX), public physx::PxSimulationEventCallback {
    MM_MODULE_CLASS(PhysX)

    using Static = million::WrapPtr<physx::PxRigidStatic>;
    using Dynamic = million::WrapPtr<physx::PxRigidDynamic>;
public:

    void advancedModuleSetup (million::api::internal::ModuleManager* mm) override
    {
        // Components that aren't defined in TOML (ie components meant for internal use by the module) must be registered with the engine
        mm->registerInternalComponents<Static, Dynamic>();
    }
 
    bool onLoad (million::Setup* api)
    {
        debug("Loading");
        m_foundation = PxCreateFoundation(PX_PHYSICS_VERSION, m_default_allocator_callback, m_default_error_callback);
        if (!m_foundation) {
            return false;
        }
// #ifdef DEBUG_BUILD
//         m_pvd = PxCreatePvd(*m_foundation);
//         physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
//         m_pvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);
// #endif
        physx::PxTolerancesScale tolerance_scale;
        tolerance_scale.length = 1;        // typical length of an object
        tolerance_scale.speed = 981;       // typical speed of an object, gravity*1s is a reasonable choice
        m_physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_foundation, tolerance_scale, true, m_pvd);

        m_shape_loader = new PhysicsShapeLoader();
        api->registerResourceLoader(m_shape_loader);
        // static_friction, dynamic_friction, restitution 

        return true;
    }

    void onGameSetup (million::Setup*)
    {
        debug("Setting up scene");
        physx::PxSceneDesc scene_desc (m_physics->getTolerancesScale());
        scene_desc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f); // TODO: Read from game config
        m_dispatcher = physx::PxDefaultCpuDispatcherCreate(2); // TODO: Read from engine config
        scene_desc.cpuDispatcher = m_dispatcher;
        scene_desc.broadPhaseType = physx::PxBroadPhaseType::eABP;
        scene_desc.filterShader	= contactReportFilterShader; //physx::PxDefaultSimulationFilterShader;
        scene_desc.simulationEventCallback = this;
        m_scene = m_physics->createScene(scene_desc);
        m_material = m_physics->createMaterial(0.5f, 0.5f, 0.5f);

        float half_extent = 0.2f;
        m_shape = m_physics->createShape(physx::PxBoxGeometry(half_extent, half_extent, half_extent), *m_material);
        filter::setup(m_shape, 1, 1);
// #ifdef DEBUG_BUILD
//         physx::PxPvdSceneClient* pvd_client = context->m_scene->getScenePvdClient();
//         if(pvd_client) {
//             pvd_client->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
//             pvd_client->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
//             pvd_client->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
//         }
// #endif
    }
    // physx::PxDefaultSimulationFilterShader::PxFilterData::PxFlags::PxPairFlag


    void onUnload ()
    {
        debug("Unloading");
        if (m_shape_loader) {
            delete m_shape_loader;
        }
        if (m_scene) {
            m_scene->release();
        }
        if (m_dispatcher) {
            m_dispatcher->release();
        }
        m_physics->release();
        m_foundation->release();
    }

    void onPhysicsStep (timing::Delta time_step)
    {
        m_scene->simulate(time_step);
        m_scene->fetchResults(true);
    }

    void onLoadScene (million::Setup* api, entt::hashed_string::hash_type scene_id, const std::string& scene_name)
    {
        {
            auto& organizer = api->organizer(million::SystemStage::GameLogic);
            // organizer.emplace<&PhysX::read_physics, components::core::physics::StaticBody, const components::core::physics::DynamicBody, const components::core::Position>(*this, "Read Physics");
            organizer.emplace<&PhysX::create_new_static_bodies, Static>(*this, "Create Static Bodies");
            organizer.emplace<&PhysX::create_new_dynamic_bodies, Dynamic>(*this, "Create Dynamic Bodies");
            organizer.emplace<&PhysX::read_physics_objects, const components::core::physics::StaticBody, const components::core::physics::DynamicBody, const components::core::Position, Static, Dynamic>(*this, "Read Physics Bodies");
        }
        {
            auto& organizer = api->organizer(million::SystemStage::Update);
            organizer.emplace<&PhysX::write_dynamic_objects>(*this, "Write Dynamic Bodies");
        }
    }

    void create_new_static_bodies (entt::registry& registry, entt::view<entt::get_t<const components::core::physics::StaticBody, const components::core::Position>, entt::exclude_t<Static>> view)
    {
        for (const auto&& [entity, body, position] : view.each()) {
            physx::PxTransform tx(physx::PxVec3(position.x, position.y, position.z));
            physx::PxRigidStatic* px_body = m_physics->createRigidStatic(tx);
            px_body->attachShape(*m_shape);
            EntityInfo* entity_info = new EntityInfo;
            entity_info->entity = entity;
            px_body->userData = reinterpret_cast<void*>(entity_info);
            m_scene->addActor(*px_body); 
            registry.emplace<Static>(entity, px_body);
        }
    }

    void create_new_dynamic_bodies (entt::registry& registry, entt::view<entt::get_t<const components::core::physics::DynamicBody, const components::core::Position>, entt::exclude_t<Dynamic>> view)
    {
        for (const auto&& [entity, body, position] : view.each()) {
            physx::PxTransform tx(physx::PxVec3(position.x, position.y, position.z));
            physx::PxRigidDynamic* px_body = m_physics->createRigidDynamic(tx);
            px_body->attachShape(*m_shape);
            EntityInfo* entity_info = new EntityInfo;
            entity_info->entity = entity;
            px_body->userData = reinterpret_cast<void*>(entity_info);
            physx::PxRigidBodyExt::updateMassAndInertia(*px_body, body.mass);
            m_scene->addActor(*px_body); 
            registry.emplace<Dynamic>(entity, px_body);
        }
    }

    void read_physics_objects (const entt::registry& registry)
    {
        // Static bodies
        for (const auto& [entity, position, instance] : registry.view<const components::core::Position, Static>().each()) {
            instance.ptr->setGlobalPose(physx::PxTransform{
                physx::PxVec3(position.x, position.y, position.z)
            });
        }
        // Dynamic bodies
        for (const auto& [entity, body, position, instance] : registry.view<const components::core::physics::DynamicBody, const components::core::Position, Dynamic>().each()) {
            instance.ptr->setGlobalPose(physx::PxTransform{
                physx::PxVec3(position.x, position.y, position.z)
            });
            instance.ptr->setMass(body.mass);
        }
    }

    void write_dynamic_objects (entt::view<entt::get_t<const Dynamic, components::core::Position>> view)
    {
        view.each([this](const auto entity, const auto& instance, auto& position){
            auto p = instance.ptr->getGlobalPose().p;
            position.x = p.x;
            position.y = p.y;
            position.z = p.z;
        });
    }

	void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count) override
    {

    }

	void onWake(physx::PxActor** actors, physx::PxU32 count) override
    {
        info("On Wake!");
    }

	void onSleep(physx::PxActor** actors, physx::PxU32 count) override
    {
        info("On Sleep!");
    }

	void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count) override
    {
        info("On Trigger!");
    }

	void onAdvance(const physx::PxRigidBody*const*, const physx::PxTransform*, const physx::PxU32) override
    {
        info("On Advance!");
    }

    void onContact(const physx::PxContactPairHeader& header, const physx::PxContactPair* pairs, physx::PxU32 num_pairs) override
    {
        info("Contact!");
        m_contacts.emplace_back(reinterpret_cast<EntityInfo*>(header.actors[0]->userData));
        m_contacts.emplace_back(reinterpret_cast<EntityInfo*>(header.actors[1]->userData));
    }

    void onAfterFrame (million::Runtime* api)
    {
        for (auto& contact : m_contacts) {
            info("Contacting: {}", api->findEntityName(contact->entity));
        }
        m_contacts.clear();
    }

private:
    physx::PxDefaultAllocator m_default_allocator_callback;
    physx::PxDefaultErrorCallback  m_default_error_callback;
    physx::PxDefaultCpuDispatcher* m_dispatcher = nullptr;
    physx::PxFoundation* m_foundation = nullptr;
    physx::PxPhysics* m_physics = nullptr;
    physx::PxScene* m_scene = nullptr;
    physx::PxMaterial* m_material = nullptr;
    physx::PxPvd* m_pvd = nullptr;

    physx::PxShape* m_shape = nullptr;

    PhysicsShapeLoader* m_shape_loader = nullptr;

    std::vector<EntityInfo*> m_contacts;
};

MM_MODULE(PhysX)

