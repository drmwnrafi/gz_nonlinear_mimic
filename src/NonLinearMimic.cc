#include <gz/sim/System.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/plugin/Register.hh>
#include <sstream>

#include <regex>
#include <vector>
#include <algorithm>

#include "exprtk.hpp"

using namespace gz;
using namespace sim;
using namespace systems;

class NonLinearMimic :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
    using SymbolTable = exprtk::symbol_table<double>;
    using Expression  = exprtk::expression<double>;
    using Parser      = exprtk::parser<double>;

public:
    NonLinearMimic() = default;

    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &) override
    {
        if (!_ecm.EntityHasComponentType(_entity, components::Model::typeId)) {
            gzerr << "[NonLinearMimic] Plugin must be inside <model>!" << std::endl;
            return;
        }

        if (_sdf->HasElement("debug"))
            this->debug = _sdf->Get<bool>("debug");

        // -------- follower joint --------
        std::string followerName = _sdf->Get<std::string>("joint");

        this->followerEntity = _ecm.EntityByComponents(
            components::ParentEntity(_entity),
            components::Name(followerName),
            components::Joint());

        if (this->followerEntity == kNullEntity) {
            gzerr << "[NonLinearMimic] Follower joint not found: " << followerName << std::endl;
            return;
        }

        gzmsg << "[NonLinearMimic] Found follower joint: " << followerName << std::endl;

        // -------- gains --------
        if (_sdf->HasElement("kp")) this->kp = std::max(0.0, _sdf->Get<double>("kp"));
        if (_sdf->HasElement("ki")) this->ki = std::max(0.0, _sdf->Get<double>("ki"));  
        if (_sdf->HasElement("kd")) this->kd = std::max(0.0, _sdf->Get<double>("kd"));
        if (_sdf->HasElement("max_torque")) this->maxTorque = std::max(0.0, _sdf->Get<double>("max_torque"));

        gzmsg << "[NonLinearMimic] Gains: kp=" << kp
              << ", ki=" << ki
              << ", kd=" << kd
              << ", maxTorque=" << maxTorque << std::endl;

        // -------- mimic joints --------
        int n = _sdf->Get<int>("n_mimic");

        this->jointValues.resize(n);
        this->jointEntities.resize(n);

        for (int i = 0; i < n; ++i) {
            std::string key = "mimicJoint" + std::to_string(i + 1);
            std::string jointName = _sdf->Get<std::string>(key);

            Entity ent = _ecm.EntityByComponents(
                components::ParentEntity(_entity),
                components::Name(jointName),
                components::Joint());

            if (ent == kNullEntity) {
                gzerr << "[NonLinearMimic] Mimic joint not found: " << jointName << std::endl;
                continue;
            }

            this->jointEntities[i] = ent;
            this->jointValues[i] = 0.0;

            this->symbolTable.add_variable(key, this->jointValues[i]);

            gzmsg << "[NonLinearMimic] Added mimic: " << jointName
                  << " -> " << key << std::endl;
        }

        // -------- expression --------
        std::string rawEq = _sdf->Get<std::string>("eq");

        std::string processedEq = std::regex_replace(rawEq, std::regex("\\{|\\}"), "");
        processedEq = std::regex_replace(processedEq, std::regex("math::"), "");

        this->symbolTable.add_constants();
        this->expression.register_symbol_table(this->symbolTable);

        if (!this->parser.compile(processedEq, this->expression)) {
            gzerr << "[NonLinearMimic] Failed to parse: " << processedEq << std::endl;
        } else {
            gzmsg << "[NonLinearMimic] Compiled: " << processedEq << std::endl;
        }
    }

    void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
    {
        if (_info.paused) return;
        if (this->followerEntity == kNullEntity) return;

        for (size_t i = 0; i < jointEntities.size(); ++i) {
            auto comp = _ecm.Component<components::JointPosition>(jointEntities[i]);
            if (comp && !comp->Data().empty()) {
                jointValues[i] = comp->Data()[0];
            }
        }

        double target = this->expression.value();

        if (std::isnan(target) || std::isinf(target)) {
            gzerr << "[NonLinearMimic] NaN/Inf detected!" << std::endl;
            return;
        }

        double pos = 0.0;
        double vel = 0.0;

        auto posComp = _ecm.Component<components::JointPosition>(this->followerEntity);
        if (posComp && !posComp->Data().empty())
            pos = posComp->Data()[0];

        auto velComp = _ecm.Component<components::JointVelocity>(this->followerEntity);
        if (velComp && !velComp->Data().empty())
            vel = velComp->Data()[0];

        double error = target - pos;

        this->integral += error * _info.dt.count();  
        this->integral = std::clamp(this->integral, -10.0, 10.0); // anti-windup

        double torque =
            kp * error +
            ki * this->integral -
            kd * vel;

        torque = std::clamp(torque, -maxTorque, maxTorque);

        if (!_ecm.EntityHasComponentType(this->followerEntity,
                                         components::JointForceCmd::typeId)) {
            _ecm.CreateComponent(this->followerEntity,
                                 components::JointForceCmd({0.0}));
        }

        _ecm.SetComponentData<components::JointForceCmd>(
            this->followerEntity, {torque});

        static int step = 0;
        if (debug && (++step % 100 == 0)) {
            std::ostringstream oss;

            oss << "[NonLinearMimic] step=" << step << " |" ;

            for (size_t i = 0; i < jointValues.size(); ++i)
                oss << " j" << (i+1) << "=" << jointValues[i];

            oss << " | target=" << target
                << " | pos=" << pos
                << " | err=" << error
                << " | torque=" << torque;

            gzdbg << oss.str() << std::endl;
        }
    }

private:
    Entity followerEntity{kNullEntity};

    std::vector<Entity> jointEntities;
    std::vector<double> jointValues;

    double kp{50.0};
    double ki{0.0};   
    double kd{5.0};
    double maxTorque{100.0};

    double integral{0.0};  

    bool debug{false};    

    SymbolTable symbolTable;
    Expression expression;
    Parser parser;
};

GZ_ADD_PLUGIN(
    NonLinearMimic,
    gz::sim::System,
    NonLinearMimic::ISystemConfigure,
    NonLinearMimic::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(NonLinearMimic, "gz::sim::NonLinearMimic")