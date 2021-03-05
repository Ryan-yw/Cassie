#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

namespace robot
{

using Size = std::size_t;
constexpr double PI = 3.141592653589793;

class MoveJS : public aris::core::CloneObject<MoveJS, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit MoveJS(const std::string &name = "MoveJS_plan");

};

class MoveJoint : public aris::core::CloneObject<MoveJoint, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit MoveJoint(const std::string &name = "MoveJiont");
private:
    double dir_;
};


class DogForward :public aris::core::CloneObject<DogForward, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;

    virtual ~DogForward();
    explicit DogForward(const std::string& name = "dog_forward");
private:
    double step_;
};


    auto createControllerCassie()->std::unique_ptr<aris::control::Controller>;
    auto createPlanCassie()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
