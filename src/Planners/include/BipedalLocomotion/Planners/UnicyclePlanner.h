/**
 * @file UnicyclePlanner.h
 * @authors Diego Ferigo, Stefano Dafarra
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H
#define BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H

#include "BipedalLocomotion/Contacts/ContactList.h"
#include "BipedalLocomotion/ParametersHandler/IParametersHandler.h"
#include "BipedalLocomotion/System/Advanceable.h"

#include <Eigen/Core>

#include <memory>

namespace BipedalLocomotion::Planners
{
struct UnicycleKnot;
class UnicyclePlanner;
struct UnicyclePlannerInput;
struct UnicyclePlannerOutput;
} // namespace BipedalLocomotion::Planners

struct BipedalLocomotion::Planners::UnicycleKnot
{
    UnicycleKnot(const double x = 0.0,
                 const double y = 0.0,
                 const double dx = 0.0,
                 const double dy = 0.0,
                 const double t = 0.0)
        : x(x)
        , y(y)
        , dx(dx)
        , dy(dy)
        , time(t)
    {
    }

    UnicycleKnot(const Eigen::Vector2d& position = {0, 0},
                 const Eigen::Vector2d& velocity = {0, 0},
                 const double time = 0.0)
        : x(position[0])
        , y(position[1])
        , dx(velocity[0])
        , dy(velocity[1])
        , time(time)
    {
    }

    bool operator==(const UnicycleKnot& rhs)
    {
        return this->x == rhs.x && this->y == rhs.y && this->dx == rhs.dx && this->dy == rhs.dy
               && this->time == rhs.time;
    }

    double x;
    double y;

    double dx = 0.0;
    double dy = 0.0;

    double time = 0.0;
};

struct BipedalLocomotion::Planners::UnicyclePlannerInput
{
    UnicyclePlannerInput(const std::vector<UnicycleKnot>& knots,
                         const double tf = 0.0,
                         const double t0 = 0.0)
        : t0(t0)
        , tf(tf)
        , knots(knots)
    {
    }

    double t0;
    double tf;

    std::vector<UnicycleKnot> knots;
};

struct BipedalLocomotion::Planners::UnicyclePlannerOutput
{
    UnicyclePlannerOutput(const Contacts::ContactList& left = {},
                          const Contacts::ContactList& right = {})
        : left(left)
        , right(right)
    {
    }

    Contacts::ContactList left;
    Contacts::ContactList right;
};

class BipedalLocomotion::Planners::UnicyclePlanner final
    : public System::Advanceable<UnicyclePlannerInput, UnicyclePlannerOutput>
{
public:
    UnicyclePlanner();

    virtual ~UnicyclePlanner();

    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

    const UnicyclePlannerOutput& getOutput() const override;

    bool isOutputValid() const override;

    bool setInput(const UnicyclePlannerInput& input) override;

    bool advance() override;

private:
    class Impl;
    std::unique_ptr<Impl> m_pImpl;
};

#endif // BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H
