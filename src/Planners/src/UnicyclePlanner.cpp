/**
 * @file UnicyclePlanner.h
 * @authors Diego Ferigo, Stefano Dafarra
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include "BipedalLocomotion/Planners/UnicyclePlanner.h"
#include "BipedalLocomotion/TextLogging/Logger.h"

#include <FootPrint.h>
#include <UnicyclePlanner.h>
#include <iDynTree/Core/VectorFixSize.h>

#include <cassert>
#include <limits>
#include <optional>

using namespace BipedalLocomotion;

class Planners::UnicyclePlanner::Impl
{
public:
    struct
    {
        double planner;
    } dt;

    struct
    {
        double t0;
        double tf;
    } horizon;

    struct
    {
        struct
        {
            double x = 0.10;
            double y = 0.00;
        } reference;

        struct
        {
            double unicycle = 10.0;
            double slowWhenTurning = 0.0;
        } gains;
    } controller;

    struct
    {
        double time = 1.0;
        double position = 1.0;
    } weights;

    struct
    {
        double min;
        double max;
        double nominal;
    } duration;

    struct
    {
        double min;
        double max;
    } stepLength;

    struct
    {
        double min;
        double nominal;
    } feetDistance;

    struct
    {
        double min;
        double max;
    } angleVariation;

    struct
    {
        double stancePhaseRatio;
        bool startWithLeft = false;
        bool terminalStep = true;
        bool resetStartingFootIfStill = false;
    } gait;

    UnicyclePlannerOutput outputRef;
    std::optional<UnicyclePlannerOutput> output = std::nullopt;

    std::shared_ptr<::FootPrint> left;
    std::shared_ptr<::FootPrint> right;
    std::unique_ptr<::UnicyclePlanner> planner;
};

Planners::UnicyclePlanner::UnicyclePlanner()
    : m_pImpl{std::make_unique<Impl>()}
{
}

Planners::UnicyclePlanner::~UnicyclePlanner() = default;

bool Planners::UnicyclePlanner::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[UnicyclePlanner::initialize]";

    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} The handler has to point to an already initialized IParametersHandler.",
                     logPrefix);
        return false;
    }

    bool okPlanner = true;
    m_pImpl->planner = std::make_unique<::UnicyclePlanner>();

    // ==
    // dt
    // ==

    if (!ptr->getParameter("sampling_time", m_pImpl->dt.planner))
    {
        log()->error("{} Unable to load the sampling time of the planner (sampling_time).",
                     logPrefix);
        return false;
    }

    okPlanner = okPlanner && m_pImpl->planner->setPlannerPeriod(m_pImpl->dt.planner);
    okPlanner = okPlanner && m_pImpl->planner->setMaximumIntegratorStepSize(m_pImpl->dt.planner);

    // ==========
    // controller
    // ==========

    if (!ptr->getParameter("unicycleGain", m_pImpl->controller.gains.unicycle))
    {
        log()->info("{} Using default unicycleGain={}.",
                    logPrefix,
                    m_pImpl->controller.gains.unicycle);
    }

    if (!ptr->getParameter("slowWhenTurningGain", m_pImpl->controller.gains.slowWhenTurning))
    {
        log()->info("{} Using default slowWhenTurningGain={}.",
                    logPrefix,
                    m_pImpl->controller.gains.slowWhenTurning);
    }

    okPlanner = okPlanner
                && m_pImpl->planner->setControllerGain(m_pImpl->controller.gains.unicycle);
    okPlanner = okPlanner
                && m_pImpl->planner->setSlowWhenTurnGain(m_pImpl->controller.gains.slowWhenTurning);

    std::vector<double> reference;

    if (!(ptr->getParameter("referencePosition", reference) && reference.size() == 2))
    {
        log()->info("{} Using default referencePosition=({}, {}).",
                    logPrefix,
                    m_pImpl->controller.reference.x,
                    m_pImpl->controller.reference.y);
    } else
    {
        m_pImpl->controller.reference.x = reference[0];
        m_pImpl->controller.reference.y = reference[1];
    }

    okPlanner = okPlanner
                && m_pImpl->planner->setDesiredPersonDistance(m_pImpl->controller.reference.x,
                                                              m_pImpl->controller.reference.y);

    // =======
    // weights
    // =======

    if (!ptr->getParameter("timeWeight", m_pImpl->weights.time))
    {
        log()->info("{} Using default timeWeight={}.", logPrefix, m_pImpl->weights.time);
    }

    if (!ptr->getParameter("positionWeight", m_pImpl->weights.position))
    {
        log()->info("{} Using default positionWeight={}.", logPrefix, m_pImpl->weights.position);
    }

    okPlanner
        = okPlanner
          && m_pImpl->planner->setCostWeights(m_pImpl->weights.position, m_pImpl->weights.time);

    // ========
    // duration
    // ========

    if (!ptr->getParameter("minStepDuration", m_pImpl->duration.min))
    {
        log()->error("{} Unable to load the min step duration (minStepDuration).", logPrefix);
        return false;
    }

    if (!ptr->getParameter("maxStepDuration", m_pImpl->duration.max))
    {
        log()->error("{} Unable to load the max step duration (maxStepDuration).", logPrefix);
        return false;
    }

    if (!ptr->getParameter("nominalDuration", m_pImpl->duration.nominal))
    {
        log()->error("{} Unable to load the nominal step duration (nominalDuration).", logPrefix);
        return false;
    }

    okPlanner = okPlanner
                && m_pImpl->planner->setStepTimings(m_pImpl->duration.min,
                                                    m_pImpl->duration.max,
                                                    m_pImpl->duration.nominal);

    // ==========
    // stepLength
    // ==========

    if (!ptr->getParameter("minStepLength", m_pImpl->stepLength.min))
    {
        log()->error("{} Unable to load the min step length (minStepLength).", logPrefix);
        return false;
    }

    if (!ptr->getParameter("maxStepLength", m_pImpl->stepLength.max))
    {
        log()->error("{} Unable to load the max step length (maxStepLength).", logPrefix);
        return false;
    }

    okPlanner = okPlanner && m_pImpl->planner->setMaxStepLength(m_pImpl->stepLength.max);
    okPlanner = okPlanner && m_pImpl->planner->setMinimumStepLength(m_pImpl->stepLength.min);

    // ============
    // feetDistance
    // ============

    if (!ptr->getParameter("minWidth", m_pImpl->feetDistance.min))
    {
        log()->error("{} Unable to load the min feet distance (minWidth).", logPrefix);
        return false;
    }

    if (!ptr->getParameter("nominalWidth", m_pImpl->feetDistance.nominal))
    {
        log()->error("{} Unable to load the nominal feet distance (nominalWidth).", logPrefix);
        return false;
    }

    okPlanner = okPlanner
                && m_pImpl->planner->setWidthSetting( //
                    m_pImpl->feetDistance.min,
                    m_pImpl->feetDistance.nominal);

    // ==============
    // angleVariation
    // ==============

    if (!ptr->getParameter("minAngleVariation", m_pImpl->angleVariation.min))
    {
        log()->error("{} Unable to load the min foot angle variation (minAngleVariation).",
                     logPrefix);
        return false;
    }

    if (!ptr->getParameter("maxAngleVariation", m_pImpl->angleVariation.max))
    {
        log()->error("{} Unable to load the max foot angle variation (maxAngleVariation).",
                     logPrefix);
        return false;
    }

    okPlanner = okPlanner && m_pImpl->planner->setMaxAngleVariation(m_pImpl->angleVariation.max);
    okPlanner = okPlanner
                && m_pImpl->planner->setMinimumAngleForNewSteps(m_pImpl->angleVariation.min);

    // ====
    // gait
    // ====

    if (!ptr->getParameter("switchOverSwingRatio", m_pImpl->gait.stancePhaseRatio))
    {
        log()->error("{} Unable to load the stance phase ratio (switchOverSwingRatio).", logPrefix);
        return false;
    }

    if (m_pImpl->gait.stancePhaseRatio <= 0)
    {
        log()->error("{} The switchOverSwingRatio cannot be <= 0.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("swingLeft", m_pImpl->gait.startWithLeft))
    {
        log()->info("{} Using default swingLeft={}.", logPrefix, m_pImpl->gait.startWithLeft);
    }

    if (!ptr->getParameter("terminalStep", m_pImpl->gait.terminalStep))
    {
        log()->info("{} Using default terminalStep={}.", logPrefix, m_pImpl->gait.terminalStep);
    }

    if (!ptr->getParameter("startAlwaysSameFoot", m_pImpl->gait.resetStartingFootIfStill))
    {
        log()->info("{} Using default startAlwaysSameFoot={}.",
                    logPrefix,
                    m_pImpl->gait.resetStartingFootIfStill);
    }

    m_pImpl->planner->startWithLeft(m_pImpl->gait.startWithLeft);
    m_pImpl->planner->addTerminalStep(m_pImpl->gait.terminalStep);
    m_pImpl->planner->resetStartingFootIfStill(m_pImpl->gait.resetStartingFootIfStill);

    // =============================
    // UnicyclePlanner configuration
    // =============================

    if (!okPlanner)
    {
        log()->error("{} Failed to configure UnicyclePlanner.", logPrefix);
        return false;
    }

    return true;
}

const Planners::UnicyclePlannerOutput& Planners::UnicyclePlanner::getOutput() const
{
    constexpr auto logPrefix = "[UnicyclePlanner::getOutput]";

    if (!this->isOutputValid())
    {
        log()->warn("{} Returning an empty output.", logPrefix);
        return m_pImpl->outputRef;
    }

    m_pImpl->outputRef = *m_pImpl->output;
    return m_pImpl->outputRef;
}

bool Planners::UnicyclePlanner::isOutputValid() const
{
    constexpr auto logPrefix = "[UnicyclePlanner::isOutputValid]";

    if (!m_pImpl->planner)
    {
        log()->error("{} The Unicycle planner has never been initialized.", logPrefix);
        return false;
    }

    if (!m_pImpl->left || !m_pImpl->right)
    {
        log()->error("{} The Unicycle planner never computed the foot steps.", logPrefix);
        return false;
    }

    if (!m_pImpl->output)
    {
        log()->error("{} The output has never been computed.", logPrefix);
        return false;
    }

    return true;
}

bool Planners::UnicyclePlanner::setInput(const UnicyclePlannerInput& input)
{
    constexpr auto logPrefix = "[UnicyclePlanner::setInput]";

    if (!m_pImpl->planner)
    {
        log()->error("{} The Unicycle planner has never been initialized.", logPrefix);
        return false;
    }

    auto getMaxKnotTime = [](const UnicyclePlannerInput& input) -> double {
        double maxKnotTime = 0.0;

        for (const auto& knot : input.knots)
        {
            if (knot.time > maxKnotTime)
                maxKnotTime = knot.time;
        }

        return maxKnotTime;
    };

    if (input.tf < getMaxKnotTime(input))
    {
        log()->error("{} The input contains a knot whose time is over the planner horizon.",
                     logPrefix);
        return false;
    }

    m_pImpl->planner->clearDesiredTrajectory();

    for (const auto& knot : input.knots)
    {
        auto position = iDynTree::Vector2();
        position[0] = knot.x;
        position[1] = knot.y;

        auto velocity = iDynTree::Vector2();
        velocity[0] = knot.dx;
        velocity[1] = knot.dy;

        if (!m_pImpl->planner->addDesiredTrajectoryPoint(knot.time, position, velocity))
        {
            m_pImpl->planner->clearDesiredTrajectory();
            log()->error("{} Failed to insert knot in the Unicycle planner.", logPrefix);
            return false;
        }
    }

    m_pImpl->horizon.t0 = input.t0;
    m_pImpl->horizon.tf = input.tf;

    return true;
}

bool Planners::UnicyclePlanner::advance()
{
    constexpr auto logPrefix = "[UnicyclePlanner::advance]";

    if (!m_pImpl->planner)
    {
        log()->error("{} The Unicycle planner has never been initialized.", logPrefix);
        return false;
    }

    m_pImpl->output = std::nullopt;

    m_pImpl->left = std::make_shared<FootPrint>();
    m_pImpl->left->setFootName("left");

    m_pImpl->right = std::make_shared<FootPrint>();
    m_pImpl->right->setFootName("right");

    if (!m_pImpl->planner->computeNewSteps(m_pImpl->left,
                                           m_pImpl->right,
                                           m_pImpl->horizon.t0,
                                           m_pImpl->horizon.tf))
    {
        m_pImpl->left = nullptr;
        m_pImpl->right = nullptr;
        log()->error("{} Failed to compute new steps.", logPrefix);
        return false;
    }

    auto convertStep
        = [&](const ::Step& step, const ::Step& nextStep) -> const Contacts::PlannedContact {
        Contacts::PlannedContact contact;
        contact.name = step.footName;
        contact.activationTime = step.impactTime;

        contact.pose.quat(Eigen::AngleAxisd(step.angle, Eigen::Vector3d::UnitZ()));
        contact.pose.translation({step.position[0], step.position[1], 0.0});

        // Note: we use the impact time to compute the duration of the step.
        //       If the step is the last one, the deactivation is the farther representable time.
        const double duration = nextStep.impactTime - step.impactTime;
        assert(duration >= 0);
        // This is the ratio [0, 1] between the stance and swing phases of an individual step.
        // Small stepRatio means short stance and long swing phases.
        const auto stepRatio
            = m_pImpl->gait.stancePhaseRatio / (1 + m_pImpl->gait.stancePhaseRatio);
        contact.deactivationTime = duration > 0.0 ? contact.activationTime + duration * stepRatio
                                                  : std::numeric_limits<double>::max();

        return contact;
    };

    auto convertSteps
        = [&convertStep](decltype(
              ::FootPrint().getSteps())& steps) -> std::optional<const Contacts::ContactList> {
        Contacts::ContactList list;

        for (std::size_t i = 0; i < steps.size(); ++i)
        {
            // Last step
            if (i == steps.size() - 1)
            {
                if (!list.addContact(convertStep(steps[i], steps[i])))
                {
                    return std::nullopt;
                }
                break;
            }

            // All steps but last
            if (!list.addContact(convertStep(steps[i], steps[i + 1])))
            {
                return std::nullopt;
            }
        }

        return list;
    };

    const auto& leftContactList = convertSteps(m_pImpl->left->getSteps());
    const auto& rightContactList = convertSteps(m_pImpl->right->getSteps());

    if (!leftContactList || !rightContactList)
    {
        log()->error("{} Failed to convert unicycle steps.", logPrefix);
        return false;
    }

    m_pImpl->output = std::make_optional<UnicyclePlannerOutput>();
    m_pImpl->output->left.setDefaultName("left");
    m_pImpl->output->right.setDefaultName("right");

    m_pImpl->output->left = *leftContactList;
    m_pImpl->output->right = *rightContactList;

    return true;
}
