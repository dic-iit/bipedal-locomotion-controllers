/**
 * @file MomentumBasedTorqueControl.tpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 * @date 2020
 */

#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotionControllers/WholeBodyControllers/MomentumBasedTorqueControlWithCompliantContacts.h>
#include <BipedalLocomotionControllers/ContactModels/ContinuousContactModel.h>


namespace BipedalLocomotionControllers
{
namespace WholeBodyControllers
{

template <class T>
bool MomentumBasedTorqueControl::addFeetTypeIdentifiers(ParametersHandler::IParametersHandler<T>* handler, const FootType& type)
{
    bool isSwing = type == FootType::Swing;
    const std::string feetType = isSwing ? "swing" : "stance";
    auto& feetIdentifiers = isSwing ? m_swingFeetIdetrifiers : m_stanceFeetIdetrifiers;

    std::vector<std::string> feetVariablesName;
    if (!handler->getParameter(feetType + "_feet_name", feetVariablesName))
    {
        std::cerr << "[MomentumBasedTorqueControl::addFeetTypeIdentifiers] Unable to find the "
                  << feetType << "feet names" << std::endl;
        return false;
    }

    std::vector<std::string> feetFramesName;
    if (!handler->getParameter(feetType + "_feet_frame", feetFramesName))
    {
        std::cerr << "[MomentumBasedTorqueControl::addFeetTypeIdentifiers] Unable to find the "
                  << feetType << "feet names" << std::endl;
        return false;
    }

    if (feetFramesName.size() != feetVariablesName.size())
    {
        std::cerr << "[MomentumBasedTorqueControl::addFeetTypeIdentifiers] The number of identifiers "
                     "is different. For the "
                  << feetType << " feet" << std::endl;
        return false;
    }

    // add the identifiers
    for (std::size_t i = 0; i < feetFramesName.size(); i++)
        feetIdentifiers.emplace_back(feetVariablesName[i], feetFramesName[i]);

    return true;
}

template <class T>
bool MomentumBasedTorqueControl::addFeetIdentifiers(std::unique_ptr<ParametersHandler::IParametersHandler<T>> handler)
{
    if(handler->isEmpty())
    {
        std::cerr << "[MomentumBasedTorqueControl::addFeetIdentifiers] The handler is empty. "
                     "Unable to retrieve the parameters related to stance and swing feet"
                  << std::endl;
        return false;
    }

    if (!addFeetTypeIdentifiers(handler.get(), FootType::Swing))
    {
        std::cerr << "[MomentumBasedTorqueControl::addFeetIdentifiers] Unable to add the Swing feet "
                     "identifiers"
                  << std::endl;
        return false;
    }

    if (!addFeetTypeIdentifiers(handler.get(), FootType::Stance))
    {
        std::cerr << "[MomentumBasedTorqueControl::addFeetIdentifiers] Unable to add the Stance "
                     "feet identifiers"
                  << std::endl;
        return false;
    }

    return true;
}

template <class T>
bool MomentumBasedTorqueControl::addLinearMomentumElement(
    std::unique_ptr<ParametersHandler::IParametersHandler<T>> handler)
{
    using namespace OptimalControlUtilities;

    // the frame in contact are two (left and right foot)
    std::vector<FrameInContact<std::string, std::string>> framesInContact;

    bool isCompliantContact = true;
    for (const auto& stanceFoot : m_stanceFeetIdetrifiers)
        framesInContact.emplace_back(stanceFoot.identifierInVariableHandler(),
                                     stanceFoot.identifierInModel(),
                                     isCompliantContact);

    // gains and weights
    // Gain
    iDynTree::Vector3 kp, kd, ki;
    if (!handler->getParameter("kp", kp) || !handler->getParameter("kd", kd) || !handler->getParameter("ki", ki))
    {
        std::cerr << "[MomentumBasedTorqueControl::addCentroidalLinearMomentumElement] Unable to "
                     "get the gains.";
        return false;
    }

    // create the pd controller
    PIDController<iDynTree::Vector3> pidController(kd, kp, ki);

    m_centroidalLinearMomentumElement
        = std::make_unique<CentroidalLinearMomentumRateOfChangeElement>(m_kinDyn,
                                                                        pidController,
                                                                        m_variableHandler,
                                                                        framesInContact);

    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight(3);
        if (!handler->getParameter("weight", rawWeight))
        {
            std::cerr << "[MomentumBasedTorqueControl::addCentroidalLinearMomentumElement] Unable "
                         "to get the Weight."
                      << std::endl;
            return false;
        }

        m_costFunction->addCostFunction(m_centroidalLinearMomentumElement.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        "centroidal_linear_momentum");

    } else
        m_constraints->addConstraint(m_centroidalLinearMomentumElement.get());

    return true;
}

template <typename T>
bool MomentumBasedTorqueControl::addAngularMomentumElement(
    std::unique_ptr<ParametersHandler::IParametersHandler<T>> handler)
{
    // get all the required parameters
    using namespace OptimalControlUtilities;

    // the frame in contact are two (left and right foot)
    std::vector<FrameInContact<std::string, std::string>> framesInContact;

    bool isCompliantContact = true;
    for (const auto& stanceFoot : m_stanceFeetIdetrifiers)
        framesInContact.emplace_back(stanceFoot.identifierInVariableHandler(),
                                     stanceFoot.identifierInModel(),
                                     isCompliantContact);

    // gains and weights
    // Gain
    iDynTree::Vector3 kp, kd, ki;
    if (!handler->getParameter("kp", kp) || !handler->getParameter("kd", kd) || !handler->getParameter("ki", ki))
    {
        std::cerr << "[MomentumBasedTorqueControl::addCentroidalAngulatMomentumElement] Unable to "
                     "get the gains.";
        return false;
    }

    // create the pid controller
    PIDController<iDynTree::Vector3> pidController(kd, kp, ki);

    double samplingTime;
    if (!handler->getParameter("sampling_time", samplingTime))
    {
        std::cerr << "[MomentumBasedTorqueControl::addCentroidalAngulatMomentumElement] Unable to "
                     "find the sampling time"
                  << std::endl;
        return false;
    }


    m_centroidalAngularMomentumElement
        = std::make_unique<CentroidalAngularMomentumRateOfChangeElement>(m_kinDyn,
                                                                         pidController,
                                                                         m_variableHandler,
                                                                         framesInContact,
                                                                         samplingTime);

    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight(3);
        if (!handler->getParameter("weight", rawWeight))
        {
            std::cerr << "[MomentumBasedTorqueControl::addCentroidalAngularMomentumElement] Unable "
                         "to get the Weight."
                      << std::endl;
            return false;
        }

        m_costFunction->addCostFunction(m_centroidalAngularMomentumElement.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        "centroidal_angular_momentum");

    } else
        m_constraints->addConstraint(m_centroidalAngularMomentumElement.get());

    return true;
}

template <class T>
bool MomentumBasedTorqueControl::addOrientationElement(unique_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                       const std::string& label)
{
    // get all the required parameters
    using namespace OptimalControlUtilities;

    auto type = CartesianElementType::ORIENTATION;
    auto axis = CartesianElementAxisName::ALL;

    std::string frameInModel;
    if (!handler->getParameter("frame_name", frameInModel))
    {
        std::cerr << "[MomentumBasedTorqueControl::addCartesianElement] The frame_name cannot be "
                     "found"
                  << std::endl;
        return false;
    }

    double kp, kd, c0;
    if (!handler->getParameter("kp", kp))
    {
        std::cerr << "[MomentumBasedTorqueControl::addCartesianElement] The kp cannot be "
                     "found"
                  << std::endl;
        return false;
    }
    bool useDefaultKp = false;
    handler->getParameter("use_default_kd", useDefaultKp);
    if (useDefaultKp)
    {
        double scaling = 1.0;
        handler->getParameter("scaling", scaling);
        kd = 2 / scaling * std::sqrt(kp);
    } else
    {
        if (!handler->getParameter("kd", kd))
        {
            std::cerr << "[MomentumBasedTorqueControl::addCartesianElement] The kd cannot be "
                     "found"
                  << std::endl;
            return false;
        }
    }

    if (!handler->getParameter("c0", c0))
    {
        std::cerr << "[MomentumBasedTorqueControl::addCartesianElement] The c0 cannot be "
                     "found"
                  << std::endl;
        return false;
    }

    auto pdController = std::make_unique<OrientationPD>();
    pdController->setGains(c0, kd, kp);

    if (m_cartesianElements.find(label) != m_cartesianElements.end())
        throw std::runtime_error("[TaskBasedTorqueControl::addCartesianElement] The element named "
                                 + label + " has been already added.");

    m_cartesianElements.emplace(label,
                                std::make_unique<CartesianElement<CartesianElementType::ORIENTATION>>(m_kinDyn,
                                                                                                      std::move(pdController),
                                                                                                      m_variableHandler,
                                                                                                      frameInModel));

    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight;
        if (!handler->getParameter("weight", rawWeight))
        {
            std::cerr << "[MomentumBasedTorqueControl::addCentroidalLinearMomentumElement] Unable "
                         "to get the Weight."
                      << std::endl;
            return false;
        }
        m_costFunction->addCostFunction(m_cartesianElements.find(label)->second.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        label + "_cartesian_element");
    } else
        m_constraints->addConstraint(m_cartesianElements.find(label)->second.get());

    return true;
}

template <class T>
bool MomentumBasedTorqueControl::addSystemDynamicsElement(unique_ptr<ParametersHandler::IParametersHandler<T>> handler)
{
    using namespace OptimalControlUtilities;

    // get the frames in contact name
    std::vector<FrameInContact<std::string, std::string>> framesInContact;
    bool isCompliantContact = true;
    for (const auto& stanceFoot : m_stanceFeetIdetrifiers)
        framesInContact.emplace_back(stanceFoot.identifierInVariableHandler(),
                                     stanceFoot.identifierInModel(),
                                     isCompliantContact);

    m_floatingBaseDynamics = std::make_unique<FloatingBaseDynamicsElement>(m_kinDyn,
                                                                           m_variableHandler,
                                                                           framesInContact);

    // initialize joint dynamics element
    VariableHandler tempVariableHandler(m_variableHandler);
    size_t jointAccelerationSize = m_variableHandler.getVariable("joint_accelerations").size;
    tempVariableHandler.addVariable("joint_torques", jointAccelerationSize);

    m_jointDynamics = std::make_unique<JointSpaceDynamicsElement>(m_kinDyn,
                                                                  tempVariableHandler,
                                                                  framesInContact);

    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight;
        if (!handler->getParameter("weight", rawWeight))
        {
            std::cerr << "[MomentumBasedTorqueControl::addCentroidalLinearMomentumElement] Unable "
                         "to get the Weight."
                      << std::endl;
            return false;
        }
        m_costFunction->addCostFunction(m_floatingBaseDynamics.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        "floating_base_dynamics");
    } else
        m_constraints->addConstraint(m_floatingBaseDynamics.get());

    return true;
}


template <class T>
bool MomentumBasedTorqueControl::addRegularizationElement(std::unique_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                          const std::string& label)
{
    using namespace BipedalLocomotionControllers::OptimalControlUtilities;

    if (m_regularizationElements.find(label) != m_regularizationElements.end())
    {
        std::cerr << "[MomentumBasedTorqueControl::addRegularizationElement] The element named "
                  << label << " has been already added" << std::endl;
        return false;
    }


    m_regularizationElements.emplace(label,
                                     std::make_unique<RegularizationElement>(m_kinDyn,
                                                                             m_variableHandler,
                                                                             label));
    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight;
        if (!handler->getParameter("weight", rawWeight))
        {
            std::cerr << "[MomentumBasedTorqueControl::addCentroidalLinearMomentumElement] Unable "
                         "to get the Weight."
                      << std::endl;
            return false;
        }
        m_costFunction->addCostFunction(m_regularizationElements.find(label)->second.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        label + "_regularization");
    } else
        m_constraints->addConstraint(m_regularizationElements.find(label)->second.get());

    return true;
}

template <class T>
bool MomentumBasedTorqueControl::addRegularizationWithControlElement( std::unique_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                                      const std::string& label)
{
    using namespace BipedalLocomotionControllers::OptimalControlUtilities;

    if (m_regularizationWithControlElements.find(label) != m_regularizationWithControlElements.end())
    {
        std::cerr << "[MomentumBasedTorqueControl::addRegularizationWithControlElement] The "
                     "element named "
                  << label << " has been already added" << std::endl;
        return false;
    }

    // instantiate gains
    iDynTree::VectorDynSize kp, kd;
    bool outcome = true;
    outcome = handler->getParameter("kp", kp);

    if (!outcome)
    {
        std::cerr << "[MomentumBasedTorqueControl::addRegularizationWithControlElement] The Kp of "
                     "the "
                  << label << " cannot be found" << std::endl;
        return outcome;
    }

    bool useDefaultKp = false;
    handler->getParameter("use_default_kd", useDefaultKp);
    if (useDefaultKp)
    {
        double scaling = 1.0;
        handler->getParameter("scaling", scaling);

        kd.resize(kp.size());
        iDynTree::toEigen(kd) = 2 / scaling * iDynTree::toEigen(kp).array().sqrt();
    } else
    {
        outcome = handler->getParameter("kd", kd);
        if (!outcome)
        {
            std::cerr << "[MomentumBasedTorqueControl::addRegularizationWithControlElement] The kd "
                         "cannot be found"
                      << std::endl;
            return outcome;
        }
    }
    auto pdController = std::make_unique<LinearPD<iDynTree::VectorDynSize>>(kp, kd);

    m_regularizationWithControlElements.emplace(label,
                                                std::make_unique<RegularizationWithControlElement>(m_kinDyn,
                                                                                                   std::move(pdController),
                                                                                                   m_variableHandler,
                                                                                                   label));
    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight;
        if (!handler->getParameter("weight", rawWeight))
        {
            std::cerr << "[MomentumBasedTorqueControl::addCentroidalLinearMomentumElement] Unable "
                         "to get the Weight."
                      << std::endl;
            return false;
        }
        m_costFunction->addCostFunction(m_regularizationWithControlElements.find(label)->second.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        label + "_regularization_with_constraints");
    } else
        m_constraints->addConstraint(m_regularizationWithControlElements.find(label)->second.get());

    return true;
}

template <class T>
bool MomentumBasedTorqueControl::addJointValuesFeasibilityElement(unique_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                                  const iDynTree::VectorDynSize& maxJointsPosition,
                                                                  const iDynTree::VectorDynSize& minJointsPosition)
{
    using namespace BipedalLocomotionControllers::OptimalControlUtilities;

    double samplingTime;
    if (!handler->getParameter("sampling_time", samplingTime))
    {
        std::cerr << "[MomentumBasedTorqueControl::addJointValuesFeasibilityElement] Unable to "
                     "find the sampling time"
                  << std::endl;
        return false;
    }

    m_jointValuesFeasibilityElement = std::make_unique<JointValuesFeasibilityElement>(m_kinDyn,
                                                                                      m_variableHandler,
                                                                                      "joint_accelerations",
                                                                                      maxJointsPosition,
                                                                                      minJointsPosition,
                                                                                      samplingTime);

    m_constraints->addConstraint(m_jointValuesFeasibilityElement.get());

    return true;
}

template <class T>
bool MomentumBasedTorqueControl::addContactWrenchFeasibilityElement(unique_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                                    const OptimalControlUtilities::Frame<std::string, std::string>& frame)
{
    using namespace BipedalLocomotionControllers::OptimalControlUtilities;

    const auto& label = frame.identifierInVariableHandler();

    if (m_contactWrenchFeasibilityElements.find(label) != m_contactWrenchFeasibilityElements.end())
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactWrenchFeasibilityElement] This "
                     "element named "
                  << label << "has been already added" << std::endl;
        return false;
    }

    double samplingTime;
    if (!handler->getParameter("sampling_time", samplingTime))
    {
        std::cerr << "[MomentumBasedTorqueControl::addJointValuesFeasibilityElement] Unable to "
                     "find the sampling time"
                  << std::endl;
        return false;
    }

    double staticFrictionCoefficient;
    if (!handler->getParameter("static_friction_coefficient", staticFrictionCoefficient))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactWrenchFeasibilityElement] "
                     "static_friction_coefficient of "
                         + label + " cannot be found"
                  << std::endl;
        return false;
    }

    int numberOfPoints;
    if (!handler->getParameter("number_of_points", numberOfPoints))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactWrenchFeasibilityElement] "
                     "static_friction_coefficient of "
                         + label + " cannot be found"
                  << std::endl;
        return false;
    }

    double torsionalFrictionCoefficient;
    if (!handler->getParameter("torsional_friction_coefficient", torsionalFrictionCoefficient))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactWrenchFeasibilityElement] "
                     "torsional_friction_coefficient of "
                  << label << " cannot be found" << std::endl;
        return false;
    }

    iDynTree::Vector2 footLimitsX;
    if (!handler->getParameter("foot_limits_x", footLimitsX))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactWrenchFeasibilityElement] "
                     "foot_limits_x of "
                  << label << " cannot be found" << std::endl;
        return false;
    }

    iDynTree::Vector2 footLimitsY;
    if (!handler->getParameter("foot_limits_y", footLimitsY))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactWrenchFeasibilityElement] "
                     "foot_limits_y of "
                  << label << " cannot be found" << std::endl;
        return false;
    }

    double minimalNormalForce;
    if (!handler->getParameter("minimal_normal_force", minimalNormalForce))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactWrenchFeasibilityElement] "
                     "foot_limits_y of "
                  << label << " cannot be found" << std::endl;
        return false;
    }

    m_contactWrenchFeasibilityElements.insert(
        {label,
         std::make_unique<ContactWrenchRateOfChangeFeasibilityElement>(m_kinDyn,
                                                                       m_variableHandler,
                                                                       frame,
                                                                       numberOfPoints,
                                                                       staticFrictionCoefficient,
                                                                       torsionalFrictionCoefficient,
                                                                       minimalNormalForce,
                                                                       footLimitsX,
                                                                       footLimitsY,
                                                                       OsqpEigen::INFTY,
                                                                       samplingTime)});

    m_constraints->addConstraint(m_contactWrenchFeasibilityElements.find(label)->second.get());

    return true;
}

template <class T>
bool MomentumBasedTorqueControl::addContactModelElement(unique_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                        const OptimalControlUtilities::Frame<std::string, std::string>& frame)
{
    using namespace BipedalLocomotionControllers::OptimalControlUtilities;
    using namespace BipedalLocomotionControllers::ContactModels;

    const auto& label = frame.identifierInVariableHandler();

    if (m_contactModelElements.find(label) != m_contactModelElements.end())
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactModelElement] This element named "
                  << label << "has been already added" << std::endl;
        return false;
    }

    // get contact models parameters
    double length, width, springCoeff, damperCoeff;
    if (!handler->getParameter("length", length) || !handler->getParameter("width", width)
        || !handler->getParameter("spring_coeff", springCoeff)
        || !handler->getParameter("damper_coeff", damperCoeff))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactModelElement] Unable to get the "
            "contact parameters." << std::endl;
        return false;
    }
    const std::unordered_map<std::string, std::any> parameters({{"length", length},
                                                                {"width", width},
                                                                {"spring_coeff", springCoeff},
                                                                {"damper_coeff", damperCoeff}});

    FrameInContactWithContactModel<std::string, std::string> frameInContact;
    frameInContact.identifierInVariableHandler() = label;
    frameInContact.identifierInModel() = frame.identifierInModel();
    frameInContact.isInCompliantContact() = true;
    frameInContact.contactModel() = std::make_shared<ContinuousContactModel>(parameters);

    m_contactModelElements.insert(
        {label,
         std::make_unique<ContactModelElement>(m_kinDyn, m_variableHandler, frameInContact)});

    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight;
        if (!handler->getParameter("weight", rawWeight))
        {
            std::cerr << "[MomentumBasedTorqueControl::addContactModelElement] Unable to get the "
                         "Weight."
                      << std::endl;
            return false;
        }
        m_costFunction->addCostFunction(m_contactModelElements.find(label)->second.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        label);
    } else
        m_constraints->addConstraint(m_contactModelElements.find(label)->second.get());


    return true;
}

template<class T>
bool MomentumBasedTorqueControl::initialize(std::unique_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                            const std::string& controllerType,
                                            const iDynTree::VectorDynSize& maxJointsPosition,
                                            const iDynTree::VectorDynSize& minJointsPosition)
{
    if (!addFeetIdentifiers(handler->getGroup(controllerType)))
    {
        std::cerr << "[MomentumBasedTorqueControl::initialize] Unable to load the feet identifiers"
                  << std::endl;
        return false;
    }

    initializeVariableHandler();

    double samplingTime;
    if(!handler->getParameter("sampling_time", samplingTime))
    {
        std::cerr << "[MomentumBasedTorqueControl::initialize] Unable to find the sampling time"
                  << std::endl;
        return false;
    }

    auto linearMomentumOptions = handler->getGroup("CENTROIDAL_LINEAR_MOMENTUM");
    if (!linearMomentumOptions->isEmpty())
        if (!addLinearMomentumElement(std::move(linearMomentumOptions)))
        {
            std::cerr << "[MomentumBasedTorqueControl::initialize] Unable to add the linear "
                         "momentum element"
                      << std::endl;
            return false;
        }

    auto angularMomentumOptions = handler->getGroup("CENTROIDAL_ANGULAR_MOMENTUM");
    if(!angularMomentumOptions->isEmpty())
    {
        angularMomentumOptions->setParameter("sampling_time", samplingTime);
        if(!addAngularMomentumElement(std::move(angularMomentumOptions)))
        {
            std::cerr << "[MomentumBasedTorqueControl::initialize] Unable to add the angular "
                         "momentum element"
                      << std::endl;
            return false;
        }
    }

    auto torsoOptions = handler->getGroup("TORSO");
    if (!torsoOptions->isEmpty())
        if (!addOrientationElement(std::move(torsoOptions), "torso"))
        {
            std::cerr << "[MomentumBasedTorqueControl::initialize] Unable to add the torso element"
                      << std::endl;
            return false;
        }

    auto systemDynamicsOptions = handler->getGroup("SYSTEM_DYNAMICS");
    if (!systemDynamicsOptions->isEmpty())
        if (!addSystemDynamicsElement(std::move(systemDynamicsOptions)))
        {
            std::cerr << "[MomentumBasedTorqueControl::initialize] Unable to add the system dynamics"
                      << std::endl;
            return false;
        }

    auto jointRegularizationOptions = handler->getGroup("JOINT_REGULARIZATION");
    if (!jointRegularizationOptions->isEmpty())
        if (!addRegularizationWithControlElement(std::move(jointRegularizationOptions),
                                                 "joint_accelerations"))
        {
            std::cerr << "[MomentumBasedTorqueControl::initialize] Unable to add the joint "
                         "regularization element"
                      << std::endl;
            return false;
        }

    for (const auto& identifier : m_stanceFeetIdetrifiers)
    {
        std::string upperIdentifier = identifier.identifierInVariableHandler();
        std::transform(upperIdentifier.begin(),
                       upperIdentifier.end(),
                       upperIdentifier.begin(),
                       [](unsigned char c) { return std::toupper(c); });

        // add the regularization element
        auto stanceFootRegularizationOptions = handler->getGroup(upperIdentifier + "_WRENCH_REGULARIZATION");
        if (!stanceFootRegularizationOptions->isEmpty())
            if (!addRegularizationElement(std::move(stanceFootRegularizationOptions),
                                          identifier.identifierInVariableHandler()))
            {
                std::cerr << "[MomentumBasedTorqueControl::initialize] Unable to add the wrench "
                             "regularization element for the "
                          << identifier.identifierInVariableHandler() << std::endl;
                return false;
            }

        // add the wrench feasibility element
        auto stanceFootWrenchOptions = handler->getGroup(upperIdentifier + "_WRENCH_FEASIBILITY");
        if (!stanceFootWrenchOptions->isEmpty())
        {
            stanceFootWrenchOptions->setParameter("sampling_time", samplingTime);

            if (!addContactWrenchFeasibilityElement(std::move(stanceFootWrenchOptions), identifier))
            {
                std::cerr << "[MomentumBasedTorqueControl::initialize] Unable to add the wrench "
                             "feasibility element for the "
                          << identifier.identifierInVariableHandler() << std::endl;
                return false;
            }
        }

        // add the contact model
        auto stanceFootContactModelOptions = handler->getGroup(upperIdentifier + "_CONTACT_MODEL");
        if (!stanceFootContactModelOptions->isEmpty())
            if (!addContactModelElement(std::move(stanceFootContactModelOptions), identifier))
            {
                std::cerr << "[MomentumBasedTorqueControl::initialize] Unable to add the contact "
                             "model for the "
                          << identifier.identifierInVariableHandler() << std::endl;
                return false;
            }
    }

    addRegularizationElement(handler->getGroup("JOINT_ACCELERATION_REGULARIZATION"), "joint_accelerations");

    addRegularizationElement(handler->getGroup("BASE_ACCELERATION_REGULARIZATION"), "base_acceleration");


    // auto jointFeasibilityOptions = handler->getGroup("JOINT_FEASIBILITY");
    // jointFeasibilityOptions->setParameter("sampling_time", samplingTime);
    // addJointValuesFeasibilityElement(std::move(jointFeasibilityOptions),
    //                                  maxJointsPosition,
    //                                  minJointsPosition);


    initialzeSolver();
    printElements();

    return true;
}

} // namespace WholeBodyControllers
} // namespace BipedalLocomotionControllers