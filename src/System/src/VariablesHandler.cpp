/**
 * @file VariablesHandler.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::ParametersHandler;

bool VariablesHandler::VariableDescription::isValid() const
{
    return (offset >= 0) && (size >= 0);
}

VariablesHandler::VariableDescription VariablesHandler::VariableDescription::InvalidVariable()
{
    VariablesHandler::VariableDescription tmp;
    tmp.offset = tmp.size = -1;
    return tmp;
}

bool VariablesHandler::initialize(std::weak_ptr<const IParametersHandler> handler) noexcept
{
    // clear the content of the handler
    this->clear();

    constexpr auto logPrefix = "[VariablesHandler::initialize]";
    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid ParametersHandler pointer.", logPrefix);
        return false;
    }

    std::vector<std::string> names;
    std::vector<int> sizes;
    if (!ptr->getParameter("variables_name", names))
    {
        log()->error("{} Unable to get the name of the variables.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("variables_size", sizes))
    {
        log()->error("{} Unable to get the size of the variables.", logPrefix);
        return false;
    }

    if (names.size() != sizes.size())
    {
        log()->error("{} The size of the list containing the names is different from the size of "
                     "the list containing the sizes. Size of variables_name: {}. Size of "
                     "variables_size: {}",
                     logPrefix,
                     names.size(),
                     sizes.size());
        return false;
    }

    // the size must be a strictly positive number
    auto iterator = std::find_if(sizes.begin(), sizes.end(), [](int size) { return size <= 0; });
    if (iterator != sizes.end())
    {
        log()->error("{} There exist a non positive element in the variables_size list. The size "
                     "must be a strictly positive number.",
                     logPrefix);
        return false;
    }

    for (int i = 0; i < names.size(); i++)
    {
        if (!this->addVariable(names[i], sizes[i]))
        {
            log()->error("{} Unable to add the variable named {} having a size equal to {}.",
                         logPrefix,
                         names[i],
                         sizes[i]);
            return false;
        }
    }

    return true;
}

bool VariablesHandler::addVariable(const std::string& name, const std::size_t& size) noexcept
{
    // if the variable already exist cannot be added again.
    if (m_variables.find(name) != m_variables.end())
    {
        log()->error("[VariableHandler::addVariable] The variable named {} already exists.", name);
        return false;
    }

    VariablesHandler::VariableDescription description;
    description.size = size;
    description.offset = m_numberOfVariables;
    description.name = name;

    m_variables.emplace(name, description);
    m_numberOfVariables += size;

    return true;
}

const VariablesHandler::VariableDescription&
VariablesHandler::getVariable(const std::string& name) const noexcept
{
    auto variable = m_variables.find(name);

    // if the variable is present its IndexRange is returned otherwise an
    // invalid IndexRange is provided to the user

    if (variable != m_variables.end())
        return variable->second;
    else
        return m_invalidVariable;
}

bool VariablesHandler::getVariable(const std::string& name, //
                                   VariablesHandler::VariableDescription& description) const noexcept
{
    return (description = this->getVariable(name)).isValid();
}

const std::size_t& VariablesHandler::getNumberOfVariables() const noexcept
{
    return m_numberOfVariables;
}

std::string VariablesHandler::toString() const noexcept
{
    std::string out;
    for (const auto& [key, variable] : m_variables)
    {
        out += key + " size: " + std::to_string(variable.size)
              + ", offset: " + std::to_string(variable.offset) + ". ";
    }

    return out;
}

void VariablesHandler::clear() noexcept
{
    m_numberOfVariables = 0;
    m_variables.clear();
}
