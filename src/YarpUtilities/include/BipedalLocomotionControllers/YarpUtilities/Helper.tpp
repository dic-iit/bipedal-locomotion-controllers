/**
 * @file Helper.tpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// std
#include <iostream>
#include <stdexcept>

// clang-format off

#define YARP_UTILITES_GET_ELEMENT_TYPE(type)                            \
    ((std::is_same<type, double>::value) ? "double" :                   \
    ((std::is_same<type, int>::value) ? "int" :                         \
    ((std::is_same<type, std::string>::value) ? "string" :              \
    ((std::is_same<type, bool>::value) ? "bool" :                       \
     "undefined" ))))

#define YARP_UTILITES_CHECK_ELEMENT_SUPPORT(type)                       \
    ((std::is_same<type, double>::value) ? true :                       \
    ((std::is_same<type, int>::value) ? true :                          \
    ((std::is_same<type, std::string>::value) ? true :                  \
    ((std::is_same<type, bool>::value) ? true :                         \
     false ))))

#define YARP_UTILITES_GET_CHECKER_NAME(type)                                                  \
    ((std::is_same<type, int>::value) ? &yarp::os::Value::isInt :                             \
    ((std::is_same<type, double>::value) ? &yarp::os::Value::isDouble :                       \
    ((std::is_same<type, std::string>::value) ? &yarp::os::Value::isString :                  \
    ((std::is_same<type, bool>::value) ? &yarp::os::Value::isBool :                           \
     &yarp::os::Value::isDouble ))))

// clang-format on

namespace BipedalLocomotionControllers
{

/**
 * Helper for YARP library.
 */
namespace YarpUtilities
{

template <typename T> T convertValue(const yarp::os::Value& value)
{
    throw std::runtime_error("[BipedalLocomotionControllers::YarpUtilities::convertValue] The "
                             "non specialized version has not been implemented");

    return T();
}

template <> int convertValue<int>(const yarp::os::Value& value)
{
    return value.asInt();
}

template <> double convertValue<double>(const yarp::os::Value& value)
{
    return value.asDouble();
}

template <> std::string convertValue<std::string>(const yarp::os::Value& value)
{
    return value.asString();
}

template <> bool convertValue<bool>(const yarp::os::Value& value)
{
    return value.asBool();
}

template <typename T>
bool getElementFromSearchable(const yarp::os::Searchable& config,
                              const std::string& key,
                              T& element)
{

    static_assert(YARP_UTILITES_CHECK_ELEMENT_SUPPORT(T),
                  "[BipedalLocomotionControllers::YarpUtilities::getElementFromSearchable] The "
                  "function getElementFromSearchable() cannot be called with the desired "
                  "element type");

    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getElementFromSearchable] "
                     "Missing field named "
                  << key << std::endl;
        return false;
    }

    if (!(value->*YARP_UTILITES_GET_CHECKER_NAME(T))())
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getElementFromSearchable] The "
                     "value named "
                  << key << " is not a " << YARP_UTILITES_GET_ELEMENT_TYPE(T) << "." << std::endl;
        return false;
    }

    element = convertValue<T>(*value);
    return true;
}

template <typename T>
bool getVectorFromSearchable(const yarp::os::Searchable& config, const std::string& key, T& vector)
{

    using elementType = typename std::pointer_traits<decltype(vector.data())>::element_type;

    static_assert(YARP_UTILITES_CHECK_ELEMENT_SUPPORT(elementType),
                  "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] The "
                  "function getElementFromSearchable() cannot be called with the desired "
                  "element type");

    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] "
                     "Missing field "
                  << key << std::endl;
        return false;
    }

    if (value->isNull())
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] Empty "
                     "input value named "
                  << key << std::endl;
        return false;
    }

    if (!value->isList())
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] The "
                     "value named "
                  << key << "is not associated to a list." << std::endl;
        return false;
    }

    yarp::os::Bottle* inputPtr = value->asList();
    if (inputPtr == nullptr)
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] The "
                     "list associated to the value named "
                  << key << " is empty." << std::endl;
        return false;
    }

    // resize the vector
    vector.resize(inputPtr->size());

    for (int i = 0; i < inputPtr->size(); i++)
    {
        if (!(inputPtr->get(i).*YARP_UTILITES_GET_CHECKER_NAME(elementType))())
        {
            std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] "
                         "The element of the list associated to the value named "
                      << key << " is not a " << YARP_UTILITES_GET_ELEMENT_TYPE(elementType) << "."
                      << std::endl;
            return false;
        }

        vector[i] = convertValue<elementType>(inputPtr->get(i));
    }
    return true;
}

template <>
bool getVectorFromSearchable<std::vector<bool>>(const yarp::os::Searchable& config,
                                                const std::string& key,
                                                std::vector<bool>& vector)
{

    static_assert(YARP_UTILITES_CHECK_ELEMENT_SUPPORT(bool),
                  "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] The "
                  "function getElementFromSearchable() cannot be called with the desired "
                  "element type");

    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] "
                     "Missing field "
                  << key << std::endl;
        return false;
    }

    if (value->isNull())
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] Empty "
                     "input value named "
                  << key << std::endl;
        return false;
    }

    if (!value->isList())
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] The "
                     "value named "
                  << key << "is not associated to a list." << std::endl;
        return false;
    }

    yarp::os::Bottle* inputPtr = value->asList();
    if (inputPtr == nullptr)
    {
        std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] The "
                     "list associated to the value named "
                  << key << " is empty." << std::endl;
        return false;
    }

    // resize the vector
    vector.resize(inputPtr->size());

    for (int i = 0; i < inputPtr->size(); i++)
    {
        if (!(inputPtr->get(i).isBool()) && !(inputPtr->get(i).isInt()))
        {
            std::cerr << "[BipedalLocomotionControllers::YarpUtilities::getVectorFromSearchable] "
                         "The element of the list associated to the value named "
                      << key << " is not a boolean ." << std::endl;
            return false;
        }

        vector[i] = convertValue<bool>(inputPtr->get(i));
    }
    return true;
}

template <typename T> void mergeSigVector(yarp::sig::Vector& vector, const T& t)
{
    for (int i = 0; i < t.size(); i++)
        vector.push_back(t(i));

    return;
}

template <typename T, typename... Args>
void mergeSigVector(yarp::sig::Vector& vector, const T& t, const Args&... args)
{
    for (int i = 0; i < t.size(); i++)
        vector.push_back(t(i));

    mergeSigVector(vector, args...);

    return;
}

template <typename... Args>
void sendVariadicVector(yarp::os::BufferedPort<yarp::sig::Vector>& port, const Args&... args)
{
    yarp::sig::Vector& vector = port.prepare();
    vector.clear();

    mergeSigVector(vector, args...);

    port.write();
}
} // namespace YarpUtilities
} // namespace BipedalLocomotionControllers