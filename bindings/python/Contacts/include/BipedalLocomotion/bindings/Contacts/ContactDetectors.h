/**
 * @file ContactDetectors.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_CONTACTS_CONTACT_DETECTORS_H
#define BIPEDAL_LOCOMOTION_BINDINGS_CONTACTS_CONTACT_DETECTORS_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Contacts
{

void CreateContactDetector(pybind11::module& module);
void CreateSchmittTriggerUnit(pybind11::module& module);
void CreateSchmittTriggerDetector(pybind11::module& module);
void CreateFixedFootDetector(pybind11::module& module);

} // namespace Contacts
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_CONTACTS_CONTACT_DETECTORS_H
