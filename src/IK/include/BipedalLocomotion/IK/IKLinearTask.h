/**
 * @file IKLinearTask.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_IK_LINEAR_TASK_H
#define BIPEDAL_LOCOMOTION_SYSTEM_IK_LINEAR_TASK_H

#include <BipedalLocomotion/System/LinearTask.h>

namespace BipedalLocomotion
{
namespace IK
{

/**
 * IKLinearTask specializes a LinearTask in the case of Inverse Kinematics.
 */
struct IKLinearTask : public BipedalLocomotion::System::LinearTask
{
};

} // namespace IK
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_IK_LINEAR_TASK_H
