/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "dancer_raisim/LeggedRobotRaisimVisualizer.h"

#include <ocs2_raisim_ros/RaisimHeightmapRosConverter.h>

namespace ocs2 {
namespace dancer {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotRaisimVisualizer::LeggedRobotRaisimVisualizer(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                                                         const PinocchioEndEffectorKinematics& endEffectorKinematics,
                                                         ros::NodeHandle& nodeHandle, scalar_t maxUpdateFrequency)
    : LeggedRobotVisualizer(pinocchioInterface, centroidalModelInfo, endEffectorKinematics, nodeHandle, maxUpdateFrequency) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotRaisimVisualizer::update(const SystemObservation& observation, const PrimalSolution& primalSolution,
                                         const CommandData& command) {
  SystemObservation raisimObservation = observation;
  PrimalSolution raisimPrimalSolution = primalSolution;
  CommandData raisimCommand = command;
  // height relative to terrain
  if (terrainPtr_ != nullptr) {
    raisimObservation.state(8) += terrainPtr_->getHeight(observation.state(6), observation.state(7));
    for (size_t i = 0; i < primalSolution.stateTrajectory_.size(); i++) {
      raisimPrimalSolution.stateTrajectory_[i](8) +=
          terrainPtr_->getHeight(primalSolution.stateTrajectory_[i](6), primalSolution.stateTrajectory_[i](7));
    }
    raisimCommand.mpcInitObservation_.state(8) +=
        terrainPtr_->getHeight(command.mpcInitObservation_.state(6), command.mpcInitObservation_.state(7));
    for (size_t i = 0; i < command.mpcTargetTrajectories_.stateTrajectory.size(); i++) {
      raisimCommand.mpcTargetTrajectories_.stateTrajectory[i](8) += terrainPtr_->getHeight(
          command.mpcTargetTrajectories_.stateTrajectory[i](6), command.mpcTargetTrajectories_.stateTrajectory[i](7));
    }
  }
  LeggedRobotVisualizer::update(raisimObservation, raisimPrimalSolution, raisimCommand);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotRaisimVisualizer::updateTerrain(double timeout) {
  terrainPtr_ = std::move(RaisimHeightmapRosConverter::getHeightmapFromRos(timeout).first);
}

}  // namespace dancer
}  // namespace ocs2
