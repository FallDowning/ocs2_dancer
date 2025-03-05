/*
 * LeggedRobotMrtInterface.h
 *
 *  Created on: 2024-5-19
 *      Author: Ren kunzhao
 */

#pragma once

#include <dancer_interface/LeggedRobotInterface.h>
#include <dancer_interface/gait/MotionPhaseDefinition.h>

#include <dancer_raisim/LeggedRobotRaisimConversions.h>
#include <dancer_raisim/LeggedRobotRaisimVisualizer.h>

// #include <dancer_hw_ros/CentroidalModelMrtInterface.h>

namespace CentroidalModel {

class LeggedRobotMrtInterface : public CentroidalModelMrtInterface{
public:
    LeggedRobotMrtInterface(ros::NodeHandle& nodeHandle): CentroidalModelMrtInterface(nodeHandle){

        std::string taskFile, urdfFile, referenceFile;
        nodeHandle.getParam("/taskFile", taskFile);
        nodeHandle.getParam("/urdfFile", urdfFile);
        nodeHandle.getParam("/referenceFile", referenceFile);

        leggedRobotInterface_ = std::make_unique<dancer::LeggedRobotInterface>(taskFile, urdfFile, referenceFile);

        centroidalModelInfo_ = leggedRobotInterface_->getCentroidalModelInfo();

        pinocchioInterfacePtr_ = std::make_unique<PinocchioInterface>(leggedRobotInterface_->getPinocchioInterface());

        CentroidalModelPinocchioMapping pinocchioMapping(leggedRobotInterface_->getCentroidalModelInfo());

        eeKinematics_ = std::make_unique<PinocchioEndEffectorKinematics>(leggedRobotInterface_->getPinocchioInterface(),
                                                                pinocchioMapping, leggedRobotInterface_->modelSettings().contactNames6DoF);
        rbdConversions_ = std::make_unique<CentroidalModelRbdConversions>(leggedRobotInterface_->getPinocchioInterface(),
                                                                    leggedRobotInterface_->getCentroidalModelInfo());
        rbdConversions_->loadSettings(mrtFile_, rbdFieldName_, true);
        
        auto leggedRobotRaisimVisualizer_ = std::make_shared<dancer::LeggedRobotRaisimVisualizer>(
            leggedRobotInterface_->getPinocchioInterface(), leggedRobotInterface_->getCentroidalModelInfo(), *eeKinematics_, nodeHandle);
        leggedRobotRaisimVisualizer_->updateTerrain();
        observers_.push_back(leggedRobotRaisimVisualizer_);

    }

    const RolloutBase& getRollout() const override {return leggedRobotInterface_->getRollout();};    

    vector<bool> getContactFlags(size_t mode) override {
        auto contactFlags = ocs2::dancer::modeNumber2StanceLeg(mode);
        return std::vector<bool>(contactFlags.begin(), contactFlags.end());
    };

    size_t getContactMode(vector<bool> contactFlags) override{
        return ocs2::dancer::stanceLeg2ModeNumber(ocs2::dancer::contact_flag_t{contactFlags[0], contactFlags[1]});
    }

    SystemObservation getInitObs(const scalar_t& time, const vector_t& rbdState) override {          // TODO
        SystemObservation initObs;
        initObs.mode = ocs2::dancer::ModeNumber::STANCE;
        initObs.time = time;
        initObs.state.setZero(getCentroidalModelInfo().stateDim);
        // initObs.state = rbdStateToState(rbdState);
        initObs.state <<    0, 0, 0, 0, 0, 0,
                            0, 0, 0.73, 0, 0, 0,
                            0, 0, -0.4791, 0.83075, -0.35165, 0,
                            0, 0, -0.4791, 0.83075, -0.35165, 0;
        initObs.input = vector_t::Zero(getCentroidalModelInfo().inputDim);
        return initObs;
    }

    std::pair<vector_t, vector_t> rbdStateToStateInput(const vector_t& rbdState) override{
        std::lock_guard<std::mutex> lk(rbdMutex_);  
        auto state = rbdConversions_->computeCentroidalStateFromRbdModel(rbdState);

        auto info = getCentroidalModelInfo();
        vector_t input = vector_t::Zero(info.inputDim);
        input.tail(info.actuatedDofNum) = rbdState.tail(info.actuatedDofNum);

        // check rbd conversion, but this may take some time because invoving conversion between baseVel and comMomentum, only use it when needed
        // auto newRbdState = rbdConversions_->computeRbdStateFromCentroidalModel(state, input);
        // if(!newRbdState.isApprox(rbdState)){
        //     std::cout << "newRbdState: " << newRbdState << std::endl;
        //     std::cout << "rbdState:    " << rbdState << std::endl;
        // }

        return {state, input};
    };

    vector_t inputToTorque( const vector_t& desiredState, const vector_t& desiredInput,
                            const vector_t& desiredJointAccelerations,
                            const vector_t& measuredRbdState) override {
        std::lock_guard<std::mutex> lk(rbdMutex_);                        
        return rbdConversions_->computeRbdTorqueFromCentroidalModelPD(desiredState, desiredInput, desiredJointAccelerations, measuredRbdState);
    };

private:
    std::unique_ptr<dancer::LeggedRobotInterface> leggedRobotInterface_;
};

}
