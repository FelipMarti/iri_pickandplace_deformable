// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _pickandplace_deformable_alg_node_h_
#define _pickandplace_deformable_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "pickandplace_deformable_alg.h"

// [publisher subscriber headers]

// [service client headers]
#include <iri_common_drivers_msgs/QueryJointsMovement.h>

// [action server client headers]
#include <iri_action_server/iri_action_server.h>
#include <iri_pickandplace_deformable/PaPDeformableAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iri_wam_generic_pickandplace/PickAndPlaceAction.h>

#include <tf/transform_listener.h>

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class PickandplaceDeformableAlgNode:public algorithm_base::IriBaseAlgorithm <
    PickandplaceDeformableAlgorithm > {
 private:
	// [publisher attributes]

	// [subscriber attributes]

	// [service attributes]

	// [client attributes]
	ros::ServiceClient wam_joints_pose_client_;
	iri_common_drivers_msgs::QueryJointsMovement wam_joints_pose_srv_;

	// [action server attributes]
	IriActionServer < iri_pickandplace_deformable::PaPDeformableAction >
	    pick_place_defo_aserver_;
	void pick_place_defoStartCallback(const
					  iri_pickandplace_deformable::PaPDeformableGoalConstPtr
					  & goal);
	void pick_place_defoStopCallback(void);
	bool pick_place_defoIsFinishedCallback(void);
	bool pick_place_defoHasSucceedCallback(void);
	void pick_place_defoGetResultCallback(iri_pickandplace_deformable::
					      PaPDeformableResultPtr & result);
	void pick_place_defoGetFeedbackCallback(iri_pickandplace_deformable::
						PaPDeformableFeedbackPtr &
						feedback);

	// [action client attributes]
	actionlib::SimpleActionClient <
	    iri_wam_generic_pickandplace::PickAndPlaceAction >
	    pick_and_place_client_;
	iri_wam_generic_pickandplace::PickAndPlaceGoal pick_and_place_goal_;
	void pick_and_placeMakeActionRequest();
	void pick_and_placeDone(const actionlib::SimpleClientGoalState & state,
				const
				iri_wam_generic_pickandplace::PickAndPlaceResultConstPtr
				& result);
	void pick_and_placeActive();
	void pick_and_placeFeedback(const
				    iri_wam_generic_pickandplace::PickAndPlaceFeedbackConstPtr
				    & feedback);

	// MY CLASS VARS
	int PickPlace_Defo_State;
	int PickPlace_Defo_Result;

	int Pick_and_Place_Feedback;
	int Pick_or_Place_Feedback;

	struct point_XYZ {	//or point_RPY
		float X;	//or R
		float Y;	//or P
		float Z;	//or Y
	};

	// Pick and Drop Robot 3D position   
	point_XYZ pick_pos;
	point_XYZ place_pos;
	float yaw;

	//Create a TransformListener object
	tf::TransformListener tflistener_rob_H_cam;
	//Create a StampedTransformr object
	tf::StampedTransform transform_rob_H_cam;

 public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
	PickandplaceDeformableAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
	~PickandplaceDeformableAlgNode(void);

 protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
	void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
	void node_config_update(Config & config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
	void addNodeDiagnostics(void);

	// [diagnostic functions]

	// [test functions]

	// TODO: Move to test_grasp_deformable_alg
	/* Set Robot to the INIT pose FUNCTION */
	bool My_Init_Robot_Pose();

};

#endif
