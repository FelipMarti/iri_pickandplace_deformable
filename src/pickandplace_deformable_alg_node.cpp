#include "pickandplace_deformable_alg_node.h"

PickandplaceDeformableAlgNode::PickandplaceDeformableAlgNode
    (void):algorithm_base::IriBaseAlgorithm < PickandplaceDeformableAlgorithm >
    (), pick_place_defo_aserver_(public_node_handle_, "pick_place_defo"),
pick_and_place_client_("iri_wam_generic_pickandplace/pick_and_place", true)
{
	//init class attributes if necessary
	PickPlace_Defo_State = 0;
	PickPlace_Defo_Result = 0;

	Pick_and_Place_Feedback = 0;
	Pick_or_Place_Feedback = 0;

	// Transformation from estirabot frame to camera frame worldHcam or robotHcam 
	// So, camera position from the point of view of the robot
	//
	// PickAndPlace algorithm requires all the points in estirabot reference frame
	// All the points are obtained from the camera reference frame. So, this transformation
	// is used to calculate pregras postgrasp preungrasp postungrasp points in robot coord
	try {
		tflistener_rob_H_cam.waitForTransform("estirabot_link_base",
						      "camera_rgb_optical_frame",
						      ros::Time(0),
						      ros::Duration(10.0));
		tflistener_rob_H_cam.lookupTransform("estirabot_link_base",
						     "camera_rgb_optical_frame",
						     ros::Time(0),
						     transform_rob_H_cam);
		ROS_INFO("1st Transformation DONE");
	}
	catch(tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		PickPlace_Defo_State = 100;	//ERROR
		ROS_ERROR
		    ("Error when transforming from camera_rgb_optical_frame to camera_rgb_optical_frame");
		ROS_ERROR
		    ("Please connect and check the KINECT, and Start WAM robot");
		ROS_ERROR("Then launch this node again");
	}

	//this->loop_rate_ = 2;//in [Hz]

	// [init publishers]

	// [init subscribers]

	// [init services]

	// [init clients]
	wam_joints_pose_client_ =
	    this->public_node_handle_.serviceClient <
	    iri_common_drivers_msgs::QueryJointsMovement > ("wam_joints_pose");

	// [init action servers]
	pick_place_defo_aserver_.registerStartCallback(boost::bind
						       (&PickandplaceDeformableAlgNode::
							pick_place_defoStartCallback,
							this, _1));
	pick_place_defo_aserver_.
	    registerStopCallback(boost::
				 bind(&PickandplaceDeformableAlgNode::
				      pick_place_defoStopCallback, this));
	pick_place_defo_aserver_.
	    registerIsFinishedCallback(boost::
				       bind(&PickandplaceDeformableAlgNode::
					    pick_place_defoIsFinishedCallback,
					    this));
	pick_place_defo_aserver_.
	    registerHasSucceedCallback(boost::
				       bind(&PickandplaceDeformableAlgNode::
					    pick_place_defoHasSucceedCallback,
					    this));
	pick_place_defo_aserver_.
	    registerGetResultCallback(boost::
				      bind(&PickandplaceDeformableAlgNode::
					   pick_place_defoGetResultCallback,
					   this, _1));
	pick_place_defo_aserver_.
	    registerGetFeedbackCallback(boost::
					bind(&PickandplaceDeformableAlgNode::
					     pick_place_defoGetFeedbackCallback,
					     this, _1));
	pick_place_defo_aserver_.start();

	// [init action clients]
}

PickandplaceDeformableAlgNode::~PickandplaceDeformableAlgNode(void)
{
	// [free dynamic memory]
}

void PickandplaceDeformableAlgNode::mainNodeThread(void)
{
	// [fill msg structures]

	// [fill srv structure and make request to the server]

	// [fill action structure and make request to the action server]

	// [publish messages]

	//  
	// PickPlace_Defo_State=0 => INIT, set robot out of the camera scene for the 1st time 
	// PickPlace_Defo_State=1 => Robot initialized, ready for a new action 
	// PickPlace_Defo_State=2 => Calc all the variables to perform PandP 
	// PickPlace_Defo_State=3 => Robot performs PickAndPlace  
	// PickPlace_Defo_State=4 => Wait until the robot finish PickAndPlace action 
	// PickPlace_Defo_State=5 => Action has finished OK, robot going to INIT pose
	// PickPlace_Defo_State=8 => END OK! 
	// PickPlace_Defo_State=9 => END NOK 
	// PickPlace_Defo_State=10 => STOP 

	if (this->PickPlace_Defo_State == 0) {	//INIT

		//Move robot out of the camera
		//Wait here until the robot is ON!
		while (!My_Init_Robot_Pose()) ;
		PickPlace_Defo_State = 1;

	}
	else if (this->PickPlace_Defo_State == 2) {	//Clac and fill Vars 

		//Transformations
		tf::Transform camHgp;	//Transf. from camera to grasp point
		camHgp.setIdentity();
		camHgp.setOrigin(tf::
				 Vector3(pick_pos.X, pick_pos.Y, pick_pos.Z));

		tf::Transform camHugp;	//Transf. from camera to grasp point
		camHugp.setIdentity();
		camHugp.setOrigin(tf::Vector3
				  (place_pos.X, place_pos.Y, place_pos.Z));

		tf::Transform robotHgp;	//Transf. from robot to grasp point
		robotHgp = transform_rob_H_cam * camHgp;

		tf::Transform robotHugp;	//Transf. from robot to ungrasp point
		robotHugp = transform_rob_H_cam * camHugp;

		const float PRE_POST_GRASP_DISTANCE = 0.15;   // 15cm

		//PreGrasp point
		pick_and_place_goal_.pregrasp_point[0] =
		    robotHgp.getOrigin()[0];
		pick_and_place_goal_.pregrasp_point[1] =
		    robotHgp.getOrigin()[1];
		pick_and_place_goal_.pregrasp_point[2] =
		    robotHgp.getOrigin()[2] + PRE_POST_GRASP_DISTANCE;

		//Grasp point, with a little offset due to the calibration of the cam
		pick_and_place_goal_.grasp_point[0] = robotHgp.getOrigin()[0];
		pick_and_place_goal_.grasp_point[1] = robotHgp.getOrigin()[1];
		pick_and_place_goal_.grasp_point[2] = robotHgp.getOrigin()[2] - 0.005;

		//PostGrasp point, middle point
		point_XYZ middle_point;
		middle_point.X =
		    (robotHugp.getOrigin()[0] -
		     robotHgp.getOrigin()[0]) / 2 + robotHgp.getOrigin()[0];
		middle_point.Y =
		    (robotHugp.getOrigin()[1] -
		     robotHgp.getOrigin()[1]) / 2 + robotHgp.getOrigin()[1];
		middle_point.Z =
		    (robotHugp.getOrigin()[2] -
		     robotHgp.getOrigin()[2]) / 2 +
		    robotHgp.getOrigin()[2] + PRE_POST_GRASP_DISTANCE;

		pick_and_place_goal_.postgrasp_point[0] = middle_point.X;
		pick_and_place_goal_.postgrasp_point[1] = middle_point.Y;
		pick_and_place_goal_.postgrasp_point[2] = middle_point.Z;

		//PreGrasp point, middle point
		pick_and_place_goal_.preungrasp_point[0] = middle_point.X;
		pick_and_place_goal_.preungrasp_point[1] = middle_point.Y;
		pick_and_place_goal_.preungrasp_point[2] = middle_point.Z;

		//UnGrasp point
		pick_and_place_goal_.ungrasp_point[0] =
		    robotHugp.getOrigin()[0];
		pick_and_place_goal_.ungrasp_point[1] =
		    robotHugp.getOrigin()[1];
		pick_and_place_goal_.ungrasp_point[2] =
		    robotHugp.getOrigin()[2];

		//PostUnGrasp point
		pick_and_place_goal_.postungrasp_point[0] =
		    robotHugp.getOrigin()[0];
		pick_and_place_goal_.postungrasp_point[1] =
		    robotHugp.getOrigin()[1];
		pick_and_place_goal_.postungrasp_point[2] =
		    robotHugp.getOrigin()[2] + PRE_POST_GRASP_DISTANCE;

		const float PITCH_ANGLE = 3.14159;  // PI radians, EF pointing down

		//Initial Graping EF Orientation
		pick_and_place_goal_.ini_grasp_EF_rpy[0] = 0;
		pick_and_place_goal_.ini_grasp_EF_rpy[1] = PITCH_ANGLE;
		pick_and_place_goal_.ini_grasp_EF_rpy[2] = 0;

		//Final Graping EF Orientation
		pick_and_place_goal_.end_grasp_EF_rpy[0] = 0;
		pick_and_place_goal_.end_grasp_EF_rpy[1] = PITCH_ANGLE;
		pick_and_place_goal_.end_grasp_EF_rpy[2] = 0;

		//Initial UnGraping EF Orientation
		pick_and_place_goal_.ini_ungrasp_EF_rpy[0] = 0;
		pick_and_place_goal_.ini_ungrasp_EF_rpy[1] = PITCH_ANGLE;
		pick_and_place_goal_.ini_ungrasp_EF_rpy[2] = 0;

		//Final UnGraping EF Orientation
		pick_and_place_goal_.end_ungrasp_EF_rpy[0] = 0;
		pick_and_place_goal_.end_ungrasp_EF_rpy[1] = PITCH_ANGLE;
		pick_and_place_goal_.end_ungrasp_EF_rpy[2] = 0;

		//Change state
		if (PickPlace_Defo_State == 2)
			PickPlace_Defo_State = 3;

	}
	else if (this->PickPlace_Defo_State == 3) {	//Robot Starts PickAndPlace 

		pick_and_placeMakeActionRequest();
		if (PickPlace_Defo_State == 3)
			PickPlace_Defo_State = 4;

	}
	else if (this->PickPlace_Defo_State == 5) {	//Robot go to init pos 

		My_Init_Robot_Pose();
		PickPlace_Defo_Result = 1;
		if (PickPlace_Defo_State == 5)
			PickPlace_Defo_State = 8;

	}
	else if (this->PickPlace_Defo_State == 10) {	//STOP

		pick_and_place_client_.cancelGoal();
		My_Init_Robot_Pose();
		PickPlace_Defo_State = 9;

	}

}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */
void PickandplaceDeformableAlgNode::
pick_place_defoStartCallback(const
			     iri_pickandplace_deformable::
			     PaPDeformableGoalConstPtr & goal)
{
	alg_.lock();
	//check goal 
	//execute goal 
	if (PickPlace_Defo_State == 1 || PickPlace_Defo_State == 9) {

		PickPlace_Defo_Result = 0;

		pick_pos.X = goal->pick_pos[0];
		pick_pos.Y = goal->pick_pos[1];
		pick_pos.Z = goal->pick_pos[2];

		place_pos.X = goal->place_pos[0];
		place_pos.Y = goal->place_pos[1];
		place_pos.Z = goal->place_pos[2];

		yaw = goal->yaw;

		PickPlace_Defo_State = 2;

	}
	alg_.unlock();
}

void PickandplaceDeformableAlgNode::pick_place_defoStopCallback(void)
{
	alg_.lock();
	//stop action 
	if (PickPlace_Defo_State == 4)
		PickPlace_Defo_State = 10;

	alg_.unlock();
}

bool PickandplaceDeformableAlgNode::pick_place_defoIsFinishedCallback(void)
{
	bool ret = false;

	alg_.lock();
	//if action has finish for any reason 
	if (PickPlace_Defo_State == 8 || PickPlace_Defo_State == 9) {
		PickPlace_Defo_State = 0;
		ret = true;
	}
	alg_.unlock();

	return ret;
}

bool PickandplaceDeformableAlgNode::pick_place_defoHasSucceedCallback(void)
{
	bool ret = false;

	alg_.lock();
	//if goal was accomplished 
	ret = PickPlace_Defo_Result;
	alg_.unlock();

	return ret;
}

void PickandplaceDeformableAlgNode::
pick_place_defoGetResultCallback(iri_pickandplace_deformable::
				 PaPDeformableResultPtr & result)
{
	alg_.lock();
	//update result data to be sent to client 
	//result->data = data; 
	result->successful = PickPlace_Defo_Result;
	alg_.unlock();
}

void PickandplaceDeformableAlgNode::
pick_place_defoGetFeedbackCallback(iri_pickandplace_deformable::
				   PaPDeformableFeedbackPtr & feedback)
{
	alg_.lock();
	//keep track of feedback 
	feedback->PaPDeformable_state = PickPlace_Defo_State;
	feedback->PandP_state = Pick_and_Place_Feedback;
	feedback->PorP_state = Pick_or_Place_Feedback;
	//ROS_INFO("feedback: %s", feedback->data.c_str()); 
	alg_.unlock();
}

void PickandplaceDeformableAlgNode::
pick_and_placeDone(const actionlib::SimpleClientGoalState & state,
		   const
		   iri_wam_generic_pickandplace::PickAndPlaceResultConstPtr &
		   result)
{
	alg_.lock();
	if (state.toString().compare("SUCCEEDED") == 0) {
		ROS_INFO
		    ("PickandplaceDeformableAlgNode::pick_and_placeDone: Goal Achieved!");
		//Change State
		if (PickPlace_Defo_State == 4)
			PickPlace_Defo_State = 5;
	}
	else {
		ROS_INFO
		    ("PickandplaceDeformableAlgNode::pick_and_placeDone: %s",
		     state.toString().c_str());
		PickPlace_Defo_State = 9;
	}

	//copy & work with requested result 
	alg_.unlock();
}

void PickandplaceDeformableAlgNode::pick_and_placeActive()
{
	alg_.lock();
	//ROS_INFO("PickandplaceDeformableAlgNode::pick_and_placeActive: Goal just went active!"); 
	alg_.unlock();
}

void PickandplaceDeformableAlgNode::
pick_and_placeFeedback(const
		       iri_wam_generic_pickandplace::
		       PickAndPlaceFeedbackConstPtr & feedback)
{
	alg_.lock();
	//ROS_INFO("PickandplaceDeformableAlgNode::pick_and_placeFeedback: Got Feedback!"); 

	bool feedback_is_ok = true;

	//analyze feedback 
	//my_var = feedback->var; 
	Pick_and_Place_Feedback = feedback->PandP_state;
	Pick_or_Place_Feedback = feedback->PorP_state;

	//if feedback is not what expected, cancel requested goal 
	if (!feedback_is_ok) {
		pick_and_place_client_.cancelGoal();
		//ROS_INFO("PickandplaceDeformableAlgNode::pick_and_placeFeedback: Cancelling Action!"); 
	}
	alg_.unlock();
}

/*  [action requests] */
void PickandplaceDeformableAlgNode::pick_and_placeMakeActionRequest()
{
	ROS_INFO
	    ("PickandplaceDeformableAlgNode::pick_and_placeMakeActionRequest: Starting New Request!");

	//wait for the action server to start 
	//will wait for infinite time 
	pick_and_place_client_.waitForServer();
	ROS_INFO
	    ("PickandplaceDeformableAlgNode::pick_and_placeMakeActionRequest: Server is Available!");

	//send a goal to the action 
	//pick_and_place_goal_.data = my_desired_goal; 
	pick_and_place_client_.sendGoal(pick_and_place_goal_,
					boost::bind
					(&PickandplaceDeformableAlgNode::
					 pick_and_placeDone, this, _1, _2),
					boost::
					bind(&PickandplaceDeformableAlgNode::
					     pick_and_placeActive, this),
					boost::
					bind(&PickandplaceDeformableAlgNode::
					     pick_and_placeFeedback, this, _1));
	ROS_INFO
	    ("PickandplaceDeformableAlgNode::pick_and_placeMakeActionRequest: Goal Sent. Wait for Result!");
}

void PickandplaceDeformableAlgNode::node_config_update(Config & config,
						       uint32_t level)
{
	this->alg_.lock();

	this->alg_.unlock();
}

void PickandplaceDeformableAlgNode::addNodeDiagnostics(void)
{
}

/* Set Robot to the INIT pose FUNCTION */
bool PickandplaceDeformableAlgNode::My_Init_Robot_Pose()
{

	//Set the robot pose MOVE TO 0,0,0,2,0,0,0
	wam_joints_pose_srv_.request.positions.resize(7);
	wam_joints_pose_srv_.request.positions[0] = 0.0;
	wam_joints_pose_srv_.request.positions[1] = 0.0;
	wam_joints_pose_srv_.request.positions[2] = 0.0;
	wam_joints_pose_srv_.request.positions[3] = 2.0;
	wam_joints_pose_srv_.request.positions[4] = 0.0;
	wam_joints_pose_srv_.request.positions[5] = 0.0;
	wam_joints_pose_srv_.request.positions[6] = 0.0;
	wam_joints_pose_srv_.request.velocity = 1.0;
	wam_joints_pose_srv_.request.acceleration = 1.0;

	//Move robot out of the camera
	ROS_INFO
	    ("TestGraspDeformableAlgNode:: Sending INITIAL POSE to the robot!");
	if (wam_joints_pose_client_.call(wam_joints_pose_srv_)) {
		return true;
	}
	else {
		ROS_INFO
		    ("TestGraspDeformableAlgNode:: Failed to Call Server on topic wam_joints_pose ");
		return false;
	}

}

/* main function */
int main(int argc, char *argv[])
{
	return algorithm_base::main < PickandplaceDeformableAlgNode > (argc,
								       argv,
								       "pickandplace_deformable_alg_node");
}
