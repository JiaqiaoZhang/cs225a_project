// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

// regular panda + gripper
// const string robot_file = "./resources/panda_arm_hand.urdf";
// panda + mobile base + gripper
//spatula
const string robot_file = "./resources/mmp_panda.urdf";
// const string spatula_file = "./resources/spatula.urdf";
// //burger
const string burger_file = "./resources/burger.urdf";
const string burger_name = "burger"; 
// bun 1
const string bottom_bun_file = "./resources/bottom_bun.urdf";
const string bottom_bun_name = "bot_bun"; 
// // bun 2
const string top_bun_file = "./resources/top_bun.urdf";
const string top_bun_name = "top_bun"; 
// cheese
// tomato
// lettuce

// states
#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1
// tasks
#define IDLE                  0
#define SPATULA_PRE_POS       1
#define SPATULA_GRASP_POS	  2
#define SLIDE		  		  3
#define LIFT_SPATULA		  4
#define DROP_FOOD			  5
#define RELAX_WRIST			  6
#define FLEX_WRIST			  7
#define RESET			      8
#define ALIGN                 9
#define PLATE                10
// gripper states
#define OPEN                  0
#define CLOSED                1
// stations
#define STATION_1             1
#define STATION_2             2

// print
#define VERBOSE				  0

int state = JOINT_CONTROLLER;
int task = SPATULA_PRE_POS;
int station = STATION_2;
int gripper_state = OPEN;
// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
std::string SPATULA_POSITION_KEY;
std::string SPATULA_ORIENTATION_KEY;
std::string SPATULA_JOINT_ANGLES_KEY;
std::string BURGER_POSITION_KEY;
std::string BURGER_ORIENTATION_KEY;
std::string BURGER_JOINT_ANGLES_KEY;
std::string BOTTOM_BUN_POSITION_KEY;
std::string TOP_BUN_POSITION_KEY;
std::string BOTTOM_BUN_TORQUES_COMMANDED_KEY;
std::string BURGER_TORQUES_COMMANDED_KEY;
std::string TOP_BUN_TORQUES_COMMANDED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

unsigned long long controller_counter = 0;
int relax_counter = 0;
const bool inertia_regularization = true;


int main() {

	JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
	JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
	JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";
	// KEY FOR SPATULA POSITION
	SPATULA_POSITION_KEY = "sai2::cs225a::spatula::sensors::r_spatula";
	SPATULA_ORIENTATION_KEY = "sai2::cs225a::spatula::sensors::ori_spatula";
	SPATULA_JOINT_ANGLES_KEY = "sai2::cs225a::spatula::sensors::spatula_q";
	// KEY POSITION FOR BURGER
	BURGER_POSITION_KEY = "sai2::cs225a::burger::sensors::r_burger";
	TOP_BUN_POSITION_KEY = "sai2::cs225a::top_bun::sensors::r_top_bun";
	// KEY POSITION FOR BUNS
	BOTTOM_BUN_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::bottom_bun";

	BOTTOM_BUN_POSITION_KEY = "sai2::cs225a::bottom_bun::sensors::r_bottom_bun";
	BURGER_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::burger";
	TOP_BUN_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::top_bun";

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	// cout << initial_q << endl << endl;
	robot->updateModel();
	//----------------------------------------***** KITCHEN FOOD ROBOTS *****-----------------------------------------------
	auto bottom_bun = new Sai2Model::Sai2Model(bottom_bun_file, false);
	Vector3d r_bottom_bun = redis_client.getEigenMatrixJSON(BOTTOM_BUN_POSITION_KEY);
	VectorXd bottom_bun_command_torques = VectorXd::Zero(6);

	auto burger = new Sai2Model::Sai2Model(burger_file, false);
	Vector3d r_burger = redis_client.getEigenMatrixJSON(BURGER_POSITION_KEY);
	VectorXd burger_command_torques = VectorXd::Zero(6);

	auto top_bun = new Sai2Model::Sai2Model(top_bun_file, false);
	Vector3d r_top_bun = redis_client.getEigenMatrixJSON(TOP_BUN_POSITION_KEY);
	VectorXd top_bun_command_torques = VectorXd::Zero(6);


	// for (int i=0; i<3; i++)
	// {
	// 	bottom_bun->_q(i)=r_bottom_bun(i);

	// }
	bottom_bun->updateModel();
	bool bottom_bun_actuate = false;
	MatrixXd N_bottom_bun = MatrixXd::Identity(6, 6);

	burger->updateModel();
	bool burger_actuate = false;
	MatrixXd N_burger = MatrixXd::Identity(6, 6);

	top_bun->updateModel();
	bool top_bun_actuate = false;
	MatrixXd N_top_bun = MatrixXd::Identity(6, 6);

	// auto bottom_bun = new Sai2Model::Sai2Model(bottom_bun_file, false);
	// Vector3d r_bottom_bun = redis_client.getEigenMatrixJSON(BOTTOM_BUN_POSITION_KEY);
	// for (int i=0; i<3; i++)
	// {
	// 	bottom_bun->_q(i)=r_bottom_bun(i);
	// }

	// auto bottom_bun = new Sai2Model::Sai2Model(bottom_bun_file, false);
	// Vector3d r_bottom_bun = redis_client.getEigenMatrixJSON(BOTTOM_BUN_POSITION_KEY);
	// for (int i=0; i<3; i++)
	// {
	// 	bottom_bun->_q(i)=r_bottom_bun(i);
	// }
	// bool food_actuate[] = {bottom_bun_actuate};
	// use plate food index

	auto bottom_bun_task = new Sai2Primitives::JointTask(bottom_bun);
	bottom_bun_task->_kp = 80.0;
	bottom_bun_task->_kv = 50.0;

	auto burger_task = new Sai2Primitives::JointTask(burger);
	burger_task->_kp = 80.0;
	burger_task->_kv = 50.0;

	auto top_bun_task = new Sai2Primitives::JointTask(top_bun);
	top_bun_task->_kp = 75.0;
	top_bun_task->_kv = 50.0;

	bool food_actuate[] = {bottom_bun_actuate, burger_actuate, top_bun_actuate};
	MatrixXd N_food[] = {N_bottom_bun, N_burger, N_top_bun};
	Sai2Primitives::JointTask * food_task[] = {bottom_bun_task, burger_task, top_bun_task};
	Sai2Model::Sai2Model * food_robot[] = {bottom_bun, burger, top_bun};
	// use plate_index

	//----------------------------------------***** KITCHEN FOOD ROBOTS *****-----------------------------------------------
	
	// from world urdf
	Vector3d base_origin;
	base_origin << 0.0, -0.05, 0.3514;

	Vector3d spatula_handle_pre_grasp_local;
	spatula_handle_pre_grasp_local << -0.35, 0, 0.1;
	// spatula_handle_pre_grasp_local << -0.4, 0, 0.27;

	Vector3d spatula_handle_grasp_local;
	spatula_handle_grasp_local << -0.25, 0, 0.04;
	// spatula_handle_grasp_local << -0.25, 0, 0.184;

	Vector3d base_offset;
	base_offset << 0.0, 0.0, 0.1757;
	// base_offset << 0.0, -0.05, 0.3514;

	Matrix3d handle_rot_local;
	handle_rot_local << -0.3553997, -0.3516974,  0.8660254,
  						-0.7033947,  0.7107995,  0.0000000,
  						-0.6155704, -0.6091577, -0.5000000; 

  	double finger_rest_pos = 0.02;
	double finger_closed_pos = -0.01;
 	// spatula
	Vector3d r_spatula = Vector3d::Zero();
	Matrix3d ori_spatula = Matrix3d::Zero();
	VectorXd spatula_q(6);
	r_spatula = redis_client.getEigenMatrixJSON(SPATULA_POSITION_KEY);	
	Vector3d r_spatula_init = r_spatula;
	ori_spatula = redis_client.getEigenMatrixJSON(SPATULA_ORIENTATION_KEY);	
	Vector3d del_r; // r_spatula - r_end_effector
	Matrix3d ori_spatula_level = ori_spatula;
	// spatula_q = redis_client.getEigenMatrixJSON(SPATULA_JOINT_ANGLES_KEY);	
	// burger

	// bun 1
	// bun 2
	// cheese	
	// tomato
	// lettuce
	
	int grill_index = 0;
	int plate_index = 0;
	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0,0,0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

#ifdef USING_OTG
	posori_task->_use_interpolation_flag = true;
#else
	posori_task->_use_velocity_saturation_flag = true;
#endif
	
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 200.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 20.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
	joint_task->_use_interpolation_flag = true;
#else
	joint_task->_use_velocity_saturation_flag = true;
#endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 200.0;
	joint_task->_kv = 40.0;


	VectorXd q_init_desired = initial_q;
	joint_task->_desired_position = q_init_desired;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	Matrix3d good_ee_rot;
	good_ee_rot << 0.703586,  -0.710608, -0.0017762,
					-0.337309,  -0.336174,   0.879324,
					-0.625451,  -0.618081,  -0.476221;

	Vector3d slide;
	slide << 0.0, 0.25, 0.0;
	// slide << 0.0, 0.28, 0.02;
	Matrix3d slide_ori;
	double slide_angle = -6 * M_PI / 180.0;
	slide_ori << 	1.0000000, 0.0000000,  		 0.0000000,
   					0.0000000, cos(slide_angle), -sin(slide_angle),
   					0.0000000, sin(slide_angle), cos(slide_angle);

	slide_ori *= good_ee_rot;

   	double y_slide = 0.44;  // based off of backstop location and thickness

   	Matrix3d lift_ori;
	double lift_angle = 20 * M_PI / 180.0;
	// double lift_angle = 6 * M_PI / 180.0;

	lift_ori << 	1.0000000, 0.0000000,  		 0.0000000,
   					0.0000000, cos(lift_angle), -sin(lift_angle),
   					0.0000000, sin(lift_angle), cos(lift_angle);
	lift_ori *= good_ee_rot;
	// Vector3d lift_height;
	// lift_height << 0.0, 0.05, 0.25;
	double z_lift = 0.6;
	Vector3d drop_food;
	drop_food << 0.0, 0.25, 0.53;
	Matrix3d relax_ori;
	double relax_angle = -30 * M_PI / 180.0;
	// relax_ori = lift_ori.transpose();
	relax_ori << 	1.0000000, 0.0000000,  		 0.0000000,
					0.0000000, cos(relax_angle), -sin(relax_angle),
					0.0000000, sin(relax_angle), cos(relax_angle);
	relax_ori *= good_ee_rot;

	Matrix3d flex_ori;
	// double flex_angle = 0 * M_PI / 180.0;
	flex_ori = relax_ori.transpose();
	
	Vector3d plate_food;
	plate_food << -0.45, 0.5-0.221, 0.48;
	// plate_food << -0.415193+0.02, 0.481433-0.21, 0.53;
	
	double y_offset_tip = 0.051;  // according to onshape - distance between spatula origin and front of spatula base
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		r_spatula = redis_client.getEigenMatrixJSON(SPATULA_POSITION_KEY);		
		ori_spatula = redis_client.getEigenMatrixJSON(SPATULA_ORIENTATION_KEY);

		r_bottom_bun = redis_client.getEigenMatrixJSON(BOTTOM_BUN_POSITION_KEY);
		r_top_bun = redis_client.getEigenMatrixJSON(TOP_BUN_POSITION_KEY);
		r_burger = redis_client.getEigenMatrixJSON(BURGER_POSITION_KEY);

		Vector3d grill_foods[] = {r_burger, r_bottom_bun, r_top_bun};
		Vector3d foods[] = {r_bottom_bun, r_burger, r_top_bun};
		int plate_shift[] = {1, 0, 2};
		// update model
		robot->updateModel();
		
		VectorXd q_curr_desired(12);
		q_curr_desired = robot->_q;

		Vector3d ee_pos;
		robot->positionInWorld(ee_pos, control_link, control_point);
		Matrix3d ee_rot;
		robot->rotationInWorld(ee_rot, control_link);

		if(state == JOINT_CONTROLLER)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			joint_task->_use_velocity_saturation_flag = false;

			// if(task == IDLE)
			// {
			// 	q_curr_desired = q_init_desired;
			// }

			if(station == STATION_1) 
			{
				q_curr_desired(0) = -0.3514;
				joint_task->_use_velocity_saturation_flag = true;
				joint_task->_saturation_velocity(0) = 0.2;
			}
	
			if(station == STATION_2)
			{
				q_curr_desired(0) = 0.3514;
				joint_task->_use_velocity_saturation_flag = true;
				joint_task->_saturation_velocity(0) = 0.2;
			}
			
			if(gripper_state == OPEN)
			{
				q_curr_desired(10) = finger_rest_pos;
				q_curr_desired(11) = -finger_rest_pos;
			}
			if(gripper_state == CLOSED)
			{
				q_curr_desired(10) = finger_closed_pos;
				q_curr_desired(11) = -finger_closed_pos;
			}




			joint_task->_desired_position = q_curr_desired;
			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;

			if( (robot->_q - q_curr_desired).norm() < 0.05 )
			{
				if (task == SPATULA_PRE_POS) {
					state = POSORI_CONTROLLER;
				}
				if (task == SPATULA_GRASP_POS) {
					state = POSORI_CONTROLLER;
					task = SLIDE;

				}
				if (station == STATION_1 && task == LIFT_SPATULA) {
					state = POSORI_CONTROLLER;
					cout << "Dropping food on grill..." << endl << endl;
					task = DROP_FOOD;
					posori_task->reInitializeTask();
					posori_task->_desired_position = drop_food;
					posori_task->_desired_position(0) += (0.11 * grill_index);
				}

				if (task == SLIDE) 
				{
					cout << "Sliding..." << endl << endl;
					posori_task->reInitializeTask();
					posori_task->_use_velocity_saturation_flag = true;
					posori_task->_linear_saturation_velocity = 0.3;
					posori_task->_desired_position(1) = y_slide;
					posori_task->_desired_orientation = slide_ori; 
				}
				else if (task == RESET)
				{
					// grill_index++;
					if (grill_index < 3)
					{
						cout << "Aligning..." << endl << endl;
						task = ALIGN;
						state = POSORI_CONTROLLER;
						posori_task->reInitializeTask();
						
						Vector3d r_food = grill_foods[grill_index];
						cout << "Current Food..." << grill_index << endl << endl;
						posori_task->_desired_orientation = good_ee_rot;
					}
					else 
					{
						cout << "FUCKED UP";
						task = IDLE;
					}
				}
			}
		}
//-----------------------------------------***** POSORI CONTROLLER *****--------------------------------------------------------------
		else if(state == POSORI_CONTROLLER)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
	
			// FIX BASE
			joint_task->_use_velocity_saturation_flag = true;
			joint_task->_saturation_velocity(0) = 0.0;
			joint_task->_saturation_velocity(1) = 0.0;
			if(gripper_state == OPEN)
			{
				q_curr_desired(10) = finger_rest_pos;
				q_curr_desired(11) = -finger_rest_pos;
			}
			if(gripper_state == CLOSED)
			{
				q_curr_desired(10) = finger_closed_pos;
				q_curr_desired(11) = -finger_closed_pos;
			}
			joint_task->_desired_position = q_curr_desired;
			joint_task->updateTaskModel(posori_task->_N);

			if (task == SPATULA_PRE_POS)
			{
				// cout << "Pre" << endl << endl;
				// need to maintain the finger position while moving to the spatula
				posori_task->reInitializeTask();
				// want this to be spatula position + local vector * local to world rotation
				posori_task->_desired_position = r_spatula + ori_spatula.transpose() * spatula_handle_pre_grasp_local - base_offset;
				// go to spatula position
				posori_task->_desired_orientation = ori_spatula.transpose() * handle_rot_local;
			} 
			else if (task == SPATULA_GRASP_POS) 
			{
				// need to maintain the finger position while moving to the spatula
				posori_task->reInitializeTask();
				// want this to be spatula position + local vector * local to world rotation
				posori_task->_desired_position = r_spatula + ori_spatula.transpose() * spatula_handle_grasp_local - base_offset;
				posori_task->_desired_orientation = ori_spatula.transpose() * handle_rot_local;

			} 
			else if (task == ALIGN)
			{
				Vector3d r_align;
				if (grill_index < 3)
				{
					// Vector3d r_food = grill_foods[grill_index];					
					// r_align(0) = r_food(0) - ((r_food(0) - r_spatula(0))/2) + 0.04*grill_index;
					// r_align(1) = r_food(1) - 0.45;
					// r_align(2) = r_food(2) + (r_food(2)-r_spatula(2)) - 0.1;

					// // new
					Vector3d robot_offset = Vector3d(0.0, -0.05, 0.3514);
					Vector3d r_food = grill_foods[grill_index];		
					r_align = r_food - robot_offset;	
					double sim_offset = 0.005;
					r_align(1) -= y_slide; 
					r_align(2) += 0.11683695 + (0.17 - 0.107) * cos(30 * M_PI / 180) + sim_offset;
				}

				else if (plate_index < 3)
				{
					// Vector3d r_food = foods[plate_index];
					// r_align(0) = r_food(0) - ((r_food(0) - r_spatula(0))/2) - 0.3*plate_shift[plate_index];
					// r_align(1) = r_food(1) - 0.45;
					// r_align(2) = r_food(2) + (r_food(2)-r_spatula(2)) - 0.1 + (0.0254);

					Vector3d robot_offset = Vector3d(0.0, -0.05, 0.3514);
					Vector3d r_food = foods[plate_index];		
					r_align = r_food - robot_offset;	
					double sim_offset = 0.005;
					r_align(1) -= y_slide; 
					r_align(2) += 0.11683695 + (0.17 - 0.107) * cos(30 * M_PI / 180) + sim_offset;
				}
				posori_task->_desired_position = r_align;
				posori_task->_desired_orientation = good_ee_rot;
			}
			
			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques;
			
			// if we have reached the desired position and orientation
			if(posori_task->goalPositionReached(0.01) && posori_task->goalOrientationReached(0.05))
			{
				// if we have moved into the pre-grasp position (essentially you position slightly away from the spatula to not contact it)
				// else if we have moved to a position with the spatula handle between the jaws of the gripper
				if (task == SPATULA_PRE_POS)
				{
					cout << "Moving to Grasp Position" << endl << endl;
					// move inwards to the grasp position
					task = SPATULA_GRASP_POS;					
				} 
				else if (task == SPATULA_GRASP_POS) 
				{
					cout << "Closing Gripper..." << endl << endl;
					// if we are in the grasp position, go to a joint task and close the jaws
					state = JOINT_CONTROLLER;
					gripper_state = CLOSED;
				} 
				else if (task == SLIDE) 
				{
					// state = POSORI_CONTROLLER;
					cout << "Lifting..." << endl << endl;
					posori_task->reInitializeTask();
					posori_task->_use_velocity_saturation_flag = true;
					posori_task->_linear_saturation_velocity = 0.1;
					task =  LIFT_SPATULA;
					// z_lift<Matrix>(posori_task->_desired_position, posori_task->_desired_orientation);
					posori_task->_desired_position(2) = z_lift;
					posori_task->_desired_orientation = lift_ori;
				} 
				else if (task == LIFT_SPATULA) 
				{
					if (grill_index < 3)
					{
						state = JOINT_CONTROLLER;
						cout << "Changing station..." << endl << endl;
						joint_task->reInitializeTask();
						station = STATION_1;
					}
					else if (plate_index < 3)
					{
						state = POSORI_CONTROLLER;
						cout << "Plating food #" << plate_index << " ..." << endl << endl;
						task = PLATE;
						posori_task->reInitializeTask();
						posori_task->_desired_position(0) = plate_food(0);
						posori_task->_desired_position(1) = plate_food(1);
						posori_task->_desired_position(2) = plate_food(2) + (0.0254*plate_index);
					}
					
				}
				else if (task == DROP_FOOD)
				{
					cout << "\t(Relaxing wrist...)" << endl << endl;
					task = RELAX_WRIST;
					posori_task->reInitializeTask();
					// state = POSORI_CONTROLLER;
					posori_task->_desired_orientation = relax_ori;
				}
				else if (task == PLATE)
				{
					cout << "\t(Relaxing wrist...)" << endl << endl;
					task = RELAX_WRIST;
					posori_task->reInitializeTask();
					state = POSORI_CONTROLLER;
					posori_task->_desired_orientation = relax_ori;
				}
				else if (task == RELAX_WRIST)
				{
					// start counter for relaxing					
					if(relax_counter < 1500) 
					{
						relax_counter++;
					} 
					else 
					{
						cout << "\t(Flexing wrist...)" << endl << endl;
						task = FLEX_WRIST;
						posori_task->reInitializeTask();
						
						// posori_task->_desired_orientation *= flex_ori;
						posori_task->_desired_orientation=good_ee_rot;
						relax_counter=0;
					}
				}
				else if (task == FLEX_WRIST)
				{
					if(grill_index < 3) 
					{
						grill_index++;
					} else if (plate_index <3) 
					{
						food_actuate[plate_index] = true;
						plate_index++;
					}

					if(grill_index < 3)
					{
						state = JOINT_CONTROLLER;
						task = RESET;
						cout << "Moving to initial station..." << endl << endl;
						joint_task->reInitializeTask();
						station = STATION_2;
					}
					else if (plate_index < 3)
					{
						state = POSORI_CONTROLLER;
						cout << "Aligning for plate#" << plate_index << "..." << endl << endl;
						task = ALIGN;
						posori_task->reInitializeTask();
						//if(plate_index == 1){
						//	bottom_bun_actuate = true;
						//}
						
					}
					else if (plate_index == 3)
					{
						state = JOINT_CONTROLLER;
						task = IDLE;
						// food_actuate[plate_index-1] = true;
					}
				}
				else if (task == ALIGN)
				{
					task = SLIDE;
					if (grill_index < 3)
					{
						cout << "Sliding for Grill Food " << grill_index << "..." << endl << endl;
					} 
					else if (plate_index < 3)
					{
						cout << "Sliding for Plate Food " << plate_index << "..." << endl << endl;
					}						
					posori_task->reInitializeTask();
					posori_task->_use_velocity_saturation_flag = true;
					posori_task->_linear_saturation_velocity = 0.3;
					
					posori_task->_desired_position(1) = y_slide;
					posori_task->_desired_orientation = slide_ori; 

				}


			} // goal reached if-statement
		}// posori if-statement
//-----------------------------------------------*******STACKING FOOD CONTROL********---------------------------------------------------------
		// //if(food_actuate[plate_index])
		for(int f = 0; f < 3; f++)
		{
			//if(bottom_bun_actuate)

			if(food_actuate[f] == true)
			{
				Sai2Primitives::JointTask * curr_food_task;
				curr_food_task = food_task[f];

				N_bottom_bun.setIdentity();
				curr_food_task->updateTaskModel(N_bottom_bun);

				Vector3d q_food_desired;

				if(f == 0)
				{
					q_food_desired(0) = 0.458 + (f * 0.027);
					q_food_desired(1) = 0.5+0.01;
					q_food_desired(2) = -0.45;
				}

				else if(f==1 or f==2)
				{
					q_food_desired(0) = food_robot[f-1]->_q(0) + 0.027;
					q_food_desired(1) = food_robot[f-1]->_q(1);
					q_food_desired(2) = food_robot[f-1]->_q(2);
				}			

				Vector3d r_food;
				r_food = foods[f];
				food_robot[f]->_q(0) = r_food(2);
				food_robot[f]->_q(1) = r_food(1);
				food_robot[f]->_q(2) = r_food(0);

				for(int i = 0; i < 3; i++)
				{
					curr_food_task->_desired_position(i) = q_food_desired(i);
				}

				Eigen::VectorXd g_food(6);
				g_food << 9.81, 0, 0, 0, 0, 0;
				g_food *= 0.173;
				
				if(f == 0)
				{
					curr_food_task->computeTorques(bottom_bun_command_torques);
					redis_client.setEigenMatrixJSON(BOTTOM_BUN_TORQUES_COMMANDED_KEY, bottom_bun_command_torques + g_food);
					if(controller_counter % 10000 == 0){
					cout << "bottom_bun_actuate = " << bottom_bun_actuate << "... bottom_bun_command_torques = " << bottom_bun_command_torques.transpose() << endl << endl;
					}
				}

				else if(f == 1)
				{
					curr_food_task->computeTorques(burger_command_torques);
					redis_client.setEigenMatrixJSON(BURGER_TORQUES_COMMANDED_KEY, burger_command_torques + g_food);
					if(controller_counter % 10000 == 0){
					cout << "burger_actuate = " << burger_actuate << "... burger_command_torques = " << burger_command_torques.transpose() << endl << endl;
					}
				}
				else if(f == 2)
				{
					curr_food_task->computeTorques(top_bun_command_torques);
					redis_client.setEigenMatrixJSON(TOP_BUN_TORQUES_COMMANDED_KEY, top_bun_command_torques + g_food);
					if(controller_counter % 10000 == 0){
					cout << "top_bun_actuate = " << top_bun_actuate << "... top_bun_command_torques = " << top_bun_command_torques.transpose() << endl << endl;
					}
				}
			}
			
			
		}
			
//-----------------------------------------------*******STACKING FOOD CONTROL********---------------------------------------------------------
		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
