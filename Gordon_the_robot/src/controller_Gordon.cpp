#include <Sai2Model.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>
using namespace std;
using namespace Eigen;

bool runloop = true;
void sighandler(int sig)
{
	runloop = false;
}
/*
Define tasks: idle, align, slide, lift_spatula, drop_food, flip_food, reset
Define states: state1, state2(assume task2 is one assembled object), state3(serve food)
Set task as idle
Set initial grill_index to 1
Set initial flipping_index to 1
Set flip flag to false
Set time as global counting variable
while(robot visualization is on and data is transmitting){
Receive robot and sensor data from redis
Start counting time
if(time is passed 5 seconds){
Set task to first task “align”
}
If( state is state1 for preparing):{
Update posori_controller
If(task is to align){
align the spatula to the object[grill_index]
if (position and ori reached){change task to slide}
}
Else if(task is to slide){
slide the spatula under the object[grill_index]
if (position and ori reached){change task to lift_spatula}
}
Else if(task is to lift_spatula){
hold the object[grill_index] and lift
if (position and ori reached){change task to drop_food}
}
Else if(task is to drop_food){
Drop the object[grill_index] on cutting board
if (position and ori reached){
If (grill object is less than 3){
change task to align
Grill_index ++
Else{
change state to state2,
change task to align
change the object into assembled one}
}
reset
}
}
Else If(state is state2 for flipping):{Update posori_controller
If(task is to align){
align the spatula to the assembled object
if (position and ori reached){change task to slide}
}
Else if(task is to slide){
slide the spatula under the assembled object
if (position and ori reached){change task to lift_spatula}
}
Else if(task is to lift_spatula){
hold the assembled object and lift
if (position and ori reached){change task to flip_food}
}
Else if (task is to flip_food){
Flip the assembled object
if (position and ori reached){
change task to drop_food
}
}
Else if(task is to drop_food){
drop the assembled object
if (position and ori reached){
wait a few minutes for cooking
change state to state 3
}
reset
}
}
Else if(state = state3){
Update posori_controller
If(task is to align){
align the spatula to the assembled object
if (position and ori reached){change task to slide}
}
Else if(task is to slide){
slide the spatula under the assembled object
if (position and ori reached){change task to lift_spatula}
}
Else if(task is to lift_spatula){
hold the assembled object and lift
if (position and ori reached){change task to drop_food}
}
Else if(task is to drop_food){
Drop the object[grill_index] on dishif (position and ori reached){
Change task to reset
}
}
}
*/

// tasks
#define IDLE 0
#define ALIGN 1
#define SLIDE 2
#define LIFT_SPATULA 3
#define DROP_FOOD 4
#define FLIP_FOOD 5
#define RESET_TASK 6
#define MOVE_TO_BOARD 12
#define MOVE_TO_GRILL 13
#define MOVE_TO_CORNER 14

// states
#define STACKING 7
#define FLIPPING 8
#define SERVING 9

// base
#define STACK_BASE 10
#define GRILL_BASE 11

const std::string JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";
// KEY FOR SPATULA POSITION
// const std::string SPATULA_POSITION_KEY = "sai2::cs225a::spatula::sensors::r_spatula";
// const std::string SPATULA_ORIENTATION_KEY = "sai2::cs225a::spatula::sensors::ori_spatula";
// const std::string SPATULA_JOINT_ANGLES_KEY = "sai2::cs225a::spatula::sensors::spatula_q";
// KEY POSITION FOR BURGER
const std::string BURGER_POSITION_KEY = "sai2::cs225a::burger::sensors::r_burger";
const std::string TOP_BREAD_POSITION_KEY = "sai2::cs225a::top_bun::sensors::r_top_bun";
// KEY POSITION FOR BUNS
const std::string BOTTOM_BREAD_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::bottom_bun";

const std::string BOTTOM_BREAD_POSITION_KEY = "sai2::cs225a::bottom_bun::sensors::r_bottom_bun";
const std::string BURGER_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::burger";
const std::string TOP_BREAD_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::top_bun";

const std::string NEW_OBJECT_KEY = "sai2::cs225a::project::new_object";

//robot
const std::string robot_file = "./resources/mmp_panda.urdf";

// //burger
const std::string burger_file = "./resources/burger.urdf";
const std::string burger_name = "burger";
// bun 1
const std::string bottom_bread_file = "./resources/bottom_bun.urdf";
const std::string bottom_bread_name = "bot_bun";
// // bun 2
const std::string top_bread_file = "./resources/top_bun.urdf";
const std::string top_bread_name = "top_bun";

unsigned long long controller_counter = 0;
int main()
{
	int task = ALIGN;
	int state = STACKING;
	int base = STACK_BASE;
	int stack_idx = 2;
	int flipping_idx = 0;
	bool flip_flag = false;
	unsigned long long counter = 0;
	const int object_nums = 3;

	string switch_food_flag = "false";

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

	robot->updateModel();
	//----------------------------------------***** KITCHEN FOOD ROBOTS *****-----------------------------------------------
	auto bottom_bread = new Sai2Model::Sai2Model(bottom_bread_file, false);
	Vector3d r_bottom_bread = redis_client.getEigenMatrixJSON(BOTTOM_BREAD_POSITION_KEY);
	VectorXd bottom_bread_command_torques = VectorXd::Zero(6);

	auto burger = new Sai2Model::Sai2Model(burger_file, false);
	Vector3d r_burger = redis_client.getEigenMatrixJSON(BURGER_POSITION_KEY);
	VectorXd burger_command_torques = VectorXd::Zero(6);

	auto top_bread = new Sai2Model::Sai2Model(top_bread_file, false);
	Vector3d r_top_bread = redis_client.getEigenMatrixJSON(TOP_BREAD_POSITION_KEY);
	VectorXd top_bread_command_torques = VectorXd::Zero(6);

	bottom_bread->updateModel();
	bool bottom_bread_actuate = false;
	MatrixXd N_bottom_bread = MatrixXd::Identity(6, 6);

	burger->updateModel();
	bool burger_actuate = false;
	MatrixXd N_burger = MatrixXd::Identity(6, 6);

	top_bread->updateModel();
	bool top_bread_actuate = false;
	MatrixXd N_top_bread = MatrixXd::Identity(6, 6);

	auto bottom_bread_task = new Sai2Primitives::JointTask(bottom_bread);
	bottom_bread_task->_kp = 80.0;
	bottom_bread_task->_kv = 50.0;

	auto burger_task = new Sai2Primitives::JointTask(burger);
	burger_task->_kp = 80.0;
	burger_task->_kv = 50.0;

	auto top_bread_task = new Sai2Primitives::JointTask(top_bread);
	top_bread_task->_kp = 75.0;
	top_bread_task->_kv = 50.0;

	bool food_actuate[] = {bottom_bread_actuate, burger_actuate, top_bread_actuate};
	MatrixXd N_food[] = {N_bottom_bread, N_burger, N_top_bread};

	Sai2Primitives::JointTask *food_task[] = {bottom_bread_task, burger_task, top_bread_task};
	Sai2Model::Sai2Model *food_robot[] = {bottom_bread, burger, top_bread};
	// use plate_index

	//----------------------------------------***** KITCHEN FOOD ROBOTS *****-----------------------------------------------

	// from world urdf
	Vector3d base_origin;
	base_origin << 0.0, -0.05, 0.3514;

	Vector3d base_offset;
	base_offset << 0.0, 0.0, 0.1757;
	// base_offset << 0.0, -0.05, 0.3514;

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task for spatula
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, -0.203, 0.21764); // 0 0.203 0.21764
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
	// timer.setLoopFrequency(1000);
	timer.setLoopFrequency(200);
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	Matrix3d good_ee_rot; //
	// good_ee_rot << 0.703586, -0.710608, -0.0017762,
	// 		-0.337309, -0.336174, 0.879324,
	// 		-0.625451, -0.618081, -0.476221;
	good_ee_rot << 1, 0, 0,
			0, -1, 0,
			0, 0, -1;

	Vector3d slide;
	slide << 0.0, 0.25, 0.0;
	// slide << 0.0, 0.28, 0.02;
	Matrix3d slide_ori;
	double slide_angle = (180 - 6) * M_PI / 180.0; // 180 degrees is the starting position, instead of 0.
	slide_ori << 1.0000000, 0.0000000, 0.0000000,
			0.0000000, cos(slide_angle), -sin(slide_angle),
			0.0000000, sin(slide_angle), cos(slide_angle);

	// slide_ori *= good_ee_rot;

	double y_slide = 0.54; // based off of backstop location and thickness

	Matrix3d lift_ori;
	double lift_angle = (180) * M_PI / 180.0;
	// double lift_angle = 6 * M_PI / 180.0;

	lift_ori << 1.0000000, 0.0000000, 0.0000000,
			0.0000000, cos(lift_angle), -sin(lift_angle),
			0.0000000, sin(lift_angle), cos(lift_angle);
	// lift_ori *= good_ee_rot;
	// Vector3d lift_height;
	// lift_height << 0.0, 0.05, 0.25;
	double z_lift = 0.45;

	Vector3d drop_food;
	drop_food << 0.12, 0.70, 0.22;

	Vector3d des_vel;
	des_vel << 0.2, 0.2, 0.2;

	Vector3d reset_pos;
	reset_pos << 0.1, 0.2, 0.5;

	Matrix3d relax_ori;
	double relax_angle = (180 - 70) * M_PI / 180.0;
	// relax_ori = lift_ori.transpose();
	relax_ori << 1.0000000, 0.0000000, 0.0000000,
			0.0000000, cos(relax_angle), -sin(relax_angle),
			0.0000000, sin(relax_angle), cos(relax_angle);
	// relax_ori *= good_ee_rot;

	Matrix3d flex_ori;
	// double flex_angle = 0 * M_PI / 180.0;
	flex_ori = relax_ori.transpose();

	Vector3d plate_food;
	plate_food << -0.45, 0.5 - 0.221, 0.48;
	// plate_food << -0.415193+0.02, 0.481433-0.21, 0.53;

	double y_offset_tip = 0.051; // according to onshape - distance between spatula origin and front of spatula base
	bool taskFinished = false;
	int taskIndex = 7;

	posori_task->_use_interpolation_flag = true;
	posori_task->_use_velocity_saturation_flag = false;

	Vector3d init_spatula_pos;
	robot->positionInWorld(init_spatula_pos, control_link, control_point);
	Matrix3d init_spatula_rot;
	robot->rotationInWorld(init_spatula_rot, control_link);

	while (runloop)
	{
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		// r_spatula = redis_client.getEigenMatrixJSON(SPATULA_POSITION_KEY);
		// ori_spatula = redis_client.getEigenMatrixJSON(SPATULA_ORIENTATION_KEY);

		r_bottom_bread = redis_client.getEigenMatrixJSON(BOTTOM_BREAD_POSITION_KEY);
		r_top_bread = redis_client.getEigenMatrixJSON(TOP_BREAD_POSITION_KEY);
		r_burger = redis_client.getEigenMatrixJSON(BURGER_POSITION_KEY);

		//Vector3d stack_foods[] = {r_bottom_bread, r_burger, r_top_bread};
		Vector3d stack_foods[] = {Vector3d(0.5, 0.5, 0.5), Vector3d(0.9, 0.5, 0.5), Vector3d(0.7, 0.5, 0.5)};

		Vector3d robot_offset[] = {Vector3d(0, 0.15, 0.37114), Vector3d(0, 0.15, 0.37114), Vector3d(0, 0.15, 0.37114)};

		robot->updateModel();

		VectorXd q_curr_desired = VectorXd::Zero(10); // container  7 for joints, 3 for mobile base
		q_curr_desired = robot->_q;

		// Vector3d spatula_pos;
		// robot->positionInWorld(spatula_pos, control_link, control_point);
		// Matrix3d spatula_rot;
		// robot->rotationInWorld(spatula_rot, control_link);
		// cout << state << endl;

		switch (state)
		{
		case STACKING:
		{
			/* code */
			std::vector<int> tasks = {RESET_TASK, MOVE_TO_BOARD, ALIGN, SLIDE, LIFT_SPATULA, MOVE_TO_GRILL, DROP_FOOD};
			//std::vector<int> tasks = {MOVE_TO_BOARD, ALIGN, SLIDE, LIFT_SPATULA, MOVE_TO_GRILL, DROP_FOOD};

			if (taskFinished)
			{
				taskIndex++;
				taskFinished = false;
				if (taskIndex == tasks.size() && stack_idx < 2)
				{
					stack_idx++;
					//cout << "stack index: " << stack_idx << endl;
					taskIndex = 0;
					cout << "pick another food" << endl;
				}
				cout << "switch to task number " << taskIndex << endl;
			}
			if (taskIndex == tasks.size() && stack_idx == 2)
			{
				state = FLIPPING;
				cout << "switch to FLIPPING state" << endl;
				taskIndex = 0;
				stack_idx = 0;

				// send flag to simviz
				switch_food_flag = "true";
				continue;
			}

			task = tasks[taskIndex];
		}
		break;
		case FLIPPING:
		{
			std::vector<int> tasks = {RESET_TASK, ALIGN, SLIDE, LIFT_SPATULA, FLIP_FOOD, DROP_FOOD};
			task = tasks[taskIndex];
			if (taskFinished)
			{
				taskIndex++;
				taskFinished = false;
				cout << "switch to task number " << taskIndex << endl;
			}
			if (taskIndex == tasks.size())
			{
				state = SERVING;
				cout << "switch to SERVING state" << endl;
				taskIndex = 0;
			}
		}
		break;
		case SERVING:
		{
			std::vector<int> tasks = {ALIGN, SLIDE, LIFT_SPATULA, DROP_FOOD};
			task = tasks[taskIndex];
			if (taskFinished)
			{
				taskIndex++;
				taskFinished = false;
				cout << "switch to task number " << taskIndex << endl;
			}
			if (taskIndex == tasks.size())
			{
				// state = default;
				cout << "Finish" << endl;
				runloop = false;
				taskIndex = 0;
			}
		}
		break;
		default:
		{
			task = IDLE;
		}
		break;
		}

		switch (task)
		{
		case IDLE:
		{
			// VectorXd dq_curr_desired = VectorXd::Zero(12);
			// robot->_dq = dq_curr_desired;
			q_curr_desired = robot->_q;
		}
		break;
		case RESET_TASK:
		{
			posori_task->reInitializeTask();
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);
			posori_task->_desired_position = init_spatula_pos;
			posori_task->_desired_orientation = init_spatula_rot;
			if (posori_task->goalPositionReached(0.01) && posori_task->goalOrientationReached(0.05))
			{
				cout << "RESET Finished" << endl;
				taskFinished = true;
				// state = SLIDE;
			}

			// posori_task->reInitializeTask();

			// joint_task->reInitializeTask();
			// // q_curr_desired(8) = M_PI/2;
			// N_prec.setIdentity();
			// posori_task->updateTaskModel(N_prec);
			// posori_task->_desired_position = reset_pos;
			// // joint_task->_desired_position = initial_q;
			// // joint_task->_use_velocity_saturation_flag = false;
			// //N_prec = joint_task->_N;

			// // posori_task->_desired_position = reset_pos;
			// // N_prec = joint_task->_N;
			// posori_task->updateTaskModel(N_prec);
			// //joint_task->_desired_position =  q_curr_desired;
			// //cout << "robot position: " << robot->_q << endl;
			// //if ((robot->_q - q_curr_desired).norm() < 0.05)
			// if ((robot->_q - initial_q).norm() < 0.1)
			// {
			// 	cout << "Reset Finished" << endl;
			// 	taskFinished = true;
			// 	//state = -1;
			// }
			// // joint_task->_desired_position = q_curr_desired;
		}
		break;
		case MOVE_TO_BOARD:
		{
			posori_task->reInitializeTask();
			joint_task->reInitializeTask();

			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			joint_task->_use_velocity_saturation_flag = false;
			q_curr_desired(0) = 0.55;
			q_curr_desired(1) = -0.2;
			joint_task->_use_velocity_saturation_flag = true;
			joint_task->_saturation_velocity(0) = 0.2;
			if ((robot->_q - q_curr_desired).norm() < 0.05)
			{
				cout << "Move to Board Finished" << endl;
				taskFinished = true;
				// state = SLIDE;
			}
			joint_task->_desired_position = q_curr_desired;
			// compute torques
		}
		break;
		case ALIGN:
		{
			// align the spatula with the objects
			// joint_task->reInitializeTask();
			// joint_task->_kp = 0;

			posori_task->reInitializeTask();
			// posori_task->_otg->setMaxLinearVelocity(0.8);
			// posori_task->_otg->setMaxAngularVelocity(M_PI / 2);
			// posori_task->_desired_velocity = des_vel;
			// posori_task->_desired_angular_velocity = des_vel;
			//cout << "stack_index:\n\r" << stack_idx << endl;
			Vector3d r_food = stack_foods[stack_idx];
			if (controller_counter % 100 == 0)
			{
				cout << "r_food position" << r_food << endl;
			}

			//cout << "r_food:\n\r" << r_food << endl; //0.6 0.5 0.48
			//cout << "pos_spatula: " << spatula_pos << endl;

			//Vector3d robot_offset = Vector3d(0.1, 0.15, 0.3514); // where did this number come from
			Vector3d r_align = r_food - robot_offset[stack_idx];
			//Vector3d r_align = r_food;
			posori_task->_desired_position = r_align;
			posori_task->_desired_orientation = good_ee_rot;
			if (posori_task->goalPositionReached(0.01) && posori_task->goalOrientationReached(0.05))
			{
				cout << "ALIGN Finished" << endl;
				taskFinished = true;
				// state = SLIDE;
			}
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);
		}
		break;
		case SLIDE:
		{
			posori_task->reInitializeTask();

			posori_task->_desired_position(1) = y_slide;
			posori_task->_desired_orientation = slide_ori;

			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			if (posori_task->goalPositionReached(0.01) && posori_task->goalOrientationReached(0.05))
			{
				cout << "SLIDE Finished" << endl;
				taskFinished = true;
				// state = -1;
				continue;
			}
		}
		break;
		case LIFT_SPATULA:
			posori_task->reInitializeTask();

			posori_task->_desired_position(2) = z_lift;
			posori_task->_desired_orientation = lift_ori;

			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;

			joint_task->updateTaskModel(N_prec);

			if (posori_task->goalPositionReached(0.01) && posori_task->goalOrientationReached(0.05))
			{
				cout << "LIFT Finished" << endl;
				taskFinished = true;
				// state = -1;
				continue;
			}
			break;
		case MOVE_TO_GRILL:
			//cout << "pos_spatula: " << spatula_pos << endl;
			// set velocity to zero
			joint_task->reInitializeTask();
			posori_task->reInitializeTask();
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			joint_task->_use_velocity_saturation_flag = false;
			q_curr_desired(0) = 0.1;
			q_curr_desired(1) = 0;
			posori_task->_desired_position = drop_food;
			posori_task->_desired_orientation = lift_ori;
			// q_curr_desired(9) = -M_PI;/
			joint_task->_use_velocity_saturation_flag = true;
			joint_task->_saturation_velocity(0) = 0.2;
			if (posori_task->goalPositionReached(0.01) && posori_task->goalOrientationReached(0.05) && (robot->_q - q_curr_desired).norm() < 0.05)
			{
				cout << "Move to Grill finished" << endl;
				taskFinished = true;
				// state = SLIDE;
				// state = -1;
			}
			joint_task->_desired_position = q_curr_desired;
			// compute torques
			break;

			break;
		case DROP_FOOD:
			// drop the food to the grill
			posori_task->reInitializeTask();
			// posori_task->_desired_position(1) = y_slide + 0.1;
			posori_task->_desired_position(2) = z_lift - 0.1;
			posori_task->_desired_orientation = relax_ori;

			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;

			joint_task->updateTaskModel(N_prec);

			if (posori_task->goalPositionReached(0.01) && posori_task->goalOrientationReached(0.05))
			{
				cout << "Drop Food Finished" << endl;
				taskFinished = true;
				//state = -1;
				continue;
			}
			break;
		case SERVING:
			break;
		default:
			// task = IDLE;
			break;
		}

		posori_task->computeTorques(posori_task_torques);
		joint_task->computeTorques(joint_task_torques);

		command_torques = posori_task_torques + joint_task_torques;
		// cout << command_torques(0) << endl;
		// command_torques = posori_task_torques + joint_task_torques;

		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.set(NEW_OBJECT_KEY, switch_food_flag);
		controller_counter++;
	}
}
