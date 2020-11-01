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
#define RESET 6

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
	int task = IDLE;
	int state = ALIGN;
	int base = STACK_BASE;
	int grill_idx = 0;
	int flipping_idx = 0;
	bool flip_flag = false;
	unsigned long long counter = 0;
	const int object_nums = 3;

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
	const Vector3d control_point = Vector3d(0, 0.203, 0.21764); // 0 0.203 0.21764
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

	Matrix3d good_ee_rot; //
	good_ee_rot << 0.703586, -0.710608, -0.0017762,
			-0.337309, -0.336174, 0.879324,
			-0.625451, -0.618081, -0.476221;

	Vector3d slide;
	slide << 0.0, 0.25, 0.0;
	// slide << 0.0, 0.28, 0.02;
	Matrix3d slide_ori;
	double slide_angle = -6 * M_PI / 180.0;
	slide_ori << 1.0000000, 0.0000000, 0.0000000,
			0.0000000, cos(slide_angle), -sin(slide_angle),
			0.0000000, sin(slide_angle), cos(slide_angle);

	slide_ori *= good_ee_rot;

	double y_slide = 0.44; // based off of backstop location and thickness

	Matrix3d lift_ori;
	double lift_angle = 20 * M_PI / 180.0;
	// double lift_angle = 6 * M_PI / 180.0;

	lift_ori << 1.0000000, 0.0000000, 0.0000000,
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
	relax_ori << 1.0000000, 0.0000000, 0.0000000,
			0.0000000, cos(relax_angle), -sin(relax_angle),
			0.0000000, sin(relax_angle), cos(relax_angle);
	relax_ori *= good_ee_rot;

	Matrix3d flex_ori;
	// double flex_angle = 0 * M_PI / 180.0;
	flex_ori = relax_ori.transpose();

	Vector3d plate_food;
	plate_food << -0.45, 0.5 - 0.221, 0.48;
	// plate_food << -0.415193+0.02, 0.481433-0.21, 0.53;

	double y_offset_tip = 0.051; // according to onshape - distance between spatula origin and front of spatula base

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

		Vector3d stack_foods[] = {r_bottom_bread, r_burger, r_top_bread};

		robot->updateModel();

		VectorXd q_curr_desired(10); // container  7 for joints, 3 for mobile base
		q_curr_desired = robot->_q;

		VectorXd dq_curr_desired = VectorXd::Zero(12);

		Vector3d spatula_pos;
		robot->positionInWorld(spatula_pos, control_link, control_point);
		Matrix3d spatula_rot;
		robot->rotationInWorld(spatula_rot, control_link);
		// cout << time << endl;

		switch (state)
		{
		case IDLE:
			robot->_dq = dq_curr_desired;
			break;
		case ALIGN:
			// If(task is to align){
			// moving to the chopping board area
			// align the spatula to the object[grill_index]
			// if (position and ori reached){change task to slide}
			// }
			if (time > 2.0)
			{
				cout << "swich to Align" << endl;
				q_curr_desired(0) = -0.3514; // move to cutting board
				joint_task->_use_velocity_saturation_flag = true;
				joint_task->_saturation_velocity(0) = 0.2;
				state = IDLE;
			}
			break;
		case STACKING:
			// set velocity to zero
			break;
		case FLIPPING:
			// align the
			break;
		case SERVING:
			break;
		case RESET:
			break;
		}
		joint_task->_desired_position = q_curr_desired;
		// compute torques
		joint_task->computeTorques(joint_task_torques);
		command_torques = joint_task_torques;
		if ((robot->_q - q_curr_desired).norm() < 0.05)
		{
			state = IDLE;
		}

		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}
}
