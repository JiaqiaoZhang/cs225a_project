// #include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew
#include <cmath>
#include "uiforce/UIForceWidget.h"

#include <iostream>
#include <string>

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

const string world_file = "./resources/world_panda_gripper.urdf";
// const string robot_file = "./resources/panda_arm_hand.urdf";
// const string robot_name = "panda_arm_hand";
const string robot_file = "./resources/mmp_panda.urdf";
const string robot_name = "mmp_panda";
const string camera_name = "camera_fixed";
const string spatula_file = "./resources/spatula.urdf";
const string spatula_name = "spatula"; 
const string burger_file = "./resources/burger.urdf";
const string burger_name = "burger"; 
const string tomato_file = "./resources/tomato.urdf";
const string tomato_name = "tomato"; 
const string cheese_file = "./resources/cheese.urdf";
const string cheese_name = "cheese"; 
const string lettuce_file = "./resources/lettuce.urdf";
const string lettuce_name = "lettuce";
const string top_bun_file = "./resources/top_bun.urdf";
const string top_bun_name = "top_bun";
const string bottom_bun_file = "./resources/bottom_bun.urdf";
const string bottom_bun_name = "bottom_bun"; 

// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
const std::string SPATULA_POSITION_KEY = "sai2::cs225a::spatula::sensors::r_spatula";
const std::string SPATULA_ORIENTATION_KEY = "sai2::cs225a::spatula::sensors::ori_spatula";
const std::string SPATULA_JOINT_ANGLES_KEY = "sai2::cs225a::spatula::sensors::spatula_q";
const std::string BURGER_POSITION_KEY = "sai2::cs225a::burger::sensors::r_burger";
// const std::string BURGER_ORIENTATION_KEY = "sai2::cs225a::burger::sensors::q_burger";
// const std::string BURGER_JOINT_ANGLES_KEY = "sai2::cs225a::burger::sensors::burger_q";
const std::string TOMATO_POSITION_KEY = "sai2::cs225a::tomato::sensors::r_tomato";
const std::string CHEESE_POSITION_KEY = "sai2::cs225a::cheese::sensors::r_cheese";
const std::string LETTUCE_POSITION_KEY = "sai2::cs225a::lettuce::sensors::r_lettuce";
const std::string TOP_BUN_POSITION_KEY = "sai2::cs225a::top_bun::sensors::r_top_bun";
const std::string BOTTOM_BUN_POSITION_KEY = "sai2::cs225a::bottom_bun::sensors::r_bottom_bun";

// - read
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";
const std::string BOTTOM_BUN_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::bottom_bun";
const std::string BURGER_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::burger";
const std::string TOP_BUN_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::top_bun";
RedisClient redis_client;

// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, 
				Sai2Model::Sai2Model* spatula, 
				Sai2Model::Sai2Model* burger, 
				Sai2Model::Sai2Model* tomato, 
				Sai2Model::Sai2Model* cheese, 
				Sai2Model::Sai2Model* lettuce, 
				Sai2Model::Sai2Model* top_bun,
				Sai2Model::Sai2Model* bottom_bun, 
				Simulation::Sai2Simulation* sim, 
				UIForceWidget *ui_force_widget);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->updateKinematics();

	auto spatula = new Sai2Model::Sai2Model(spatula_file, false);
	spatula->updateModel();
	spatula->updateKinematics();

	auto burger = new Sai2Model::Sai2Model(burger_file, false);
	burger->updateModel();
	burger->updateKinematics();

	auto tomato = new Sai2Model::Sai2Model(tomato_file, false);
	tomato->updateModel();
	tomato->updateKinematics();

	auto cheese = new Sai2Model::Sai2Model(cheese_file, false);
	cheese->updateModel();
	cheese->updateKinematics();

	auto lettuce = new Sai2Model::Sai2Model(lettuce_file, false);
	lettuce->updateModel();
	lettuce->updateKinematics();

	auto top_bun = new Sai2Model::Sai2Model(top_bun_file, false);
	top_bun->updateModel();
	top_bun->updateKinematics();

	auto bottom_bun = new Sai2Model::Sai2Model(bottom_bun_file, false);
	bottom_bun->updateModel();
	bottom_bun->updateKinematics();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0.1);
	sim->setCoeffFrictionStatic(0.9);
	sim->setCoeffFrictionDynamic(0.2);


	// read joint positions, velocities, update model
	sim->getJointPositions(robot_name, robot->_q);
	sim->getJointVelocities(robot_name, robot->_dq);
	robot->updateKinematics();
	// get position and orientation of spatula from sim
	//set initial position of spatula in world
	Eigen::Vector3d r_spatula;
	Eigen::Matrix3d ori_spatula;
	// get position and orientation of burger from sim
	//set initial position of burger in world
	Eigen::Vector3d r_burger;
	// Eigen::Matrix3d q_burger;
	// get position of all other objects in the world
	Eigen::Vector3d r_tomato;
	Eigen::Vector3d r_cheese;
	Eigen::Vector3d r_lettuce;
	Eigen::Vector3d r_top_bun;
	Eigen::Vector3d r_bottom_bun;

	spatula->positionInWorld(r_spatula, "link6", Vector3d(0, 0, 0));
	spatula->rotationInWorld(ori_spatula, "link6");
	spatula->updateModel();

	burger->positionInWorld(r_burger, "link6", Vector3d(0, 0, 0));
	// burger->rotationInWorld(q_burger, "link6");
	burger->updateModel();

	tomato->positionInWorld(r_tomato, "link6", Vector3d(0, 0, 0));
	tomato->updateModel();
	cheese->positionInWorld(r_cheese, "link6", Vector3d(0, 0, 0));
	cheese->updateModel();
	lettuce->positionInWorld(r_lettuce, "link6", Vector3d(0, 0, 0));
	lettuce->updateModel();
	top_bun->positionInWorld(r_top_bun, "link6", Vector3d(0, 0, 0));
	top_bun->updateModel();
	bottom_bun->positionInWorld(r_bottom_bun, "link6", Vector3d(0, 0, 0));
	bottom_bun->updateModel();

	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - PandaApplications", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// init click force widget 
	auto ui_force_widget = new UIForceWidget(robot_name, robot, graphics);
	ui_force_widget->setEnable(false);

	// cache variables
	double last_cursorx, last_cursory;

	// initialize glew
	// glewInitialize();

	fSimulationRunning = true;

	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q); 
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq); 
	redis_client.setEigenMatrixJSON(SPATULA_JOINT_ANGLES_KEY, spatula->_q); 
	// redis_client.setEigenMatrixJSON(SPATULA_JOINT_ANGLES_KEY, burger->_q); 

	thread sim_thread(simulation, robot, spatula, burger, tomato, cheese, lettuce, top_bun, bottom_bun, sim, ui_force_widget);
	
	// while window is open:
	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->updateGraphics(spatula_name, spatula);
		graphics->updateGraphics(burger_name, burger);
		graphics->updateGraphics(tomato_name, tomato);
		graphics->updateGraphics(cheese_name, cheese);
		graphics->updateGraphics(lettuce_name, lettuce);
		graphics->updateGraphics(top_bun_name, top_bun);
		graphics->updateGraphics(bottom_bun_name, bottom_bun);
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();
		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}	    
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);

		ui_force_widget->setEnable(fRobotLinkSelect);
		if (fRobotLinkSelect)
		{
			double cursorx, cursory;
			int wwidth_scr, wheight_scr;
			int wwidth_pix, wheight_pix;
			std::string ret_link_name;
			Eigen::Vector3d ret_pos;

			// get current cursor position
			glfwGetCursorPos(window, &cursorx, &cursory);

			glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
			glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);

			int viewx = floor(cursorx / wwidth_scr * wwidth_pix);
			int viewy = floor(cursory / wheight_scr * wheight_pix);

			if (cursorx > 0 && cursory > 0)
			{
				ui_force_widget->setInteractionParams(camera_name, viewx, wheight_pix - viewy, wwidth_pix, wheight_pix);
				//TODO: this behavior might be wrong. this will allow the user to click elsewhere in the screen
				// then drag the mouse over a link to start applying a force to it.
			}
		}
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwSetWindowShouldClose(window,GL_TRUE);
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
// void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* spatula, Sai2Model::Sai2Model* burger, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget) {
void simulation(Sai2Model::Sai2Model* robot, 
				Sai2Model::Sai2Model* spatula, 
				Sai2Model::Sai2Model* burger, 
				Sai2Model::Sai2Model* tomato, 
				Sai2Model::Sai2Model* cheese, 
				Sai2Model::Sai2Model* lettuce, 
				Sai2Model::Sai2Model* top_bun,
				Sai2Model::Sai2Model* bottom_bun, 
				Simulation::Sai2Simulation* sim, 
				UIForceWidget *ui_force_widget) {


	int dof = robot->dof();

	VectorXd bottom_bun_command_torques = VectorXd::Zero(6);
	redis_client.setEigenMatrixJSON(BOTTOM_BUN_TORQUES_COMMANDED_KEY, bottom_bun_command_torques);

	VectorXd burger_command_torques = VectorXd::Zero(6);
	redis_client.setEigenMatrixJSON(BURGER_TORQUES_COMMANDED_KEY, burger_command_torques);

	VectorXd top_bun_command_torques = VectorXd::Zero(6);
	redis_client.setEigenMatrixJSON(TOP_BUN_TORQUES_COMMANDED_KEY, top_bun_command_torques);

	VectorXd command_torques = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	// For David's Laptop: slow_down_factor = 3
	double slow_down_factor = 2;
	timer.setLoopFrequency(1000); 
	double last_time = timer.elapsedTime()/slow_down_factor; //secs
	bool fTimerDidSleep = true;

	// init variables
	VectorXd g(dof);

	Eigen::Vector3d ui_force;
	ui_force.setZero();

	Eigen::VectorXd ui_force_command_torques;
	ui_force_command_torques.setZero();

	Eigen::Vector3d r_spatula;
	Eigen::Matrix3d ori_spatula_local;
	Eigen::Matrix3d ori_spatula;
	Eigen::Vector3d spatula_offset;

	spatula_offset << 0.5, 0.4, 0.46;
	Matrix3d spatula_rot_init;
	spatula_rot_init << 0.0, 1.0, 0.0, 
						-1.0, 0.0, 0.0,
						0.0, 0.0, 1.0;

	Eigen::Vector3d r_burger;
	// Eigen::Matrix3d q_burger_local;
	// Eigen::Matrix3d q_burger;
	Eigen::Vector3d burger_offset;

	burger_offset << 0.5, 0.5, 0.5;
	// Matrix3d burger_rot_init;
	// burger_rot_init << 1.0, 0.0, 0.0, 
	// 					0.0, 1.0, 0.0,
	// 					0.0, 0.0, 1.0;


	Eigen::Vector3d r_bottom_bun = Vector3d::Zero();
	Eigen::Vector3d bottom_bun_offset;
	bottom_bun_offset << 0.6, 0.5, 0.5;

	Eigen::Vector3d r_top_bun = Vector3d::Zero();
	Eigen::Vector3d top_bun_offset;
	top_bun_offset << 0.7, 0.5, 0.5;
	
	Eigen::Vector3d r_lettuce = Vector3d::Zero();
	Eigen::Vector3d lettuce_offset;
	lettuce_offset << 0.8, 0.5, 0.5;

	Eigen::Vector3d r_cheese = Vector3d::Zero();
	Eigen::Vector3d cheese_offset;
	cheese_offset << 0.9, 0.5, 0.5;

	Eigen::Vector3d r_tomato = Vector3d::Zero();
	Eigen::Vector3d tomato_offset;
	tomato_offset << 1.0, 0.5, 0.5;



	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// get gravity torques
		robot->gravityVector(g);

		// g.setZero();
		// read arm torques from redis and apply to simulated robot
		command_torques = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY);
		bottom_bun_command_torques = redis_client.getEigenMatrixJSON(BOTTOM_BUN_TORQUES_COMMANDED_KEY);
		burger_command_torques = redis_client.getEigenMatrixJSON(BURGER_TORQUES_COMMANDED_KEY);
		top_bun_command_torques = redis_client.getEigenMatrixJSON(TOP_BUN_TORQUES_COMMANDED_KEY);

		
		ui_force_widget->getUIForce(ui_force);
		ui_force_widget->getUIJointTorques(ui_force_command_torques);

		if (fRobotLinkSelect)
			sim->setJointTorques(robot_name, command_torques + ui_force_command_torques + g);
		else
			sim->setJointTorques(robot_name, command_torques + g);

		sim->setJointTorques(bottom_bun_name, bottom_bun_command_torques);
		sim->setJointTorques(burger_name, burger_command_torques);
		sim->setJointTorques(top_bun_name, top_bun_command_torques);



		// integrate forward

		double curr_time = timer.elapsedTime() / slow_down_factor;
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		// update joint positions for the spatula
		sim->getJointPositions(spatula_name, spatula->_q);
		sim->getJointVelocities(spatula_name, spatula->_dq);
		spatula->updateModel();

		spatula->positionInWorld(r_spatula, "link6");
		spatula->rotationInWorld(ori_spatula_local, "link6");
		r_spatula += spatula_offset;
		ori_spatula = ori_spatula_local * spatula_rot_init;
		spatula->updateModel();

		// update joint positions for the burger
		sim->getJointPositions(burger_name, burger->_q);
		sim->getJointVelocities(burger_name, burger->_dq);
		burger->updateModel();

		burger->positionInWorld(r_burger, "link6");
		// burger->rotationInWorld(q_burger_local, "link6");
		r_burger += burger_offset;
		// q_burger = q_burger_local * burger_rot_init;
		// burger->updateModel();

		// update graphics and positions for all other objects
		sim->getJointPositions(tomato_name, tomato->_q);
		sim->getJointVelocities(tomato_name, tomato->_dq);
		tomato->updateModel();
		tomato->positionInWorld(r_tomato, "link6");
		r_tomato += tomato_offset;

		sim->getJointPositions(cheese_name, cheese->_q);
		sim->getJointVelocities(cheese_name, cheese->_dq);
		cheese->updateModel();
		cheese->positionInWorld(r_cheese, "link6");
		r_cheese += cheese_offset;

		sim->getJointPositions(lettuce_name, lettuce->_q);
		sim->getJointVelocities(lettuce_name, lettuce->_dq);
		lettuce->updateModel();
		lettuce->positionInWorld(r_lettuce, "link6");
		r_lettuce += lettuce_offset;

		sim->getJointPositions(top_bun_name, top_bun->_q);
		sim->getJointVelocities(top_bun_name, top_bun->_dq);
		top_bun->updateModel();
		top_bun->positionInWorld(r_top_bun, "link6");
		r_top_bun += top_bun_offset;

		sim->getJointPositions(bottom_bun_name, bottom_bun->_q);
		sim->getJointVelocities(bottom_bun_name, bottom_bun->_dq);
		bottom_bun->updateModel();
		bottom_bun->positionInWorld(r_bottom_bun, "link6");
		r_bottom_bun += bottom_bun_offset;

		// write new robot state to redis
		redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
		redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);
		redis_client.setEigenMatrixJSON(SPATULA_POSITION_KEY, r_spatula);
		redis_client.setEigenMatrixJSON(SPATULA_ORIENTATION_KEY, ori_spatula);
		redis_client.setEigenMatrixJSON(SPATULA_JOINT_ANGLES_KEY, spatula->_q);
		redis_client.setEigenMatrixJSON(BURGER_POSITION_KEY, r_burger);
		// redis_client.setEigenMatrixJSON(BURGER_ORIENTATION_KEY, q_burger);
		// redis_client.setEigenMatrixJSON(BURGER_JOINT_ANGLES_KEY, burger->_q);
		redis_client.setEigenMatrixJSON(TOMATO_POSITION_KEY, r_tomato);
		redis_client.setEigenMatrixJSON(CHEESE_POSITION_KEY, r_cheese);
		redis_client.setEigenMatrixJSON(LETTUCE_POSITION_KEY, r_lettuce);
		redis_client.setEigenMatrixJSON(TOP_BUN_POSITION_KEY, r_top_bun);
		redis_client.setEigenMatrixJSON(BOTTOM_BUN_POSITION_KEY, r_bottom_bun);

		//update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime() / slow_down_factor;
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize() {
	bool ret = false;
	#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK) {
		cout << "Failed to initialize GLEW library" << endl;
		cout << glewGetErrorString(ret) << endl;
		glfwTerminate();
	} else {
		ret = true;
	}
	#endif
	return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			fSimulationRunning = false;
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			fRobotLinkSelect = set;
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}

