#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <GLFW/glfw3.h>									 // must be loaded after loading opengl/glew
#include "uiforce/UIForceWidget.h"			 // used for right-click drag interaction in window									
#include <signal.h>

#include <cmath>
#include <iostream>
#include <string>
#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

using namespace std;
using namespace Eigen;

const string world_file = "./resources/world_panda_gripper.urdf";

const string robot_file = "./resources/mmp_panda.urdf";
const string robot_name = "mmp_panda";
const string camera_name = "camera_fixed";

const string burger_file = "./resources/burger.urdf";
const string burger_name = "burger";

const string top_bun_file = "./resources/top_bun.urdf";
const string top_bun_name = "top_bun";
const string bottom_bun_file = "./resources/bottom_bun.urdf";
const string bottom_bun_name = "bottom_bun";

const string grill_cheese_file = "./resources/grill_cheese.urdf";
const string grill_cheese_name = "grill_cheese";

// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";

const std::string BURGER_POSITION_KEY = "sai2::cs225a::burger::sensors::r_burger";

const std::string TOP_BUN_POSITION_KEY = "sai2::cs225a::top_bun::sensors::r_top_bun";
const std::string BOTTOM_BUN_POSITION_KEY = "sai2::cs225a::bottom_bun::sensors::r_bottom_bun";
const std::string GRILL_CHEESE_POSITION_KEY = "sai2::cs225a::grill_cheese::sensors::r_grill_cheese";

// - read
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";
const std::string BOTTOM_BUN_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::bottom_bun";
const std::string BURGER_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::burger";
const std::string TOP_BUN_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::top_bun";
const std::string GRILL_CHEESE_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::grill_cheese";

const std::string SWITCH_OBJECT_KEY = "sai2::cs225a::project::switch_object";

RedisClient redis_client;

// simulation function prototype
void simulation(Sai2Model::Sai2Model *robot,
								Sai2Model::Sai2Model *burger,
								Sai2Model::Sai2Model *top_bun,
								Sai2Model::Sai2Model *bottom_bun,
								Sai2Model::Sai2Model *grill_cheese,
								Sai2Graphics::Sai2Graphics *graphics,
								Simulation::Sai2Simulation *sim,
								UIForceWidget *ui_force_widget);

// callback to print glfw errors
void glfwError(int error, const char *description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow *window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow *window, int button, int action, int mods);

// callback for changing dynamics and graphics
void ChangeObject(Sai2Graphics::Sai2Graphics *graphics, Simulation::Sai2Simulation *sim, string robot_name, bool enable_dynamic_flag);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

int main()
{
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
	robot->updateModel();

	auto burger = new Sai2Model::Sai2Model(burger_file, false);
	burger->updateModel();

	auto top_bun = new Sai2Model::Sai2Model(top_bun_file, false);
	top_bun->updateModel();

	auto bottom_bun = new Sai2Model::Sai2Model(bottom_bun_file, false);
	bottom_bun->updateModel();

	auto grill_cheese = new Sai2Model::Sai2Model(grill_cheese_file, false);
	grill_cheese->updateModel();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.9);
	sim->setCoeffFrictionDynamic(0.2);

	// read joint positions, velocities, update model
	sim->getJointPositions(robot_name, robot->_q);
	sim->getJointVelocities(robot_name, robot->_dq);
	robot->updateModel();

	// set initial position of burger in world
	Eigen::Vector3d r_burger;
	Eigen::Vector3d r_top_bun;
	Eigen::Vector3d r_bottom_bun;
	Eigen::Vector3d r_grill_cheese;

	burger->positionInWorld(r_burger, "link6", Vector3d(0, 0, 0));
	burger->updateModel();

	top_bun->positionInWorld(r_top_bun, "link6", Vector3d(0, 0, 0));
	top_bun->updateModel();

	bottom_bun->positionInWorld(r_bottom_bun, "link6", Vector3d(0, 0, 0));
	bottom_bun->updateModel();

	grill_cheese->positionInWorld(r_grill_cheese, "link6", Vector3d(0, 0, 0));
	grill_cheese->updateModel();

	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor *primary = glfwGetPrimaryMonitor();
	const GLFWvidmode *mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow *window = glfwCreateWindow(windowW, windowH, "SAI2.0 - PandaApplications", NULL, NULL);
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

	fSimulationRunning = true;

	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);

	// thread sim_thread(simulation, robot, spatula, burger, tomato, cheese, lettuce, top_bun, bottom_bun, sim, ui_force_widget);
	//thread sim_thread(simulation, robot, burger, top_bun, bottom_bun, graphics, sim, ui_force_widget);
	thread sim_thread(simulation, robot, burger, top_bun, bottom_bun, grill_cheese, graphics, sim, ui_force_widget);

	// initialize glew
	glewInitialize();

	// while window is open:
	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->updateGraphics(burger_name, burger);
		graphics->updateGraphics(top_bun_name, top_bun);
		graphics->updateGraphics(bottom_bun_name, bottom_bun);
		graphics->updateGraphics(grill_cheese_name, grill_cheese);
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
		if (fTransXp)
		{
			camera_pos = camera_pos + 0.05 * cam_roll_axis;
			camera_lookat = camera_lookat + 0.05 * cam_roll_axis;
		}
		if (fTransXn)
		{
			camera_pos = camera_pos - 0.05 * cam_roll_axis;
			camera_lookat = camera_lookat - 0.05 * cam_roll_axis;
		}
		if (fTransYp)
		{
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05 * cam_up_axis;
			camera_lookat = camera_lookat + 0.05 * cam_up_axis;
		}
		if (fTransYn)
		{
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05 * cam_up_axis;
			camera_lookat = camera_lookat - 0.05 * cam_up_axis;
		}
		if (fTransZp)
		{
			camera_pos = camera_pos + 0.1 * cam_depth_axis;
			camera_lookat = camera_lookat + 0.1 * cam_depth_axis;
		}
		if (fTransZn)
		{
			camera_pos = camera_pos - 0.1 * cam_depth_axis;
			camera_lookat = camera_lookat - 0.1 * cam_depth_axis;
		}
		if (fRotPanTilt)
		{
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006 * (cursorx - last_cursorx);
			double azimuth = 0.006 * (cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt;
			m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt * (camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan;
			m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan * (camera_pos - camera_lookat);
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
	glfwSetWindowShouldClose(window, GL_TRUE);
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
// void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* spatula, Sai2Model::Sai2Model* burger, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget) {
void simulation(Sai2Model::Sai2Model *robot,
								Sai2Model::Sai2Model *burger,
								Sai2Model::Sai2Model *top_bun,
								Sai2Model::Sai2Model *bottom_bun,
								Sai2Model::Sai2Model *grill_cheese,
								Sai2Graphics::Sai2Graphics *graphics,
								Simulation::Sai2Simulation *sim,
								UIForceWidget *ui_force_widget)
{

	int dof = robot->dof();

	VectorXd bottom_bun_command_torques = VectorXd::Zero(6);
	redis_client.setEigenMatrixJSON(BOTTOM_BUN_TORQUES_COMMANDED_KEY, bottom_bun_command_torques);

	VectorXd burger_command_torques = VectorXd::Zero(6);
	redis_client.setEigenMatrixJSON(BURGER_TORQUES_COMMANDED_KEY, burger_command_torques);

	VectorXd top_bun_command_torques = VectorXd::Zero(6);
	redis_client.setEigenMatrixJSON(TOP_BUN_TORQUES_COMMANDED_KEY, top_bun_command_torques);

	VectorXd grill_cheese_command_torques = VectorXd::Zero(6);
	redis_client.setEigenMatrixJSON(GRILL_CHEESE_TORQUES_COMMANDED_KEY, grill_cheese_command_torques);

	VectorXd command_torques = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	//switch food flag set to false
	string switch_food_flag = "false";
	redis_client.set(SWITCH_OBJECT_KEY, switch_food_flag);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();

	double slow_down_factor = 2;
	timer.setLoopFrequency(1000);
	double last_time = timer.elapsedTime() / slow_down_factor; //secs
	bool fTimerDidSleep = true;

	// init variables
	VectorXd g(dof);

	Eigen::Vector3d ui_force;
	ui_force.setZero();

	Eigen::VectorXd ui_force_command_torques;
	ui_force_command_torques.setZero();

	Eigen::Vector3d r_burger;
	Eigen::Vector3d burger_offset;

	burger_offset << 0.5, 0.5, 0.5;

	Eigen::Vector3d r_bottom_bun = Vector3d::Zero();
	Eigen::Vector3d bottom_bun_offset;
	bottom_bun_offset << 0.6, 0.5, 0.5;

	Eigen::Vector3d r_top_bun = Vector3d::Zero();
	Eigen::Vector3d top_bun_offset;
	top_bun_offset << 0.7, 0.5, 0.5;

	Eigen::Vector3d r_grill_cheese = Vector3d::Zero();
	Eigen::Vector3d grill_cheese_offset;
	// grill_cheese_offset << 0.12, 0.7, 0.48;
	grill_cheese_offset << 0.12, 0.65, 0.50;

	std::vector<std::string> get_redis_data(6);  // set with the number of keys to read
	std::vector<std::pair<std::string, std::string>> set_redis_data(6);  // set with the number of keys to write 
	std::vector<std::string> read_keys(6);  // redis get keys 
	read_keys.at(0) = JOINT_TORQUES_COMMANDED_KEY;
	read_keys.at(1) = SWITCH_OBJECT_KEY;
	read_keys.at(2) = BOTTOM_BUN_TORQUES_COMMANDED_KEY;
	read_keys.at(3) = BURGER_TORQUES_COMMANDED_KEY;
	read_keys.at(4) = TOP_BUN_TORQUES_COMMANDED_KEY;
	read_keys.at(5) = GRILL_CHEESE_TORQUES_COMMANDED_KEY;

	while (fSimulationRunning)
	{
		fTimerDidSleep = timer.waitForNextLoop();

		// get gravity torques
		robot->gravityVector(g);

		// read arm torques from redis and apply to simulated robot
		// command_torques = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY);
		// switch_food_flag = redis_client.get(SWITCH_OBJECT_KEY);
		// bottom_bun_command_torques = redis_client.getEigenMatrixJSON(BOTTOM_BUN_TORQUES_COMMANDED_KEY);
		// burger_command_torques = redis_client.getEigenMatrixJSON(BURGER_TORQUES_COMMANDED_KEY);
		// top_bun_command_torques = redis_client.getEigenMatrixJSON(TOP_BUN_TORQUES_COMMANDED_KEY);
		// grill_cheese_command_torques = redis_client.getEigenMatrixJSON(GRILL_CHEESE_TORQUES_COMMANDED_KEY);

		get_redis_data = redis_client.pipeget(read_keys);
		sim->setJointTorques(robot_name, redis_client.decodeEigenMatrixJSON(get_redis_data.at(0)) + g);
		switch_food_flag = get_redis_data.at(1);  
		sim->setJointTorques(bottom_bun_name, redis_client.decodeEigenMatrixJSON(get_redis_data.at(2)));
		sim->setJointTorques(burger_name, redis_client.decodeEigenMatrixJSON(get_redis_data.at(3)));
		sim->setJointTorques(top_bun_name, redis_client.decodeEigenMatrixJSON(get_redis_data.at(4)));
		sim->setJointTorques(grill_cheese_name, redis_client.decodeEigenMatrixJSON(get_redis_data.at(5)));

		// integrate forward
		double curr_time = timer.elapsedTime() / slow_down_factor;
		double loop_dt = curr_time - last_time;
		// sim->integrate(loop_dt);
		sim->integrate(0.001);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		sim->getJointPositions(burger_name, burger->_q);
		sim->getJointVelocities(burger_name, burger->_dq);
		burger->updateModel();

		burger->positionInWorld(r_burger, "link6");
		r_burger += burger_offset;

		// update graphics and positions for all other objects
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

		sim->getJointPositions(grill_cheese_name, grill_cheese->_q);
		sim->getJointVelocities(grill_cheese_name, grill_cheese->_dq);
		grill_cheese->updateModel();
		grill_cheese->positionInWorld(r_grill_cheese, "link6");
		r_grill_cheese += grill_cheese_offset;

		if (switch_food_flag == "true")
		{
			ChangeObject(graphics, sim, top_bun_name, false);
			ChangeObject(graphics, sim, burger_name, false);
			ChangeObject(graphics, sim, bottom_bun_name, false);
			ChangeObject(graphics, sim, grill_cheese_name, true);
		}
		else
		{
			//make grill cheese object invisible at start up
			ChangeObject(graphics, sim, grill_cheese_name, false);
		}

		// write new robot state to redis
		// redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
		// redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);
		// redis_client.setEigenMatrixJSON(BURGER_POSITION_KEY, r_burger);
		// redis_client.setEigenMatrixJSON(TOP_BUN_POSITION_KEY, r_top_bun);
		// redis_client.setEigenMatrixJSON(BOTTOM_BUN_POSITION_KEY, r_bottom_bun);
		// redis_client.setEigenMatrixJSON(GRILL_CHEESE_POSITION_KEY, r_grill_cheese);

		set_redis_data.at(0) = std::pair<string, string>(JOINT_ANGLES_KEY, redis_client.encodeEigenMatrixJSON(robot->_q));
		set_redis_data.at(1) = std::pair<string, string>(JOINT_VELOCITIES_KEY, redis_client.encodeEigenMatrixJSON(robot->_dq));
		set_redis_data.at(2) = std::pair<string, string>(BURGER_POSITION_KEY, redis_client.encodeEigenMatrixJSON(r_burger));
		set_redis_data.at(3) = std::pair<string, string>(TOP_BUN_POSITION_KEY, redis_client.encodeEigenMatrixJSON(r_top_bun));
		set_redis_data.at(4) = std::pair<string, string>(BOTTOM_BUN_POSITION_KEY, redis_client.encodeEigenMatrixJSON(r_bottom_bun));
		set_redis_data.at(5) = std::pair<string, string>(GRILL_CHEESE_POSITION_KEY, redis_client.encodeEigenMatrixJSON(r_grill_cheese));
		redis_client.pipeset(set_redis_data);

		//update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime() / slow_down_factor;
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles() / end_time << "Hz\n";
}

//------------------------------------------------------------------------------

void glfwError(int error, const char *description)
{
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize()
{
	bool ret = false;
#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK)
	{
		cout << "Failed to initialize GLEW library" << endl;
		cout << glewGetErrorString(ret) << endl;
		glfwTerminate();
	}
	else
	{
		ret = true;
	}
#endif
	return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch (key)
	{
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

void mouseClick(GLFWwindow *window, int button, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button)
	{
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

// change dynamics and graphics
void ChangeObject(Sai2Graphics::Sai2Graphics *graphics, Simulation::Sai2Simulation *sim, string robot_name, bool enable_dynamic_flag)
{

	for (auto robot : sim->_world->m_dynamicObjects)
	{
		if (robot->m_name == robot_name)
		{
			// cout << "change robot name " << robot_name << endl;
			robot->enableDynamics(enable_dynamic_flag);
			break;
		}
	}

	for (unsigned int i = 0; i < graphics->_world->getNumChildren(); ++i)
	{
		if (robot_name == graphics->_world->getChild(i)->m_name)
		{
			// cast to cRobotBase
			graphics->_world->getChild(i)->setEnabled(enable_dynamic_flag, true);
			break;
		}
	}
}
