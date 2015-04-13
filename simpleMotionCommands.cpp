/*
Adept MobileRobots Robotics Interface for Applications (ARIA)
Copyright (C) 2004-2005 ActivMedia Robotics LLC
Copyright (C) 2006-2010 MobileRobots Inc.
Copyright (C) 2011-2014 Adept Technology

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960
*/
#include "Aria.h"
#include <iostream>
#include <sicklms-1.0/SickLMS.hh>

/** @example simpleMotionCommands.cpp example showing how to connect and send
 * basic motion commands to the robot
 *
 * ARIA provides two levels of robot motion control, direct motion commands, and
 * actions. This example shows direct motion commands. See actionExample.cpp,
 * actionGroupExample.cpp, and others for examples on how to use actions.
 * Actions provide a more modular way of performing more complex motion
 * behaviors than the simple imperitive style used here.  
 *
 * See the ArRobot class documentation, as well as the overview of robot motion,
 * for more information.
 *
 * WARNING: this program does no sensing or avoiding of obstacles, the robot WILL
 * collide with any objects in the way!   Make sure the robot has about 2-3
 * meters of free space around it before starting the program.
 *
 * This program will work either with the MobileSim simulator or on a real
 * robot's onboard computer.  (Or use -remoteHost to connect to a wireless
 * ethernet-serial bridge.)
 */

using namespace std;
using namespace SickToolbox;

ArRobot initializeRobot(int argc, char ** argv);

int main(int argc, char **argv)
{
  string dev_path = "/dev/ttyUSB3";
  sick_lms_baud_t lms_baud = SickLMS::SICK_BAUD_38400;
  unsigned int num_range_values;
  unsigned int num_reflect_values;
  unsigned int range_values[SickLMS::SICK_MAX_NUM_MEASUREMENTS] = {0};
  unsigned int reflect_values[SickLMS::SICK_MAX_NUM_MEASUREMENTS] = {0};

  //-----Connect to robot
  Aria::init();
  ArRobot robot = initializeRobot(argc, argv);
  //---------------connect to SICK
  SickLMS sick_lms(dev_path);
  sick_lms.Initialize(lms_baud);


	int avg_reading_num = 0;
	int num_readings = 0;
	int ref_range = 0;
	bool complete = false; 
	while(!complete){	
		sick_lms.GetSickScan(range_values,reflect_values,num_range_values,num_reflect_values);
				
		num_readings = 0;
		avg_reading_num = 0;
		
		for(unsigned int j = 0; j< num_reflect_values; j++){
 			
				
		if(reflect_values[j] > 180){
			    num_readings++;
			    avg_reading_num += j;
			    cout << "Point #" << j << "reflectivity = " << reflect_values[j] << "\n";
			   cout << j;
			}

			
		}
		if (num_readings > 0){
			avg_reading_num = avg_reading_num / num_readings; 
		
		cout << "Avg_reading = " << avg_reading_num << "Num_readings = " << num_readings << "\n";
		
		if(avg_reading_num > 92){
			int degreesOff = (avg_reading_num - 90)/2;
			cout << "Degrees off = " << degreesOff<<endl;
			robot.lock();
			robot.enableMotors();
			robot.setRotVel(10);
			robot.unlock();
			ArUtil::sleep(degreesOff * 80);

			robot.lock();
			robot.enableMotors();
			robot.setRotVel(0);
			robot.unlock();
			ArUtil::sleep(100);

		}
		else if(avg_reading_num < 88){
			int degreesOff = (90 - avg_reading_num)/2;
			cout << "Degrees off = " << degreesOff<<endl;
			robot.lock();
			robot.enableMotors();
			robot.setRotVel(-10);
			robot.unlock();
			ArUtil::sleep(degreesOff * 80);

			robot.lock();
			robot.enableMotors();
			robot.setRotVel(0);
			robot.unlock();
			ArUtil::sleep(100);

		}
		else{
			ref_range = range_values[avg_reading_num];
			if (ref_range > 40){
				robot.lock();
				robot.enableMotors();
				robot.setVel(200);
				robot.unlock();
				cout << 5*ref_range << endl;
			}
			else {
				complete = true; 
			}				
		}
		}
		else{
			robot.lock();
			robot.enableMotors();
			robot.setVel(0);
			robot.unlock();
		}
	}
	
				
			

  sick_lms.Uninitialize();
 
  /*Aria::init();
  ArRobot robot;
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();*/
/*
  ArLog::log(ArLog::Terse, "WARNING: this program does no sensing or avoiding of obstacles, the robot WILL collide with any objects in the way! Make sure the robot has approximately 3 meters of free space on all sides.");

  // ArRobotConnector connects to the robot, get some initial data from it such as type and name,
  // and then loads parameter files for this robot.
  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "simpleMotionCommands: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
        return 1;
    }
  }
  if (!Aria::parseArgs())
  {
    Aria::logOptions();
    Aria::exit(1);
    return 1;
  }
  
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Connected.");

  // Start the robot processing cycle running in the background.
  // True parameter means that if the connection is lost, then the 
  // run loop ends.
  robot.runAsync(true);

  // Print out some data from the SIP.  

  // We must "lock" the ArRobot object
  // before calling its methods, and "unlock" when done, to prevent conflicts
  // with the background thread started by the call to robot.runAsync() above.
  // See the section on threading in the manual for more about this.
  // Make sure you unlock before any sleep() call or any other code that will
  // take some time; if the robot remains locked during that time, then
  // ArRobot's background thread will be blocked and unable to communicate with
  // the robot, call tasks, etc.
  
  robot.lock();
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
  robot.unlock();

  // Sleep for 3 seconds.
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Will start driving in 3 seconds...");
  robot.lock();
  robot.comInt(28,0);
  robot.unlock();
  ArUtil::sleep(3000);

  // Set forward velocity to 50 mm/s
 for(int i=0; i<12;i++){
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Driving forward at 250 mm/s for 5 sec...");
  robot.lock();
  robot.enableMotors();
  //robot.setVel(10);
  robot.unlock();
  ArUtil::sleep(5000);

}


  // Other motion command functions include move(), setHeading(),
  // setDeltaHeading().  You can also adjust acceleration and deceleration
  // values used by the robot with setAccel(), setDecel(), setRotAccel(),
  // setRotDecel().  See the ArRobot class documentation for more.

  
  robot.lock();
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
  robot.unlock();

  */
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Ending robot thread...");
  robot.stopRunning();

  // wait for the thread to stop
  robot.waitForRunExit();

  // exit
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
  Aria::exit(0);

  return 0;
}

ArRobot initializeRobot(int argc, char ** argv){

  Aria::init();
  ArRobot robot;
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();

  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "simpleMotionCommands: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
        return NULL;
    }
  }
  if (!Aria::parseArgs())
  {
    Aria::logOptions();
    Aria::exit(1);
    return NULL;
  }

  robot.runAsync(true);

  robot.lock();
  robot.comInt(28,0);
  robot.unlock();

  return robot;
}
