/** 
 *In the program's current state, the robot will follow a reflective point using the sick lidar
 */
#include "Aria.h"
#include <iostream>
#include <sicklms-1.0/SickLMS.hh>



using namespace std;
using namespace SickToolbox;

ArRobot initializeRobot(int argc, char ** argv);

int main(int argc, char **argv)
{

  //-----Connect to robot
  Aria::init();
  ArRobot robot = initializeRobot(argc, argv);

  //Connect to SICK
  string dev_path = "/dev/ttyUSB3";
  sick_lms_baud_t lms_baud = SickLMS::SICK_BAUD_38400;
  SickLMS sick_lms(dev_path);
  sick_lms.Initialize(lms_baud);

  //Instantiate variables for SICK
  unsigned int num_range_values;
  unsigned int num_reflect_values;
  unsigned int range_values[SickLMS::SICK_MAX_NUM_MEASUREMENTS] = {0};
  unsigned int reflect_values[SickLMS::SICK_MAX_NUM_MEASUREMENTS] = {0};

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
