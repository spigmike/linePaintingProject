/** 
 *In the program's current state, the robot will follow a reflective point using the sick lidar
 */
#include "Aria.h"
#include <iostream>
#include <math.h>
#include <sicklms-1.0/SickLMS.hh>
#include <fstream>
#include <sstream>
#include "gnuplot_i.hpp"

#define PI 3.14159265



using namespace std;
using namespace SickToolbox;

class Stake
{
	public:
	  double x;
	  double y;
	  Stake(double x,double y){this->x = x; this->y = y;};
};



class Line
{
	public:
	  double x1;
	  double y1;
	  double x2;
	  double y2; 
	  Line(double x1, double y1, double x2, double y2){this->x1=x1;this->y1=y1;this->x2=x2;this->y2=y2;};
};

class Plot
{
	public:
	  Plot(string style) : myPlot(style) { myPlot.cmd("set style arrow 1 nohead ls 1"); myPlot.cmd("set size square"); };
	  void plotStakes(vector<Stake> stake_locations);
	  void plotLines(vector<Line> line_locations);
	  void setRangeFromStakes(vector<Stake> stakes, double buffer);
	  void plotRobot(double x, double y, double a);
	private:
	  Gnuplot myPlot;
};

void Plot::plotStakes (vector<Stake> stake_locations){
	vector<double> stake_x;
  	vector<double> stake_y;
 	for(unsigned int i=0; i< stake_locations.size(); i++){
		cout << "x = " << stake_locations[i].x << "y = " << stake_locations[i].y;
		stake_x.push_back(stake_locations[i].x);
		stake_y.push_back(stake_locations[i].y);
	}
	myPlot.plot_xy(stake_x,stake_y,"Stake Locations");
	
}

void Plot::plotLines (vector<Line> line_locations){
	for(unsigned int i=0; i< line_locations.size(); i++){
	stringstream command; 
	command << "set arrow from " << line_locations[i].x1 << "," << line_locations[i].y1 << " to " << line_locations[i].x2 << "," << line_locations[i].y2 << " as 1";
	string thisString = command.str();
	const char * c = thisString.c_str();
	myPlot.cmd(c);
	}
}

void Plot::setRangeFromStakes(vector<Stake> stakes, double buffer){
	double x_min;
	double x_max;
	double y_min;
	double y_max;
	if(stakes.size() > 0){
		x_min = stakes[0].x;
		x_max = stakes[0].x;
		y_min = stakes[0].y;
		y_max = stakes[0].y;
		for(unsigned int i=1; i<stakes.size(); i++){
		if (stakes[i].x > x_max) x_max = stakes[i].x;
		if (stakes[i].x < x_min) x_min = stakes[i].x;
		if (stakes[i].y > y_max) y_max = stakes[i].y;
		if (stakes[i].y < y_min) y_min = stakes[i].y;
		}
		cout << x_min << " " << y_min << " " << x_max << " " << y_max << endl;
	}
	myPlot.set_xrange(x_min-buffer,x_max+buffer);
	myPlot.set_yrange(y_min-buffer,y_max+buffer);
}

void Plot::plotRobot(double x,double y, double a){
	stringstream command;
	a = a*PI / 180;
	double x1 = (-.1*cos(a)) - (-.1*sin(a)) + x;
	double y1 = (-.1*sin(a)) + (-.1*cos(a)) + y;
	double x2 = (-.1*cos(a)) - (.1*sin(a)) + x;
	double y2 = (-.1*sin(a)) + (.1*cos(a)) + y;
	double x3 = (.2*cos(a)) - (0*sin(a)) + x;
	double y3 = (.2*sin(a)) + (0*cos(a)) + y;
	command << "set object 1 polygon from " << x1 << "," << y1 << " to " << x2 << "," << y2 << " to " << x3 << "," << y3 << " to " << x1 << "," << y1;
	string thisString = command.str();
	const char * c = thisString.c_str();
	myPlot.cmd(c);
	myPlot.cmd("set object 1 fc rgb '#0000FF' fillstyle solid lw 0");
}

class LIDAR
{
	public:

	void connect();
	void disconnect();
	vector<Stake> getActualStakeLocations();
        LIDAR(string path) : sick_lms(path) { };   // PATH TO LIDAR GOES HERE
	SickLMS sick_lms;

	private:

         unsigned int num_range_values;
  	 unsigned int num_reflect_values;
  	 unsigned int range_values[SickLMS::SICK_MAX_NUM_MEASUREMENTS];
  	 unsigned int reflect_values[SickLMS::SICK_MAX_NUM_MEASUREMENTS];

};

void LIDAR::connect(){
	sick_lms_baud_t lms_baud = SickLMS::SICK_BAUD_38400;
  	sick_lms.Initialize(lms_baud);
}

void LIDAR::disconnect(){
	 sick_lms.Uninitialize();
}

vector<Stake> LIDAR::getActualStakeLocations(){

	 sick_lms.GetSickScan(range_values,reflect_values,num_range_values,num_reflect_values);
	 vector<Stake> actualStakeLocations;
	 bool onStake = false;
	 int stakeNumMeas = 0;  
	 double stakeTotalRange = 0.0;
	 for (unsigned int i=0; i<num_range_values; i++){
		if(reflect_values[i] > 150 && onStake == false){
			onStake = true;
			stakeNumMeas = 1;
			stakeTotalRange += range_values[i];
		}
		else if (reflect_values[i] > 150 && onStake == true){
			stakeNumMeas++;
			stakeTotalRange += range_values[i];
		}
		else if (onStake == true){
			double avg_dist = (double) stakeTotalRange / (double) stakeNumMeas;
			double avg_angle = 45.0 + (90.0/181.0) * ((double) i - (stakeNumMeas / 2.0));
			double this_x = avg_dist * cos(PI * avg_angle / 180.0);
			double this_y = avg_dist * sin(PI * avg_angle / 180.0);
			Stake this_stake(this_x,this_y);
			actualStakeLocations.push_back(this_stake);
			onStake = false;
			stakeTotalRange = 0;
			stakeNumMeas = 0;
		}
	 }
	 return actualStakeLocations; 
}

vector<Stake> loadStakeLocations(const char *filename);
vector<Line> loadLineLocations(const char *filename);
vector<bool> visibleStakes(double x_pos, double y_pos, double angle, vector<Stake> stakes);

int main(int argc, char **argv)
{
  vector<Stake> stake_locations = loadStakeLocations("stakes.txt");
  vector<Line> line_locations = loadLineLocations("lines.txt");
  vector<bool> visible_stakes = visibleStakes(1,0,90,stake_locations);
  Plot initialPlot("points");
  initialPlot.setRangeFromStakes(stake_locations,1.0);
  initialPlot.plotRobot(1,0,90); 
  initialPlot.plotLines(line_locations);
  initialPlot.plotStakes(stake_locations);
  usleep(10000000);
  
  
  //-----Connect to robot
  Aria::init();					//initialize Aria interface	
  ArRobot robot;				//create robot object
  ArArgumentParser parser(&argc, argv);		//Load command line args into parser (use -rp /dev/ttyUSB0 or other device path)
  parser.loadDefaultArguments();		 

  ArRobotConnector robotConnector(&parser, &robot); //connect to robot using parsed arguments

  if(!robotConnector.connectRobot())		//check connection
  {
    ArLog::log(ArLog::Terse, "simpleMotionCommands: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
    }
  }
  if (!Aria::parseArgs())
  {
    Aria::logOptions();
    Aria::exit(1);
  }

  robot.runAsync(true);

  robot.lock();
  robot.comInt(28,0);				//turn off sonar - robot.comInt(28,1) to reenable
  robot.unlock();


  //Connect to SICK
  LIDAR myLidar("/dev/ttyUSB0");
  myLidar.connect();		//dev path of SICK lidar
  

  /* BODY OF MAIN CODE WILL GO HERE */		
			

  myLidar.disconnect();

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Ending robot thread...");
  robot.stopRunning();

  // wait for the thread to stop
  robot.waitForRunExit();

  // exit
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
  Aria::exit(0);

  return 0;
}

vector<Stake> loadStakeLocations(const char *filename){
	vector<Stake> loadedStakes; 
	double x;
	double y;
	ifstream infile;
	infile.open(filename); 
	string line;
	while(getline(infile,line)){
		istringstream istr(line);
		istr >> x;
		istr >> y;
		Stake thisStake(x,y);
		cout << thisStake.x;
		cout << "Loaded stake w/ pos x = " << x << " y = " << y <<endl; 
		loadedStakes.push_back(thisStake);
	}
	infile.close();
	return loadedStakes; 		
}

vector<Line> loadLineLocations(const char *filename){
	vector<Line> loadedLines;
	double x1;
	double y1;
	double x2;
	double y2;
	ifstream infile;
	infile.open(filename);
	string line;
	while(getline(infile,line)){
		istringstream istr(line);
		istr >> x1;
		istr >> y1;
		istr >> x2;
		istr >> y2;
		Line thisLine(x1,y1,x2,y2);
		cout << "Loaded line with x1,y1 = " << x1 << "," << y1 << " x2,y2 = " << x2 << "," << y2 << endl;
		loadedLines.push_back(thisLine);
	}
	infile.close();
	return loadedLines;
}

vector<bool> visibleStakes(double x_pos, double y_pos, double angle, vector<Stake> stakes){
	vector<bool> visibility_vector;
	for(unsigned int i=0; i<stakes.size(); i++){
		cout << "x = " << stakes[i].x << " y = " << stakes[i].y << endl;
		double stake_angle = 0;
		if (stakes[i].x - x_pos == 0){
			if(stakes[i].y - y_pos >= 0) stake_angle = 90;
			else stake_angle = 270;
			cout << stake_angle << endl;
		}
		else{
			cout << 180 * atan2((stakes[i].y - y_pos),(stakes[i].x - x_pos)) / PI << endl;
			stake_angle = 180 * atan2((stakes[i].y - y_pos),(stakes[i].x - x_pos)) / PI;
		}
		if (fabs(angle - stake_angle) < 45){
			 visibility_vector.push_back(true); 
			 cout << "Visible Stake @ " << stakes[i].x << "," << stakes[i].y;
		}
		else visibility_vector.push_back(false);
	}
	return visibility_vector;
	
}
	


