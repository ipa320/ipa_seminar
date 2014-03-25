#pragma once
#include <string>
#include <iostream>
#include <sstream>
#include "pugixml/pugixml.hpp"
#include <map>
#include <vector>
#include <boost/algorithm/string.hpp>
#include "KukaFrame.h"
#include "KukaAxis.h"

using namespace boost;
using namespace std;


class XmlStringHandler
{
public:

	XmlStringHandler(void);
	~XmlStringHandler(void);

	std::string float2String(float value);

	int initAll(string filepath);
	
	int parseDataFromRobot(string inputStr, bool clear = true);//reads the string sended by the robot and extract the data, clears (if wanted structure before)

	void initSendStructure(string key, string valueType);//initialize the send structure manually ,.arg=Path , Value
	void initReceiveStructure(string key, string valueType);//initialize the receive manually ,.arg=Path , Value

	int setDataToRobot(string key, string value);//sets the data that is going to be send to the Robot,.arg=Path , Value
	int setDataToRobot(string key, const KukaFrame& value);//sets the data that is going to be send to the Robot,.arg=Path , Value
	int setDataToRobot(string key, const KukaAxis& value);//sets the data that is going to be send to the Robot,.arg=Path , Value

	void clearDataToRobot();

	string getDataToRobotValue(string key, int &error);//retrieve a (previously set) value from the data thats is going to be send to the Robot
	string getDataToRobotValueType(string key, int &error);//retrieve a (previously set) valueType of the data thats is going to be send to the Robot
	
	string createDataToRobotXml(); //prints out the hole XML output string
	string getAllDataFromRobot();//prints out the hole DatafromRobot map 
	string getDataFromRobotValue(string key, int &error);//retrieve a value from the data that was send by the Robot

	string getAttributeFromRobotValue(string key, string attr, int &error);//retrieve a attribute from the data that was send by the Robot

	string getDataFromRobotValueType(string key, int &error);//retrieve a value from the data that was send by the Robot
	string createXML(std::string File);

	int getIntFromRobot(string key, int &error);
	double getRealFromRobot(string key, int &error);
	KukaFrame getFrameFromRobot(string key, int &error);
	KukaAxis getAxisFromRobot(string key, int &error);

private:
	map <string,string> _receiveStructure;
	map <string,string> _dataToRobot;
	map <string,string> _dataFromRobot;
	map <string,string> _sendStructure;

};

