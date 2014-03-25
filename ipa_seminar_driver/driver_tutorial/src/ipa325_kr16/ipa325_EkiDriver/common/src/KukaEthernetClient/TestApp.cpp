// KukaEthernetAdapter.cpp : Defines the entry point for the console application.
//
#include "KukaEthernetClient.h"
#include <iostream>
#include "Poco\Logger.h"
#include "Poco\File.h"
#include "Poco\Path.h"
#include "XmlDefines.h"
#include "../GUI/src/LogManager.h"
#include "SILIA_Pathplanner.h"
#include <functional>

//#define FLUSHTEST

int currentMsgID;
clock_t start, finish;


void testCallback(int msgID)
{
	std::cout << "Called back with MsgID: " << msgID << std::endl;
	if (msgID == currentMsgID)
{
		finish = clock();
		cout << "Time needed:" << ( (finish - start)/CLOCKS_PER_SEC ) << endl;
	}
	}

float computeVelo(float x, float y, float z) 
		{
	//std::cout << "callback called - compute velocity!" << std::endl;
	float world_x = y;
	float world_y = -x;
	return 0.5f;
		}
	
int main(int argc, char* argv[])
{
		
	//init Pathplanner
	cout << "start planner ... " << endl;
	std::tr1::function<float(float, float, float)> callback;
	callback = &computeVelo;
	SILIA_Pathplanner planner(callback);
			
	vector<float> vec1; //<! start configuration
	vector<float> vec2; //<! goal configuration
	vector< pair<vector<float>, bool> > path; //<! target path, consists of start and goal configuration

	/*
	vec1.push_back(90);
	vec1.push_back(30);
	vec1.push_back(30);
	vec1.push_back(0);
	vec1.push_back(0);
	vec1.push_back(0);

	vec2.push_back(0);
	vec2.push_back(90);
	vec2.push_back(-90);
	vec2.push_back(90);
	vec2.push_back(90);
	vec2.push_back(90);
	path.push_back(pair<vector<float>, bool> (vec1, true));
	path.push_back(pair<vector<float>, bool> (vec2, true));

	*/

	vector<float> a2,b2;

	a2.push_back(-26.7);
	a2.push_back(-72.4);
	a2.push_back(78.7);
	a2.push_back(-0.5);
	a2.push_back(82.3);
	a2.push_back(17.9);
	
	/*b2.push_back(35.1);
	b2.push_back(-75.1);
	b2.push_back(84.3);
	b2.push_back(0.1);
	b2.push_back(79.3);
	b2.push_back(-3.6);
*/
	
	b2.push_back(-39.1);
	b2.push_back(-45.1);
	b2.push_back(34.3);
	b2.push_back(-0.1);
	b2.push_back(97.3);
	b2.push_back(-3.8);
	
		
	path.push_back(pair<vector<float>, bool> (a2, true));
	path.push_back(pair<vector<float>, bool> (b2, true));
	
	vector<pair<vector<float>,float>> solution = planner.computePath(path); //<! solution path consists of start, goal and intermediate configurations
	
	cout << "Path: " << endl;
	for(unsigned int i=0; i<solution.size(); i++)
	{
		for(unsigned int k=0; k<solution[i].first.size(); k++)
		{
			cout << solution[i].first[k] << " ";
		}
		cout << "|| Velocity: " << solution[i].second;
		cout << endl;
	}

	LogManager * _logManager;
	_logManager = new LogManager(NULL);

	KukaEthernetClient client;	

	cout << "start client..." << endl;
	getchar();

	client.Initialize("EKIServerFrame.xml", SocketAddress("192.1.10.20", 49152));
	//client.Initialize("TestKRL.xml", SocketAddress("localhost", 9978));
	client.setCallbackFcn(testCallback);

	cout << "start sending messages..." << endl;
	getchar();

	//client.movePTP(currentMsgID, KukaAxis(-33, -113, 100, 0, 100, 11), 10);// oben
	
	for(unsigned int i=0; i<solution.size(); i++)
	{
		currentMsgID = client.getMessageID();
		client.movePTP(currentMsgID, KukaAxis(solution[i].first), solution[i].second * 100);// oben
	}

	//float axSpeed_base = 10;
	//for (float i = 1; i <= 10; i++)
	//{	
	//	currentMsgID = client.getMessageID();
	//	client.movePTP(currentMsgID, KukaAxis(-33, -113, 100, 0, 100, 11), axSpeed);// oben
	//	//client.movePTP(client.getMessageID(), KukaAxis(-53, -114, 110, 0, 92,-9), axSpeed);// mitte rechts
	//	//client.movePTP(client.getMessageID(), KukaAxis(-33, -113, 118, 0, 83, 11), axSpeed);//unten
	//	//client.movePTP(client.getMessageID(), KukaAxis(-20, -109, 107, 0, 90, 24), axSpeed);// mitte links
	//}
	//getchar();
	////cout << "Press to close..." << endl;
	getchar();
	return 0;
}