#include <KukaEthernetClient/XmlStringHandler.h>
#include <cstdio>
#define OUTPUT true
XmlStringHandler::XmlStringHandler(void)
{
}


XmlStringHandler::~XmlStringHandler(void)
{
}

std::string XmlStringHandler::float2String(float value)
{
	char buffer[50];
	int n;
	n = sprintf(buffer, "%f", value);
	return std::string(buffer, n);
}

int XmlStringHandler::parseDataFromRobot(string inputStr, bool clear)
{
        if (clear)
	{
		_dataFromRobot.clear();
	}
		
	map <string,string>::iterator iterator;
	pugi::xml_document doc;	
	pugi::xml_parse_result result = doc.load_buffer(inputStr.data(), inputStr.length());
	for (iterator = _receiveStructure.begin(); iterator != _receiveStructure.end(); iterator++)
	{//goes trough all the sendstructure map to retrieve the data that we have.
		vector<string> strs;//create vector strs
		boost::split(strs,iterator->first,boost::is_any_of("@")); //save in strs the result of string1 taking the "@".
		string path, attribute, child;
		if(iterator->first!=strs[0])
		{
			//stop
			attribute=strs[1]; //saves the attribute.
			trim_right_if(strs[0],is_any_of("/")); //remove the last / of the path. 
			path=strs[0]; // path have the value of strs[0].
			pugi::xpath_node build_tool = doc.select_single_node(path.data());
			_dataFromRobot.insert(make_pair(iterator->first,build_tool.node().attribute(attribute.data()).value()));
		}
		else
		{
			boost::split(strs,iterator->first,boost::is_any_of("/"));
			path=strs[0];
			child=strs[1];
			pugi::xml_node tool = doc.child(path.data());
			_dataFromRobot.insert(make_pair(iterator->first,tool.child_value(child.data())));
		}
	}
	return 0;
}

void XmlStringHandler::initSendStructure(string key, string valueType)
{  //make the item in the Send Structure map.
	_receiveStructure.insert(make_pair(key.data(), valueType.data()));
	// cout <<"the value that i have inserted is  "<<Key.data()<<" and value  "<<Sendstructure[Key.data()] <<endl;
}


void XmlStringHandler::initReceiveStructure(string key, string valueType)
{ //make the item in the Receive Structure map
	_sendStructure.insert(make_pair(key.data(), valueType.data()));
	// cout <<"the value that i have in "<<Key.data()<<" is "<< Receivestructure[Key.data()] <<endl;
}

void XmlStringHandler::clearDataToRobot()
{
	_dataToRobot.clear();
}

int XmlStringHandler::setDataToRobot(string key, string value)
{ // get the new values, if not in structure send 1 as an error , else he creates or modify the existent one 
	map <string,string>::iterator iterator;
	iterator=_sendStructure.find(key.data());
	if (iterator==_sendStructure.end())
	{
		std::cout << "Cannot find Value @ "<< key << " in DataToRobot structure" << endl;
		return -1;
	}
	else 
	{
		iterator=_dataToRobot.find(key.data());
		if (iterator==_dataToRobot.end())
		{
			_dataToRobot.insert(make_pair(key.data(),value.data()));

		} 
		else
		{
			_dataToRobot[key.data()]=value.data();

		}
	}
	return 0;
}

int XmlStringHandler::setDataToRobot(string key, const KukaAxis& value)
{
	map <string,string>::iterator iterator;
	iterator=_sendStructure.find(key.data());
	if (iterator==_sendStructure.end())
	{
		std::cout << "Cannot find Value  "<< key << " in DataToRobot structure" << endl;
		return -1;
	}
	else 
	{
		iterator=_dataToRobot.find(key.data());
		if (iterator==_dataToRobot.end())
		{
			_dataToRobot.insert(make_pair(key.data(), ""));
			_dataToRobot.insert(make_pair((key + "/@A1").data(), float2String(value.a[0]).data()));
			_dataToRobot.insert(make_pair((key + "/@A2").data(), float2String(value.a[1]).data()));
			_dataToRobot.insert(make_pair((key + "/@A3").data(), float2String(value.a[2]).data()));
			_dataToRobot.insert(make_pair((key + "/@A4").data(), float2String(value.a[3]).data()));
			_dataToRobot.insert(make_pair((key + "/@A5").data(), float2String(value.a[4]).data()));
			_dataToRobot.insert(make_pair((key + "/@A6").data(), float2String(value.a[5]).data()));
		} 
		else
		{
			_dataToRobot[(key + "/@A1").data()]=float2String(value.a[0]).data();
			_dataToRobot[(key + "/@A2").data()]=float2String(value.a[1]).data();
			_dataToRobot[(key + "/@A3").data()]=float2String(value.a[2]).data();
			_dataToRobot[(key + "/@A4").data()]=float2String(value.a[3]).data();
			_dataToRobot[(key + "/@A5").data()]=float2String(value.a[4]).data();
			_dataToRobot[(key + "/@A6").data()]=float2String(value.a[5]).data();
		}
	}
	return 0;
}

int XmlStringHandler::setDataToRobot(string key, const KukaFrame& value)
{
	map <string,string>::iterator iterator;
	iterator=_sendStructure.find(key.data());
	if (iterator==_sendStructure.end())
	{
		std::cout << "Cannot find Value  "<< key << " in DataToRobot structure" << endl;
		return -1;
	}
	else 
	{
		iterator=_dataToRobot.find(key.data());
		if (iterator==_dataToRobot.end())
		{
			_dataToRobot.insert(make_pair(key.data(), ""));
			_dataToRobot.insert(make_pair((key + "/@X").data(), float2String(value.a[0]).data()));
			_dataToRobot.insert(make_pair((key + "/@Y").data(), float2String(value.a[1]).data()));
			_dataToRobot.insert(make_pair((key + "/@Z").data(), float2String(value.a[2]).data()));
			_dataToRobot.insert(make_pair((key + "/@A").data(), float2String(value.a[3]).data()));
			_dataToRobot.insert(make_pair((key + "/@B").data(), float2String(value.a[4]).data()));
			_dataToRobot.insert(make_pair((key + "/@C").data(), float2String(value.a[5]).data()));
		} 
		else
		{
			_dataToRobot[(key + "/@X").data()]=float2String(value.a[0]).data();
			_dataToRobot[(key + "/@Y").data()]=float2String(value.a[1]).data();
			_dataToRobot[(key + "/@Z").data()]=float2String(value.a[2]).data();
			_dataToRobot[(key + "/@A").data()]=float2String(value.a[3]).data();
			_dataToRobot[(key + "/@B").data()]=float2String(value.a[4]).data();
			_dataToRobot[(key + "/@C").data()]=float2String(value.a[5]).data();
		}
	}
	return 0;
}

string XmlStringHandler::getDataToRobotValueType(string key, int &error)
{
	map <string,string>::iterator iterator;
	string value = ""; 
	iterator = _sendStructure.find(key.data());
	if (iterator == _sendStructure.end())
	{
		std::cout << "Cannot find ValueType @ "<< key << " in DataToRobot structure" << endl;
		error = -1;
	}
	else
	{
		value=iterator->second;
		error = 0;
		//cout << value.data() <<endl;
	}
	return value;
}

int XmlStringHandler::getIntFromRobot(string key, int &error)
{
	int parsingError = -1;
	int structureError = -1;

	int result = 0;
	string value = "";
	value = getDataFromRobotValue(key, parsingError);
	if (!getDataFromRobotValueType(key, parsingError).compare("INT"))
	{
		structureError = 0;
	}
	if (!parsingError)
	{
		result = atoi(value.c_str());
		error = structureError;
	}
	else
	{
		error = parsingError;
	}

	return result;
}

KukaFrame XmlStringHandler::getFrameFromRobot(string key, int &error)
{
	int parsingError = -1;
	int structureError = -1;

	float x,y,z,a,b,c;
	string value = "";

// Typenkontrolle �bergangen weil KUKA kein wirklichen Typ f�hrt
/*
	if (!getDataFromRobotValueType(key, parsingError).compare("FRAME"))
	{
		structureError = 0;
	}
*/
	value = getAttributeFromRobotValue(key, "X", parsingError);
	x = atof(value.c_str());
	value = getAttributeFromRobotValue(key, "Y", parsingError);
	y = atof(value.c_str());
	value = getAttributeFromRobotValue(key, "Z", parsingError);
	z = atof(value.c_str());
	value = getAttributeFromRobotValue(key, "A", parsingError);
	a = atof(value.c_str());
	value = getAttributeFromRobotValue(key, "B", parsingError);
	b = atof(value.c_str());
	value = getAttributeFromRobotValue(key, "C", parsingError);
	c = atof(value.c_str());

	return KukaFrame(x,y,z,a,b,c);
}

KukaAxis XmlStringHandler::getAxisFromRobot(string key, int &error)
{
	int parsingError = -1;
	int structureError = -1;

	float a1, a2, a3, a4, a5, a6;
	string value = "";

// Typenkontrolle �bergangen weil KUKA kein wirklichen Typ f�hrt
/*
	if (!getDataFromRobotValueType(key, parsingError).compare("FRAME"))
	{
		structureError = 0;
	}
*/
	value = getAttributeFromRobotValue(key, "A1", parsingError);
	a1 = atof(value.c_str());
	value = getAttributeFromRobotValue(key, "A2", parsingError);
	a2 = atof(value.c_str());
	value = getAttributeFromRobotValue(key, "A3", parsingError);
	a3 = atof(value.c_str());
	value = getAttributeFromRobotValue(key, "A4", parsingError);
	a4 = atof(value.c_str());
	value = getAttributeFromRobotValue(key, "A5", parsingError);
	a5 = atof(value.c_str());
	value = getAttributeFromRobotValue(key, "A6", parsingError);
	a6 = atof(value.c_str());

	return KukaAxis(a1,a2,a3,a4,a5,a6);
}

double XmlStringHandler::getRealFromRobot(string key, int &error)
{
	int parsingError = -1;
	int structureError = -1;

	double result = 0;
	string value = "";
	value = getDataFromRobotValue(key, parsingError);
	if (!getDataFromRobotValueType(key, parsingError).compare("REAL"))
	{
		structureError = 0;
	}
	if (!parsingError)
	{
		result = atof(value.c_str());
		error = structureError;
	}
	else
	{
		error = parsingError;
	}

	return result;

}

string XmlStringHandler::getDataToRobotValue(string key, int &error)
{
	map <string,string>::iterator iterator;
	string value = ""; 
	iterator=_dataToRobot.find(key.data());
	if (iterator==_dataToRobot.end())
	{
		std::cout << "Cannot find Value @ "<< key << " in DataToRobot" << endl;
		error = -1;
	}
	else
	{
		value=iterator->second;
		error = 0;
		//cout << value.data() <<endl;
	}
	return value;
}

string XmlStringHandler::getDataFromRobotValueType(string key, int &error)
{
	map <string,string>::iterator iterator;
	string value = ""; 
	iterator=_receiveStructure.find(key.data());
	if (iterator==_receiveStructure.end())
	{
		std::cout << "Cannot find ValueType @ "<< key << " in DataFromRobot structure" << endl;
		error = -1;
	}
	else
	{
		value=iterator->second;
		error = 0;
	}
	return value;
}

string XmlStringHandler::getDataFromRobotValue(string key, int &error)
{
	map <string,string>::iterator iterator;
	string value = ""; 
	iterator=_dataFromRobot.find(key.data());
	if (iterator==_dataFromRobot.end())
	{
		std::cout << "Cannot find Value @ "<< key << " in DataFromRobot" << endl;
		error = -1;
	}
	else
	{
		value=iterator->second;
		error = 0;
	}
	return value;
}

string XmlStringHandler::getAttributeFromRobotValue(string key, string attr, int &error)
{
	map <string,string>::iterator iterator;
	string value = "";
	key.append("/@");
	key.append(attr);

	iterator=_dataFromRobot.find(key.data());
	if (iterator==_dataFromRobot.end())
	{
		std::cout << "Cannot find Value @ "<< key << " in DataFromRobot" << endl;
		error = -1;
	}
	else
	{
		value=iterator->second;
		error = 0;
	}
	return value;
}


int XmlStringHandler::initAll(string filePath)
{
	pugi::xml_document doc;
	string path;
	string Value;
	if (!doc.load_file(filePath.c_str()))
	{
		std::cout << "Cannot find file: " << filePath << endl;;
		return -1;
	}
	//selecting send (from KUKA point of view)
	pugi::xml_node node = doc.child("ETHERNETKRL").child("SEND").child("XML");

	for (pugi::xml_node node2 = node.first_child(); node2; node2= node2.next_sibling())
	{
		pugi::xml_attribute attr = node2.first_attribute();
		path=attr.value();
		attr = attr.next_attribute();
		Value=attr.value();
		_receiveStructure.insert(make_pair(path.data(),Value.data()));
	}
	//selecting receive (from KUKA point of view)
	pugi::xml_node node3 = doc.child("ETHERNETKRL").child("RECEIVE").child("XML");
	for (pugi::xml_node node4 = node3.first_child(); node4; node4= node4.next_sibling())
	{
		pugi::xml_attribute attr = node4.first_attribute();
		path=attr.value();
		attr = attr.next_attribute();
		Value=attr.value();
		_sendStructure.insert(make_pair(path.data(),Value.data()));
	}

	return 0;
}

string XmlStringHandler::createDataToRobotXml()
{
	map <string,string>::iterator iterator;
	pugi::xml_document doc;
	ostringstream outs;
	iterator=_dataToRobot.begin();
	vector<string> strs1;
	boost::split(strs1,iterator->first,boost::is_any_of("/")); 
	string starter;
	starter=strs1[0];
	pugi::xml_node node = doc.append_child(starter.data());
	for (iterator=_dataToRobot.begin();iterator!=_dataToRobot.end();iterator++)
	{     
		vector<string> strs;
		boost::split(strs,iterator->first,boost::is_any_of("@")); 
		string path, attribute, child,Key,search;
		Key=_dataToRobot[iterator->first];
		if(iterator->first!=strs[0])//function for paths with attributes
		{
			attribute=strs[1];
			boost::split(strs,iterator->first,boost::is_any_of("/")); 
			child=strs[1];
			for (pugi::xml_node tool = node.child(child.data()); tool; tool = tool.next_sibling(child.data()))
			{
				search=tool.name();

			}
			if(search.empty())
				node.append_child(child.data());
			node.child(child.data()).append_attribute(attribute.data()) =Key.data();
			//cout<<DatatoRobot[Key.data()]<<endl;
		}
		else//function for paths without attributes
		{
			boost::split(strs,iterator->first, boost::is_any_of("/"));
			child=strs[1];
			node.append_child(child.data());
			node.child(child.data()).append_child(pugi::node_pcdata).set_value(Key.data());

		}
	}
	string s;
	doc.save(outs);
	s=outs.str();
	return s;
}



string XmlStringHandler::getAllDataFromRobot()
{
	map <string,string>::iterator iterator;
	string res = "";
	for (iterator=_dataFromRobot.begin(); iterator!=_dataFromRobot.end(); iterator++)
	{
		cout << iterator->first << "->" << iterator->second << endl;
		res.append(iterator->first);
		res.append("->");
		res.append(iterator->second);
	}
	return res;
}


std::string XmlStringHandler::createXML(std::string File)
{
	std::map <std::string,std::string>::iterator iterator;
 pugi::xml_document doc;
 ostringstream outs;
 iterator=_dataToRobot.begin();
 pugi::xml_node node = doc.append_child("ETHERNETKRL");
 node = node.append_child("CONFIGURATION");
	node.append_child("EXTERNAL");
	node.child("EXTERNAL").append_child("IP").append_child(pugi::node_pcdata).set_value("172.1.10.5");
	node.child("EXTERNAL").append_child("PORT").append_child(pugi::node_pcdata).set_value("9978");
	node.child("EXTERNAL").append_child("TYPE").append_child(pugi::node_pcdata).set_value("Client");
 node.append_child("INTERNAL");
 node.child("INTERNAL").append_child("ENVIRONMENT").append_child(pugi::node_pcdata).set_value("172.1.10.5");
 node.child("INTERNAL").append_child("IP").append_child(pugi::node_pcdata).set_value("172.1.10.5");
 node.child("INTERNAL").append_child("BUFFERING").append_attribute("Mode").set_value("FIFO");
 node.child("INTERNAL").append_child("BUFFERING").append_attribute("Limit").set_value("10");
 node.child("INTERNAL").append_child("BUFFSIZE").append_attribute("Limit").set_value("16384");
  node.child("INTERNAL").append_child("TIMEOUT").append_attribute("Connect").set_value("60000");
	node.child("INTERNAL").append_child("ALIVE").append_attribute("Set_Out").set_value("666");
	node.child("INTERNAL").append_child("ALIVE").append_attribute("Ping").set_value("200");
	  node.child("INTERNAL").append_child("PORT").append_child(pugi::node_pcdata).set_value("192.1.10.20");
 node.child("INTERNAL").append_child("PORT").append_child(pugi::node_pcdata).set_value("49152");
 node.child("INTERNAL").append_child("PROTOCOL").append_child(pugi::node_pcdata).set_value("TCP");
 node=node.parent();
 node=node.append_child("SEND");
 node=node.append_child("XML");

  for (iterator=_dataToRobot.begin();iterator!=_dataToRobot.end();iterator++)
  {     
	  node= node.append_child("Element");
		node.append_attribute("Tag").set_value(iterator->first.data());
		node.append_attribute("Type").set_value(iterator->second.data());
		node=node.parent();
		
	}
  node=node.parent();
  node=node.parent();
  node=node.append_child("RECEIVE");
  node=node.append_child("XML");

  for (iterator=_dataFromRobot.begin();iterator!=_dataFromRobot.end();iterator++)
  {     
	  node= node.append_child("Element");
		node.append_attribute("Tag").set_value(iterator->first.data());
		node.append_attribute("Type").set_value(iterator->second.data());
		node=node.parent();
		
	}
  std::string s;
  doc.save(outs);
  doc.save_file(File.data());
  s=outs.str();
  return s;
   
}


