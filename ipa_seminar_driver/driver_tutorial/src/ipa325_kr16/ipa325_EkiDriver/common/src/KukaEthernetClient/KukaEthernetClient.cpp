#define SENDING_SLEEPING_TIME_MS 1000
#define NO_LOG_AVAILABLE
#pragma once
#include <KukaEthernetClient/KukaEthernetClient.h>
#include <KukaEthernetClient/XmlDefines.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <functional>
#include <cstdio>
#include "boost/lexical_cast.hpp"


KukaEthernetClient::SenderRunnable::SenderRunnable(KukaEthernetClient* parent)
{
	_parent = parent;
	_run = true;
	_pause = false;
}

KukaEthernetClient::SenderRunnable::~SenderRunnable()
{

}

bool KukaEthernetClient::SenderRunnable::isRunning()
{
	return _run;
}

void KukaEthernetClient::SenderRunnable::stop()
{
	_pause = true;
}

void KukaEthernetClient::SenderRunnable::start()
{
	_pause = false;
}

void KukaEthernetClient::SenderRunnable::run()
{
	Poco::Mutex mutex;
	mutex.lock();
	while (_run)
	{
		if (!_pause)
		{
			_parent->_messageMutex.lock();
			if (_parent->_messagePendingList.size() == 0)
			{
				_parent->_messageMutex.unlock();
				mutex.lock();
				_parent->_listPendingNotEmptyCondition.wait(mutex);
			}
			else
			{
				_parent->_messageMutex.unlock();		
				_parent->sendNextMessage();
				Poco::Thread::sleep(SENDING_SLEEPING_TIME_MS);
			}
		}
		else
		{
			Poco::Thread::sleep(250);
		}
	}
}

//******************* KUKA Client *************************//

KukaEthernetClient::KukaEthernetClient()
	:	_isConnected(false),
	_isXMLInit(false),
	_activeMessages(0),
        _messageCount(1),
	_intermediateCallbackFunction(NULL)
{
	_ethernetClient.reset(new StringEthernetClient());
	_xmlHandler.reset(new XmlStringHandler());
	_workerThread.reset(new Thread);
	_senderWorker.reset(new SenderRunnable(this));
	_workerThread->start(*_senderWorker);

	//_messageMutex.unlock();
}

KukaEthernetClient::~KukaEthernetClient()
{
	_messageMutex.unlock();
}


void KukaEthernetClient::setCallbackFcn(IntermediateCallbackFunctionPtr callback)
{
	_intermediateCallbackFunction = callback;
}

void KukaEthernetClient::pause()
{
	_senderWorker->stop();
}

void KukaEthernetClient::start()
{
	_senderWorker->start();
}

int KukaEthernetClient::getError()
{
    int error = 0;
    if(_isConnected && _isXMLInit)
    {
        return error;
    }
    else
    {
        if (!_isConnected)
        {
            error += 1;
        }
        if (!_isXMLInit)
        {
            error += 2;
        }
    }
    return error;

}

bool KukaEthernetClient::checkRepliedMsgID(int messageID, list<commandMessage>& list)
{
    if(_ethernetClient->isDummy())
    {
        return true;
    }

	bool result = false;
	if (list.size() > 0)
	{
		result = (messageID == list.begin()->messageID);
		if (!result)
		{
#ifdef NO_LOG_AVAILABLE
                        std::cout << "Message ID compare failed (Exp: " << list.begin()->messageID << ", Recv: " << messageID << ")" << std::endl;
#else
                        logs.warning() << "Message ID compare failed (Exp: " << list.begin()->messageID << ", Recv: " << messageID << ")" << std::endl;
#endif
		}
	}
	else
	{
#ifdef NO_LOG_AVAILABLE
		std::cout << "Message ID compare failed (List was empty)" << std::endl;
#else
		logs.warning() << "Message ID compare failed (List was empty)" << std::endl;
#endif
	}
	return result;
}

int KukaEthernetClient::messageCallBack(std::string data)
{
	_xmlHandler->parseDataFromRobot(data);

#ifdef NO_LOG_AVAILABLE
	std::cout << "Received Message:" << endl << data << endl;
#else
	logs.warning() << "Received Message:" << endl << data << endl;
#endif

	int parsingError = -1;
	int resultError = 0;
        int messageID = 0;

        if (!_ethernetClient->isDummy())
        {
            messageID = _xmlHandler->getIntFromRobot(TAG_RECV_MSG_ID, parsingError);
        }
        else
        {
            // When Client is in Dummy Mode check MsgID against the one that was send (Dummy mode will only echo back)
            messageID = _xmlHandler->getIntFromRobot(TAG_SEND_MSG_ID, parsingError);
        }

	int confirmation = _xmlHandler->getIntFromRobot(TAG_RECV_CONFIRM, parsingError);
        int jobId = _xmlHandler->getIntFromRobot(TAG_RECV_SUB_ID, parsingError);


	// for debugging purposes (Check Sync between KRC and PC)
	int runPointer = _xmlHandler->getIntFromRobot(TAG_RECV_RUN_POINTER, parsingError);
	int recvPointer = _xmlHandler->getIntFromRobot(TAG_RECV_RECV_POINTER, parsingError);
	int slot[3] = {-2};
	slot[0] = _xmlHandler->getIntFromRobot(TAG_RECV_SLOT1, parsingError);
	slot[1] = _xmlHandler->getIntFromRobot(TAG_RECV_SLOT2, parsingError);
	slot[2] = _xmlHandler->getIntFromRobot(TAG_RECV_SLOT3, parsingError);

	KukaFrame myframe;
        myframe = _xmlHandler->getFrameFromRobot(TAG_RECV_ACTPOS, parsingError);
        setCurrentFrame(myframe);

        KukaAxis myAxis;
        myAxis = _xmlHandler->getAxisFromRobot(TAG_RECV_ACTAXIS, parsingError);
        setCurrentAxis(myAxis);

	if (!checkSync(&slot[0], 3, recvPointer, runPointer))
	{
#ifdef NO_LOG_AVAILABLE
		std::cout <<  "KukaEthernetClient and KRL seem out of Sync!" << endl;
#else
		logs.critical() <<  "KukaEthernetClient and KRL seem out of Sync!" << endl;
#endif
	}

	if(!parsingError)
	{
		_messageMutex.lock();
		if (checkRepliedMsgID(messageID, _messageActiveList))	// messageID wurde gefunden und vom Kuka durchgef�hrt
		{
			_messageDoneList.push_back(*_messageActiveList.begin());
			_messageActiveList.pop_front();
			_activeMessages--;

			if (_intermediateCallbackFunction != NULL)
			{
				_intermediateCallbackFunction(messageID);
			}
		}
		else
		{
#ifdef NO_LOG_AVAILABLE
                        std::cout << "Message " << messageID << ", JobId: " << jobId << " was performed BUT cannot be found to be sent before! (activeList)" << endl;
#else
                        logs.warning() <<  "Message " << messageID << ", JobId: " << jobId << " was performed BUT cannot be found to be sent before! (activeList)" << endl;
#endif
			resultError = 3;
		}
		_messageMutex.unlock();
	}
	else
	{
#ifdef NO_LOG_AVAILABLE
		std::cout << "Parsing Error. Message: " << messageID << "ignored." << endl;
#else
		logs.warning() << "Parsing Error. Message: " << messageID << "ignored." << endl;
#endif
		resultError = -1;
	}

	return resultError;
}

bool KukaEthernetClient::Initialize(std::string pathKrlEthernet, const SocketAddress& sa, bool autoReconnect)
{

	boost::function<int (std::string)> f3( boost::bind( &KukaEthernetClient::messageCallBack, this, _1 ) );
	_ethernetClient->setCallbackFcn(f3);

	std::cout << "Connecting KukaEthernetClient" << endl;
        _isConnected = _ethernetClient->connect(sa, autoReconnect);

	std::cout << "Initializing XmlStringHandler" << endl;
	_isXMLInit = !_xmlHandler->initAll(pathKrlEthernet);
	return (_isConnected && _isXMLInit);
}

void KukaEthernetClient::setClientDummy(bool dummy)
{
    _ethernetClient->setDummy(dummy);
}

StringEthernetClientPtr KukaEthernetClient::getEthernetClient()
{
	return _ethernetClient;
}

XmlStringHandlerPtr KukaEthernetClient::getXmlhandler()
{
	return _xmlHandler;
}

void KukaEthernetClient::addMessage(const commandMessage& i_message, bool flush)
{
	_messageMutex.lock();
	if (!flush)
	{
		_messagePendingList.push_back(i_message);
	}
	else
	{
		_messagePendingList.clear();
		_messagePendingList.push_back(i_message);
		_messageActiveList.resize(1);
		_activeMessages = _messageActiveList.size();
	}
	_messageMutex.unlock();

	_listPendingNotEmptyCondition.broadcast();
}

bool KukaEthernetClient::checkSync(int* slot, int count, int recvPointer, int runPointer)
{
	if (_messageActiveList.size() > count)
	{
#ifdef NO_LOG_AVAILABLE
		std::cout <<  "More messages in active list than defined!" << endl;
#else
		logs.critical() <<  "More messages in active list than defined!" << endl;
#endif  
		return false;
	}

	string info = "";

	bool result = true;
	list<commandMessage>::iterator iterActivList = _messageActiveList.begin();
	while (iterActivList != _messageActiveList.end())
	{
		result &= (iterActivList->messageID == slot[runPointer-1]);
		info.append(boost::lexical_cast<string>(iterActivList->messageID) + "<c-r>" + boost::lexical_cast<string>(slot[runPointer-1]) + "; ");
		iterActivList++;
		//wrap around
		if(runPointer < count)
		{
			runPointer++;
		}
		else
		{
			runPointer = 1;
		}
	}

	if (!result)
	{
#ifdef NO_LOG_AVAILABLE
		std::cout <<  "Sync Problem! ["<< info << "]" << endl;
#else
		logs.critical() <<  "Sync Problem! ["<< info << "]" << endl;
#endif
	}
	else
	{
#ifdef NO_LOG_AVAILABLE
		std::cout <<  "Sync OK! ["<< info << "]" << endl;
#else
		logs.debug() <<  "Sync OK! ["<< info << "]" << endl;
#endif
	}
	return result;
}

int KukaEthernetClient::getMessageID()
{
	int msgID = _messageCount;

	if (_messageCount < INT_MAX)
	{
		_messageCount++;
	}
	else
	{
                _messageCount = 1;
	}

	return msgID;
}

string KukaEthernetClient::int2String(int value)
{
	char buffer[50];
	int n;
	n = sprintf(buffer, "%d", value);
	return string(buffer, n);
}

string KukaEthernetClient::float2String(float value)
{
	char buffer[50];
	int n;
	n = sprintf(buffer, "%f", value);
	return string(buffer, n);
}

void KukaEthernetClient::sendNextMessage()
{
	_messageMutex.lock();
	if (_messagePendingList.size() > 0
		&& _activeMessages < MAX_ACTIVE_MESSAGES)
	{	
#ifdef NO_LOG_AVAILABLE
		std::cout << "Try to send messageID " << _messagePendingList.begin()->messageID << endl;
#else
		logs.warning() << "Try to send messageID " << _messagePendingList.begin()->messageID << endl;
#endif
		_activeMessages++;

		_ethernetClient->send(_messagePendingList.begin()->message);

		_messageActiveList.push_back(*_messagePendingList.begin());
		_messagePendingList.pop_front();
	}
	_messageMutex.unlock();
}

//************************** Main Add Message Function *************************//
void KukaEthernetClient::addMessage(int msgID, int jobId, int param1, int param2, KukaAxis axis, KukaFrame frame, float linVel, float axVel, bool flush)
{
	commandMessage commandMsg;
        commandMsg = createNewCommandMessage(msgID, jobId, param1, param2, axis, frame, linVel, axVel);

#ifdef NO_LOG_AVAILABLE
	std::cout << "Adding messageID " << commandMsg.messageID << ",  string = " << endl << commandMsg.message << endl;
#else
	logs.debug() << "Adding messageID " << commandMsg.messageID << ",  string = " << endl << commandMsg.message << endl;
#endif
	addMessage(commandMsg, flush);
}
//******************************************************************************//

void KukaEthernetClient::addMessage(int msgID, int jobId, KukaFrame frame, float linVel, bool flush)
{
	KukaAxis axis;
        addMessage(msgID, jobId, 0, 0, axis, frame, linVel, 0, flush);
}

void KukaEthernetClient::addMessage(int msgID, int jobId, KukaAxis axis, float axVel, bool flush)
{
	KukaFrame frame;
        addMessage(msgID, jobId, 0, 0, axis, frame, 0, axVel, flush);
}

void KukaEthernetClient::addMessage(int msgID, int jobId, int param1, bool flush)
{
	KukaAxis axis;
	KukaFrame frame;
        addMessage(msgID, jobId, param1, 0, axis, frame, 0, 0, flush);
}

void KukaEthernetClient::addMessage(int msgID, int jobId, int param1, int param2, bool flush)
{
	KukaAxis axis;
	KukaFrame frame;
        addMessage(msgID, jobId, param1, param2, axis, frame, 0, 0, flush);
}

void KukaEthernetClient::addMessage(int msgID, int jobId, bool flush)
{
	KukaAxis axis;
	KukaFrame frame;
        addMessage(msgID, jobId, 0, 0, axis, frame, 0, 0, flush);
}




//******************************************************************************//

void KukaEthernetClient::setIo(int msgID, int ioNr, bool newState)
{
    int intState = newState?1:0;
    addMessage(msgID, (int)SubID::SetIo, ioNr, intState, false);
}

void KukaEthernetClient::movePTP(int msgID, float a1, float a2, float a3, float a4, float a5, float a6, float vel)
{
	KukaAxis axis(a1, a2, a3, a4, a5, a6); 
        movePTP(msgID, axis, vel);
}

void KukaEthernetClient::movePTP(int msgID, KukaAxis axis, float vel)
{
	if (vel > 100.0)
	{
		std::cout << "Axis Movement Speed adjusted higher than 100%; Value truncated to 90%" << endl;
		vel = 90.0;
	}
        addMessage(msgID, (int)SubID::MovePTP_AXIS, axis, vel, false);
}

void KukaEthernetClient::movePTP(int msgID, KukaFrame frame, float vel)
{
        if (vel > 2.0)
        {
                std::cout << "Linear Movement Speed adjusted higher than 2m/s; Value truncated to 1.9m/s" << endl;
                vel = 1.9;
        }
        addMessage(msgID, (int)SubID::MovePTP_FRAME, frame, vel, false);
}

void KukaEthernetClient::moveLIN(int msgID, KukaFrame frame, float vel)
{
	if (vel > 2.0)
	{
		std::cout << "Linear Movement Speed adjusted higher than 2m/s; Value truncated to 1.9m/s" << endl;
		vel = 1.9;
	}
        addMessage(msgID, (int)SubID::MoveLIN_FRAME, frame, vel, false);
}

void KukaEthernetClient::pauseMsg( int msgID )
{
	commandMessage commandMsg;
        commandMsg = createNewCommandMessage(msgID, (int)SubID::Pause, 0, 0, KukaAxis(), KukaFrame(), 0, 0);
	_ethernetClient->send(commandMsg.message);
}

void KukaEthernetClient::resumeMsg( int msgID )
{
	commandMessage commandMsg;
        commandMsg = createNewCommandMessage(msgID, (int)SubID::Resume, 0, 0, KukaAxis(), KukaFrame(), 0, 0);
	_ethernetClient->send(commandMsg.message);
}

KukaEthernetClient::commandMessage KukaEthernetClient::createNewCommandMessage( int msgID, int jobId, int param1, int param2, KukaAxis axis, KukaFrame frame, float linVel, float axVel )
{
        _xmlHandler->setDataToRobot(TAG_SEND_SUB_ID, int2String(jobId));
	_xmlHandler->setDataToRobot(TAG_SEND_MSG_ID, int2String(msgID));

	_xmlHandler->setDataToRobot(TAG_SEND_PARAM1, int2String(param1));
	_xmlHandler->setDataToRobot(TAG_SEND_PARAM2, int2String(param2));
	_xmlHandler->setDataToRobot(TAG_SEND_FRAME, frame);
	_xmlHandler->setDataToRobot(TAG_SEND_AXIS, axis);
	_xmlHandler->setDataToRobot(TAG_SEND_AXVEL, float2String(axVel));
	_xmlHandler->setDataToRobot(TAG_SEND_LINVEL, float2String(linVel));

	commandMessage commandMsg;
	commandMsg.messageID = msgID;
	commandMsg.message = _xmlHandler->createDataToRobotXml();

	return commandMsg;
}

KukaFrame KukaEthernetClient::getCurrentFrame()
{
	_frameMutex.lock();
        return _currentFrame;
	_frameMutex.unlock();
}
void KukaEthernetClient::setCurrentFrame(KukaFrame frame)
{
	_frameMutex.lock();
        _currentFrame = frame;
	_frameMutex.unlock();
}

KukaAxis KukaEthernetClient::getCurrentAxis()
{
        _axisMutex.lock();
        return _currentAxis;
        _axisMutex.unlock();
}
void KukaEthernetClient::setCurrentAxis(KukaAxis axis)
{
        _axisMutex.lock();
        _currentAxis = axis;
        _axisMutex.unlock();
}
