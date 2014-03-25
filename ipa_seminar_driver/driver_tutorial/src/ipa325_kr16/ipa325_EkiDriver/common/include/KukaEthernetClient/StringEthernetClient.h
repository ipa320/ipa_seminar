#pragma once
#include "Poco/Net/SocketAddress.h"
#include "Poco/Net/StreamSocket.h"

#include "Poco/Thread.h"
#include "Poco/Runnable.h"
#include "Poco/Mutex.h"
#include <list>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

using namespace Poco::Net;
using namespace std;
using Poco::Thread;

class StringEthernetClient
{
	typedef boost::shared_ptr<StreamSocket> StreamSocketPtr;
	typedef boost::function<int (std::string)> CallbackFunctionPtr;
	
	class WorkerRunnable : public Poco::Runnable
	{
	private:
		enum
		{
			BUFFER_SIZE = 1024
		};

		StreamSocketPtr	_socket;
		Poco::Mutex		_mutex;
		bool			_run;
		bool			_isConnected;
		bool			_sendPending;
                bool                    _autoReconnect;

                bool                    _isDummy;
		
		char*	_pBuffer;
		SocketAddress	_sa;

		list<string> _sendList;

		CallbackFunctionPtr _callback;

	public:
                WorkerRunnable(const SocketAddress& sa, bool autoReconnect, bool isDummy = false);
		~WorkerRunnable();
		void start();
		void stop();
		void reconnect();
		bool isRunning();
		bool isConnected();
                void setDummy(bool dummy);
                bool isDummy();

		int send(string);

		void setCallbackFcn(CallbackFunctionPtr callback);

		virtual void run();


	};

	typedef boost::shared_ptr<WorkerRunnable> WorkerRunnablePtr;
	typedef boost::shared_ptr<Thread> ThreadPtr;
		

public:
	StringEthernetClient();
	~StringEthernetClient();
	void setCallbackFcn(CallbackFunctionPtr callback);

        int connect(const SocketAddress& sa, bool autoReconnect = true);
        void setDummy(bool dummy);
        bool isDummy();

	int disconnect();
	int send(std::string data);

private:
        ThreadPtr		_workerThread;
	WorkerRunnablePtr	_workerRunnable;
        CallbackFunctionPtr     _callback;
        bool                    _isDummy;
};
