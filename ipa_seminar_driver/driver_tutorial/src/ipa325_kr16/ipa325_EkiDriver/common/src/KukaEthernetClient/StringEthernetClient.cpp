#include <KukaEthernetClient/StringEthernetClient.h>
#include <cstdio>
#include <iostream>

//********** Worker *************//
StringEthernetClient::WorkerRunnable::WorkerRunnable(const SocketAddress& sa, bool autoReconnect, bool isDummy):
    _pBuffer(new char[BUFFER_SIZE])
{
    _sa = sa;
    _run = true;
    _sendPending = false;
    _isConnected = false;
    _autoReconnect = autoReconnect;
    _isDummy = false;

    setDummy(isDummy);

    if (!_isDummy)
    {
        try
        {
            _socket.reset(new StreamSocket(_sa));
            _isConnected = true;
        }
        catch (...)
        {
            std::cout << "Cannot connect to TCP server" << std::endl;
        }
    }
    else
    {
        _isConnected = true;
    }

}

StringEthernetClient::WorkerRunnable::~WorkerRunnable()
{
    _mutex.unlock();
}

void StringEthernetClient::WorkerRunnable::start()
{
    _run = true;
    _sendPending = false;
    _mutex.unlock();
}

bool StringEthernetClient::WorkerRunnable::isRunning()
{
    return _run;
}

bool StringEthernetClient::WorkerRunnable::isConnected()
{
    return _isConnected;
}
void StringEthernetClient::WorkerRunnable::stop()
{
    _run = false;
    _socket->close();
    _isConnected = false;
}

void StringEthernetClient::WorkerRunnable::reconnect()
{
    if (_isDummy)
    {
        std::cout << "Client in Dummy-Mode: Not connecting to KRC" << std::endl;
        _isConnected = true;
    }
    else
    {
        std::cout << "Trying to reconnect..." << std::endl;
        //_socket->close();
        _isConnected = false;
        try
        {
            _socket.reset(new StreamSocket(_sa));
            _isConnected = true;
        }
        catch (...)
        {
            std::cout << "Cannot connect to TCP server" << std::endl;
            Poco::Thread::sleep(5000);
        }
    }
}	

void StringEthernetClient::WorkerRunnable::setCallbackFcn(CallbackFunctionPtr callback)
{
    _callback = callback;
}

int StringEthernetClient::WorkerRunnable::send(string message)
{
    _mutex.lock();
    int queue = _sendList.size();
    _sendList.push_back(message);
    _mutex.unlock();

    _sendPending = true;
    return queue;
}

void StringEthernetClient::WorkerRunnable::setDummy(bool dummy)
{
    if (dummy != _isDummy)
    {
        std::cout << "Dummy-Mode: "<< dummy << std::endl;
    }
    _isDummy = dummy;
}

bool StringEthernetClient::WorkerRunnable::isDummy()
{
    return _isDummy;
}

void StringEthernetClient::WorkerRunnable::run()
{
    while (_run)
    {
        if (_isDummy)
        {
            if (_sendPending)
            {
                std::cout << "Client in Dummy-Mode: Not connected to KRC; Just echoing messages! Please also doublecheck XMLDefines.h!" << std::endl;
                Poco::Thread::sleep(5000);
                _mutex.lock();

                _callback(_sendList.front());

                _sendList.pop_front();
                if (_sendList.size() == 0)
                {
                    _sendPending = false;
                }
                _mutex.unlock();
            }
            continue;
        }

        if (!_isConnected)
        {
            std::cout << "Communication error (outer Workerloop)" << std::endl;
            Poco::Thread::sleep(5000);
            if (_autoReconnect)
            {
                reconnect();
            }
        }
        else
        {
            try
            {
                if (_sendPending)
                {
                    if (_socket->poll(100, Socket::SELECT_WRITE))
                    {
                        _mutex.lock();
                        _socket->sendBytes(_sendList.begin()->data(), _sendList.begin()->length());
                        _sendList.pop_front();
                        if (_sendList.size() == 0)
                        {
                            _sendPending = false;
                        }
                        _mutex.unlock();
                    }
                }
                else
                {
                    if (_socket->poll(100, Socket::SELECT_READ))
                    {
                        int n = _socket->receiveBytes(_pBuffer, BUFFER_SIZE);
                        if (n > 0)
                        {
                            int result = _callback(string(_pBuffer, n));
                        }
                    }
                }
            }
            catch (...)
            {
                std::cout << "Communication error (Worker while sending/receiving)" << std::endl;
            }
        }
    }
}

//*************** Ethernet Client ***************//
StringEthernetClient::StringEthernetClient():
    _callback(NULL),
    _isDummy(false)
{

}

StringEthernetClient::~StringEthernetClient()
{
    disconnect();
}

void StringEthernetClient::setDummy(bool dummy)
{
    _isDummy = dummy;
    if (!!_workerRunnable)
    {
        _workerRunnable->setDummy(dummy);
    }
}

bool StringEthernetClient::isDummy()
{
    return _isDummy;
}

void StringEthernetClient::setCallbackFcn(boost::function<int (std::string)> callback)
{
    _callback = callback;
    if (!!_workerRunnable)
    {
        _workerRunnable->setCallbackFcn(callback);
    }
}

int StringEthernetClient::connect(const SocketAddress& sa, bool autoReconnect)
{
    _workerThread.reset(new Thread);
    _workerRunnable.reset(new WorkerRunnable(sa, autoReconnect, _isDummy));
    if (_callback != NULL)
    {
        _workerRunnable->setCallbackFcn(_callback);
    }

    _workerThread->start(*_workerRunnable);
    return _workerRunnable->isConnected();
}

int StringEthernetClient::disconnect()
{
    _workerRunnable->stop();
    _workerThread->join();
    return 0;
}

int StringEthernetClient::send(std::string data)
{
    return _workerRunnable->send(data);
}

