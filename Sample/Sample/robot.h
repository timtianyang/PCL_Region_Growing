#pragma once
#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <boost/array.hpp>
#include "ctrData.h"
class robot
{

public:
	robot();
	~robot();
	int waitForCommand();
	void SendMessage(sMsgSend* msg_send);
private:
	boost::asio::io_service io_service;

	boost::asio::ip::tcp::acceptor acceptor;
	boost::shared_ptr<boost::asio::ip::tcp::socket> client;
};

