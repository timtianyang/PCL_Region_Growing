#include "stdafx.h"
#include "robot.h"

using boost::asio::ip::tcp;
using namespace std;
robot::robot() : acceptor(io_service, tcp::endpoint(tcp::v4(), 3200)), client(new tcp::socket(acceptor.get_io_service()))
{
	acceptor.accept(*client);
}
int robot::waitForCommand(){
	while (1){
		boost::array<char, 1024> buf;
		boost::system::error_code error;
		size_t len = (client)->read_some(boost::asio::buffer(buf), error);
		string strCmd = "";
		std::copy(buf.begin(), buf.begin() + len, std::back_inserter(strCmd));
		if (strCmd == "grab")
		{
			cout << "grabbing an image" << endl;
			return 1;
		}

		else if (strCmd == "exit"){
			return 100;
		}
		else 
		{
			boost::system::error_code ignored_error;
			(client)->write_some(boost::asio::buffer("Command Error"), ignored_error);
			continue;
		}
		//Yudl,20150521,Mode3 with manually put boxes
		cout << strCmd << endl;
	}
	return 0;
}
//Send message
void robot::SendMessage(sMsgSend* msg_send)
{
	time_t t;
	struct tm * ptr;
	time(&t);
	ptr = localtime(&t);
	strftime(msg_send->time, 20, "%Y%m%d%H%M%S", ptr);
	void* fullData = new char[sizeof(*msg_send)];
	memset(fullData, '\0', sizeof(*msg_send));
	memcpy(fullData, msg_send, sizeof(*msg_send));
	boost::asio::write(*client, boost::asio::buffer(fullData, sizeof(*msg_send)));
	delete[]fullData;
}
robot::~robot()
{
}
