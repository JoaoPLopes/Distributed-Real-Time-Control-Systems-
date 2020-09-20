#ifndef TCP_SERVER
#define TCP_SERVER

#include <atomic>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>


using namespace boost::asio;
using boost::system::error_code;

class session {
	enum { max_len = 1024 };
	char data[max_len];
	void interpret_request();
	void send_reply(std::string& response);
	int nDesks;
};



#endif //TCP_SERVER#pragma once
