#include <iostream>
#include <string>
#include<boost/asio/basic_streambuf.hpp>
#include <boost/asio/buffer.hpp>
#include <sstream>
#include <boost/asio.hpp>
#include "Office.h"
using namespace boost::system;
using namespace boost::asio;

//GLOBALS
boost::asio::io_context io;
boost::asio::serial_port sp{ io };
boost::asio::deadline_timer tim{ io };
boost::asio::streambuf read_buf; //read buffer
boost::asio::const_buffer;
int counter = 0;
std::string text,test;
std::ostringstream os;
enum { max_len = 1024 };

office this_office(3);


void interpret_request(std::string request){

	int n = request.length();

	// declaring character array 
	char data[max_len];

	// copying the contents of the 
	// string to char array 
	strcpy(data, request.c_str());

	std::string invalid = "invalid request";
	int desk = data[4];
	std::string response = "";
	response += data[2];
	

	switch (data[0]) {
		//-------------------STANDART PARAMETERS---------------------
	case 'g': {
		switch (data[2]) {
		case 'l': {
			uint32_t lux = this_office.get_lux(desk);
			response = response + std::to_string(lux) + '\n';
			async_write(sp, buffer(response), write_handler);
			break;
		}
		case 'd': {
			uint32_t duty_cycle = this_office.get_duty_cycle(desk);
			response = response + std::to_string(duty_cycle) + '\n';
			send_reply(response);
			break;
		}
		case 'o': {
			bool state = this_office.get_occupancy(desk);
			response = response + std::to_string(state) + '\n';
			send_reply(response);
			break;
		}
		case 'O': {
			float lower_bound = this_office.get_lower_bound(desk);
			response = response + std::to_string(lower_bound) + '\n';
			send_reply(response);
			break;
		}
		case 'U': {
			float lower_bound = this_office.get_lower_bound(desk);
			response = response + std::to_string(lower_bound) + '\n';
			send_reply(response);
			break;
		}
		case 'L': {
			float lower_bound = this_office.get_lower_bound(desk);
			response = response + std::to_string(lower_bound) + '\n';
			send_reply(response);
			break;
		}
		case 'x': {
			float ext_lux = this_office.get_ext_lux(desk);
			response = response + std::to_string(ext_lux) + '\n';
			send_reply(response);
			break;
		}
		case 'r': {
			float control_ref = this_office.get_control_ref(desk);
			response = response + std::to_string(control_ref) + '\n';
			send_reply(response);
			break;
		}
		case 'c': {
			float power = 0;
			for (; desk <= nDesks; desk++)
				power += office.get_cost(desk);

			response = response + std::to_string(power) + '\n';
			send_reply(response);
			break;
		}
		case 'p': {
			if (data[4] == 'T') {
				float power = 0;
				for (; desk <= nDesks; desk++)
					power += this_office.get_power(desk);

				response = response + std::to_string(power) + '\n';
				send_reply(response);
			}
			else {
				float power = this_office.get_power(desk);
				response = response + std::to_string(power) + '\n';
				send_reply(response);
			}
			break;
		}
		case 't': {
			float elapsed_time = office.get_elapsed_time(desk);
			response = response + std::to_string(elapsed_time) + '\n';
			send_reply(response);
			break;
		}
		case 'e': {
			if (data[4] == 'T') {
				float energy = 0;
				for (; desk <= nDesks; desk++)
					energy += this_office.get_energy(desk);

				response = response + std::to_string(energy) + '\n';
				send_reply(response);
			}
			else {
				float energy = 0;
				for (; desk <= nDesks; desk++)
					energy += this_office.get_energy(desk);

				response = response + std::to_string(energy) + '\n';
				send_reply(response);
			}
			break;


			break;
		}
		case 'v': {
			if (data[4] = 'T') {
				float visibility_error = 0;
				for (; desk <= nDesks; desk++)
					visibility_error += this_office.get_vis_error(desk);

				response = response + std::to_string(visibility_error) + '\n';
				send_reply(response);
			}
			else {
				float visibility_error = this_office.get_vis_error(desk);

				response = response + std::to_string(visibility_error) + '\n';
				send_reply(response);
			}

			break;
		}
		case 'f': {
			if (data[4] = 'T') {
				float flicker_error = 0;
				for (; desk <= nDesks; desk++)
					flicker_error += this_office.get_flicker_error(desk);

				response = response + std::to_string(flicker_error) + '\n';
				send_reply(response);
			}
			else {

				float flicker_error = this_office.get_flicker_error(desk);

				response = response + std::to_string(flicker_error) + '\n';
				send_reply(response);
			}

			break;
		}
		default: {
			send_reply(invalid);
			break;
		}
		}
		break;
	}

	default: {
		send_reply(invalid);
		break;
	}
	}
	return;
}

//HANDLERS FOR ASYNC CALLBACKS
//forward declaration of write_handler to timer_handler
void write_handler(const error_code& ec, size_t nbytes);
//timer_handler
void timer_handler(const error_code& ec)
{
	//timer expired – launch new write operation
	std::cout << "Insert command:";
	std::cin >> counter;
	os << "Counter = " << counter;
	text = os.str();
	interpret_request(text);
}

void write_handler(const error_code& ec, size_t nbytes) {
	//writer done – program new deadline
	tim.expires_from_now(boost::posix_time::seconds(5));
	tim.async_wait(timer_handler);
	}
void read_handler(const error_code& ec, size_t nbytes) {
	//data is now available at read_buf
	std::cout << &read_buf;
	size_t n = 0;
	boost::asio::streambuf::const_buffers_type bufs = read_buf.data();
	boost::asio::streambuf::const_buffers_type::const_iterator i = bufs.begin();
	while (i != bufs.end())
	{
		const_buffer buf(*i++);
		n += buf.size();
	}	
	read_buf.consume(n);
	//program new read cycle
	async_read_until(sp, read_buf, '\n', read_handler);
	

}
int main() {
	boost::system::error_code ec;
	sp.open("COM5", ec); //connect to port
	if (ec)
		std::cout << "could not open serial port" << std::endl;
	sp.set_option(serial_port_base::baud_rate(9600), ec);
	//program timer for write operations
	tim.expires_from_now(boost::posix_time::seconds(5));
	tim.async_wait(timer_handler);
	//program chain of read operations
	async_read_until(sp, read_buf, '\n', read_handler);
	io.run(); //get things rolling
}