#pragma once
#include <cstdint>
#ifndef OFFICE
#define OFFICE

class office {

	int nLuminaires;
	int *occupancy;
	float *lowerBoundO;
	float *lowerBoundU;
	float *lowerBound;
	float *energyCost;
	uint32_t message;

public:

	office(int d);

	//desksDB access
	uint32_t get_lux(int desk);
	uint32_t get_duty_cycle(int desk);
	int get_occupancy(int desk);
	uint32_t set_occupancy(int desk, int value);
	float get_lower_bound_O(int desk);
	uint32_t set_lower_bound_O(int desk, float value);
	float get_lower_bound_U(int desk);
	uint32_t set_lower_bound_U(int desk, float value);
	float get_lower_bound(int desk);
	uint32_t get_ext_lux(int desk);
	uint32_t get_control_ref(int desk);
	uint32_t get_power(int desk);
	float get_cost(int desk);
	uint32_t set_cost(int desk, float value);
	uint32_t get_elapsed_time(int desk);
	uint32_t get_energy(int desk);
	uint32_t get_vis_error(int desk);
	uint32_t get_flicker_error(int desk);
	uint32_t restart();
};

#endif //Office