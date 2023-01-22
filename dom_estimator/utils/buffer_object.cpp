#include "utils/database/buffer_object.h"

using namespace dom_estimator;

// BufferElements
BufferElements::BufferElements() :
	x(0.0), y(0.0) {}

void BufferElements::add_element(double _time,double _credibility,double _x,double _y)
{
	this->emplace_back(Element(_time,_credibility,_x,_y));
	calc_center();
}

void BufferElements::calc_center()
{
	if(this->empty()){
		std::cerr << "'Buffer Elements' is empty!" << std::endl;
		return;
	}

	double sum_x = 0.0;
	double sum_y = 0.0;
	double max_credibility = this->at(0).credibility;
	for(size_t i = 0; i < this->size(); i++){
		sum_x += this->at(i).x;
		sum_y += this->at(i).y;
		if(max_credibility < this->at(i).credibility) max_credibility = this->at(i).credibility;
	}
	time = this->back().time;
	credibility = max_credibility;
	x = sum_x/(double)this->size();
	y = sum_y/(double)this->size();
}

size_t BufferElements::get_buffer_size() { return this->size(); }

double BufferElements::get_distance(double _x,double _y)
{
	calc_center();
	double diff_x = x - _x;
	double diff_y = y - _y;
	return std::sqrt(diff_x*diff_x + diff_y*diff_y);
}

void BufferElements::print_elements()
{
	for(auto it = this->begin(); it != this->end(); it++){
		it->print_element();
	}
}

// BufferObject
BufferObject::BufferObject() {}

void BufferObject::add_buffer(double _time,double _credibility,double _x,double _y)
{
	if(this->empty()){
		BufferElements buffer_elements;
		buffer_elements.add_element(_time,_credibility,_x,_y);
		this->emplace_back(buffer_elements);
		return;
	}

	std::vector<double> distance_list;
	distance_list.resize(this->size());
	for(size_t i = 0; i < this->size(); i++){
		distance_list.at(i) = this->at(i).get_distance(_x,_y);
	}
	size_t min_index = std::distance(distance_list.begin(),std::min_element(distance_list.begin(),distance_list.end()));
	if(distance_list.at(min_index) < distance_th){
		this->at(min_index).add_element(_time,_credibility,_x,_y);
	}
	else{
		BufferElements buffer_elements;
		buffer_elements.add_element(_time,_credibility,_x,_y);
		this->emplace_back(buffer_elements);
	}
}

bool BufferObject::are_elements_to_add(std::vector<Element>& elements)
{
	elements.clear();
	for(auto it = this->begin(); it != this->end();){
		if((int)it->size() > size_th){
			Element element;
			element.time = it->time;
			element.credibility = it->credibility;
			element.x = it->x;
			element.y = it->y;
			elements.emplace_back(element);
			it = this->erase(it);
		}
		else it++;
	}

	if(elements.empty()) return false;
	else return true;
}

void BufferObject::print_bufffer_elements()
{
	std::cout << "Bufffers Size: " << this->size() << std::endl;
	int id = 0;
	for(auto it = this->begin(); it != this->end(); it++, id++){
		std::cout << "Buffer[" << id << "]: ("
		          << it->x << "," << it->y << ")" << std::endl;
		it->print_elements();
		std::cout << std::endl;
	}
}