#ifndef DOM_RECORDER_H_
#define DOM_RECORDER_H_

#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>

namespace dom_estimator
{
class DomRecord
{
public:
    DomRecord() :
        time(0.0), dom(0.0) {}
    DomRecord(double _time,double _dom) :
        time(_time), dom(_dom) {}
    DomRecord(double _time,double _dom,double _credibility,double _distance) :
        time(_time), dom(_dom), credibility(_credibility), distance(_distance) {}

    double time;
    double dom;
    double credibility;
    double distance;

private:
};

// class DomRecorder : public std::map<std::string,std::vector<DomRecord>>
class DomRecorder : public std::map<int,std::vector<DomRecord>>
{
public:
    DomRecorder() {}

    void set_path(std::string path) { RECORD_PATH_ = path; }

    // void add_data(std::string name,DomRecord dom_record)
    // {
    //     for(auto it = this->begin(); it != this->end(); it++){
    //         if(it->first == name){
    //             it->second.emplace_back(dom_record);
    //         }
    //     }
    // }

    void add_data(int id,DomRecord dom_record)
    {
        for(auto it = this->begin(); it != this->end(); it++){
            if(it->first == id){
                it->second.emplace_back(dom_record);
            }
        }
    }

    void output_data()
    {
        for(auto it = this->begin(); it != this->end(); it++){
            // std::string file_name = RECORD_PATH_ + it->first + ".csv";
            std::string file_name = RECORD_PATH_ + std::to_string(it->first) + ".csv";
            std::ofstream ofs(file_name);
            for(auto sit = it->second.begin(); sit != it->second.end(); sit++){
                // ofs << sit->time << "," << sit->dom << std::endl;
                ofs << sit->time << "," << sit->dom << "," << sit->credibility << "," << sit->distance << std::endl;
            }
            ofs.close();
        }
    }

private:
    std::string RECORD_PATH_;

};
} // namespace dom_estimator

#endif  // DOM_RECORDER_H_
