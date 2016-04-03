//
// Created by nikitas on 28.03.16.
//

#include <sstream>

#include "boost/shared_ptr.hpp"
#include "include/LineDetectorManager.h"

std::vector<boost::shared_ptr<manager::BaseManager> > managers;

std::string convertInt(int number) {
    std::stringstream ss;//create a stringstream
    ss << number;//add number to the stream
    return ss.str();//return a string with the contents of the stream
}

int main(int argv, char **argc) {
    cv::Mat mat(10, 50, CV_8U);

    std::vector<cv::Vec3b> v;

    std::vector<std::string> paths(100);
    for (int i = 0; i < 100; i++)
        paths[i] = "/home/nikitas/bases/" + convertInt(i) + ".png";

    managers.push_back(boost::shared_ptr<manager::BaseManager>
                               (new manager::LineDetectorManager(paths)));
    std::cout << "1 - LineDetector\nEsc - choose manager\nq - exit program";
    while (true) {
        int key = cv::waitKey();
        if (key == 27) continue;
        if (key == 'q') break;
        try {
            managers.at((unsigned int) key)->run();
        }
        catch (std::out_of_range &) {
            std::cerr << "not correct choose\n";
        }
    }
    return 0;
}