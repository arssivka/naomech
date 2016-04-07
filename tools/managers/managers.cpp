//
// Created by nikitas on 4/6/16.
//

#include "Vision.h"

int main(int argv, char **argc) {
    rd::Vision vision;
    cv::Mat frame = cv::Mat::zeros(240, 380, CV_8UC(1));
    vision.setFrame(frame);
    std::cout << vision.ballDetect();
    cv::imshow("cv::imshow", frame);
    return 0;
}