#ifndef GESTURE_RECOGNITION_H
#define GESTURE_RECOGNITION_H

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/public/session.h"

#include <ros/ros.h>

using namespace std;

namespace rhome_perception
{
const string GESTURE[6] = {"Four", "Garbage", "Rock", "Startrek", "Fist", "Palm"};    // 6 classes

typedef std::unique_ptr<tensorflow::Session> SessionPtr;

class GestureRecognizer
{
private:
    /* data */
    const int WIDTH;
    const int HEIGHT;
    const int DEPTH;

    SessionPtr sess_det, sess_cls;
    string graph_path_det, graph_path_cls;

    /* function */
    bool createSession(string graph_path, SessionPtr& session);

    void getTensor(tensorflow::Tensor &tensor, const cv::Mat& image);
    void getTensor(tensorflow::Tensor &tensor, const std::vector<cv::Mat>& images);

    void detect(SessionPtr& session, tensorflow::Tensor input, vector<vector<double>>& output, int num_hand, int img_width, int img_height, double score_thresh = 0.2);
    void classify(SessionPtr& session, tensorflow::Tensor input, vector<string>& labels);

public:
    GestureRecognizer(string graph_path_det, string graph_path_cls);
    GestureRecognizer(string graph_path);

    ~GestureRecognizer();

    void run(const cv::Mat& input, vector<vector<double>>& boxes, vector<string>& labels);
};

} // namespace rhome_perception



#endif // GESTURE_RECOGNITION_H