#ifndef MODEL_H
#define MODEL_H

#include <iostream>
#include <vector>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/public/session.h"

using namespace std;

using tensorflow::Tensor;

namespace rhome_perception
{

typedef std::unique_ptr<tensorflow::Session> SessionPtr;
    
class Model
{
protected:
    /* data */
    const int WIDTH;
    const int HEIGHT;
    const int DEPTH;

    SessionPtr sess;
    string graph_path;

    /* function */
    bool createSession(string graph_path, SessionPtr& session);

    void getTensor(tensorflow::Tensor &tensor, const cv::Mat& image);
    void getTensor(tensorflow::Tensor &tensor, const std::vector<cv::Mat>& images);

    virtual void predict(SessionPtr& session, tensorflow::Tensor input, vector<vector<double>>& output) = 0;

public:
    Model(int width, int height, int depth, string graph_path);
    ~Model();

    virtual string run(const cv::Mat& input) = 0;


};

} // namespace rhome_perception


#endif // MODEL_H