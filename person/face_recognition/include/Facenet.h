#include <iostream>
#include <fstream>
#include <math.h>
// #define NDEBUG
#include <cassert>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/public/session.h"

using namespace cv;
using namespace std;
using namespace tensorflow;

namespace rhome_perception
{
const int NUM_DIM = 128;
const double THRESHOLD = 0.53;
const string UNKNOWN = "Unknown Person";
const string input_layer_1 = "input:0";
const string input_layer_2 = "phase_train:0";
const string output_layer = "embeddings:0";

typedef std::unique_ptr<tensorflow::Session> SessionPtr;

class Facenet
{
private:
    const int width;
    const int height;
    const int depth;

    string graph_path, database_path;

    SessionPtr session;

    vector<vector<double>> db_emb;
    vector<string> db_name;
    
    void createSession(string graph_path, SessionPtr& session);
    void loadDatabase(string database_path, SessionPtr& session);

    void getTensor(tensorflow::Tensor &tensor, const cv::Mat& image);
    void getTensor(tensorflow::Tensor &tensor, const std::vector<cv::Mat>& images);
    void predict(SessionPtr& session, tensorflow::Tensor input, vector<vector<double>>& output);
    double getDist(const vector<double>& emb1, const vector<double>& emb2);



public:
    Facenet();
    Facenet(string graph_path, string database_path);
    ~Facenet();

    string recognize(const cv::Mat& image);
};

} // namespace rhome_percepetion



