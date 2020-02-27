#include "model.h"

namespace rhome_perception
{

Model::Model(int width, int height, int depth, string graph_path) : WIDTH(width), HEIGHT(height), DEPTH(depth), graph_path(graph_path)
{
    // Create session
    bool status = createSession(graph_path, sess);
    
    if (sess==nullptr || status==false) {
        cerr << "Failed creating session" << endl;
        exit;
    } else
        cout << "Create new session successfully " << endl;
}

Model::~Model()
{
}

bool Model::createSession(string graph_path, SessionPtr& session)
{
    tensorflow::GraphDef graph_def;

    // load frozen graph
    tensorflow::Status status1 = tensorflow::ReadBinaryProto(tensorflow::Env::Default(), graph_path, &graph_def);
    if (!status1.ok()) {
        cerr << "Read Proto Error " << status1.ToString() << endl;
        cerr << "graph_path: " << graph_path << endl;
        return false;
    }

    tensorflow::SessionOptions sess_opt;

    (&session)->reset(tensorflow::NewSession(sess_opt));

    tensorflow::Status status2 = session->Create(graph_def);
    if (!status2.ok()) {
        cerr << "Create Graph Error " << status2.ToString() << endl;
        return false;
    }

    cout << "---------------SUMMARY---------------" << endl;
	cout << "Graph loading path: " << graph_path << endl;
    cout << "Session creation Status: " << status2.ToString() << endl;
    cout << "Number of nodes in model_graph: " << graph_def.node_size() << endl;
    cout << "Load graph Status: " << status1.ToString() << endl;
    cout << "-------------------------------------" << endl;

    return true;
}

void Model::getTensor(tensorflow::Tensor &tensor, const cv::Mat& image)
{
    cv::Mat clipped_image;
    cv::resize(image, clipped_image, cv::Size(WIDTH, HEIGHT));

    auto tensor_mapped = tensor.tensor<float, 4>();

    cv::Mat temp = clipped_image.reshape(1, clipped_image.rows*DEPTH);
    cv::Mat mean3;
    cv::Mat stddev3;
    cv::meanStdDev(temp, mean3, stddev3);

    double mean = mean3.at<double>(0);
    double stddev = stddev3.at<double>(0);

    // Prewhiten
    clipped_image.convertTo(clipped_image, CV_64FC1);
    clipped_image = clipped_image - mean;
    clipped_image = clipped_image/stddev;

    // Copy
    for (int y = 0; y < HEIGHT; ++y) {
        const double* row_ptr = clipped_image.ptr<double>(y);

        for (int x = 0; x < WIDTH; ++x) {
            const double* pixel_ptr = row_ptr + x * DEPTH;

            for (int c = 0; c < DEPTH; ++c) {
                const double* value_ptr = pixel_ptr + c; 

                tensor_mapped(0, y, x, c) = *value_ptr;
            }
        }
    }
}

void Model::getTensor(tensorflow::Tensor &tensor, const std::vector<cv::Mat>& images)
{
    auto tensor_mapped = tensor.tensor<float, 4>();

    for (int n = 0; n < images.size(); ++n) {
        cv::Mat clipped_image;

        cv::resize(images[n], clipped_image, cv::Size(WIDTH, HEIGHT));

        auto tensor_mapped = tensor.tensor<float, 4>();

        cv::Mat temp = clipped_image.reshape(1, clipped_image.rows*DEPTH);

        cv::Mat mean3;
        cv::Mat stddev3;
        cv::meanStdDev(temp, mean3, stddev3);

        double mean = mean3.at<double>(0);
        double stddev = stddev3.at<double>(0);

        // Prewhiten
        clipped_image.convertTo(clipped_image, CV_64FC1);
        clipped_image = clipped_image - mean;
        clipped_image = clipped_image/stddev;

        // Copy
        for (int y = 0; y < HEIGHT; ++y) {
            const double* row_ptr = clipped_image.ptr<double>(y);

            for (int x = 0; x < WIDTH; ++x) {
                const double* pixel_ptr = row_ptr + x * DEPTH;

                for (int c = 0; c < DEPTH; ++c) {
                    const double* value_ptr = pixel_ptr + c; // RGB->BGR

                    tensor_mapped(n, y, x, c) = *value_ptr;
                }
            }
        }
    }
}

} // namespace rhome_perception
