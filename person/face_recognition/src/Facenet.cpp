#include "Facenet.h"

namespace rhome_perception 
{

Facenet::Facenet() : width(160), height(160), depth(3)
{}

Facenet::Facenet(string graph_path, string database_path) : width(160), height(160), depth(3), graph_path(graph_path), database_path(database_path)
{
    // Create session
    createSession(graph_path, session);
    cout << "Create new session successfully \n";

    // Load the database
    loadDatabase(database_path, session);
    cout << "Load database successfully \n";
}

Facenet::~Facenet()
{
}

void Facenet::createSession(string graph_path, SessionPtr& session)
{   
    tensorflow::GraphDef graph_def;

    if (!tensorflow::ReadBinaryProto(tensorflow::Env::Default(), graph_path, &graph_def).ok()) {
        LOG(ERROR) << "Read Proto \n";
        return;
    }

    tensorflow::SessionOptions sess_opt;

    (&session)->reset(tensorflow::NewSession(sess_opt));

    if (!session->Create(graph_def).ok()) {
        LOG(ERROR) << "Create graph \n";
        return;
    }

}

void Facenet::loadDatabase(string database_path, SessionPtr& session)
{
    // Visit folder
    vector<cv::Mat> db_images;
    
    // Read CSV file
    std::ifstream file(database_path, ifstream::in);
    
    if (!file) {
        string error_message = "No valid input file was given, please check the given filename.\n";
        CV_Error(Error::StsBadArg, error_message);
    }

    string line, path, classlabel;
    while (getline(file, line)) {
        stringstream liness(line);
        getline(liness, path, ',');
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty()) {
            db_images.push_back(cv::imread(path, cv::IMREAD_COLOR));
            db_name.push_back(classlabel);
        }
    }

    tensorflow::Tensor input_tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({db_images.size(), height, width, depth}));

    // Compute Embeddings
    if (db_images.empty()) {
        string error_message = "No images found in this folder.\n";
        CV_Error(Error::StsBadArg, error_message);
    }

    cout << "Converting images to tensors ... " << endl;
    getTensor(input_tensor, db_images);

    cout << "Encoding database ... " << endl;
    if (!db_emb.empty())
        db_emb.clear();

    predict(session, input_tensor, db_emb);

    assert(db_name.size()==db_emb.size());

    return;
}

// Get tensor of one images
void Facenet::getTensor(tensorflow::Tensor &tensor, const cv::Mat& image)
{
    cv::Mat clipped_image;
    cv::resize(image, clipped_image, Size(width, height));

    auto tensor_mapped = tensor.tensor<float, 4>();

    cv::Mat temp = clipped_image.reshape(1, clipped_image.rows*3);
    cv::Mat mean3;
    cv::Mat stddev3;
    cv::meanStdDev(temp, mean3, stddev3);

    double mean = mean3.at<double>(0);
    double stddev = stddev3.at<double>(0);

    // Prewhiten
    clipped_image.convertTo(clipped_image, CV_64FC1);
    clipped_image = clipped_image - cv::Vec3d(mean, mean, mean);
    clipped_image = clipped_image/stddev;

    // Copy
    for (int y = 0; y < height; ++y) {
        const double* row_ptr = clipped_image.ptr<double>(y);

        for (int x = 0; x < width; ++x) {
            const double* pixel_ptr = row_ptr + x * depth;

            for (int c = 0; c < depth; ++c) {
                const double* value_ptr = pixel_ptr + (2 - c); // RGB->BGR

                tensor_mapped(0, y, x, c) = *value_ptr;
            }
        }
    }
}

// Get tensors of images
void Facenet::getTensor(tensorflow::Tensor &tensor, const std::vector<cv::Mat>& images)
{
    auto tensor_mapped = tensor.tensor<float, 4>();

    for (int n = 0; n < images.size(); ++n) {
        cv::Mat clipped_image;

        cv::resize(images[n], clipped_image, cv::Size(width, height));

        auto tensor_mapped = tensor.tensor<float, 4>();

        cv::Mat temp = clipped_image.reshape(1, clipped_image.rows*3);

        cv::Mat mean3;
        cv::Mat stddev3;
        cv::meanStdDev(temp, mean3, stddev3);

        double mean = mean3.at<double>(0);
        double stddev = stddev3.at<double>(0);

        // Prewhiten
        clipped_image.convertTo(clipped_image, CV_64FC1);
        clipped_image = clipped_image - cv::Vec3d(mean, mean, mean);
        clipped_image = clipped_image/stddev;

        // Copy
        for (int y = 0; y < height; ++y) {
            const double* row_ptr = clipped_image.ptr<double>(y);

            for (int x = 0; x < width; ++x) {
                const double* pixel_ptr = row_ptr + x * depth;

                for (int c = 0; c < depth; ++c) {
                    const double* value_ptr = pixel_ptr + (2 - c); // RGB->BGR

                    tensor_mapped(n, y, x, c) = *value_ptr;
                }
            }
        }
    }
}

void Facenet::predict(SessionPtr& session, tensorflow::Tensor input, vector<vector<double>>& output)
{
    tensorflow::Tensor phase_train(tensorflow::DT_BOOL, tensorflow::TensorShape());

    phase_train.scalar<bool>()() = false;

    std::vector<Tensor> outputs;

    tensorflow::Status run_status = session->Run({{input_layer_1, input},
                                                  {input_layer_2, phase_train}},
                                                 {output_layer},
                                                 {},
                                                 &outputs);

    if (!run_status.ok()) {
        LOG(ERROR) << "Predicting model failed" << run_status;
        return;
    }


    for (int i = 0; i < outputs.size(); ++i) {
        auto output_mapped = outputs[i].tensor<float, 2>();

        for (int n = 0; n < outputs[0].dim_size(0); ++n) {
            vector<double> ele;

            for (int j = 0; j < NUM_DIM; ++j) {
                ele.push_back(output_mapped(n, j));
            } 
            output.push_back(ele);
        }
    }
    return;
}

double Facenet::getDist(const vector<double>& emb1, const vector<double>& emb2)
{
    double sum = 0;

    for (int i = 0; i < NUM_DIM; ++i) {
        sum += pow((emb1[i] - emb2[i]), 2);
    }
    return sqrt(sum);
}

// Interface
string Facenet::recognize(const cv::Mat& image)
{
    tensorflow::Tensor input(tensorflow::DT_FLOAT, tensorflow::TensorShape({1, height, width, depth}));;
    getTensor(input, image);

    // cout << "Computing embedding of new image ... ";
    vector<vector<double>> res;
    predict(session, input, res);

    // cout << "Comparing the database ... ";
    int min_index = -1;
    double min_dist = THRESHOLD;
    for (int i = 0; i < db_emb.size(); ++i) {
        double new_dist = getDist(res[0], db_emb[i]);
        
        if (new_dist < min_dist) {
            min_dist = new_dist;
            min_index = i;
        }
    }

    if (min_index == -1)
        return UNKNOWN;
    else
    {
        return db_name[min_index];
    }
}

} // namespace rhome_perception