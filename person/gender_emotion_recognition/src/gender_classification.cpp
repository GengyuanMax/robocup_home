#include "gender_classification/gender_classification.h"

namespace rhome_perception
{

GenderClassifier::GenderClassifier(int width, int height, int depth, string graph_path) : Model(width, height, depth, graph_path)
{
}

GenderClassifier::GenderClassifier(string graph_path) : Model(140, 140, 3, graph_path)
{
}

GenderClassifier::~GenderClassifier()
{
}

void GenderClassifier::predict(SessionPtr& session, tensorflow::Tensor input, vector<vector<double>>& output)
{

    std::vector<Tensor> outputs;

    if (session==nullptr)
    {
        throw runtime_error("Tensorflow session problem");
    }
    

    tensorflow::Status run_status = session->Run({{"input_1:0", input}},
                                                    {"predictions/Sigmoid:0"},
                                                    {},
                                                    &outputs);  

    if (!run_status.ok()) {
        LOG(ERROR) << "Predicting gender classification model failed" << run_status;
        return;
    }


    for (int i = 0; i < outputs.size(); ++i) {
        auto output_mapped = outputs[i].tensor<float, 2>();

        for (int n = 0; n < outputs[0].dim_size(0); ++n) {
            vector<double> ele;

            for (int j = 0; j < 2; ++j) {
                ele.push_back(output_mapped(n, j));
            } 
            output.push_back(ele);
        }
    }
    return;
}

// Interface 
string GenderClassifier::run(const cv::Mat& input)
{
    cv::Mat processed_input;
    cv::resize(input, processed_input, cv::Size(WIDTH, HEIGHT), cv::INTER_NEAREST);

    if (processed_input.channels()==1)
    {
        cv::cvtColor(processed_input, processed_input, cv::COLOR_GRAY2BGR);     // TODO: check RGB or BGR
    }
    tensorflow::Tensor input_tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({1, HEIGHT, WIDTH, DEPTH}));
    getTensor(input_tensor, processed_input);

    cout << "Computing embedding of new image ... " << endl;
    vector<vector<double>> res;
    predict(sess, input_tensor, res);

    // TODO:
    if (res[0][0] > res[0][1]) {
        return "male";
    } else {
        return "female";
    }

}


    
} // namespace rhome_peception

