#include "emotion_recognition/emotion_recognition.h"
 
namespace rhome_perception
{    

EmotionRecognizer::EmotionRecognizer(int width, int height, int depth, string graph_path) : Model(width, height, depth, graph_path)
{
}

EmotionRecognizer::EmotionRecognizer(string graph_path) : Model(71, 71, 3, graph_path)     // fixed input size
{
}

EmotionRecognizer::~EmotionRecognizer()
{
}

void EmotionRecognizer::predict(SessionPtr& session, tensorflow::Tensor input, vector<vector<double>>& output)
{

    std::vector<Tensor> outputs;

    if (session==nullptr)
    {
        throw runtime_error("Tensorflow session problem");
    }
    

    tensorflow::Status run_status = session->Run({{"input_1:0", input}},
                                                    {"predictions/Softmax:0"},
                                                    {},
                                                    &outputs);

    if (!run_status.ok()) {
        LOG(ERROR) << "Predicting emotion recognition model failed" << run_status;
        return;
    }


    for (int i = 0; i < outputs.size(); ++i) {
        auto output_mapped = outputs[i].tensor<float, 2>();

        for (int n = 0; n < outputs[0].dim_size(0); ++n) {
            vector<double> ele;

            for (int j = 0; j < 7; ++j) {
                ele.push_back(output_mapped(n, j));
            } 
            output.push_back(ele);
        }
    }
    return;
    }


// Interface
string EmotionRecognizer::run(const cv::Mat& input)
{
    cv::Mat processed_input;
    cv::resize(input, processed_input, cv::Size(WIDTH, HEIGHT), cv::INTER_NEAREST);

    tensorflow::Tensor input_tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({1, HEIGHT, WIDTH, DEPTH}));;
    getTensor(input_tensor, processed_input);

    cout << "Computing embedding of new image ... " << endl;
    vector<vector<double>> res;
    predict(sess, input_tensor, res);

    auto pos = std::max_element(res[0].begin(), res[0].end());
    return emo[std::distance(res[0].begin(), pos)];

}

} // namespace rhome_perception
