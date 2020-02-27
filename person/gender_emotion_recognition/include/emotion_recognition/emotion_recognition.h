#ifndef EMOTION_RECOGNITION_H
#define EMOTION_RECOGNITION_H

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>

#include <model.h>
#include <ros/ros.h>

using namespace std;

namespace rhome_perception
{
const string emo[7] = {"Angry", "Disgust", "Fear", "Happy", "Sad", "Surprise", "Neutral"};

class EmotionRecognizer : public Model
{
private:
    void predict(SessionPtr& session, tensorflow::Tensor input, vector<vector<double>>& output);

public:
    EmotionRecognizer(int width, int height, int depth, string graph_path);
    EmotionRecognizer(string graph_path);

    ~EmotionRecognizer();

    string run(const cv::Mat& input);
};

} // namespace rhome_perception



#endif // EMOTION_RECOGNITION_H