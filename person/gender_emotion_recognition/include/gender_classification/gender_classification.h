#ifndef GENDER_CLASSIFICATION_H
#define GENDER_CLASSIFICATION_H

#include <iostream>
#include <vector>
#include <string>
#include <memory>

#include <model.h>
#include <ros/ros.h>

using namespace std;

namespace rhome_perception
{

class GenderClassifier : public Model
{
private:
    void predict(SessionPtr& session, tensorflow::Tensor input, vector<vector<double>>& output);

public:
    GenderClassifier(int width, int height, int depth, string graph_path);
    GenderClassifier(string graph_path);

    ~GenderClassifier();

    string run(const cv::Mat& input);
};

} // namespace rhome_perception



#endif // GENDER_CLASSIFICATION_H