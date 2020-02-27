#include "gesture_recognition.h"
 
namespace rhome_perception
{    

GestureRecognizer::GestureRecognizer(string graph_path_det, string graph_path_cls) : 
WIDTH(28), HEIGHT(28), DEPTH(1), graph_path_det(graph_path_det), graph_path_cls(graph_path_cls)
{
	// Create detection session
    bool status1 = createSession(graph_path_det, sess_det);
    
    if (sess_det==nullptr || status1==false) {
        cerr << "Failed creating detection session" << endl;
        exit;
    } else
        cout << "Create new detection session successfully " << endl;

	// Create classification session
    bool status2 = createSession(graph_path_cls, sess_cls);
    
    if (sess_cls==nullptr || status1==false) {
        cerr << "Failed creating classification session" << endl;
        exit;
    } else
        cout << "Create new classification session successfully " << endl;
}

GestureRecognizer::~GestureRecognizer()
{
}

bool GestureRecognizer::createSession(string graph_path, SessionPtr& session)
{
    tensorflow::GraphDef graph_def;

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

void GestureRecognizer::getTensor(tensorflow::Tensor &tensor, const cv::Mat& image)
{
    auto tensor_mapped = tensor.tensor<uint8_t, 4>();
	int row = image.rows;
	int col = image.cols;
	int ch = image.channels();

    // Copy
    for (int y = 0; y < row; ++y) {
        const uint8_t* row_ptr = image.ptr<uint8_t>(y);

        for (int x = 0; x < col; ++x) {
            const uint8_t* pixel_ptr = row_ptr + x * ch;

            for (int c = 0; c < ch; ++c) {
                const uint8_t* value_ptr = pixel_ptr + c; 

                tensor_mapped(0, y, x, c) = *value_ptr;
            }
        }
    }
}

void GestureRecognizer::getTensor(tensorflow::Tensor &tensor, const std::vector<cv::Mat>& images)
{
    auto tensor_mapped = tensor.tensor<float, 4>();

    for (int n = 0; n < images.size(); ++n) {
		int row = images[n].rows;
		int col = images[n].cols;
		int ch = images[n].channels();	
		
        auto tensor_mapped = tensor.tensor<float, 4>();

        // Copy
        for (int y = 0; y < row; ++y) {
            const double* row_ptr = images[n].ptr<double>(y);

            for (int x = 0; x < col; ++x) {
                const double* pixel_ptr = row_ptr + x * ch;

                for (int c = 0; c < ch; ++c) {
                    const double* value_ptr = pixel_ptr + c;

                    tensor_mapped(n, y, x, c) = *value_ptr;
                }
            }
        }
    }
}

// detect net
void GestureRecognizer::detect(SessionPtr& session, tensorflow::Tensor input, vector<vector<double>>& output, int num_hand, int img_width,int img_height, double score_thresh)
{
	string input_name = "image_tensor:0";
	vector<string> output_name = {"detection_boxes:0", "detection_scores:0"};

    std::vector<tensorflow::Tensor> outputs;

    if (session==nullptr)
    {
        throw runtime_error("Tensorflow session problem");
    }
    

    tensorflow::Status run_status = session->Run({{input_name, input}},
                                                    output_name, //output_name,
                                                    {},
                                                    &outputs);
 
    if (!run_status.ok()) {
        LOG(ERROR) << "Predicting gesture recognition model failed : " << run_status;
        return;
    }

	auto output_mapped_boxes = outputs[0].tensor<float, 3>();
	auto output_mapped_scores = outputs[1].tensor<float, 2>();

	for (int n = 0; n < num_hand; ++n) {
		vector<double> ele;
		if (output_mapped_scores(0, n) > score_thresh) {
			ele.push_back(floor(output_mapped_boxes(0, n, 1) * img_width));
			ele.push_back(floor(output_mapped_boxes(0, n, 3) * img_width));
			ele.push_back(floor(output_mapped_boxes(0, n, 0) * img_height));
			ele.push_back(floor(output_mapped_boxes(0, n, 2) * img_height));

			output.push_back(ele);
		}
	}
}

// classify net
void GestureRecognizer::classify(SessionPtr& session, tensorflow::Tensor input, vector<string>& labels)
{
	string input_name = "conv2d_1_input:0";
	vector<string> output_name = {"dense_2/Softmax:0"};

	std::vector<tensorflow::Tensor> outputs;

    if (session==nullptr)
    {
        throw runtime_error("Tensorflow session problem");
    }
    
    tensorflow::Status run_status = session->Run({{input_name, input}},
                                                    output_name, 
                                                    {},
                                                    &outputs);

    if (!run_status.ok()) {
        LOG(ERROR) << "Predicting gesture recognition model failed : " << run_status;
        return;
    }

	auto output_mapped_labels = outputs[0].tensor<float, 2>();

	for (int n = 0; n < input.dim_size(0); ++n) {
		vector<double> softmax;
		for (int i = 0; i < 6; ++i) {
			softmax.push_back(output_mapped_labels(n, i));
		}

		int pos = max_element(softmax.begin(), softmax.end()) - softmax.begin();
		labels.push_back(GESTURE[pos]);
	}

}

// Interface
void GestureRecognizer::run(const cv::Mat& input, vector<vector<double>>& boxes, vector<string>& labels)
{
    cv::Mat processed_input;
    
	int img_height = input.rows;
	int img_width = input.cols;
	int img_depth = input.channels();

    if (input.channels()==1)
    {
        cv::cvtColor(input, processed_input, cv::COLOR_GRAY2RGB);     
    } else {
		cv::cvtColor(input, processed_input, cv::COLOR_BGR2RGB);
	}

    tensorflow::Tensor input_tensor_det(tensorflow::DT_UINT8, tensorflow::TensorShape({1, img_height, img_width, img_depth}));
    
	getTensor(input_tensor_det, processed_input);

    cout << "Detecting and classifying hand gesture ... " << endl;
    
	// detection stage
	detect(sess_det, input_tensor_det, boxes, 2, img_width, img_height, 0.2);

	// warp the boxes
	tensorflow::Tensor input_tensor_cls(tensorflow::DT_FLOAT, tensorflow::TensorShape({boxes.size(), HEIGHT, WIDTH, DEPTH}));
	
	cv::Mat gray_input;
	vector<cv::Mat> box_images;

	cv::cvtColor(processed_input, gray_input, cv::COLOR_BGR2GRAY);

	if (boxes.size()==0)
		return;

    // crop and normalize
	for (auto iter = boxes.begin(); iter != boxes.end(); iter++) {
        int left = iter[0][0];
        int right = iter[0][1];
        int top = iter[0][2];
        int bottom = iter[0][3];

        cv::Mat cropped_input = gray_input(cv::Rect(left, top, right-left, bottom-top));
		cv::resize(cropped_input, cropped_input, cv::Size(WIDTH, HEIGHT));
		cropped_input /= 255;

        box_images.push_back(cropped_input);
    }

	getTensor(input_tensor_cls, box_images);

	// classification stage
	classify(sess_cls, input_tensor_cls, labels);

}

} // namespace rhome_perception


