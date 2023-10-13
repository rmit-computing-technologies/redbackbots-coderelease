/**
 * @author Felix Weiglhofer
 * @author Adhiraj Jain, Kah Hie Toh, Diana Louise Gaba
 */
#include "whistle/whistle_classifier.h"

#include <iterator>
#include <algorithm>
#include <unistd.h>
#include <climits>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <Model.h>
#include <CompiledNN.h>

#include "hdf5.h"
#include <fstream>
// #include <tiny_dnn/io/hdf5.h>

#include "tiny_dnn/tiny_dnn.h"
#include <tiny_dnn/layers/convolutional_layer.h>
#include <tiny_dnn/layers/max_pooling_layer.h>
#include <tiny_dnn/layers/fully_connected_layer.h>
#include <tiny_dnn/activations/relu_layer.h>


WhistleClassifier::WhistleClassifier(
        const double * const input, 
        size_t inSize,
        float whistleThreshold,
        unsigned int minWhistleLength,
        float bufferLength,
        float freqSpacing,
        float freqOffset)
    : _input(input)
    , _inSize(inSize)
    , _bufferLength(bufferLength)
    , _freqSpacing(freqSpacing)
    , _freqOffset(freqOffset)
    , _matchLength(0)
    , _minMatchLength(minWhistleLength)
    , _whistleThreshold(whistleThreshold)
    , _inputWidth(128)
    , _inputHeight(128)
    , _inputChannels(1) 
    , _numClasses(2)
{

    std::cout << "_whistleThreshold: "<<_whistleThreshold<< std::endl;
    std::cout << "_minMatchLength: "<<_minMatchLength<< std::endl;
    
    // _model.load("whistle_detection_model.h5");
    // _nn.compile(_model);

    // std::string onnxModelFile = "model.onnx";

        // Train the network
    const int num_epochs = 10;
    const int batch_size = 32;

    _net = createModel();

}

tiny_dnn::network<tiny_dnn::sequential> WhistleClassifier::createModel() {
    using conv     = tiny_dnn::convolutional_layer;
    using max_pool = tiny_dnn::max_pooling_layer;
    using fc       = tiny_dnn::fully_connected_layer;
    using relu     = tiny_dnn::relu_layer;
    using softmax  = tiny_dnn::softmax_layer;
    using padding  = tiny_dnn::padding;
    using tiny_dnn::core::connection_table;

    int n_channels = 32;
    int stride = 16;
    int kernel_size = 240;
    const int numClasses = 4; 

    tiny_dnn::network<tiny_dnn::sequential> net;
    std::cout << "net model declared: "<<_inSize<< std::endl;

    net     << conv((int)_inSize,1, kernel_size,1, 1,n_channels,padding::same, true, stride, 1)// 1D Convolutional Layer
            << tiny_dnn::batch_normalization_layer((int) _inSize/16,n_channels)

            << conv(n_channels*4, 1, 3, 1, n_channels, n_channels*2, padding::same)
            << tiny_dnn::batch_normalization_layer(n_channels*4,2*n_channels)
            << max_pool(n_channels*4, 1, n_channels*2, 4, 1, 1, 1,false, padding::same)// Max Pooling Layer

            << conv(n_channels*4, 1, 3, 1, n_channels*2, n_channels*4, padding::same)
            << tiny_dnn::batch_normalization_layer(n_channels*4,4*n_channels)
            << max_pool(n_channels*4, 1, n_channels*4, 4, 1, 1, 1,false, padding::same)// Max Pooling Layer
            
            << conv(n_channels*4, 1, 3, 1, n_channels*4, n_channels*8, padding::same)
            << tiny_dnn::batch_normalization_layer(n_channels*4,8*n_channels)
            << max_pool(n_channels*4, 1, n_channels*8, 4, 1, 1, 1,false, padding::same)

            << fc(n_channels*4*(n_channels*8), 2); // Fully Connected Layer


    std::string weight_path;
    std::string bias_path;
    std::string root_path = DNN_WEIGHTS_DIR;

    std::vector<tiny_dnn::vec_t*> weights_input_layer0 = net[0]->weights();
    weight_path = root_path + "conv_0_w.txt";
    bias_path = root_path + "conv_0_b.txt";
    Read_File(*weights_input_layer0[0], weight_path);
    Read_File(*weights_input_layer0[1], bias_path);
    std::cout << "conv-1 Read successfull !!" << std::endl;

    weight_path = root_path + "conv_1_w.txt";
    bias_path = root_path + "conv_1_b.txt";
    std::vector<tiny_dnn::vec_t*> weights_layer0_layer1 = net[2]->weights();
    Read_File(*weights_layer0_layer1[0], weight_path);
    Read_File(*weights_layer0_layer1[1], bias_path);

    weight_path = root_path + "conv_2_w.txt";
    bias_path = root_path + "conv_2_b.txt";
    std::vector<tiny_dnn::vec_t*> weights_layer1_layer2 = net[5]->weights();
    Read_File(*weights_layer1_layer2[0], weight_path);
    Read_File(*weights_layer1_layer2[1], bias_path);

    weight_path = root_path + "conv_3_w.txt";
    bias_path = root_path + "conv_3_b.txt";
    std::vector<tiny_dnn::vec_t*> weights_layer2_layer3 = net[8]->weights();
    Read_File(*weights_layer2_layer3[0], weight_path);
    Read_File(*weights_layer2_layer3[1], bias_path);

    // weight_path = root_path + "fc_1_w.txt";
    // bias_path = root_path + "fc_1_b.txt";
    // std::vector<vec_t*> weights_layer2_layer3 = nn[9]->weights();
    // Read_File(*weights_layer2_layer3[0], weight_path);
    // Read_File(*weights_layer2_layer3[1], bias_path);

    weight_path = root_path + "out_1_w.txt";
    bias_path = root_path + "out_1_b.txt";
    std::vector<tiny_dnn::vec_t*> weights_layer3_output = net[11]->weights();
    Read_File(*weights_layer3_output[0], weight_path);
    Read_File(*weights_layer3_output[1], bias_path);

            // << fc(128, 64) << relu(); // Fully Connected Layer
            // << fc(64, 2) << softmax(); // Output Layer

        // << tiny_dnn::batch_normalization_layer(128, _bufferLength / 4);
        // << max_pool(128, _bufferLength / 4, 2, 2); //0
        // << relu();
        // << fc(numClasses, _bufferLength / 8 * 128);
        // << fc(_bufferLength / 8 * 128, numClasses);
        // << softmax();
    std::cout << "batch_normalization_layer created"<< std::endl<<std::endl;

    for (size_t i = 0; i < net.layer_size(); ++i) {
        const tiny_dnn::layer* layer = net[i];
        std::cout << "Layer " << i << " architecture:" << std::endl;
        std::cout << "  Type: " << layer->layer_type() << std::endl;
        std::cout << "  Input shape: " << layer->in_shape() << std::endl;
        std::cout << "  Output shape: " << layer->out_shape() << std::endl;
        std::cout << "  kernel_header: ";
        if (layer->layer_type() == "conv") {
            const auto* conv_layer = dynamic_cast<const tiny_dnn::convolutional_layer*>(layer);
            std::cout << conv_layer->kernel_header();
        // } else if (layer->layer_type() == "max-pool") {
        //     const auto* max_pool_layer = dynamic_cast<const tiny_dnn::max_pooling_layer*>(layer);
        //     std::cout << max_pool_layer->pool_size();
        } else {
            std::cout << "N/A";
        }
        std::cout << std::endl;
        // std::cout << "  Number of parameters: " << layer->weight_init_size() << std::endl;
        std::cout << std::endl;
    }

    std::cout << "Model created."<< std::endl;

    std::string json = net.to_json();

    // Save the serialized model to a file
    std::ofstream file("model.json");
    if (file.is_open()) {
        file << json;
        file.close();
        std::cout << "Model saved successfully." << std::endl;
    } else {
        std::cerr << "Failed to save the model." << std::endl;
    }

    net.save("model.bin");

    std::cout << "Model saved." << std::endl;

    net.load("model.bin");

    std::cout << "Model loaded." << std::endl;
    return net;
}

void WhistleClassifier::execute() {
    
    // _currPeak = findPeak();
    // if (_currPeak < _whistleThreshold) {
    //     reset();
    // } else {
    //     _matchLength += _bufferLength;
    // }
    // _whistleDetected = (_matchLength >= _minMatchLength);
    // if (_whistleDetected) {
    //     reset();
    // }
    std::vector<double> inputSamples; // inputSamples(_inSize); 
    for (int i = 0; i < _inSize; ++i) {
    inputSamples.push_back(_input[i]);
}
    tiny_dnn::vec_t inputVec(inputSamples.begin(), inputSamples.end());
    std::cout << "Predicting output... "<< _inSize<< std::endl;
    tiny_dnn::vec_t outputVec = _net.predict(inputVec);
    // Print the predicted output
    for (size_t i = 0; i < outputVec.size(); ++i) {
        std::cout << "Class " << i << ": " << outputVec[i] << std::endl;
    }
}

float WhistleClassifier::findPeak() {
    size_t maxInd = std::distance(_input, std::max_element(_input, _input + _inSize));
    return (maxInd + 1) * _freqSpacing + _freqOffset;
}

void WhistleClassifier::reset() {
    _matchLength = 0;
}

// vim: set ts=4 sw=4 sts=4 expandtab:
