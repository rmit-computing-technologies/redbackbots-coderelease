"""
this file is used to re-create nao devils' 2024 whistle neural net in .h5 format for use with CompiledNN

tested with tensorflow 2.16.1 & tf-keras 2.16 & python 3.12.8

layer architecure and detailed options were taken from:
- https://link.springer.com/book/10.1007/978-3-031-55015-7 (pages 17-28)
- loading the 2024 `.tflite` whistle net into https://netron.app

https://netron.app also allows you to download the trained params for each layers.

to use this script, you will need to download the trained params for each layers as `.npy` files
and rename them to match line 95-108 of this file (or change those lines to match the lengthy original names)

Author: Kurniawan Zhong Zhen
Author: RedbackBots
"""

import os;os.environ["TF_USE_LEGACY_KERAS"]="1" # force usage of keras 2 instead of keras 3
import tf_keras as keras
from tf_keras import layers
from tf_keras import activations

import numpy as np

model = keras.Sequential()
model.add(keras.Input(shape=[513]))

# reshape into higher rank tensor so we can pass it into a conv2d layer
# NOTE: we assume a "channels last" format where the tensor axes are labeled: (height, width, channel)
model.add(layers.Reshape((1, 513 , 1)))

# conv layer 0 with 32 filters/kernels
model.add(layers.Conv2D(
    kernel_size=(1,5),
    filters=32,
    strides=(1,2),
    dilation_rate=(1,1),
    use_bias=True,
    padding="same"
))
model.add(layers.LeakyReLU(alpha=0.1))
model.add(layers.MaxPool2D(
    pool_size=(1,2),
    strides=(1,2),
    padding="valid"
))

# conv layer 1 with 64 filters/kernels
model.add(layers.Conv2D(
    kernel_size=(1,5),
    filters=64,
    strides=(1,2),
    dilation_rate=(1,1),
    use_bias=True,
    padding="same"
))
model.add(layers.LeakyReLU(alpha=0.1))
model.add(layers.MaxPool2D(
    pool_size=(1,2),
    strides=(1,2),
    padding="valid"
))

# conv layer 2 with 128 filters/kernels
model.add(layers.Conv2D(
    kernel_size=(1,5),
    filters=128,
    strides=(1,2),
    dilation_rate=(1,1),
    use_bias=True,
    padding="same"
))
model.add(layers.LeakyReLU(alpha=0.1))
model.add(layers.MaxPool2D(
    pool_size=(1,2),
    strides=(1,2),
    padding="valid"
))

# conv layer 3 with 64 filters/kernels
model.add(layers.Conv2D(
    kernel_size=(1,5),
    filters=64,
    strides=(1,1),
    dilation_rate=(1,1),
    use_bias=True,
    padding="same",
    activation=activations.elu
))

# reshape to flatten inputs for dense layer
model.add(layers.Reshape([512]))
model.add(layers.Dense(
    units=1,
    activation=activations.sigmoid
))

# load the params from naodevils 2024 tflite file
conv2d_0_filter = np.transpose(np.load("conv2d_0_filter.npy"), axes=[1,2,3,0])
conv2d_0_bias = np.transpose(np.load("conv2d_0_bias.npy"))

conv2d_1_filter = np.transpose(np.load("conv2d_1_filter.npy"), axes=[1,2,3,0])
conv2d_1_bias = np.transpose(np.load("conv2d_1_bias.npy"))

conv2d_2_filter = np.transpose(np.load("conv2d_2_filter.npy"), axes=[1,2,3,0])
conv2d_2_bias = np.transpose(np.load("conv2d_2_bias.npy"))

conv2d_3_filter = np.transpose(np.load("conv2d_3_filter.npy"), axes=[1,2,3,0])
conv2d_3_bias = np.transpose(np.load("conv2d_3_bias.npy"))

dense_weights = np.transpose(np.load("dense_weights.npy"))
dense_bias = np.load("dense_bias.npy")

model.set_weights([
    conv2d_0_filter,
    conv2d_0_bias,
    conv2d_1_filter,
    conv2d_1_bias,
    conv2d_2_filter,
    conv2d_2_bias,
    conv2d_3_filter,
    conv2d_3_bias,
    dense_weights,
    dense_bias
])
model.summary()
model.save("NaoDevilsWhistleNet.h5")
