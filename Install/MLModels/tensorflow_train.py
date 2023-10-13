import tensorflow as tf
import numpy as np
import os
import cv2
import read_from_tf as tinydnn_saver
from sklearn.model_selection import train_test_split
# import median_blur

INPUT_X = 32
INPUT_Y = 32

def loadImage(path):
        imgs = []
        labels = []
        for file in os.listdir(path):
                if file.startswith('.'):
                    continue
                file_path = os.path.join(path, file)
                temp = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
                # cv2.imshow("temp", temp)
                # cv2.waitKey(0)
                # print(file_path)
                temp = cv2.resize(temp, (INPUT_X, INPUT_Y))
                # temp = median_blur.median_filter(temp, 3)
                data = (np.array(temp)/255.0).reshape(INPUT_X, INPUT_Y, 1)
                
                # print(data)
                if file.startswith('t') or file.startswith('T'):
                        imgs.append(data)
                        labels.append(1)
                elif file.startswith('f') or file.startswith('F'):
                        imgs.append(data)
                        labels.append(0)
        imgs = np.array(imgs)
        labels = np.array(labels)
        # print(imgs.shape)
        return imgs, labels

def cnn_nets(features, labels, mode, model_path):
    data_placeholder = tf.placeholder(tf.float32, [None, 32, 32, 1])
    labels_placeholder = tf.placeholder(tf.int32, [None])
    dropout_placeholdr = tf.placeholder(tf.float32)

    conv0 = tf.layers.conv2d(data_placeholder, 4, 3, activation=tf.nn.relu, name='conv0')
    pool0 = tf.layers.max_pooling2d(inputs=conv0, pool_size=[2, 2], strides=2)

    print(pool0.shape)

    conv1 = tf.layers.conv2d(pool0, 8, 3, activation=tf.nn.relu, name='conv1')
    print(conv1.shape)
    pool1 = tf.layers.max_pooling2d(inputs=conv1, pool_size=[2, 2], strides=2)

    print(pool1.shape)

    conv2 = tf.layers.conv2d(pool1, 8, 3, strides=2, activation=tf.nn.relu, name='conv2')
    pool2 = tf.layers.max_pooling2d(inputs=conv2, pool_size=[2, 2], strides=2)

    print(pool2.shape)

    flatten = tf.layers.flatten(pool2)
    fc = tf.layers.dense(flatten, 8, activation=tf.nn.relu, name = 'fc')

    print(fc.shape)
    logits = tf.layers.dense(fc, 2, name='out')

    print(logits.shape)
    dropout_fc = tf.layers.dropout(fc, dropout_placeholdr)
    # logits = tf.layers.dense(dropout_fc, 2)
    predicted_labels = tf.argmax(logits, 1)

    losses = tf.nn.softmax_cross_entropy_with_logits_v2(
        labels=tf.one_hot(labels_placeholder, 2),
        logits=logits
    )

    # accuracy_ = tf.metrics.accuracy(labels_placeholder, predicted_labels)
    predicted_labels = tf.cast(predicted_labels, tf.int32)
    equality = tf.equal(predicted_labels, labels_placeholder)
    
    accuracy_ = tf.reduce_mean(tf.cast(equality, tf.float32))
    mean_loss = tf.reduce_mean(losses)

    optimizer = tf.train.AdamOptimizer(learning_rate=1e-3).minimize(losses)
    # optimizer = tf.train.Gra


    saver = tf.train.Saver()

    with tf.Session() as sess:

        if mode is 'train':
            print("train")

            X_train, X_test, y_train, y_test = train_test_split(features, labels, test_size=0.33, random_state=42)
            print(sum(y_test)/len(y_test))

            sess.run(tf.global_variables_initializer())
            train_feed_dict = {
                data_placeholder: X_train,
                labels_placeholder: y_train,
                dropout_placeholdr: 1.0
            }
            val_feed_dict = {
                data_placeholder: X_test,
                labels_placeholder: y_test,
                # dropout_placeholdr: 0.25
            }
            for step in range(1000):
                _, mean_loss_val, accu = sess.run([optimizer, mean_loss, accuracy_], feed_dict=train_feed_dict)

                if step % 20 == 0:
                    _, accu = sess.run([mean_loss, accuracy_], feed_dict=val_feed_dict)

                    print("step = {}\tmean accuracy = {}".format(step, accu))
            saver.save(sess, model_path)
            writer = tf.summary.FileWriter("tensorboard", sess.graph)
            # kernel = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, 'conv0/kernel')[0]
            # bias = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, 'conv0/bias')[0]
            # tinydnn_saver.save_conv_variable(kernel, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/conv_0_w.txt', bias, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/conv_0_b.txt')

            # kernel = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, 'conv1/kernel')[0]
            # bias = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, 'conv1/bias')[0]
            # tinydnn_saver.save_conv_variable(kernel, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/conv_1_w.txt', bias, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/conv_1_b.txt')

            # kernel = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, 'conv2/kernel')[0]
            # bias = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, 'conv2/bias')[0]
            # tinydnn_saver.save_conv_variable(kernel, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/conv_2_w.txt', bias, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/conv_2_b.txt')

            # kernel = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, 'fc/kernel')[0]
            # bias = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, 'fc/bias')[0]
            # tinydnn_saver.save_conv_variable(kernel, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/fc_w.txt', bias, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/fc_b.txt')
        else:
            # saver.restore(sess, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/robot_cnn_model/model.ckpt')
            # print(conv0)
            reader = tf.train.NewCheckpointReader('/media/t0b1as/Data/rUNSWift_data/robot-data-rf/robot_cnn_model/model.ckpt')
            # print(tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, 'conv0'))
            # tinydnn_saver.save_conv_variable(conv0.)
            all_variables = reader.get_variable_to_shape_map()
            for i in all_variables:
                print(i)
            w0 = reader.get_tensor("conv0/kernel")
            b0 = reader.get_tensor("conv0/bias")
            tinydnn_saver.save_conv_variable(w0, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/conv_0_w.txt', b0, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/conv_0_b.txt')

            w1 = reader.get_tensor("conv1/kernel")
            b1 = reader.get_tensor("conv1/bias")
            tinydnn_saver.save_conv_variable(w1, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/conv_1_w.txt', b1, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/conv_1_b.txt')

            w2 = reader.get_tensor("conv2/kernel")
            b2 = reader.get_tensor("conv2/bias")
            tinydnn_saver.save_conv_variable(w2, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/conv_2_w.txt', b2, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/conv_2_b.txt')

            fc1 = reader.get_tensor("fc/kernel")
            fcb1 = reader.get_tensor("fc/bias")
            tinydnn_saver.save_fc_variable(fc1, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/fc_1_w.txt', fcb1, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/fc_1_b.txt')
            
            out1 = reader.get_tensor("out/kernel")
            outb1 = reader.get_tensor("out/bias")
            tinydnn_saver.save_fc_variable(out1, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/out_1_w.txt', outb1, '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/dnn_model/out_1_b.txt')

            

if __name__ == '__main__':
    imgs, labels = loadImage('/media/t0b1as/Data/rUNSWift_data/robot-data-rf/robot-data-rf/')
    cnn_nets(imgs, labels, 'train', '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/robot_cnn_model/median_blur/model.ckpt')
    # cnn_nets(imgs, labels, 'test', '/media/t0b1as/Data/rUNSWift_data/robot-data-rf/robot_cnn_model/median_blur/model.ckpt')