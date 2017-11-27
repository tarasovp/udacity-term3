import os
import argparse
import os.path

from tqdm import tqdm
from imgaug import augmenters as iaa
import tensorflow as tf

import helper
import warnings
from distutils.version import LooseVersion
import project_tests as tests

os.environ['CUDA_VISIBLE_DEVICES']='0'



# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), 'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))


def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """
    # TODO: Implement function
    #   Use tf.saved_model.loader.load to load the model and weights
    vgg_tag = 'vgg16'
    names = ['image_input:0', 'keep_prob:0', 'layer3_out:0', 'layer4_out:0', 'layer7_out:0']

    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)
    graph = tf.get_default_graph()

    return tuple([graph.get_tensor_by_name(name) for name in names])
tests.test_load_vgg(load_vgg, tf)
tests.test_load_vgg(load_vgg, tf)


def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer7_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer3_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """
    # TODO: Implement function
    
    reg = tf.contrib.layers.l2_regularizer(1e-3)
    init = tf.random_normal_initializer(stddev=1e-2)
    
    def conv1x1 (layer):
        return tf.layers.conv2d(layer, num_classes, 1, padding='same',
                                kernel_regularizer=reg,
                                kernel_initializer= init)
    
    def upsample (layer, depth, strides):
        return tf.layers.conv2d_transpose(layer, num_classes, depth, strides, padding='same',
                                kernel_regularizer=reg,
                               kernel_initializer= init )

    #convolutions of all layers input
    conv_1x1_7 = conv1x1(vgg_layer7_out)
    conv_1x1_4 = conv1x1(vgg_layer4_out)
    conv_1x1_3 = conv1x1(vgg_layer3_out)

    #upsamples + sum
    upsample_7 = upsample(conv_1x1_7, 4, 2)
    upsample_4 = upsample(tf.add(upsample_7, conv_1x1_4), 4, 2)
    upsample_3 = upsample(tf.add(upsample_4, conv_1x1_3), 16, 8)

    return upsample_3

tests.test_layers(layers)


def optimize(nn_last_layer, correct_label, learning_rate, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """
    # TODO: Implement function


    logits = tf.reshape(nn_last_layer, (-1, num_classes))
    correct = tf.reshape(correct_label, (-1, num_classes))

    cross_entropy_loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=correct))
    io = tf.metrics.mean_iou (labels=tf.argmax(correct,1), predictions=tf.argmax(logits,1), num_classes = num_classes)
    optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate)
    train_op = optimizer.minimize(cross_entropy_loss)

    return logits, train_op, cross_entropy_loss, io
#tests.test_optimize(optimize)


def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss,  input_image,
             correct_label, keep_prob, learning_rate, 
             keep_prob_value=0.5, learning_rate_value=0.001, validate_dataset=None, io=None):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """
    # TODO: Implement function
    sess.run(tf.global_variables_initializer())
    
    #save scores for select
    scores = []
    
    for i in range(epochs):
        print("epoch %d " % i)
        j=0
        total_loss=0
        for image, label in tqdm(get_batches_fn(batch_size)):
            res, loss = sess.run([train_op, cross_entropy_loss],
                               feed_dict={input_image: image, 
                                          correct_label: label, 
                                          keep_prob: keep_prob_value,
                                          learning_rate:  learning_rate_value })
            total_loss += loss
            j+=1
        print("Average Train Loss = %.3f" % (total_loss/j if j>0 else 0))
        if validate_dataset:
            validate_loss = cross_entropy_loss.eval(feed_dict={
                input_image:validate_dataset[0], correct_label:validate_dataset[1], keep_prob: 1.0})
            
            sess.run(tf.local_variables_initializer())
            sess.run(io[1],feed_dict={
                input_image:validate_dataset[0], correct_label:validate_dataset[1], keep_prob: 1.0})
            validate_iou= io[0].eval(feed_dict={
                input_image:validate_dataset[0], correct_label:validate_dataset[1], keep_prob: 1.0})
            
            print('validate loss %.3f and validate iou %.3f' % (validate_loss,validate_iou))
            scores.append([validate_loss,validate_iou])
    
    return scores

tests.test_train_nn(train_nn)

"""
From guide:
rotation: random with angle between 0° and 360° (uniform)
translation: random with shift between -10 and 10 pixels (uniform)
rescaling: random with scale factor between 1/1.6 and 1.6 (log-uniform)
flipping: yes or no (bernoulli)
shearing: random with angle between -20° and 20° (uniform)
stretching: random with stretch factor between 1/1.3 and 1.3 (log-uniform)
"""


    

def run():
    parser = argparse.ArgumentParser(description='augmentation selector')
    parser.add_argument('--learning_rate', type=float, help='learning rate', default=0.001, required=False)
    parser.add_argument('--keep_prob', type=float, help='keep probability (droput)', default=0.5, required=False)
    parser.add_argument('--augmentation', type=int, help='1 - use augmentation 0 - do not use', default=0.001, required=False)
    
    args = parser.parse_args()
    
    
    num_classes = 2
    image_shape = (160, 576)
    data_dir = './data'
    runs_dir = './runs'
    epochs = 50
    batch_size = 4
    learning_rate_value = args.learning_rate
    keep_prob_value = args.keep_prob
    augmentation = args.augmentation
    
    print ('Run training with learning rage %.6f, dropout = %.2f and augmentation = %d' % 
           (learning_rate_value, keep_prob_value, augmentation))
    
    
    if augmentation==1:
        seq = iaa.Sequential([
            iaa.Affine(rotate=(-10, 10),  scale={"x": (1/1.6, 1.6), "y": (1/1.6, 1.6)},shear=(-20, 20)), #randomly rorate by 10px
            iaa.Crop(px=(0, 10)), # crop images from each side by 0 to 16px (randomly chosen)
            iaa.Fliplr(0.5), # horizontally flip 50% of the images
        ])
    else:
        seq = None
    
    if augmentation==1:
        f_aug = seq.augment_images
    else:
        f_aug = lambda x:x

    tests.test_for_kitti_dataset(data_dir)

    # Download pretrained vgg model
    helper.maybe_download_pretrained_vgg(data_dir)

    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/

    with tf.Session() as sess:
        # Path to vgg model
        vgg_path = os.path.join(data_dir, 'vgg')
        # Create function to get batches
        get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/training'), image_shape, seq)
        validate_dataset = helper.gen_validate_dataset(os.path.join(data_dir, 'data_road/training'), image_shape)

        # OPTIONAL: Augment Images for better results
        #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network

        correct_label = tf.placeholder(tf.int32, [None, None, None, num_classes], name='correct_label')
        learning_rate = tf.placeholder(tf.float32, name='learning_rate')

        # TODO: Build NN using load_vgg, layers, and optimize function
        input_image, keep_prob, layer3_out, layer4_out, layer7_out = load_vgg(sess, vgg_path)
        layer_output = layers(layer3_out, layer4_out, layer7_out, num_classes=num_classes)

        logits, train_op, cross_entropy_loss, io = optimize(nn_last_layer=layer_output,
                                          learning_rate=learning_rate,
                                          num_classes=num_classes,
                                          correct_label=correct_label)


        # TODO: Train NN using the train_nn function
        scores = train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob, learning_rate, keep_prob_value, learning_rate_value,
                 validate_dataset=validate_dataset, io = io)
        
        with open('scores/scores_'+str(learning_rate_value)+
                  '_'+str(keep_prob_value)+
                  '_'+str(augmentation)+'.csv','w') as f:
            for a,b in scores:
                f.write(str(a)+','+str(b)+'\n')


        # TODO: Save inference data using helper.save_inference_samples
        helper.save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob, input_image)

        # OPTIONAL: Apply the trained model to a video


if __name__ == '__main__':
    run()
