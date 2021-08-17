"""
Most codes from https://github.com/carpedm20/DCGAN-tensorflow
"""
import math
import tensorflow as tf
import numpy as np

"""Basic Operation"""


# L Regularizer
def reg_L(network, gamma=0.01, type_reg='L2'):
    reg = []
    for w in network.W:
        if type_reg == 'L1':
            reg.append(tf.reduce_sum(tf.abs(w)))
        elif type_reg == 'L2':
            reg.append(tf.reduce_sum(tf.square(w)) / 2)
    return gamma * tf.reduce_sum(reg)


"""CNN"""
# CNN
if "concat_v2" in dir(tf):
    def concat(tensors, axis, *args, **kwargs):
        return tf.concat_v2(tensors, axis, *args, **kwargs)
else:
    def concat(tensors, axis, *args, **kwargs):
        return tf.concat(tensors, axis, *args, **kwargs)


def bn(x, is_training, scope):
    return tf.contrib.layers.batch_norm(x,
                                        decay=0.9,
                                        updates_collections=None,
                                        epsilon=1e-5,
                                        scale=True,
                                        is_training=is_training,
                                        scope=scope)


def conv_out_size_same(size, stride):
    return int(math.ceil(float(size) / float(stride)))


def conv_cond_concat(x, y):
    """Concatenate conditioning vector on feature map axis."""
    x_shapes = x.get_shape()
    y_shapes = y.get_shape()
    return concat([x, y * tf.ones([x_shapes[0], x_shapes[1], x_shapes[2], y_shapes[3]])], 3)


def conv2d(input_, output_dim, k_h=5, k_w=5, d_h=2, d_w=2, stddev=0.02, name="conv2d"):
    with tf.variable_scope(name):
        w = tf.get_variable('w', [k_h, k_w, input_.get_shape()[-1], output_dim],
                            initializer=tf.truncated_normal_initializer(stddev=stddev))
        conv = tf.nn.conv2d(input_, w, strides=[1, d_h, d_w, 1], padding='SAME')

        biases = tf.get_variable('biases', [output_dim], initializer=tf.constant_initializer(0.0))
        conv = tf.reshape(tf.nn.bias_add(conv, biases), conv.get_shape())

        return conv


def deconv2d(input_, output_shape, k_h=5, k_w=5, d_h=2, d_w=2, name="deconv2d", stddev=0.02, with_w=False):
    with tf.variable_scope(name):
        # filter : [height, width, output_channels, in_channels]
        w = tf.get_variable('w', [k_h, k_w, output_shape[-1], input_.get_shape()[-1]],
                            initializer=tf.random_normal_initializer(stddev=stddev))

        try:
            deconv = tf.nn.conv2d_transpose(input_, w, output_shape=output_shape, strides=[1, d_h, d_w, 1])

        # Support for version of TensorFlow before 0.7.0
        except AttributeError:
            deconv = tf.nn.deconv2d(input_, w, output_shape=output_shape, strides=[1, d_h, d_w, 1])

        biases = tf.get_variable('biases', [output_shape[-1]], initializer=tf.constant_initializer(0.0))
        deconv = tf.reshape(tf.nn.bias_add(deconv, biases), deconv.get_shape())

        if with_w:
            return deconv, w, biases
        else:
            return deconv


def linear(input_, output_size, scope=None, with_w=False):
    shape = input_.get_shape().as_list()
    shape_output = shape[:]
    shape_output[-1] = output_size
    input_reshape = tf.reshape(input_, [-1, shape[-1]])
    with tf.variable_scope(scope or "Linear"):
        matrix = tf.get_variable("weights", [shape[-1], output_size], dtype=tf.float32,
                                 initializer=tf.contrib.layers.xavier_initializer())
        bias = tf.get_variable("bias", [output_size],
                               initializer=tf.constant_initializer(0))
        if with_w:
            return tf.reshape(tf.matmul(input_reshape, matrix) + bias, shape_output), matrix, bias
        else:
            return tf.reshape(tf.matmul(input_reshape, matrix) + bias, shape_output)


def normalize(input_, dim=-1):
    return input_ / tf.sqrt(tf.reduce_sum(tf.square(input_), dim, keepdims=True))


def random_norm(shape, mean=0.0, stddev=1.0, dtype=tf.float32, seed=None, name=None):
    return tf.random_normal(shape, mean=mean, stddev=stddev, dtype=dtype, seed=seed, name=name)


def temporal_padding(input, padding=(0, 0, 0, 0)):
    """
    :param input:   input tensor, could be 3 or 4 dim
    :param padding: must be 4, so far only pad on the L and W dim which is up/down/left/right
    :return:
    """
    len_shape = len(input.shape)
    assert len(padding) == 4
    assert len_shape == 3 or len_shape == 4
    if (len_shape == 3):
        # batch, L, W
        pattern = [[0, 0], [padding[0], padding[1]], [padding[2], padding[3]]]
    else:
        # batch, L, W, C
        pattern = [[0, 0], [padding[0], padding[1]], [padding[2], padding[3]], [0, 0]]
    return tf.pad(input, pattern)


"""RNN"""


# RNN
def GRU_cell(hidden_size):
    cell = tf.contrib.rnn.GRUCell(hidden_size)
    return cell


def LSTM_cell(hidden_size, state_is_tuple=True, activation=tf.nn.relu, keep_prob=[1, 1, 1]):
    """
    build a LSTM cell
    keep_prob is a list of 3 values
    """
    lstm_cell = tf.contrib.rnn.BasicLSTMCell(hidden_size, state_is_tuple=state_is_tuple, activation=activation)

    # drop out
    lstm_cell = tf.contrib.rnn.DropoutWrapper(cell=lstm_cell,
                                              input_keep_prob=keep_prob[0],
                                              output_keep_prob=keep_prob[1],
                                              state_keep_prob=keep_prob[2])
    return lstm_cell


def linear(input, weights, bias, keep_prob=1):
    input = tf.nn.dropout(input, keep_prob=keep_prob)
    return tf.add(tf.matmul(input, weights), bias)


def lrelu(x, leak=0.2):
    return tf.maximum(x, leak * x)


def connection_res(output, interval_con_outputs, input, interval_con_inputs):
    num_reses = len(interval_con_outputs)
    for i in range(num_reses):
        interval_con_output = interval_con_outputs[i]
        interval_con_input = interval_con_inputs[i]
        index_uncon_output_former = np.arange(0, interval_con_output[0])
        index_uncon_output_latter = np.arange(interval_con_output[-1], output.shape[-1].value)
        index_con_output = np.arange(interval_con_output[0], interval_con_output[-1])
        index_con_input = np.arange(interval_con_input[0], interval_con_input[-1])

        output_connected = tf.gather(output, index_con_output, axis=-1) + tf.gather(input, index_con_input, axis=-1)
        output = tf.concat(
            (tf.gather(output, index_uncon_output_former, axis=-1), output_connected,
             tf.gather(output, index_uncon_output_latter, axis=-1)),
            axis=-1)
    return output


def connection_res_byIndex(output, index_con_outputs, input, index_con_inputs):
    assert len(index_con_outputs) == len(index_con_inputs)
    index_uncon_output_former = np.arange(0, index_con_outputs[0])
    index_uncon_output_latter = np.arange(index_con_outputs[-1] + 1, output.shape[-1].value)
    output_resed = []
    for i in range(len(index_con_outputs)):
        output_item = output[:, index_con_outputs[i]] + input[:, index_con_inputs[i]]
        output_resed.append(tf.expand_dims(output_item, axis=-1))
        output_connected = tf.concat(output_resed, axis=-1)
    output = tf.concat(
        (tf.gather(output, index_uncon_output_former, axis=-1), output_connected,
         tf.gather(output, index_uncon_output_latter, axis=-1)),
        axis=-1)
    return output


def Film(output, ScaleOffset):
    assert ScaleOffset.shape[-1].value == 2 * output.shape[-1].value
    size_output = output.shape[-1].value
    return tf.add(tf.multiply(output, ScaleOffset[:, :size_output]), ScaleOffset[:, size_output:])


"""
Method for calculate the rotation angles for 2d vectors
Last dimension need tobe 2
"""


def cross_2d(vec1, vec2):
    return vec1[..., 0] * vec2[..., 1] - vec1[..., 1] * vec2[..., 0]


def dot_2d(vec1, vec2):
    return vec1[..., 0] * vec2[..., 0] + vec1[..., 1] * vec2[..., 1]


def norm_2d(vec):
    return tf.sqrt(tf.square(vec[..., 0]) + tf.square(vec[..., 1]))


def dot_case1(): return tf.constant(0, dtype=tf.float32)


def dot_case2(): return tf.constant(np.pi, dtype=tf.float32)


def dot_case3(dot, vec1, vec2):
    norm1 = norm_2d(vec1)
    norm2 = norm_2d(vec2)
    angle_abs = tf.math.acos(tf.div(dot, tf.multiply(norm1, norm2)))
    dir_rotation = cross_2d(vec1, vec2) / tf.abs(cross_2d(vec1, vec2))
    return angle_abs * dir_rotation


'''
def angle_rotation_2d(vec1, vec2, angle_min, angle_max, angle_gradient=2):
    """
    :param vec1: vector at t-1
    :param vec2: vector at t
    :param angle_min and angle_max: which are float Numpy in the format of Radian
    :return: list of tensors in current batch
    """
    angle_min = tf.constant(angle_min, dtype=tf.float32)
    angle_max = tf.constant(angle_max, dtype=tf.float32)
    angle_centre = (angle_min + angle_max) / 2
    angle_gradient = tf.constant(angle_gradient, dtype=tf.float32)
    vec1 = vec1 / tf.expand_dims(norm_2d(vec1), -1)
    vec2 = vec2 / tf.expand_dims(norm_2d(vec2), -1)
    batch_size = vec1.shape[0].value
    angles = []
    angles_output = []
    epsilon = tf.constant(1e-06, dtype=tf.float32)
    dot_all = dot_2d(vec1, vec2)

    """ForLoop is really bad here, need to be replaced"""
    for i in range(batch_size):
        v1 = vec1[i]
        v2 = vec2[i]
        dot = dot_all[i]

        angle = tf.case(
            pred_fn_pairs=[
                (tf.less_equal(tf.abs(tf.subtract(dot, tf.constant(1, dtype=np.float32))), epsilon),
                 lambda: dot_case1()),
                (tf.less_equal(tf.abs(tf.add(dot, tf.constant(1, dtype=np.float32))), epsilon),
                 lambda: dot_case2())],
            default=lambda: dot_case3(dot, v1, v2), exclusive=False)
        angles.append(angle)

        angle = tf.case(
            pred_fn_pairs=[
                (tf.less(angle, angle_min),
                 lambda: angle_gradient * angle),
                (tf.logical_and(tf.greater_equal(angle, angle_min), tf.less(angle, angle_centre)),
                 lambda: angle + angle_min * (angle_gradient - 1)),
                (tf.logical_and(tf.greater_equal(angle, angle_centre), tf.less(angle, angle_max)),
                 lambda: 2 * angle_centre + angle_min * (angle_gradient - 1) - angle)],
            default=lambda: 2 * angle_centre + angle_min * (angle_gradient - 1) - angle_max - angle_gradient * (
                        angle - angle_max), exclusive=False)
        angles_output.append(angle - angle_gradient)
    return angles, angles_output
'''

'''
def angle_rotation_2d(vec1, vec2, angle_min, angle_max, angle_gradient=2):
    """
    :param vec1: vector at t-1
    :param vec2: vector at t
    :param angle_min and angle_max: which are float Numpy in the format of Radian
    :return: list of tensors in current batch
    """
    angle_min = tf.constant(angle_min, dtype=tf.float32)
    angle_max = tf.constant(angle_max, dtype=tf.float32)
    angle_centre = (angle_min + angle_max) / 2
    angle_gradient = tf.constant(angle_gradient, dtype=tf.float32)
    vec1 = vec1 / tf.expand_dims(norm_2d(vec1), -1)
    vec2 = vec2 / tf.expand_dims(norm_2d(vec2), -1)
    batch_size = vec1.shape[0].value
    angles = []
    angles_output = []
    epsilon = tf.constant(1e-06, dtype=tf.float32)
    dot_all = dot_2d(vec1, vec2)

    """ForLoop is really bad here, need to be replaced"""
    for i in range(batch_size):
        v1 = vec1[i]
        v2 = vec2[i]
        dot = dot_all[i]

        angle = tf.case(
            pred_fn_pairs=[
                (tf.less_equal(tf.abs(tf.subtract(dot, tf.constant(1, dtype=np.float32))), epsilon),
                 lambda: dot_case1()),
                (tf.less_equal(tf.abs(tf.add(dot, tf.constant(1, dtype=np.float32))), epsilon),
                 lambda: dot_case2())],
            default=lambda: dot_case3(dot, v1, v2), exclusive=False)
        angles.append(angle)

        angle = tf.case(
            pred_fn_pairs=[
                (tf.less(angle, angle_centre),
                 lambda: angle_gradient * angle)],
            default=lambda: 2 * angle_centre * angle_gradient - angle_gradient * angle, exclusive=False)
        angles_output.append(angle - angle_gradient)
    return angles, angles_output
'''


def angle_rotation_2d(vec1, vec2, angle_min, angle_max, angle_gradient=2):
    """
    :param vec1: vector at t-1
    :param vec2: vector at t
    :param angle_min and angle_max: which are float Numpy in the format of Radian
    :return: list of tensors in current batch
    """
    angle_min = tf.constant(angle_min, dtype=tf.float32)
    angle_max = tf.constant(angle_max, dtype=tf.float32)
    angle_centre = (angle_min + angle_max) / 2
    angle_gradient = tf.constant(angle_gradient, dtype=tf.float32)
    vec1 = vec1 / tf.expand_dims(norm_2d(vec1), -1)
    vec2 = vec2 / tf.expand_dims(norm_2d(vec2), -1)
    batch_size = vec1.shape[0].value
    angles = []
    angles_output = []
    epsilon = tf.constant(1e-06, dtype=tf.float32)
    dot_all = dot_2d(vec1, vec2)

    """ForLoop is really bad here, need to be replaced"""
    for i in range(batch_size):
        v1 = vec1[i]
        v2 = vec2[i]
        dot = dot_all[i]

        angle = tf.case(
            pred_fn_pairs=[
                (tf.less_equal(tf.abs(tf.subtract(dot, tf.constant(1, dtype=np.float32))), epsilon),
                 lambda: dot_case1()),
                (tf.less_equal(tf.abs(tf.add(dot, tf.constant(1, dtype=np.float32))), epsilon),
                 lambda: dot_case2())],
            default=lambda: dot_case3(dot, v1, v2), exclusive=False)
        angles.append(angle)
        angles_output.append(tf.abs(angle - angle_centre))

    return angles, angles_output


def adjust_BigP(angle, center, deviation, deepness):
    return deepness * (1 / (angle + deviation - center) - 1 / (angle - deviation - center))


def angle_rotation_2d_BigP(vec1, vec2, angle_centre, angle_deviation, angle_deepness=1):
    """
    :param vec1: vector at t-1
    :param vec2: vector at t
    :param angle_centre: maxmum angle you want, which is Numpy with Radian
    :return: list of tensors in current batch
    """
    angle_centre = tf.constant(angle_centre, dtype=tf.float32)
    vec1 = vec1 / tf.expand_dims(norm_2d(vec1), -1)
    vec2 = vec2 / tf.expand_dims(norm_2d(vec2), -1)
    batch_size = vec1.shape[0].value
    angle_output = []
    angle_final = []
    epsilon = tf.constant(1e-06, dtype=tf.float32)
    dot_all = dot_2d(vec1, vec2)

    """ForLoop is really bad here, need to be replaced"""
    for i in range(batch_size):
        v1 = vec1[i]
        v2 = vec2[i]
        dot = dot_all[i]

        angle = tf.case(
            pred_fn_pairs=[
                (tf.less_equal(tf.abs(tf.subtract(dot, tf.constant(1, dtype=np.float32))), epsilon),
                 lambda: dot_case1()),
                (tf.less_equal(tf.abs(tf.add(dot, tf.constant(1, dtype=np.float32))), epsilon),
                 lambda: dot_case2())],
            default=lambda: dot_case3(dot, v1, v2), exclusive=False)
        angle_output.append(angle)

        angle = tf.case(
            pred_fn_pairs=[
                (tf.greater(angle, angle_centre + angle_deviation - tf.constant(0.1 * np.pi / 180)),
                 lambda: adjust_BigP(angle_centre + angle_deviation - tf.constant(0.1 * np.pi / 180),
                                     angle_centre, angle_deviation, angle_deepness)),
                (tf.less(angle, angle_centre - angle_deviation + tf.constant(0.1 * np.pi / 180)),
                 lambda: adjust_BigP(angle_centre - angle_deviation + tf.constant(0.1 * np.pi / 180),
                                     angle_centre, angle_deviation, angle_deepness))],
            default=lambda: adjust_BigP(angle,
                                        angle_centre, angle_deviation, angle_deepness), exclusive=False)
        angle_final.append(angle)
    return angle_output, angle_final


def clip_angle(angle_output, lower_bound=-1000, upper_bound=1000):
    angle_output = tf.clip_by_value(angle_output, lower_bound, upper_bound)
    return angle_output


def loss_log(loss_original, scale):
    scale = tf.constant(scale, dtype=tf.float32)
    return tf.log(loss_original * scale + 1)


def kron_product(m1, m2):
    m1 = tf.expand_dims(m1, -1)
    m2 = tf.expand_dims(m2, -1)

    operator_m1 = tf.linalg.LinearOperatorFullMatrix(m1)
    operator_m2 = tf.linalg.LinearOperatorFullMatrix(m2)

    kron = tf.linalg.LinearOperatorKronecker([operator_m1, operator_m2])
    kron_dense = kron.to_dense()
    kron_dense = tf.squeeze(kron_dense, -1)

    return kron_dense


def angle_rotation_2d_origin(vec1, vec2):
    """
    :param vec1: vector at t-1
    :param vec2: vector at t
    :return: list of tensors in current batch
    """
    vec1 = vec1 / tf.expand_dims(norm_2d(vec1), -1)
    vec2 = vec2 / tf.expand_dims(norm_2d(vec2), -1)
    batch_size = vec1.shape[0].value
    angle_output = []
    epsilon = tf.constant(1e-06, dtype=tf.float32)
    dot_all = dot_2d(vec1, vec2)

    for i in range(batch_size):
        v1 = vec1[i]
        v2 = vec2[i]
        dot = dot_all[i]

        angle = tf.case(
            pred_fn_pairs=[
                (tf.less_equal(tf.abs(tf.subtract(dot, tf.constant(1, dtype=np.float32))), epsilon),
                 lambda: dot_case1()),
                (tf.less_equal(tf.abs(tf.add(dot, tf.constant(1, dtype=np.float32))), epsilon),
                 lambda: dot_case2())],
            default=lambda: dot_case3(dot, v1, v2), exclusive=False)

        angle_output.append(angle)
    return angle_output


def angle_rotation_angles(a1, a2):
    rotation = tf.mod(tf.abs(a2 - a1 + tf.constant(2 * np.pi, dtype=tf.float32)),
                      tf.constant(2 * np.pi, dtype=tf.float32))
    return rotation


# Hard code for 2 Groups of optimizer
def Optimizer2Groups(list_vars, list_ops, loss):
    var1 = list_vars[0]
    var2 = list_vars[1]
    opt1 = list_ops[0]
    opt2 = list_ops[1]
    grads = tf.gradients(loss, var1 + var2)
    grads1 = grads[:len(var1)]
    grads2 = grads[len(var1):]
    train_op1 = opt1.apply_gradients(zip(grads1, var1))
    train_op2 = opt2.apply_gradients(zip(grads2, var2))
    train_op = tf.group(train_op1, train_op2)
    return train_op


# FOR SEPARATE LOSSES

""" FOR THE LOSS"""


def GetDivisionLosses(divisionIndices, loss):
    """give the list of division index and whole array of loss, return losses for different parts"""
    if len(divisionIndices) == 0:
        return [tf.reduce_mean(loss)]
    else:
        divisionLosses = []
        startIndex = 0
        endIndex = int(loss.shape[-1])
        for index in divisionIndices:
            assert index < endIndex
            currentIndices = np.arange(startIndex, index)
            divisionLosses.append(tf.reduce_mean(tf.gather(loss, currentIndices, axis=-1)))
            startIndex = index
        currentIndices = np.arange(startIndex, endIndex)
        divisionLosses.append(tf.reduce_mean(tf.gather(loss, currentIndices, axis=-1)))
        return divisionLosses


# FOR 3D TRANSFORMATION
"""FOR 3D TRANSFORMATION"""


def LookRotation3x3(forward, up):
    """
    :param forward: forward vector, N*3
    :param up: up vector,           N*3
    :return: 3x3 rotation matrix,   N*3*3
    """
    assert forward.shape[-1] == up.shape[-1] == 3
    v1 = tf.math.l2_normalize(forward, axis=-1)
    v2 = Cross(up, v1)
    v3 = Cross(v1, v2)

    """
    first  column cross
    second column up
    third  column forward  
    """
    m0 = tf.expand_dims(v2, axis=-1)  # N*3*1
    m1 = tf.expand_dims(v3, axis=-1)  # N*3*1
    m2 = tf.expand_dims(v1, axis=-1)  # N*3*1

    return tf.concat([m0, m1, m2], axis=-1)  # N*3*3


def LookRotation4x4(forward, up, position):
    """
    :param forward: forward vector,     N*3
    :param up: up vector,               N*3
    :param position: local position,    N*3
    :return: 4x4 transformation matrix, N*4*4
    """
    assert forward.shape[-1] == up.shape[-1] == position.shape[-1] == 3
    assert len(forward.shape) == len(up.shape) == len(position.shape)

    m3x3 = LookRotation3x3(forward, up)  # N*3*3
    m3x1 = tf.expand_dims(position, axis=-1)  # N*3*1
    m3x4 = tf.concat([m3x3, m3x1], axis=-1)  # N*3*4
    m_expand_shape = list(m3x4.shape)
    m_expand_shape[-2] = 1  # shape is N*1*4
    m1x4_np = np.zeros(m_expand_shape, dtype=np.float32)
    m1x4_np[..., -1] += 1  # last one is 1
    m1x4 = tf.constant(m1x4_np, dtype=tf.float32)
    m4x4 = tf.concat([m3x4, m1x4], axis=-2)
    return m4x4


def Transform(m, v):
    """
    :param m: rotation/transformation matrix
    :param v: 3d vector
    :return: return the transformed 3d vector
    """
    assert m.shape[-1] == 3 or m.shape[-1] == 4
    if m.shape[-1] == 3:
        assert m.shape[-1] == m.shape[-2] == 3
        v = tf.expand_dims(v, axis=-1)  # N*3*1
        v = tf.matmul(m, v)  # N*3*3 mul N*3*1 -> N*3*1
        v = tf.squeeze(v, axis=-1)  # N*3

    if m.shape[-1] == 4:
        assert m.shape[-1] == m.shape[-2] == 4
        v_expand_shape = list(v.shape)
        v_expand_shape[-1] = 1
        v_expand = tf.ones(v_expand_shape, dtype=tf.float32)  # N*1
        v = tf.concat([v, v_expand], axis=-1)  # N*4
        v = tf.expand_dims(v, axis=-1)  # N*4*1
        v = tf.matmul(m, v)  # N*4*4 mul N*4*1 -> N*4*1
        v = tf.squeeze(v, axis=-1)  # N*4
        v = tf.gather(v, [0, 1, 2], axis=-1)  # N*3

    return v


def Cross(v1, v2):
    """
    :param v1: N*3
    :param v2: N*3
    :return: return v3 from cross product, which is normalized vector, N*3
    """
    assert v1.shape[-1] == v2.shape[-1] == 3
    v1 = tf.math.l2_normalize(v1, axis=-1)
    v2 = tf.math.l2_normalize(v2, axis=-1)
    v3_0 = v1[..., 1:2] * v2[..., 2:3] - v1[..., 2:3] * v2[..., 1:2]
    v3_1 = v1[..., 2:3] * v2[..., 0:1] - v1[..., 0:1] * v2[..., 2:3]
    v3_2 = v1[..., 0:1] * v2[..., 1:2] - v1[..., 1:2] * v2[..., 0:1]
    v3 = tf.concat([v3_0, v3_1, v3_2], axis=-1)
    v3 = tf.math.l2_normalize(v3, axis=-1)
    return v3


# =================== FOR CONTACT LOSS BETWEEN VOXEL AND FINGER SEGMENTS ===================
import sys

sys.path.append('../Lib_Utils')
import Const as const
import Utils as utils

def HandVoxelDistanceLoss(input, output, prediction, input_norm, output_norm):
    """
    calculate the distance between each finger segment and nearest voxel
    and compare with the ground truth distance
    :param input:
    :param output:
    :param prediction:
    :param input_norm:
    :param output_norm:
    :return:
    """
    input = input * tf.constant(input_norm[1], dtype=tf.float32) + tf.constant(input_norm[0], dtype=tf.float32)
    output = output * tf.constant(output_norm[1], dtype=tf.float32) + tf.constant(output_norm[0], dtype=tf.float32)
    prediction = prediction * tf.constant(output_norm[1], dtype=tf.float32) + tf.constant(output_norm[0],
                                                                                          dtype=tf.float32)

    leftContactLoss = HandVoxelDistanceLoss_single(input, output, prediction,
                                                   const.voxelOIndex,
                                                   const.voxelPIndex_l,
                                                   const.capsuleStartEndIndexPair_l,
                                                   const.realDistanceIndex_l)

    rightContactLoss = HandVoxelDistanceLoss_single(input, output, prediction,
                                                    const.voxelOIndex,
                                                    const.voxelPIndex_r,
                                                    const.capsuleStartEndIndexPair_r,
                                                    const.realDistanceIndex_r)
    return leftContactLoss, rightContactLoss


def HandVoxelDistanceLoss_single(input, output, prediction,
                                 voxelOIndex,
                                 voxelPIndex,
                                 capsuleSE,
                                 realDisIndex
                                 ):
    # voxel
    voxelO = input[..., voxelOIndex:voxelOIndex + 1000]  # N*1000
    voxelP = input[..., voxelPIndex:voxelPIndex + 1000 * 3]  # N*3000
    voxelP = tf.reshape(voxelP, [-1, 1000, 3])  # N*1000*3
    voxelP = tf.expand_dims(voxelP, axis=2)  # N*1000*1*3
    voxelP = tf.tile(voxelP, [1, 1, 19, 1])  # N*1000*19*3

    # collider position
    capsuleIndexLeft = np.asarray(capsuleSE)  # 19*2
    capsuleIndexLeft_start = capsuleIndexLeft[:, 0]  # 19*1
    capsuleIndexLeft_start = utils.get_index_batch(capsuleIndexLeft_start, 3)  # 19*3
    capsulePosition_start = tf.gather(prediction, capsuleIndexLeft_start, axis=-1)  # N*19*3
    capsulePosition_start = tf.expand_dims(capsulePosition_start, axis=1)  # N*1*19*3
    capsulePosition_start = tf.tile(capsulePosition_start, [1, 1000, 1, 1])  # N*1000*19*3

    capsuleIndexLeft_end = capsuleIndexLeft[:, 1]  # 19*1
    capsuleIndexLeft_end = utils.get_index_batch(capsuleIndexLeft_end, 3)  # 19*3
    capsulePosition_end = tf.gather(prediction, capsuleIndexLeft_end, axis=-1)  # N*19*3
    capsulePosition_end = tf.expand_dims(capsulePosition_end, axis=1)  # N*1*19*3
    capsulePosition_end = tf.tile(capsulePosition_end, [1, 1000, 1, 1])  # N*1000*19*3

    # real shortest distance
    realDistance = output[..., realDisIndex:realDisIndex + 19]  # N*19

    # test distance
    # N*1000*19*3, N*1000*19
    predictPoints, predictDis = ClosestPointOnLineSegment(capsulePosition_start, capsulePosition_end, voxelP)
    voxelO = tf.expand_dims(voxelO, axis=-1)  # N*1000*1
    voxelO = tf.tile(voxelO, [1, 1, 19])  # N*1000*19
    voxelO = 1 - voxelO
    voxelO = voxelO * 999
    testDistance = predictDis + voxelO  # N*1000*19
    testDistance = tf.reduce_min(testDistance, axis=1)  # N*19

    results = tf.abs(testDistance - realDistance)

    return results


def HandVoxelContactLoss_test(input, output, prediction):

    leftContactLoss = HandVoxelContactLoss_single(input, output, prediction,
                                                  const.voxelOIndex,
                                                  const.voxelPIndex_l,
                                                  const.capsuleStartEndIndexPair_l,
                                                  const.realDistanceIndex_l,
                                                  const.contactIndex_l)

    rightContactLoss = HandVoxelContactLoss_single(input, output, prediction,
                                                   const.voxelOIndex,
                                                   const.voxelPIndex_r,
                                                   const.capsuleStartEndIndexPair_r,
                                                   const.realDistanceIndex_r,
                                                   const.contactIndex_r)
    return leftContactLoss, rightContactLoss



def HandVoxelContactLoss(input, output, prediction, input_norm, output_norm):
    """
    calculate the distance between contact finger segments and nearest voxel
    and compare with the ground truth distance

    :param input:
    :param output:
    :param prediction:
    :param input_norm:
    :param output_norm:
    :return:
    """
    input = input * tf.constant(input_norm[1], dtype=tf.float32) + tf.constant(input_norm[0], dtype=tf.float32)
    output = output * tf.constant(output_norm[1], dtype=tf.float32) + tf.constant(output_norm[0], dtype=tf.float32)
    prediction = prediction * tf.constant(output_norm[1], dtype=tf.float32) + tf.constant(output_norm[0],
                                                                                          dtype=tf.float32)

    leftContactLoss = HandVoxelContactLoss_single(input, output, prediction,
                                                  const.voxelOIndex,
                                                  const.voxelPIndex_l,
                                                  const.capsuleStartEndIndexPair_l,
                                                  const.realDistanceIndex_l,
                                                  const.contactIndex_l)

    rightContactLoss = HandVoxelContactLoss_single(input, output, prediction,
                                                   const.voxelOIndex,
                                                   const.voxelPIndex_r,
                                                   const.capsuleStartEndIndexPair_r,
                                                   const.realDistanceIndex_r,
                                                   const.contactIndex_r)
    return leftContactLoss, rightContactLoss


def HandVoxelContactLoss_single(input, output, prediction,
                                voxelOIndex,
                                voxelPIndex,
                                capsuleSE,
                                realDisIndex,
                                contactIndex
                                ):
    # voxel
    voxelO = input[..., voxelOIndex:voxelOIndex + 1000]  # N*1000
    voxelP = input[..., voxelPIndex:voxelPIndex + 1000 * 3]  # N*3000
    voxelP = tf.reshape(voxelP, [-1, 1000, 3])  # N*1000*3
    voxelP = tf.expand_dims(voxelP, axis=2)  # N*1000*1*3
    voxelP = tf.tile(voxelP, [1, 1, 19, 1])  # N*1000*19*3

    # collider position
    capsuleIndexLeft = np.asarray(capsuleSE)  # 19*2
    capsuleIndexLeft_start = capsuleIndexLeft[:, 0]  # 19*1
    capsuleIndexLeft_start = utils.get_index_batch(capsuleIndexLeft_start, 3)  # 19*3
    capsulePosition_start = tf.gather(prediction, capsuleIndexLeft_start, axis=-1)  # N*19*3
    capsulePosition_start = tf.expand_dims(capsulePosition_start, axis=1)  # N*1*19*3
    capsulePosition_start = tf.tile(capsulePosition_start, [1, 1000, 1, 1])  # N*1000*19*3

    capsuleIndexLeft_end = capsuleIndexLeft[:, 1]  # 19*1
    capsuleIndexLeft_end = utils.get_index_batch(capsuleIndexLeft_end, 3)  # 19*3
    capsulePosition_end = tf.gather(prediction, capsuleIndexLeft_end, axis=-1)  # N*19*3
    capsulePosition_end = tf.expand_dims(capsulePosition_end, axis=1)  # N*1*19*3
    capsulePosition_end = tf.tile(capsulePosition_end, [1, 1000, 1, 1])  # N*1000*19*3

    # real shortest distance
    realDistance = output[..., realDisIndex:realDisIndex + 19]  # N*19

    # test distance
    # N*1000*19*3, N*1000*19
    predictPoints, predictDis = ClosestPointOnLineSegment(capsulePosition_start, capsulePosition_end, voxelP)
    voxelO = tf.expand_dims(voxelO, axis=-1)  # N*1000*1
    voxelO = tf.tile(voxelO, [1, 1, 19])  # N*1000*19
    voxelO = 1 - voxelO
    voxelO = voxelO * 999
    testDistance = predictDis + voxelO  # N*1000*19
    testDistance = tf.reduce_min(testDistance, axis=1)  # N*19

    results = tf.abs(testDistance - realDistance)  # N*19

    # modify hereee!
    realContact = output[..., contactIndex:contactIndex + 19]  # N*19, 0-1, 0 means no contact, 1 means on contact
    results = results * realContact
    return results

def HandVoxelContactLossRadius_test(input, output, prediction):
    """
    calculate the distance between contact finger segments and nearest voxel
    and compare with the radius

    :param input:
    :param output:
    :param prediction:
    :param input_norm:
    :param output_norm:
    :return:
    """
    leftContactLoss = HandVoxelContactLossRadius_single(input, output, prediction,
                                                        const.voxelOIndex,
                                                        const.voxelPIndex_l,
                                                        const.capsuleStartEndIndexPair_l,
                                                        const.contactIndex_l,
                                                        const.capsuleRadius)

    rightContactLoss = HandVoxelContactLossRadius_single(input, output, prediction,
                                                         const.voxelOIndex,
                                                         const.voxelPIndex_r,
                                                         const.capsuleStartEndIndexPair_r,
                                                         const.contactIndex_r,
                                                         const.capsuleRadius)
    return leftContactLoss, rightContactLoss


def HandVoxelContactLossRadius(input, output, prediction, input_norm, output_norm):
    """
    calculate the distance between contact finger segments and nearest voxel
    and compare with the radius

    :param input:
    :param output:
    :param prediction:
    :param input_norm:
    :param output_norm:
    :return:
    """
    input = input * tf.constant(input_norm[1], dtype=tf.float32) + tf.constant(input_norm[0], dtype=tf.float32)
    output = output * tf.constant(output_norm[1], dtype=tf.float32) + tf.constant(output_norm[0], dtype=tf.float32)
    prediction = prediction * tf.constant(output_norm[1], dtype=tf.float32) + tf.constant(output_norm[0],
                                                                                          dtype=tf.float32)

    leftContactLoss = HandVoxelContactLossRadius_single(input, output, prediction,
                                                        const.voxelOIndex,
                                                        const.voxelPIndex_l,
                                                        const.capsuleStartEndIndexPair_l,
                                                        const.contactIndex_l,
                                                        const.capsuleRadius)

    rightContactLoss = HandVoxelContactLossRadius_single(input, output, prediction,
                                                         const.voxelOIndex,
                                                         const.voxelPIndex_r,
                                                         const.capsuleStartEndIndexPair_r,
                                                         const.contactIndex_r,
                                                         const.capsuleRadius)
    return leftContactLoss, rightContactLoss


def HandVoxelContactLossRadius_single(input, output, prediction,
                                      voxelOIndex,
                                      voxelPIndex,
                                      capsuleSE,
                                      contactIndex,
                                      radius
                                      ):
    # voxel
    voxelO = input[..., voxelOIndex:voxelOIndex + 1000]  # N*1000
    voxelP = input[..., voxelPIndex:voxelPIndex + 1000 * 3]  # N*3000
    voxelP = tf.reshape(voxelP, [-1, 1000, 3])  # N*1000*3
    voxelP = tf.expand_dims(voxelP, axis=2)  # N*1000*1*3
    voxelP = tf.tile(voxelP, [1, 1, 19, 1])  # N*1000*19*3

    # collider position
    capsuleIndexLeft = np.asarray(capsuleSE)  # 19*2
    capsuleIndexLeft_start = capsuleIndexLeft[:, 0]  # 19*1
    capsuleIndexLeft_start = utils.get_index_batch(capsuleIndexLeft_start, 3)  # 19*3
    capsulePosition_start = tf.gather(prediction, capsuleIndexLeft_start, axis=-1)  # N*19*3
    capsulePosition_start = tf.expand_dims(capsulePosition_start, axis=1)  # N*1*19*3
    capsulePosition_start = tf.tile(capsulePosition_start, [1, 1000, 1, 1])  # N*1000*19*3

    capsuleIndexLeft_end = capsuleIndexLeft[:, 1]  # 19*1
    capsuleIndexLeft_end = utils.get_index_batch(capsuleIndexLeft_end, 3)  # 19*3
    capsulePosition_end = tf.gather(prediction, capsuleIndexLeft_end, axis=-1)  # N*19*3
    capsulePosition_end = tf.expand_dims(capsulePosition_end, axis=1)  # N*1*19*3
    capsulePosition_end = tf.tile(capsulePosition_end, [1, 1000, 1, 1])  # N*1000*19*3

    # test distance
    # N*1000*19*3, N*1000*19
    predictPoints, predictDis = ClosestPointOnLineSegment(capsulePosition_start, capsulePosition_end, voxelP)
    voxelO = tf.expand_dims(voxelO, axis=-1)  # N*1000*1
    voxelO = tf.tile(voxelO, [1, 1, 19])  # N*1000*19
    voxelO = 1 - voxelO
    voxelO = voxelO * 999
    testDistance = predictDis + voxelO  # N*1000*19
    testDistance = tf.reduce_min(testDistance, axis=1)  # N*19
    radius = tf.constant(radius, tf.float32)  # 19
    results = tf.abs(testDistance - radius)
    # modify hereee!
    realContact = output[..., contactIndex:contactIndex + 19]  # N*19, 0-1, 0 means no contact, 1 means on contact
    results = results * realContact
    return results


def Dot(a, b):
    assert a.shape[-1] == 2 or a.shape[-1] == 3
    c = a * b
    c = tf.reduce_sum(c, axis=-1)
    return c


def ClosestPointOnLineSegment(seg_s, seg_e, p):
    wander = p - seg_s
    span = seg_e - seg_s
    t = Dot(wander, span) / Dot(span, span)
    t = tf.clip_by_value(t, tf.constant(0, dtype=tf.float32), tf.constant(1, dtype=tf.float32))
    t = tf.expand_dims(t, -1)
    newP = seg_s + t * span
    return newP, tf.linalg.norm(newP - p, axis=-1)
# =================== FOR CONTACT LOSS BETWEEN VOXEL AND FINGER SEGMENTS ===================



# =================== FOR GUASSION NOISE FOR TRAINING===================
def gaussian_noise_layer(input_layer, std):
    noise = tf.random_normal(shape=tf.shape(input_layer), mean=0.0, stddev=std, dtype=tf.float32)
    return input_layer + noise
# =================== FOR GUASSION NOISE FOR TRAINING===================








