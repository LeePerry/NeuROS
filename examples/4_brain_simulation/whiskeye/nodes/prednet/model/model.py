import numpy as np
import tensorflow.compat.v1 as tf

class Model:

    available_modality = 'visual'
    error_criterion = np.array([1e-2, 1e-2, 1e-2, 1e-2])
    max_iter = 25
    m1_inp_shape = 45 * 80 * 3
    m2_inp_shape = 180
    m1_layers = [1000, 300]
    m2_layers = [120, 60]
    msi_layers = [100]
    m1_cause_init = [0.25, 0.25]
    m2_cause_init = [0.25, 0.25]
    msi_cause_init = [0.25]
    reg_m1_causes = [0.0, 0.0]
    reg_m2_causes = [0.2, 0.2]
    reg_msi_causes = [0.0]
    lr_m1_causes = [0.0004, 0.0004]
    lr_m2_causes = [0.0004, 0.0004]
    lr_msi_causes = [0.0004]
    reg_m1_filters = [0.0, 0.0]
    reg_m2_filters = [0.2, 0.2]
    reg_msi_filters = [0.0, 0.0]
    lr_m1_filters = [0.0001, 0.0001]
    lr_m2_filters = [0.00001, 0.00001]
    lr_msi_filters = [0.0001, 0.0001]

    def __init__(self, logger, model_path):

        self._logger = logger
        self._model_path = model_path

        tf.disable_v2_behavior()
        tf.Session().close()
        tf.reset_default_graph()

        self.sess = tf.Session()
        self.x_m1 = tf.placeholder(tf.float32, shape=[1, self.m1_inp_shape])
        self.x_m2 = tf.placeholder(tf.float32, shape=[1, self.m2_inp_shape])

        self.m1_filters = []
        self.m1_causes = []

        for i in range(len(self.m1_layers)):
            filter_name = 'm1_filter_%d' % i
            cause_name = 'm1_cause_%d' % i
            if i == 0:
                self.m1_filters += [tf.get_variable(filter_name,
                                                    shape=[self.m1_layers[i],
                                                           self.m1_inp_shape])]
            else:
                self.m1_filters += [tf.get_variable(filter_name,
                                                    shape=[self.m1_layers[i],
                                                           self.m1_layers[i-1]])]
            init = tf.constant_initializer(self.m1_cause_init[i])
            self.m1_causes += [tf.get_variable(cause_name,
                                               shape=[1, self.m1_layers[i]],
                                               initializer=init)]

        self.msi_filters = []
        self.msi_causes = []

        for i in range(len(self.msi_layers)):
            if i == 0:
                filter_name = 'msi_m1_filter'
                self.msi_filters += [tf.get_variable(filter_name,
                                                     shape=[self.msi_layers[i],
                                                            self.m1_layers[-1]])]
                filter_name = 'msi_m2_filter'
                self.msi_filters += [tf.get_variable(filter_name,
                                                     shape=[self.msi_layers[i],
                                                           self.m2_inp_shape])]

            else:
                filter_name = 'msi_filter_%d' % i
                self.msi_filters += [tf.get_variable(filter_name,
                                                     shape=[self.msi_layers[i],
                                                            self.msi_layers[i - 1]])]

            cause_name = 'msi_cause_%d' % i
            init = tf.constant_initializer(self.msi_cause_init[i])
            self.msi_causes += [tf.get_variable(cause_name,
                                                shape=[1, self.msi_layers[i]],
                                                initializer=init)]

        self.m1_predictions = []

        for i in range(len(self.m1_layers)):
            self.m1_predictions += [tf.nn.leaky_relu(tf.matmul(self.m1_causes[i],
                                                               self.m1_filters[i]))]

        self.msi_predictions = []

        for i in range(len(self.msi_layers)):
            if i == 0:
                self.msi_predictions += [tf.nn.leaky_relu(tf.matmul(self.msi_causes[i],
                                                                    self.msi_filters[i]))]
                self.msi_predictions += [tf.nn.leaky_relu(tf.matmul(self.msi_causes[i],
                                                                    self.msi_filters[i+1]))]
            else:
                self.msi_predictions += [tf.nn.leaky_relu(tf.matmul(self.msi_causes[i],
                                                                    self.msi_filters[i+1]))]

        self.m1_bu_error = []
        self.m1_update_filter = []
        self.m1_cause_grad = []

        for i in range(len(self.m1_layers)):
            if i == 0:
                self.m1_bu_error += [tf.losses.mean_squared_error(
                    self.x_m1,
                    self.m1_predictions[i],
                    reduction=tf.losses.Reduction.NONE)]

            else:
                self.m1_bu_error += [tf.losses.mean_squared_error(
                    tf.stop_gradient(self.m1_causes[i - 1]),
                    self.m1_predictions[i],
                    reduction=tf.losses.Reduction.NONE)]

            if len(self.m1_layers) > (i + 1):
                td_error = tf.losses.mean_squared_error(
                    tf.stop_gradient(self.m1_predictions[i+1]),
                    self.m1_causes[i],
                    reduction=tf.losses.Reduction.NONE)
            else:
                td_error = tf.losses.mean_squared_error(
                    tf.stop_gradient(self.msi_predictions[0]),
                    self.m1_causes[i],
                    reduction=tf.losses.Reduction.NONE)

            reg_error = self.reg_m1_causes[i] * (self.m1_causes[i] ** 2)
            self.m1_cause_grad += [tf.gradients([self.m1_bu_error[i], td_error, reg_error],
                                                self.m1_causes[i])[0]]

        self.msi_bu_error = []
        self.msi_reg_error = []
        self.msi_update_filter = []
        self.msi_cause_grad = []

        for i in range(len(self.msi_layers)):
            if i == 0:
                self.msi_bu_error += [tf.losses.mean_squared_error(
                    tf.stop_gradient(self.m1_causes[-1]),
                    self.msi_predictions[i],
                    reduction=tf.losses.Reduction.NONE)]

                self.msi_bu_error += [tf.losses.mean_squared_error(
                    tf.stop_gradient(self.x_m2),
                    self.msi_predictions[i+1],
                    reduction=tf.losses.Reduction.NONE)]

                self.msi_reg_error += [self.reg_msi_causes[i] * (self.msi_causes[i] ** 2)]

                if len(self.msi_layers) > 1:
                    raise NotImplementedError
                else:
                    if self.available_modality == 'both':
                        self.msi_cause_grad += [
                            tf.gradients([self.msi_bu_error[i],
                                          self.msi_bu_error[i+1],
                                          self.msi_reg_error[i]],
                                          self.msi_causes[i])[0]]
                    elif self.available_modality == 'visual':
                        self.msi_cause_grad += [tf.gradients([self.msi_bu_error[i],
                                                              self.msi_reg_error[i]],
                                                              self.msi_causes[i])[0]]
                    elif self.available_modality == 'pose':
                        self.msi_cause_grad += [tf.gradients([self.msi_bu_error[i + 1],
                                                              self.msi_reg_error[i]],
                                                              self.msi_causes[i])[0]]
            else:
                raise NotImplementedError

        self.m1_update_cause = []
        self.msi_update_cause = []

        with tf.control_dependencies(self.m1_cause_grad + self.msi_cause_grad):
            for i in range(len(self.m1_layers)):
                self.m1_update_cause += [tf.assign_sub(self.m1_causes[i],
                    (self.lr_m1_causes[i] * self.m1_cause_grad[i]))]
            for i in range(len(self.msi_layers)):
                self.msi_update_cause += [tf.assign_sub(self.msi_causes[i],
                    (self.lr_msi_causes[i] * self.msi_cause_grad[i]))]

        saver = tf.train.Saver(self.m1_filters + self.msi_filters)
        saver.restore(self.sess, self._model_path + "/main.ckpt")

    def predict(self, visual_data=None, odometry_data=None, verbose=False):

        self.sess.run(tf.variables_initializer(self.m1_causes + self.msi_causes))
        iter = 1

        while True:
            m1_cause, msi_cause, m1_error, msi_error, m1_pred, msi_pred, m1_filter, msi_filter = self.sess.run(
                    [self.m1_update_cause, self.msi_update_cause, self.m1_bu_error, self.msi_bu_error,
                    self.m1_predictions, self.msi_predictions, self.m1_filters, self.msi_filters],
                    feed_dict={self.x_m1: visual_data, self.x_m2: odometry_data})

            m1_epoch_loss = [np.mean(item) for item in m1_error]
            msi_epoch_loss = [np.mean(item) for item in msi_error]

            if (np.all(np.array(m1_epoch_loss + msi_epoch_loss) < self.error_criterion)) or (iter >= self.max_iter):
                if verbose:
                    print_str = ', '.join(['%.8f' % elem for elem in m1_epoch_loss + msi_epoch_loss])
                    self._logger.info('(%d) %s' % (iter, print_str))
                break
            else:
                iter += 1

        recon_odom = np.dot(msi_cause[0], msi_filter[1])
        recon_vis = np.dot(msi_cause[0], msi_filter[0])
        for l in range(len(m1_filter), 0, -1):
            recon_vis = np.dot(recon_vis, m1_filter[l - 1])

        return msi_cause, recon_vis, recon_odom
