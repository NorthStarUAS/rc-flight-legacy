import copy
import lxml.etree as ET
import numpy as np
import os.path

class Calibration():
    def __init__(self, cal_file=None):
        # defined temp range
        self.min_temp = 27.0
        self.max_temp = 27.0
        # temp vs. bias fit function coefficients
        self.p_bias = np.array([0.0, 0.0, 0.0])
        self.q_bias = np.array([0.0, 0.0, 0.0])
        self.r_bias = np.array([0.0, 0.0, 0.0])
        self.ax_bias = np.array([0.0, 0.0, 0.0])
        self.ay_bias = np.array([0.0, 0.0, 0.0])
        self.az_bias = np.array([0.0, 0.0, 0.0])
        # scaling factor
        self.p_scale = np.array([0.0, 0.0, 1.0])
        self.q_scale = np.array([0.0, 0.0, 1.0])
        self.r_scale = np.array([0.0, 0.0, 1.0])
        self.ax_scale = np.array([0.0, 0.0, 1.0])
        self.ay_scale = np.array([0.0, 0.0, 1.0])
        self.az_scale = np.array([0.0, 0.0, 1.0])
        if cal_file:
            self.load(cal_file)

    # load a configuration from file
    def load_text_deprecated(self, cal_file):
        try:
            f = open(cal_file, 'r')
        except:
            print "can't open calibration file for reading: ", cal_file
            return
        p1, p2, p3 = f.readline().split()
        self.p_bias = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = f.readline().split()
        self.q_bias = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = f.readline().split()
        self.r_bias = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = f.readline().split()
        self.ax_bias = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = f.readline().split()
        self.ay_bias = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = f.readline().split()
        self.az_bias = np.array([p1, p2, p3], dtype=np.float64)

        p1, p2, p3 = f.readline().split()
        self.p_scale = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = f.readline().split()
        self.q_scale = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = f.readline().split()
        self.r_scale = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = f.readline().split()
        self.ax_scale = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = f.readline().split()
        self.ay_scale = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = f.readline().split()
        self.az_scale = np.array([p1, p2, p3], dtype=np.float64)

    def myfloat(self, input):
        if input == "":
            return 0.0
        else:
            return float(input)
        
    # load/parse an xml calibration file
    def load_xml(self, cal_file):
        try:
            self.xml = ET.parse(str(cal_file))
        except:
            print filename + ": xml parse error:\n" + str(sys.exc_info()[1])
            return
        root = self.xml.getroot()
        self.min_temp = self.myfloat(root.find('min-temp-C').text)
        self.max_temp = self.myfloat(root.find('max-temp-C').text)
        
        node = root.find('p')
        p1, p2, p3 = node.find('bias').text.split()
        self.p_bias = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = node.find('scale').text.split()
        self.p_scale = np.array([p1, p2, p3], dtype=np.float64)

        node = root.find('q')
        p1, p2, p3 = node.find('bias').text.split()
        self.q_bias = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = node.find('scale').text.split()
        self.q_scale = np.array([p1, p2, p3], dtype=np.float64)

        node = root.find('r')
        p1, p2, p3 = node.find('bias').text.split()
        self.r_bias = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = node.find('scale').text.split()
        self.r_scale = np.array([p1, p2, p3], dtype=np.float64)

        node = root.find('ax')
        p1, p2, p3 = node.find('bias').text.split()
        self.ax_bias = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = node.find('scale').text.split()
        self.ax_scale = np.array([p1, p2, p3], dtype=np.float64)

        node = root.find('ay')
        p1, p2, p3 = node.find('bias').text.split()
        self.ay_bias = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = node.find('scale').text.split()
        self.ay_scale = np.array([p1, p2, p3], dtype=np.float64)

        node = root.find('az')
        p1, p2, p3 = node.find('bias').text.split()
        self.az_bias = np.array([p1, p2, p3], dtype=np.float64)
        p1, p2, p3 = node.find('scale').text.split()
        self.az_scale = np.array([p1, p2, p3], dtype=np.float64)

    def load(self, cal_file):
        extension = os.path.splitext(cal_file)[1]
        if extension == ".txt":
            self.load_text_deprecated(cal_file)
        elif extension == ".xml":
            self.load_xml(cal_file)
            
    def update_node(self, parent, node, value):
        e = parent.find(node)
        if e == None:
            e = ET.SubElement(parent, node)
        e.text = str(value)

        # save a configuration file
    def save_xml(self, cal_file):
        root = ET.Element('PropertyList')
        self.update_node(root, 'min-temp-C', self.min_temp)
        self.update_node(root, 'max-temp-C', self.max_temp)

        sensor = ET.SubElement(root, 'p')
        p = self.p_bias
        self.update_node(sensor, 'bias', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))
        p = self.p_scale
        self.update_node(sensor, 'scale', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))

        sensor = ET.SubElement(root, 'q')
        p = self.q_bias
        self.update_node(sensor, 'bias', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))
        p = self.q_scale
        self.update_node(sensor, 'scale', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))

        sensor = ET.SubElement(root, 'r')
        p = self.r_bias
        self.update_node(sensor, 'bias', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))
        p = self.r_scale
        self.update_node(sensor, 'scale', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))

        sensor = ET.SubElement(root, 'ax')
        p = self.ax_bias
        self.update_node(sensor, 'bias', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))
        p = self.ax_scale
        self.update_node(sensor, 'scale', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))

        sensor = ET.SubElement(root, 'ay')
        p = self.ay_bias
        self.update_node(sensor, 'bias', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))
        p = self.ay_scale
        self.update_node(sensor, 'scale', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))

        sensor = ET.SubElement(root, 'az')
        p = self.az_bias
        self.update_node(sensor, 'bias', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))
        p = self.az_scale
        self.update_node(sensor, 'scale', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))

        self.xml = ET.ElementTree(root)
        try:
            self.xml.write(cal_file, encoding="us-ascii",
                           xml_declaration=False, pretty_print=True)
        except:
            print "error saving " + cal_file + ": " + str(sys.exc_info()[1])
            return

    # correct the IMU data given the current bias and scale errors
    def correct(self, imu_data):
        imu_corrected = []
        p_bias_func = np.poly1d(self.p_bias)
        q_bias_func = np.poly1d(self.q_bias)
        r_bias_func = np.poly1d(self.r_bias)
        ax_bias_func = np.poly1d(self.ax_bias)
        ay_bias_func = np.poly1d(self.ay_bias)
        az_bias_func = np.poly1d(self.az_bias)
        p_scale_func = np.poly1d(self.p_scale)
        q_scale_func = np.poly1d(self.q_scale)
        r_scale_func = np.poly1d(self.r_scale)
        ax_scale_func = np.poly1d(self.ax_scale)
        ay_scale_func = np.poly1d(self.ay_scale)
        az_scale_func = np.poly1d(self.az_scale)
        for imu in imu_data:
            newimu = copy.copy(imu)
            t = imu.temp
            if t < self.min_temp:
                t = min_temp
            if t > self.max_temp:
                t = max_temp    
            newimu.p = (imu.p - p_bias_func(t)) * p_scale_func(t)
            newimu.q = (imu.q - q_bias_func(t)) * q_scale_func(t)
            newimu.r = (imu.r - r_bias_func(t)) * r_scale_func(t)
            newimu.ax = (imu.ax - ax_bias_func(t)) * ax_scale_func(t)
            newimu.ay = (imu.ay - ay_bias_func(t)) * ay_scale_func(t)
            newimu.az = (imu.az - az_bias_func(t)) * az_scale_func(t)
            imu_corrected.append(newimu)
        return imu_corrected
    
    # back correct the IMU data given the current bias and scale errors
    # (i.e. assuming corrected data, generate the raw values)
    def back_correct(self, imu_data):
        imu_corrected = []
        p_bias_func = np.poly1d(self.p_bias)
        q_bias_func = np.poly1d(self.q_bias)
        r_bias_func = np.poly1d(self.r_bias)
        ax_bias_func = np.poly1d(self.ax_bias)
        ay_bias_func = np.poly1d(self.ay_bias)
        az_bias_func = np.poly1d(self.az_bias)
        p_scale_func = np.poly1d(self.p_scale)
        q_scale_func = np.poly1d(self.q_scale)
        r_scale_func = np.poly1d(self.r_scale)
        ax_scale_func = np.poly1d(self.ax_scale)
        ay_scale_func = np.poly1d(self.ay_scale)
        az_scale_func = np.poly1d(self.az_scale)
        for imu in imu_data:
            newimu = copy.copy(imu)
            t = imu.temp
            if t < self.min_temp:
                t = self.min_temp
            if t > self.max_temp:
                t = self.max_temp    
            newimu.p = imu.p / p_scale_func(t) + p_bias_func(t)
            newimu.q = imu.q / q_scale_func(t) + q_bias_func(t)
            newimu.r = imu.r / r_scale_func(t) + r_bias_func(t)
            newimu.ax = imu.ax / ax_scale_func(t) + ax_bias_func(t)
            newimu.ay = imu.ay / ay_scale_func(t) + ay_bias_func(t)
            newimu.az = imu.az / az_scale_func(t) + az_bias_func(t)
            imu_corrected.append(newimu)
        return imu_corrected
