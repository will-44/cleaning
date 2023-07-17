#!/usr/bin/env python3

import sys, io, datetime
from pathlib import Path
import argparse
from subprocess import DEVNULL, Popen, PIPE

import rospy
import rosbag
from importlib import import_module
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

class ExporterBase():
    def __init__(self, filename:str, fps:float, codec:str, pix_fmt:str):
        rospy.loginfo("Exporting: %s" % filename)
        self.fproc = Popen(['ffmpeg', '-hide_banner', '-loglevel', 'error',
                            '-y', '-f', 'image2pipe', '-r', str(fps), '-i', '-',
                            '-c:v', codec, '-pix_fmt', pix_fmt, filename],
                           stdin=PIPE, stdout=DEVNULL, stderr=PIPE)

        self.count = 0

    def _is_ready(self):
        return self.fproc.poll() is None

    def _handle_image(self, image):
        self.fproc.stdin.write(cv2.imencode(".png", image)[1].tobytes())
        self.count += 1

    def _shutdown(self):
        self.fproc.stdin.close()
        self.fproc.wait()

        for line in io.TextIOWrapper(self.fproc.stderr, encoding="utf-8"):
            clean_line = line.strip()
            # We don't care to report if EoF happens 
            if clean_line != 'pipe:: End of file' :
                rospy.logerr('[FFMPEG] ' + clean_line)

        rospy.loginfo('Processed %i images' % self.count)

    def _callback_image(self, msg_in):
        rospy.loginfo_once("Started recording...")
        cv_image = None
        try:
            if type(msg_in) is CompressedImage:
                cv_image = self.bridge.compressed_imgmsg_to_cv2( msg_in, "bgr8" )
            elif type(msg_in) is Image:
                cv_image = self.bridge.imgmsg_to_cv2( msg_in, "bgr8" )
            else:
                raise TypeError('Unable to decode message type: ' + type(msg_in).__name__)
        except CvBridgeError as e:
            rospy.loginfo(e)
            return

        self._handle_image(cv_image)

class GenericMessageSubscriber(object):
    def __init__(self, topic_name, callback):
        self._binary_sub = rospy.Subscriber(
            topic_name, rospy.AnyMsg, self.generic_message_callback)
        self._callback = callback

    def generic_message_callback(self, data):
        assert sys.version_info >= (2,7) #import_module's syntax needs 2.7
        connection_header =  data._connection_header['type'].split('/')
        ros_pkg = connection_header[0] + '.msg'
        msg_type = connection_header[1]
        msg_class = getattr(import_module(ros_pkg), msg_type)
        msg = msg_class().deserialize(data._buff)
        self._callback(msg)

    def unregister(self):
        if self._binary_sub is not None:
            self._binary_sub.unregister()

class ImageTopicExporter(ExporterBase):
    def __init__(self, topic_name:str, filename:str, fps:float, codec:str, pix_fmt:str):
        super().__init__(filename, fps, codec, pix_fmt)
        self.bridge = CvBridge()
        self.img_sub = GenericMessageSubscriber(topic_name, self._callback_image)

    def run(self):
        if self.is_ready():
            try:
                rospy.spin()
            except rospy.ROSInterruptException:
                pass

    def is_ready(self):
        return self._is_ready() and (self.img_sub is not None)

    def shutdown(self):
        if self.img_sub is not None:
            self.img_sub.unregister()

        self._shutdown()

class ImageBagExporter(ExporterBase):
    def __init__(self, bag_name:str, topic_name:str, filename:str, fps:float, codec:str, pix_fmt:str):
        super().__init__(filename, fps, codec, pix_fmt)
        self.bridge = CvBridge()
        self.bag = rosbag.Bag(bag_name)
        self.topic_name = topic_name

    def run(self):
        for topic, msg, stamp in self.bag.read_messages(topics=[self.topic_name]):
            self._callback_image(self.recode_message(msg))

    def is_ready(self):
        return self._is_ready() and (self.bag)

    def shutdown(self):
        self.bag.close()
        self._shutdown()

    def recode_message(self, msg):
        recode = None
        buf = io.BytesIO()
        msg.serialize(buf)
        buf.seek(0)
        if msg._md5sum == CompressedImage._md5sum:
            recode = CompressedImage()
        elif msg._md5sum == Image._md5sum:
            recode = Image()
        else:
            raise TypeError('Unable to match message type: %s (%s)' % (type(msg).__name__, msg._md5sum))

        if recode is not None:
            recode.deserialize(buf.read())

        return recode

def parse_args():
    parser = argparse.ArgumentParser(description='Extract ROS images and dump as a video')
    parser.add_argument('-topic', action="store", metavar='TOPIC',
                        help='Selects the topic to listen to (supported: sensors_msgs/Image, sensors_msgs/CompressedImage)')
    parser.add_argument('-r', '--framerate', action="store", default=30.0,
                        help='Sets the exported framerate (default: 30.0fps)')
    parser.add_argument('-c', '--codec', action="store", default='libx264',
                        help='Sets the exported video codec (default: libx264)')
    parser.add_argument('-p', '--pix-fmt', action="store", default='yuv420p',
                        help='Sets the exported pixel format (default: yuv420p)')
    parser.add_argument('-f', '--filename', action="store", type=str,
                        help='Sets the exported filename')
    parser.add_argument('-b', '--bag', action="store", type=str,
                        help='Import from a rosbag instead of a live topic')
    args = parser.parse_args(rospy.myargv()[1:])
    return args

if __name__ == '__main__':
    args = parse_args()
    rospy.init_node('image_saver_py', anonymous=True)

    if args.bag:
        if not args.filename:
            args.filename = Path(args.bag).stem + args.topic.replace('/', '_') + ".mp4"

        exporter = ImageBagExporter(args.bag, args.topic, args.filename, args.framerate, args.codec, args.pix_fmt)
    else:
        if not args.filename:
            ts = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            args.filename = ts + args.topic.replace('/', '_') + ".mp4"

        exporter = ImageTopicExporter(args.topic, args.filename, args.framerate, args.codec, args.pix_fmt)


    if exporter is not None:
        try:
            exporter.run()
        except Exception as ex:
            rospy.logerr(ex)

        rospy.loginfo("Shutting down...")
        exporter.shutdown()
