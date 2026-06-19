import os
import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from ctypes import *
from app.HiK.MvImport.MvCameraControl_class import *
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class CamNode(Node):
    def __init__(self):
        super().__init__('cam_node')
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.fps_test_pub = self.create_publisher(String, '/fps_test_recv_dec', 10)
        self.init_cam()
        self.get_logger().info('CamNode initialized')
        self.timer_period = 0.004  # 250 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def init_cam(self):
        MvCamera.MV_CC_Initialize()

        SDKVersion = MvCamera.MV_CC_GetSDKVersion()
        self.get_logger().info("SDKVersion[0x%x]" % SDKVersion)

        deviceList = MV_CC_DEVICE_INFO_LIST()
        tlayerType = (MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE)
        
        # ch:枚举设备 | en:Enum device
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0:
            self.get_logger().info("enum devices fail! ret[0x%x]" % ret)
            sys.exit()

        if deviceList.nDeviceNum == 0:
            self.get_logger().info("find no device!")
            sys.exit()

        self.get_logger().info("Find %d devices!" % deviceList.nDeviceNum)

        for i in range(0, deviceList.nDeviceNum):
            mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE or mvcc_dev_info.nTLayerType == MV_GENTL_GIGE_DEVICE:
                self.get_logger().info("\ngige device: [%d]" % i)
                strModeName = ""
                for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                    strModeName = strModeName + chr(per)
                self.get_logger().info("device model name: %s" % strModeName)

                nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
                nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
                nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
                nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
                self.get_logger().info("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
            elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
                self.get_logger().info("\nu3v device: [%d]" % i)
                strModeName = ""
                for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
                    if per == 0:
                        break
                    strModeName = strModeName + chr(per)
                self.get_logger().info("device model name: %s" % strModeName)

                strSerialNumber = ""
                for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                    if per == 0:
                        break
                    strSerialNumber = strSerialNumber + chr(per)
                self.get_logger().info("user serial number: %s" % strSerialNumber)
            elif mvcc_dev_info.nTLayerType == MV_GENTL_CAMERALINK_DEVICE:
                self.get_logger().info("\nCML device: [%d]" % i)
                strModeName = ""
                for per in mvcc_dev_info.SpecialInfo.stCMLInfo.chModelName:
                    if per == 0:
                        break
                    strModeName = strModeName + chr(per)
                self.get_logger().info("device model name: %s" % strModeName)

                strSerialNumber = ""
                for per in mvcc_dev_info.SpecialInfo.stCMLInfo.chSerialNumber:
                    if per == 0:
                        break
                    strSerialNumber = strSerialNumber + chr(per)
                self.get_logger().info("user serial number: %s" % strSerialNumber)
            elif mvcc_dev_info.nTLayerType == MV_GENTL_XOF_DEVICE:
                self.get_logger().info("\nXoF device: [%d]" % i)
                strModeName = ""
                for per in mvcc_dev_info.SpecialInfo.stXoFInfo.chModelName:
                    if per == 0:
                        break
                    strModeName = strModeName + chr(per)
                self.get_logger().info("device model name: %s" % strModeName)

                strSerialNumber = ""
                for per in mvcc_dev_info.SpecialInfo.stXoFInfo.chSerialNumber:
                    if per == 0:
                        break
                    strSerialNumber = strSerialNumber + chr(per)
                self.get_logger().info("user serial number: %s" % strSerialNumber)
            elif mvcc_dev_info.nTLayerType == MV_GENTL_CXP_DEVICE:
                self.get_logger().info("\nCXP device: [%d]" % i)
                strModeName = ""
                for per in mvcc_dev_info.SpecialInfo.stCXPInfo.chModelName:
                    if per == 0:
                        break
                    strModeName = strModeName + chr(per)
                self.get_logger().info("device model name: %s" % strModeName)

                strSerialNumber = ""
                for per in mvcc_dev_info.SpecialInfo.stCXPInfo.chSerialNumber:
                    if per == 0:
                        break
                    strSerialNumber = strSerialNumber + chr(per)
                self.get_logger().info("user serial number: %s" % strSerialNumber)

        nConnectionNum = 0
        self.cam = MvCamera()
        # ch:选择设备并创建句柄| en:Select device and create handle
        stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents

        ret = self.cam.MV_CC_CreateHandle(stDeviceList)
        if ret != 0:
            self.get_logger().info("create handle fail! ret[0x%x]" % ret)
            sys.exit()
        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            self.get_logger().info("open device fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:设置触发模式为off | en:Set trigger mode as off
        ret = self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        if ret != 0:
            self.get_logger().info("set trigger mode fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:获取数据包大小 | en:Get payload size
        stParam =  MVCC_INTVALUE()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
        
        ret = self.cam.MV_CC_GetIntValue("PayloadSize", stParam)
        if ret != 0:
            self.get_logger().info("get payload size fail! ret[0x%x]" % ret)
            sys.exit()
        nPayloadSize = stParam.nCurValue

        # ch:开始取流 | en:Start grab image
        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            self.get_logger().info("start grabbing fail! ret[0x%x]" % ret)
            sys.exit()

        self.data_buf = (c_ubyte * nPayloadSize)()

    def decode(self, data):
        ret = self.cam.MV_CC_GetImageBuffer(data, 1000)
        if ret == 0:
            pixel_type = data.stFrameInfo.enPixelType
            width = data.stFrameInfo.nWidth
            height = data.stFrameInfo.nHeight
            image_mat = None
            self.get_logger().info(f"Received image: {width}x{height}, PixelType: {pixel_type}")
            if pixel_type == PixelType_Gvsp_BayerRG8:
                bayer = np.ctypeslib.as_array(data.pBufAddr, shape=(height * width,))
                bayer = bayer.reshape((height, width))
                image_mat = cv2.cvtColor(bayer, cv2.COLOR_BAYER_RG2RGB)
            elif pixel_type == PixelType_Gvsp_BGR8_Packed:
                img = np.ctypeslib.as_array(data.pBufAddr, shape=(height * width * 3,))
                image_mat = img.reshape((height, width, 3))
            elif pixel_type == PixelType_Gvsp_BayerGB8:
                bayer = np.ctypeslib.as_array(data.pBufAddr, shape=(height * width,))
                bayer = bayer.reshape((height, width))
                image_mat = cv2.cvtColor(bayer, cv2.COLOR_BAYER_GB2RGB)
            elif pixel_type == PixelType_Gvsp_HB_RGB8_Packed:
                img = np.ctypeslib.as_array(data.pBufAddr, shape=(height * width * 3,))
                image_mat = img.reshape((height, width, 3))
                image_mat = cv2.cvtColor(image_mat, cv2.COLOR_RGB2BGR)
            elif pixel_type == PixelType_Gvsp_RGB8_Packed:
                img = np.ctypeslib.as_array(data.pBufAddr, shape=(height * width * 3,))
                image_mat = img.reshape((height, width, 3))
                image_mat = cv2.cvtColor(image_mat, cv2.COLOR_RGB2BGR)
            else:
                self.get_logger().warn("Unsupported pixel type")
            return image_mat
        return None

    def timer_callback(self):
        time0 = time.time()
        stOutFrame = MV_FRAME_OUT()
        time1 = time.time()
        memset(byref(stOutFrame), 0, sizeof(stOutFrame))
        time2 = time.time()
        image_mat = self.decode(stOutFrame) # TODO: data type for Image for minimal time of processing img
        time3 = time.time()
        if image_mat is not None and image_mat.size > 0:
            time4 = time.time()
            msg = self.bridge.cv2_to_imgmsg(image_mat, encoding="rgb8")
            time5 = time.time()
            self.publisher_.publish(msg)
            time6 = time.time()
            self.cam.MV_CC_FreeImageBuffer(stOutFrame)
            time7 = time.time()

            # cv2.imshow("Camera Feed", image_mat)
            # cv2.waitKey(1)
            self.get_logger().info(f'==='*20)
            self.get_logger().info(f'stoutframe time: {(time1 - time0)*1000:.4f} ms')
            self.get_logger().info(f'img memset time: {(time2 - time1)*1000:.4f} ms')
            self.get_logger().info(f'img decode time: {(time3 - time2)*1000:.4f} ms')
            self.get_logger().info(f'img bridge time: {(time5 - time4)*1000:.4f} ms')
            self.get_logger().info(f'img publish time: {(time6 - time5)*1000:.4f} ms')
            self.get_logger().info(f'img free time: {(time.time() - time6)*1000:.4f} ms')
            self.get_logger().info(f'img show time: {(time7 - time6)*1000:.4f} ms')
            self.get_logger().info(f'img process total time: {(time6 - time1)*1000:.4f} ms')
            self.get_logger().info(f'img total time: {(time.time() - time0)*1000:.4f} ms')
        else:
            self.get_logger().warn("No image received or image is empty")
            # not need to free stOutFrame here, as it is not used

        # Test FPS
        self.fps_test_pub.publish(String(data='FPS Test'))

    def destroy_node(self):
        self.get_logger().info('Shutting down CamNode...')
        # Camera cleanup
        try:
            self.cam.MV_CC_StopGrabbing()
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()
        except Exception as e:
            self.get_logger().warn(f"Camera cleanup error: {e}")
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    cam_node = CamNode()
    try:
        rclpy.spin(cam_node)
    except KeyboardInterrupt:
        pass
    cam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
