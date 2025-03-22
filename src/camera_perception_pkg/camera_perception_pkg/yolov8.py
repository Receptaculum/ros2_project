import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces_pkg.msg import SegmentGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from ultralytics import YOLO

import cv2
import cv_bridge

import torch

import os


## <Parameter> #####################################################################################

# 노드 이름
NODE_NAME = "yolov8"

# 발행 토픽 이름
TOPIC_NAME = "segmented_data"

# 라벨 이름 (lane_1, lane_2, traffic_light, car)
LABEL_NAME = ["lane1", "lane2", "traffic_light", "car"]

# 구독 토픽 이름
SUB_TOPIC_NAME = "image_publisher"

# PT 파일 이름 지정 (확장자 포함)
PT_NAME = "lib/pt/best.pt"

# CV 처리 영상 출력 여부
DEBUG = True

# 동작 모드 (cpu, cuda, xpu)
DEVICE = "cpu"

# Thread 수 (CPU 전용, 본인의 CPU Core 수보다 약간 적게 설정)
THREAD = 8

######################################################################################################


## <로그 출력> #########################################################################################
# DEBUG	self.get_logger().debug("msg")
# INFO	self.get_logger().info("msg")
# WARN	self.get_logger().warn("msg")
# ERROR	self.get_logger().error("msg")
# FATAL	self.get_logger().fatal("msg")
#######################################################################################################


## <QOS> ##############################################################################################
# Reliability : RELIABLE(신뢰성 보장), BEST_EFFORT(손실 감수, 최대한 빠른 전송)
# Durability : VOLATILE(전달한 메시지 제거), TRANSIENT_LOCAL(전달한 메시지 유지) / (Subscriber가 없을 때에 한함)
# History : KEEP_LAST(depth 만큼의 메시지 유지), KEEP_ALL(모두 유지)
# Liveliness : 활성 상태 감시
# Deadline : 최소 동작 보장 
#######################################################################################################


class yolov8(Node):
    def __init__(self, node_name, topic_name : list, sub_topic_name, pt_name, debug, label_name, device, thread):
        super().__init__(node_name)

        self.model = YOLO(os.path.dirname(__file__) + "/" + pt_name) # YOLO Model 선언

        if device == "cuda": # Nvidia GPU 설정
            self.model.to(device)

        elif device == "xpu": # Intel GPU 설정
            self.model.to(device)

        elif device == "cpu": # CPU 설정
            torch.set_num_threads(thread)
            self.model.to(device)


        self.qos_pub = QoSProfile( # Publisher QOS 설정
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
                )

        self.qos_sub = QoSProfile( # Subscriber QOS 설정
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
                )
        
        # Topic 이름 선언
        self.topic = topic_name

        # 라벨 이름 변수 선언
        self.label_0 = label_name[0]
        self.label_1 = label_name[1]
        self.label_2 = label_name[2]
        self.label_3 = label_name[3]

        # 디버그 변수 선언
        self.debug = debug

        # Publisher 선언
        self.publisher = self.create_publisher(SegmentGroup, self.topic, self.qos_pub) 

        # Subscriber 선언
        self.subscriber = self.create_subscription(Image, sub_topic_name, self.recognizer_callback, self.qos_sub)

        # CV Bridge Object 선언
        self.bridge = cv_bridge.CvBridge()

    def recognizer_callback(self, img_msg):
        # Publishing을 위한 Message 선언
        self.msg = SegmentGroup()

        self.frame = self.bridge.imgmsg_to_cv2(img_msg) # Frame 수령 및 처리
        self.predicted = self.model.predict(self.frame, verbose=False) # Frame Segmentation 처리

        if self.debug == True: # 디버깅(화면 출력) 여부 결정
            cv2.imshow("YOLO", self.predicted[0].plot())
            cv2.waitKey(5)

        # 카운트를 위한 변수 선언
        cnt_0 = 0
        cnt_1 = 0
        cnt_2 = 0
        cnt_3 = 0 

        # 확률에 대한 내림차순 정렬 (이미 Ultralytics의 전처리 과정에 해당 작업이 포함되어 있기에 제외함, Deprecated)
        # predict_box = self.predicted[0].boxes[torch.argsort(self.predicted[0].boxes.conf, descending=True)]
        # predict_mask = self.predicted[0].masks[torch.argsort(self.predicted[0].boxes.conf, descending=True)]

        # Box, Mask 변수 선언
        predict_box = self.predicted[0].boxes
        predict_mask = self.predicted[0].masks

        self.get_logger().info(f"{len(predict_box.conf.tolist())} object(s) detected | value = {predict_box.conf.tolist()}")

        # Box : 상자 / Keypoint : 관절 표현 / Mask : 영역 표시
        for n, predict_val in enumerate(predict_box):
            name = self.predicted[0].names[int(predict_val.cls.item())].strip()

            if  name == self.label_0.strip() and cnt_0 == 0:
                self.msg.lane_1 = torch.Tensor(predict_mask[n].xy[0]).to(torch.int16).flatten().tolist()
                cnt_0 += 1
            
            elif name == self.label_1.strip() and cnt_1 == 0:
                self.msg.lane_2 = torch.Tensor(predict_mask[n].xy[0]).to(torch.int16).flatten().tolist()    
                cnt_1 += 1

            elif name == self.label_2.strip() and cnt_2 == 0:
                self.msg.traffic_light = predict_box[n].xyxy[0].to(torch.int16).flatten().tolist()
                cnt_2 += 1          

            elif name == self.label_3.strip() and cnt_3 == 0:
                self.msg.car = predict_box[n].xyxy[0].to(torch.int16).flatten().tolist()
                cnt_3 += 1    

                # Polygon 형식
                # self.msg_2.data = self.predicted[0].masks[n].xy[0].flatten().tolist() 

                # Box 형식
                # self.msg_2.data = self.predicted[0].boxes[n].xyxy[0]       

        self.publisher.publish(self.msg)      

    def shutdown(self):
        cv2.destroyAllWindows() # CV 창 닫기


def main():
    rclpy.init()
    yolov8_node = yolov8(NODE_NAME, TOPIC_NAME, SUB_TOPIC_NAME, PT_NAME, DEBUG, LABEL_NAME, DEVICE, THREAD)
    rclpy.spin(yolov8_node)

    yolov8_node.shutdown()
    yolov8_node.destroy_node()
    rclpy.shutdown()


if __name__== "__main__":
    main() 