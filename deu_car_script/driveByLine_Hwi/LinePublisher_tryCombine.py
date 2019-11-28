#!/usr/bin/env python
# -*- coding:utf-8 -*-
'''
작성자 : 신휘정
작성일자 : 2019.11.19
용도 : 양쪽에서 차선 인식 하고 모멘텀값과 오차값을 publish
TODO
'''
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from Ros01team02.msg import DriveLineInfo

class DriveLinePublisher:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, self.detect_line)
    self.driveLineInfo_pub = rospy.Publisher('DriveLineInfo',DriveLineInfo,queue_size=1)
    self.driveLineInfo = DriveLineInfo()
    rate=rospy.Rate(2)

    '''self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.twist = Twist()'''


  def detect_line(self, msg):
    # BEGIN BRIDGE
    image = self.bridge.imgmsg_to_cv2(msg)
    # END BRIDGE

    # BEGIN HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # END HSV

  # BEGIN FILTER
    #filter white color
    lower_white = numpy.array([0, 0, 180])
    upper_white = numpy.array([255, 30, 255])
    # hsv(색상, 채도, 진하기)
    whiteMask = cv2.inRange(hsv, lower_white, upper_white)

    #filter yellow color
    lower_yellow = numpy.array([20, 160, 95])
    upper_yellow = numpy.array([40, 255, 255])
    # hsv(색상, 채도, 진하기)
    yellowMask = cv2.inRange(hsv, lower_yellow, upper_yellow)

  # END FILTER

  # BEGIN setDetectArea
    h, w, d = image.shape
    #rospy.loginfo(type(yellowMask));    -> type(yellowMask = ndarray

#    mask = cv2.bitwise_and(image, image, mask=yellowMask)   #검은바탕 노란줄만 노란색으로 나옴
    search_top = 3 * h / 4
    search_bot = 3 * h / 4 + 20

    whiteMask[0 : search_top][:]=0
    whiteMask[search_bot : h][:] = 0
    for i in range(search_top,search_bot):
      area = 0
      #왼쪽감지
      for j in range(w/2, 0, -1):
        if (whiteMask[i][j] == yellowMask[i][j]):
          whiteMask[i][j] = 0  # 검정
        else:
          whiteMask[i][j] = 255  # 흰색
      #오른쪽감지
      for j in range(2/w, w):
        if (whiteMask[i][j] == yellowMask[i][j]):
          whiteMask[i][j] = 0  # 검정
        else:
          whiteMask[i][j] = 255  # 흰색

    #BEGIN detect driveLine
    #오른쪽
    for i in range(search_top, search_bot):
      area = 0
      for j in range(w / 2, w, 1):
        if whiteMask[i, j - 1] != whiteMask[i, j]:  # 선과 선사이 감지
          area = area + 1
        if (area > 1):
          whiteMask[i, j] = 0

    #왼쪽
    for i in range(search_top, search_bot):
      area = 0
      for j in range(w / 2, 0, -1):
        if whiteMask[i, j] != whiteMask[i, j + 1]:  # 선과 선사이 감지
          area = area + 1
        if (area > 1):
          whiteMask[i, j] = 0
    #오른쪽차선모멘텀 publish
  #  self.RightmomentumPublisher(whiteMask, image )
    #왼쪽차선모멘텀 publish
  #  self.LeftmomentumPublisher(whiteMask, image)

    rightLine = self.get_rightLine(h, w, image, whiteMask)

    leftLine = self.get_leftLine(h, w, image, whiteMask)

    self.driveLineInfo_pub.publish(self.driveLineInfo)

    rightValue = self.driveLineInfo.rightLineInfo
    leftValue = self.driveLineInfo.leftLineInfo

    rospy.loginfo( rightValue )
    rospy.loginfo(leftValue)

    cv2.imshow("Robot View", image)
    cv2.imshow("Left Line View", leftLine)
    cv2.imshow("Right Line View", rightLine)

    cv2.waitKey(3)

  def get_leftLine(self, h, w, image, LineMask):
    tempMask = numpy.copy(LineMask)
    tempMask[:, 0:w / 2] = 0
    # start RightLineDetect
    for i in range(0, h):
      area = 0
      for j in range(w / 2, w, 1):
        if tempMask[i, j - 1] != tempMask[i, j]:  # 선과 선사이 감지
          area = area + 1
        if (area > 1):
          tempMask[i, j] = 0
    # end LineDetect
    # start RightLineFollow
    M = cv2.moments(tempMask)
    if M['m00'] > 0:
      # start 무게중심구하기
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      # end 무게중심구하기
      cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
      # BEGIN CONTROL
      # 왼쪽 차선의 기준선이 화면의 1/2 지점에 위치한다고 했을 때 그와의 오차
      err = cx - w / 4 * 3
      self.driveLineInfo.rightLineInfo = err
      # self.rightLineInfo_pub.publish(self.rightLineInfo)
      ''''#self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)'''
      # end RightLineFollow
      return tempMask

  def get_rightLine(self, h, w, image, LineMask):
    tempMask = numpy.copy(LineMask)
    # start LeftLineDetect
    tempMask[:, w / 2:w] = 0

    for i in range(0, h):
      area = 0
      for j in range(w / 2, 0, -1):
        if tempMask[i, j] != tempMask[i, j + 1]:  # 선과 선사이 감지
          area = area + 1
        if (area > 1):
          tempMask[i, j] = 0
    # end LeftLineDetect

    # start LeftLineFollow
    M = cv2.moments(tempMask)
    if M['m00'] > 0:
      # start 무게중심구하기
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      # end 무게중심구하기
      cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
      # BEGIN CONTROL

      # 왼쪽 차선의 기준선이 화면의 1/2지점에 위치한다고 했을 때 그와의 오차
      err = cx - w / 4
      self.driveLineInfo.leftLineInfo = err
      # self.leftLineInfo_pub.publish(self.leftLineInfo)
      ''''#self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)'''
      # end LeftLineFollow
      return tempMask

  def RightmomentumPublisher(self, maskedImg, image):
    h, w, d = image.shape

    #set detect area
    search_top = 3 * h / 4
    search_bot = 3 * h / 4 + 20
    maskedImg[search_top : search_bot][0 : w/2] = 0

    M = cv2.moments(maskedImg)
    if M['m00'] > 0:
      # start 무게중심구하기
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      # end 무게중심구하기
      cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
      # BEGIN CONTROL
      # 왼쪽 차선의 기준선이 화면의 1/2 지점에 위치한다고 했을 때 그와의 오차
      err = cx - w / 4 * 3
      self.driveLineInfo.rightLineInfo = err

  def LeftmomentumPublisher(self, maskedImg, image):
    h, w, d = image.shape

    # set detect area
    search_top = 3 * h / 4
    search_bot = 3 * h / 4 + 20
    maskedImg[search_top:search_bot][w / 2: w] = 0

    M = cv2.moments(maskedImg)
    if M['m00'] > 0:
      # start 무게중심구하기
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      # end 무게중심구하기
      cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
      # BEGIN CONTROL
      # 왼쪽 차선의 기준선이 화면의 1/2 지점에 위치한다고 했을 때 그와의 오차
      err = cx - w / 4 * 3
      self.driveLineInfo.leftLineInfo = err

'''        #차선만 인식 -> 노이즈 값 줄임
        if whiteMask[i, j] != whiteMask[i, j + 1]:  # 선과 선사이 감지
          area = area + 1
        if (area > 1):
          whiteMask[i, j] = 0       
      #차선만 인식 -> 노이즈 값 줄임
        if whiteMask[i, j - 1] != whiteMask[i, j]: 
         area = area + 1
        if (area > 1):
          whiteMask[i, j] = 0     '''


  # END setDetectArea

  #start detect rightLine


'''
  def merge_line(self, leftLine, righLine):

    height = leftLine.shape[0]   #image height
    width = leftLine.shape[1]   #image width
    # declare new parameter to store MergedLineImage
    mergedLine = numpy.zeros(height, width)

    for i in range(0, height):
      for j in range(0, width):
        if(leftLine[i][j] != 0)
          mergedLine[i][(int)]
'''

rospy.init_node('DriveLinePublisher')
driveLinePublisher = DriveLinePublisher()
rospy.spin()
# END ALL


'''
    for i in range (0,h):
      for j in range (0,w):
       self.lineMask[i][j] = whiteMask[i][j];

'''
