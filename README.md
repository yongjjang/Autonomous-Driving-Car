# Autonomous-Driving-Car
Object-Oriented Design Project

## 개요
- 주어진 시뮬레이션 환경에서 자율 주행 가능한 로봇을 구현

## 요구 사항
- 차선, 정지선, 정지 표지판, 차단바, 장애물을 인식하여 걸맞는 동작 수행
- T자, S자, 굴절 코스 극복 기능을 구현해야 함.

## 기능 목록
- 카메라를 이용한 영상 입출력 기능
- 입력된 영상을 처리 가능한 형태으로 변환
- 양 쪽 차선 인식을 통한 주행
- 사물 인식시 해당 사물에 대응하는 상태로 전이 후 동작 수행 (SMACH 사용)

## 시연 영상
### 차단바 인식
[![차단바 인식](http://img.youtube.com/vi/uLR1RNqJ1Mw/0.jpg)](https://youtu.be/ISJEHpfYz_E)

### 주행 기능
[![주행 기능](http://img.youtube.com/vi/uLR1RNqJ1Mw/0.jpg)](https://youtu.be/cXGEKiqsE1o)

### ROI 설정 후 영상 On Rviz
[![ROI 설정 후 영상 On Rviz](http://img.youtube.com/vi/uLR1RNqJ1Mw/0.jpg)](https://youtu.be/X6s7BTfWpKc)
