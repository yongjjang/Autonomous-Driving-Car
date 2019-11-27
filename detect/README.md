cv2.threshold(img, threshold_value, value, flag)
thr = cv2.threshold(mask, 127, 255, 0)

영상 문턱값으로 값 변환하는 메소드이다.  Threshold는 영상에서 이진화 등을 할 때 주로 사용하는 방식이다.

* img = GrayScale 또는 inRange(영상값 범위) 이미지
* thresold_value = 픽셀 문턱값
* value = 픽셀 문턱값보다 클 때 적용되는 최댓값(예시에서는 127을 넘으면 255로 변환)
* flag = 문턱값 적용 방법 또는 스타일 => 결과값 할당
    * cv2.THRESH_BINARY = 0 : 픽셀 값이 threshold_value 보다 크면 value, 작으면 0
    * cv2.THRESH_BINARY_INV = 1 : 픽셀 값이 threshold_value 보다 크면 0, 작으면 value
    * cv2.THRESH_TRUNC = 2 : 픽셀 값이 threshold_value 보다 크면 threshold_value, 작으면 픽셀 값
    * cv2.THRESH_TOZERO = 3 : 픽셀 값이 threshold_value 보다 크면 픽셀 값, 작으면 0
    * cv2.THRESH_TOZERO_INV = 4 : 픽셀 값이 threshold_value 보다 크면 0, 작으면 픽셀 값


cv2.findContours(image, mode, method)
contours = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAINAPPROX_SAMPLE)

컨투어(contour)란 동일한 색 또는 동일한 픽셀 값을 가지고 있는 영역의 경계선 정보이다.
물체의 윤곽선, 외형을 파악하는데 사용된다.

findContours 함수는 흑백 이미지 또는 이진화된 이미지에만 사용할 수 있다.

* image = 흑백이미지 또는 이진화된 이미지
* mode = 컨투어를 찾는 방법
    * cv2.RETR_TREE : 모든 컨투어 라인을 찾고, 모든 상하구조를 연결함.
* method = 컨투어를 찾을 때 사용하는 근사화 메소드(이론)
    * cv2.CHAIN_APPROX_NONE : 모든 컨투어 포인트를 반환
    * cv2.CHAIN_APPROX_SIMPLE : 컨투어 라인을 그릴 수 있는 포인트만 반환
