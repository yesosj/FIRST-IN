https://www.mdpi.com/2079-9292/14/24/4822

논문 요약
1. 연구 목적

이 논문의 목적은 ROS 2 환경에서 가장 많이 사용하는 두 가지 2D SLAM 알고리즘인

SLAM Toolbox
Google Cartographer

를 동일한 조건에서 비교하여

어느 알고리즘이 더 정확한지
어느 알고리즘이 더 빠른지
실제 로봇에서는 어떤 차이가 있는지

를 분석하는 것입니다.

2. 사용한 하드웨어

논문에서 사용한 센서는 다음과 같습니다.

360° LiDAR
IMU
바퀴 엔코더

그리고

IMU + 엔코더 데이터를

EKF(Extended Kalman Filter) 로 융합하여 하나의 안정적인 오드메트리를 생성합니다.

그 오드메트리를 SLAM 알고리즘에 입력합니다.

즉 구조는

Encoder
      \
       \
        EKF
       /
IMU
       ↓
   Odometry(/odom)
       ↓
SLAM Toolbox 또는 Cartographer
       ↓
지도 생성(Map)
3. 두 알고리즘의 차이
(1) SLAM Toolbox

특징

Pose Graph Optimization 사용
외부에서 제공하는 오드메트리에 크게 의존
Loop Closure 성능 우수
지도 품질이 좋음

즉,

"오드메트리가 정확하면 최고의 결과"

를 보여줍니다.

(2) Cartographer

특징

Google 개발
Submap 기반
Scan Matching
IMU를 내부적으로 적극 활용
자체 위치 추정 가능

즉,

오드메트리가 없어도 어느 정도 동작합니다.

4. EKF를 사용하는 이유

논문에서는

IMU만 사용하면

드리프트 발생

엔코더만 사용하면

바퀴 미끄러짐 발생

한다고 설명합니다.

그래서

EKF가

엔코더
IMU

를 융합하여

더 안정적인 오드메트리를 생성합니다.

5. TF 구조

논문에서 가장 강조하는 부분 중 하나입니다.

ROS에서는

Map
 ↓
Odom
 ↓
Base_footprint
 ↓
Base_link
 ↓
Laser

TF가 정확해야

SLAM이 정상적으로 수행됩니다.

TF가 잘못되면

지도가 심하게 찌그러집니다.

6. Gazebo 시뮬레이션 결과

동일한 Gazebo 환경에서 실험했습니다.

결과

SLAM Toolbox
더 부드러운 지도
더 정확한 경로
ATE(Absolute Trajectory Error) 낮음
Cartographer
시뮬레이션에서는 성능이 다소 떨어짐

이유는

Gazebo는 센서 노이즈가 거의 없기 때문입니다.

Cartographer는 실제 센서 노이즈가 있을 때 더 잘 동작하도록 설계되어 있습니다.

7. 실제 환경 결과

실제 실내에서 실험한 결과

SLAM Toolbox

장점

지도 품질 우수
Loop Closure 정확
위치 오차 적음

단점

CPU 사용량 증가
속도 느림
Cartographer

장점

빠름
CPU 사용량 적음
실시간 처리 우수

단점

파라미터 튜닝이 어려움
설정이 잘못되면 지도가 쉽게 깨짐
8. 오드메트리 중요성

논문에서 가장 흥미로운 실험입니다.

연구진은

일부러

엔코더 데이터를 틀리게 만들어

바퀴가 미끄러지는 상황을 만들었습니다.

결과

SLAM Toolbox가 만든 지도가 크게 왜곡되었습니다.

하지만

EKF 기반의 정상 오드메트리로 복원하자

바로 정상 지도가 생성되었습니다.

즉

좋은 SLAM보다 좋은 오드메트리가 먼저라는 사실을 보여줍니다.

9. 성능 비교
항목	SLAM Toolbox	Cartographer
지도 품질	★★★★★	★★★★☆
속도	★★★☆☆	★★★★★
CPU 사용	높음	낮음
메모리	많이 사용	적게 사용
오드메트리 의존성	매우 높음	낮음
Loop Closure	우수	우수
실시간성	좋음	매우 좋음

10. 논문의 결론

논문의 결론은 다음과 같습니다.

SLAM Toolbox가 적합한 경우
정확한 지도가 필요
IMU + 엔코더(EKF)가 있는 로봇
실내 자율주행
지도 품질을 가장 중요하게 생각하는 경우
Cartographer가 적합한 경우
빠른 맵 생성
CPU 자원이 부족한 임베디드 시스템
실시간성이 중요한 경우
사용자 프로젝트에 적용한다면

현재 구성은

Jetson Nano
YDLIDAR
MPU6050(IMU)
바퀴 엔코더
ROS 2 Humble

입니다.

이 논문의 결과를 그대로 적용하면 가장 추천되는 구조는

엔코더
      \
       \
        EKF(robot_localization)
       /
IMU
       ↓
Odometry (/odom)
       ↓
SLAM Toolbox
       ↓
Map 생성

입니다.

논문에서도 EKF로 IMU와 엔코더를 융합한 오드메트리를 사용한 뒤 SLAM Toolbox를 적용했을 때 가장 안정적이고 정확한 실내 지도 생성 결과를 얻었다고 보고하고 있습니다.