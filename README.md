# 제 8회 GIST 창의융합경진대회 - 탁구로봇 콘테스트 
 > 이 프로젝트는 카메라를 사용해 공을 추적하고, 추적된 공의 위치에 따라 다이나믹셀(Dynamixel) 모터를 제어하는 시스템입니다. 이 시스템은 공의 궤적을 실시간으로 분석하고, 공을 발사하는 기계적 동작을 수행합니다.

## 🙌 Members

>*제 8회 GIST 창의융합경진대회,* **럭키엣지 팀**☀😆입니다.<br>

<br>

| 유정현 | 이태웅 | 황보겸 |
| :-: | :-: | :-: |
| <img src='https://github.com/jhyoo7996.png' height=130 width=130></img> | <img src='https://github.com/taewoong1.png' height=130 width=130></img> | <img src='https://github.com/hbgyeom.png' height=130 width=130></img>
| <a href="https://github.com/jhyoo7996" target="_blank"><img src="https://img.shields.io/badge/GitHub-black.svg?&style=round&logo=github"/></a> | <a href="https://github.com/taewoong1" target="_blank"><img src="https://img.shields.io/badge/GitHub-black.svg?&style=round&logo=github"/></a> | <a href="https://github.com/hbgyeom" target="_blank"><img src="https://img.shields.io/badge/GitHub-black.svg?&style=round&logo=github"/></a> | 

<br>

## 1. Overview

<img src="https://github.com/user-attachments/assets/b27a98cd-681b-454e-ae11-3ae0179b1f77" alt="기존 탁구로봇" width="1000"/>
<br><br>
기존의 탁구로봇은 단순히 로봇 팔을 사용해 공을 위로 올려주거나 부드럽게 전달하는 방식을 취합니다. 이 방식은 사람과의 릴레이에 부적합하며, 모터를 많이 사용해야해서 축이 많아짐에 따라 제어에 어려움이 있다고 판단하였습니다.

<img src="https://github.com/user-attachments/assets/dd6b966e-aca8-41a6-b32b-53e7fb499562" alt="밀어치는 로봇" width="1000"/>
<br><br>
저희가 만든 탁구로봇 Chorle robot은 공을 정교한 타이밍에 정확하게 밀어쳐서 목표 지점으로 강력하게 보내며 실제 탁구 선수의 강력한 스매싱처럼 공을 날카롭게 날려 보냅니다. **제어해야 하는 모터는 1개**로 기존 로봇의 단점을 해결하였습니다. 

## 2. Libraries and Header files

**OpenCV (<opencv2/opencv.hpp>)**: 오픈소스 컴퓨터 비전 라이브러리로, 이미지 및 동영상 처리에 사용됩니다. 이 프로젝트에서는 카메라로 촬영한 비디오 프레임에서 공을 추적하고, 그 위치를 계산합니다.
<br><br>
**Chrono (<chrono>)**: 시간을 측정하고 처리하기 위한 표준 C++ 라이브러리입니다. 이 프로젝트에서는 공이 인식된 시점과 발사 시점 사이의 시간을 계산하는 데 사용됩니다.
<br><br>
**Vector (<vector>)**: 동적 배열을 제공하는 표준 C++ 라이브러리입니다. 필요에 따라 크기가 변경되는 배열을 사용할 수 있습니다.
<br><br>
**Dynamixel SDK ("dynamixel_sdk.h")**: 다이나믹셀 모터를 제어하는 SDK입니다. 모터와의 통신, 제어 명령 송수신 등에 사용됩니다.
<br><br>
**CLinear_actu.h**: 선형 액추에이터를 제어하는 클래스가 정의된 헤더 파일입니다. 액추에이터의 초기화, 위치 제어, 리셋 기능을 제공합니다.

## 3. Modeling

>이 프로젝트는 **3개의 Thread**를 사용하여 병렬로 작업을 처리합니다.

### thread1Function
공을 추적하고, 그에 따라 액추에이터를 제어하는 역할을 합니다.

**카메라 초기화 및 설정**

이 스레드는 두 개의 카메라(위쪽과 옆쪽)를 초기화하고 설정합니다. 카메라에서 프레임을 지속적으로 가져와서, 이미지 프로세싱을 통해 공의 위치를 추적합니다.

**공의 위치 추적**

공의 위치를 추적하기 위해 OpenCV 라이브러리를 사용하여 특정 색상 범위를 필터링하고, 필터링된 영역의 바운딩 박스를 계산합니다. 바운딩 박스의 중심 좌표를 계산하여 공의 현재 위치를 확인합니다.

**선형 액추에이터 제어**

공의 위치에 따라 선형 액추에이터를 적절한 위치로 이동시킵니다. 공이 특정 조건을 만족할 때까지 이 과정을 반복합니다.
액추에이터가 이동할 위치를 계산한 후, move_actu 함수를 호출하여 액추에이터를 해당 위치로 이동시킵니다.

**스레드 간 상호작용**

이 스레드는 fire 변수와 shared_position 변수를 사용하여 스레드 2와 정보를 공유합니다. 예를 들어, 공이 발사 준비가 되었을 때 fire 변수를 설정하여 스레드 2에게 신호를 보냅니다.

 ## 4. 개선점 및 차별점 
 
 ![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/5303d897-678a-4326-85ee-7ec79efdc4f5)
<br><br>

![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/63fa8516-6f2e-4b82-84b8-4df64f397cbc)
<br><br>
## 📈 Data
- [AI 허브-국내 여행로그 데이터(제주도 및 도서지역)](https://aihub.or.kr/aihubdata/data/view.do?currMenu=&topMenu=&aihubDataSe=realm&dataSetSn=71584)
