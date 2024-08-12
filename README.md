# 제 8회 GIST 창의융합경진대회 - 탁구로봇 콘테스트 
 > 이 프로젝트는 카메라를 사용해 공을 추적하고, 추적된 공의 위치에 따라 다이나믹셀(Dynamixel) 모터를 제어하는 시스템입니다. 이 시스템은 공의 궤적을 실시간으로 분석하고, 공을 발사하는 기계적 동작을 수행합니다.

## 🙌 Members

>*제 8회 GIST 창의융합경진대회,* **럭키엣지**☀😆입니다.<br>

<br>

| 유정현 | 이태웅 | 황보겸 |
| :-: | :-: | :-: |
| <img src='https://github.com/jhyoo7996.png' height=130 width=130></img> | <img src='https://github.com/taewoong1.png' height=130 width=130></img> | <img src='https://github.com/hbgyeom.png' height=130 width=130></img>
| <a href="https://github.com/jhyoo7996" target="_blank"><img src="https://img.shields.io/badge/GitHub-black.svg?&style=round&logo=github"/></a> | <a href="https://github.com/taewoong1" target="_blank"><img src="https://img.shields.io/badge/GitHub-black.svg?&style=round&logo=github"/></a> | <a href="https://github.com/hbgyeom" target="_blank"><img src="https://img.shields.io/badge/GitHub-black.svg?&style=round&logo=github"/></a> | 

<br>

## 1. Overview

![image](https://github.com/user-attachments/assets/b27a98cd-681b-454e-ae11-3ae0179b1f77)
<br><br>
>기존의 탁구로봇은 단순히 로봇 팔을 사용해 공을 위로 올려주거나 부드럽게 전달하는 방식을 취합니다. 이 방식은 사람과의 릴레이에 부적합하며, 모터를 많이 사용해야해서 축이 많아짐에 따라 제어에 어려움이 있다고 판단하였습니다.

![image](https://github.com/user-attachments/assets/dd6b966e-aca8-41a6-b32b-53e7fb499562)
<br><br>

>저희가 만든 로봇은 공을 정교한 타이밍에 정확하게 밀어쳐서 목표 지점으로 강력하게 보내며 실제 탁구 선수의 강력한 스매싱처럼 공을 날카롭게 날려 보냅니다. 제어해야 하는 모터는 1개로 기존 로봇의 단점을 해결하였습니다. 

## 2. Data Preprocessing

![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/816fcce9-e763-4a86-bcf0-54d72548b236)
<br><br>

>여행객의 정보가 담겨 있는 여행객master.csv의 **여행 스타일 7가지 특성** (`자연`vs`도시`, `숙박`vs`당일`, `새로운지역`vs`익숙한지역`, `휴양/휴식`vs`체험활동`, `잘 알려지지 않은 방문지`vs`알려진 방문지`, `계획에 따른 여행`vs`상황에 따른 여행`, `사진 촬영 중요하지 않음`vs`사진 촬영 중요`) 을 추출했습니다. 이때 여행객의 숙소는 이미 정해졌기 때문에 `편하지만 비싼 숙소`vs`불편하지만 저렴한 숙소` 는 추출하지 않았습니다. 이로써 **8개의 특성**(여행객ID, 여행스타일7개)과 3200개의 샘플로 전처리 하였고 이를 traveler_master.csv파일에 저장하였습니다.

![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/62edd7b6-d77c-4721-be15-f2bc5e675c5d)
<br><br>

>방문지 정보가 담겨있는 방문지 정보.csv의 62273개의 샘플 중 **위도, 경도가 없는 일부 데이터들의 값들을 추가하였습니다**. 또한 ‘올레길’, ‘올레길 주차장’처럼 같은 장소를 다른 표기로 기재된 장소를 **한 장소로 통일**했습니다. 방문지를 관광지 그리고 식당/카페로 분류하기 위해 **호텔, 집, 사적 장소들을 데이터에서 배제**하였습니다. 관광지와 식당/카페를 구분하여 식당이면 RES, 관광지면 TOU로 새로운 특성을 추가하고 **프로젝트에 활용되지 않는 특성을 제거하여 데이터를 간소화**했습니다. 이로써 **11개의 특성**(`여행객ID`, `여행ID`, `방문 순서`, `장소`, `여행 유형`, `경도`, `위도`, `만족도`, `재방문의향`, `추천의향`)과 39059개의 샘플로 전처리하였고 이를 visit_info.csv 파일에 저장하였습니다.

![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/d9e64cd8-e9bc-46a0-8153-066c94f55287)
<br><br>

>여행 정보가 담겨있는 여행.csv의 데이터 중 여행객 ID와 **주요 이동 수단 특성**을 추출했습니다. 이를 travel.csv파일에 저장하였습니다. visit_data에 travel.csv와 visit_info.csv 파일을 travel ID 기준으로 합치고 visit_final에 traveler_master.csv를 travel ID 기준으로 합쳤습니다.

## 3. Modeling

![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/b83f3754-cca8-421d-b734-c22944f19983)
<br><br>

>제주도 여행객 숙소의 위도, 경도 데이터를 활용하여 군집을 분류했습니다. 이때 최적의 클러스터 개수(k) 값을 찾기 위해 k를 **제주도 읍면리의 총 개수인 43개**까지 점차 증가시키면서 클러스터링을 수행하고, 이에 따른 SSE(Sum of Squared Errors) 값을 계산하여 그래프로 나타냈습니다.
  
![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/a1e21d9a-926b-431e-b15a-873e25995a73)
<br><br>

> k=5를 최적의 클러스터 개수로 선정하였습니다. 데이터의 위도, 경도 상 센트로이드(centroid)는 아래와 같이 나왔습니다.
각 데이터를 군집화하였을 때 위와 같이 숙소의 위치가 분류됨을 확인하였습니다.

![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/f74b1012-3624-4070-be18-1325d7117e92)
<br><br>

>사용자의 `여행 스타일` 데이터를 입력 받습니다.

![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/f8e145b8-5714-4ada-b576-80c4f6751887)
<br><br>


![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/3c5d1f03-2cd5-4dfe-8bc8-e5e0626f0469)
<br><br>

>`자동차의 유무`로 두집단을 나눕니다.

![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/b0f0f8e2-61a3-4e18-b6d9-c0d6c7c80d99)
<br><br>

>`MSE 평균 제곱 오차` 방법을 이용하여 숙소의 군집에 포함된 범위에서 **여행 스타일이 유사한 여행객 상위 5명**을 선정합니다. 이때 유사도 측정을 위해 `MSE 평균 제곱 오차` 방법을 선정한 이유는 선호도가 나타내는 특징 공간 내에서 벡터의 방향의 유사도 보다는 **벡터의 길이의 유사도**를 따져야 하기 때문입니다.

![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/e2184cc6-e2c5-4396-9be4-56ceabde1a1d)
<br><br>

>여행지 만족도는 1부터 5까지의 점수로 구성된 방문지 정보의 특성인 `만족도`, `재방문 의향` 그리고 `추천 의향` 점수의 합으로 정의했습니다.

![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/1d24980c-df22-49ae-b025-403151dc32dc)
<br><br>

>선호도가 유사한 5명의 여행객의 방문지 정보를 추출한 후 `관광지`와 `식당/카페`로 분류합니다. 그 다음 관광지 그룹을 만족도가 높은 순으로 정렬한 후 중복 위치를 제외한 `4가지 여행지`를 추출합니다.

![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/aa671968-a908-42a0-b107-30246e343149)
<br><br>

>선정한 장소 네 곳을 지도에 표시하여 숙소에서 각 장소와의 거리 계산 후, 숙소와 가장 가까운 장소 A를 선정합니다. A에서 남은 장소와 거리 계산 후 A와 가장 가까운 장소 B를 선정합니다. B에서 남은 장소와 거리 계산 후, B와 가장 가까운 장소 C를 선정한다. 남은 D는 마지막 장소로 관광지만으로 구성된 `A-B-C-D 경로`를 완성합니다.

![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/9968661a-e919-4caa-a148-d32235401e2c)
<br><br>

 ## 4. 개선점 및 차별점 
 
 ![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/5303d897-678a-4326-85ee-7ec79efdc4f5)
<br><br>

![image](https://github.com/khuda-5th/ML_team2_Recommend-Travel-Route/assets/83753041/63fa8516-6f2e-4b82-84b8-4df64f397cbc)
<br><br>
## 📈 Data
- [AI 허브-국내 여행로그 데이터(제주도 및 도서지역)](https://aihub.or.kr/aihubdata/data/view.do?currMenu=&topMenu=&aihubDataSe=realm&dataSetSn=71584)
