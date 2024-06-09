# AMR logistics automation system
<div align="center">

<img src="https://github.com/addinedu-ros-4th/ros-repo-2/assets/118419026/dd1da530-6c66-4815-94ef-1b23f385e5c4" width="700" height="300">


<div align="left">

## 목차
  * [1. 🤖프로젝트 소개](#1-프로젝트-소개)
    + [1-1. 개발 일정](#1-1-개발-일정)
    + [1-2. 기술 스택](#1-2-기술-스택)
    + [1-3. 팀원 역할](#1-3-팀원-역할)
  * [2. 📋시스템 구성](#2-시스템-구성)
    + [2-1. 시스템 구성도](#2-1-시스템-구성도)
      
## 1. 🤖프로젝트 소개
- 입고, 출고, 수거와 같은 물류 공정 과정에 지게차 프레임 기반의 자율주행 로봇을 활용
- 다중 로봇 제어 시스템을 도입하여 물류 작업의 효율성 향상
- 사람과 로봇의 상호작용(HRI)을 통해 물류 센터 운영의 최적화
- 입고 상품 등록, 진열장 재고 관리 및 소비자 주문 시스템 구현

### 1-1. 개발 일정
- **2024.04.17 ~ 2024.06.13**
![________2024-06-09_09 05pm](https://github.com/addinedu-ros-4th/ros-repo-2/assets/118419026/b24930a5-12d7-4eee-beaf-ea1b8dbec14d)

### 1-2. 기술 스택
||||
|:---:|:---|:---|
|개발환경|<img src="https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=Ubuntu&logoColor=white"> <img src="https://img.shields.io/badge/VISUAL STUDIO CODE-007ACC?style=for-the-badge&logo=VisualStudioCode&logoColor=white">|
|기술|<img src="https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54"> <img src="https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=cplusplus&logoColor=white"> <img src="https://img.shields.io/badge/ros2-%2322314E?style=for-the-badge&logo=ros&logoColor=white"> <img src="https://img.shields.io/badge/numpy-%23013243.svg?style=for-the-badge&logo=numpy&logoColor=white"> <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white"> <img src="https://img.shields.io/badge/PyQt5-%23217346.svg?style=for-the-badge&logo=Qt&logoColor=white"> <img src="https://img.shields.io/badge/mysql-4479A1.svg?style=for-the-badge&logo=mysql&logoColor=white"> |
|하드웨어|<img src="https://img.shields.io/badge/-RaspberryPi 4-C51A4A?style=for-the-badge&logo=Raspberry-Pi"> <img src="https://img.shields.io/badge/-Arduino Mega-00979D?style=for-the-badge&logo=Arduino&logoColor=white">
|COMMUNICATION|<img src="https://img.shields.io/badge/confluence-%23172BF4.svg?style=for-the-badge&logo=confluence&logoColor=white"> <img src="https://img.shields.io/badge/jira-%230A0FFF.svg?style=for-the-badge&logo=jira&logoColor=white"> <img src="https://img.shields.io/badge/Slack-4A154B?style=for-the-badge&logo=Slack&logoColor=white">  <img src="https://img.shields.io/badge/github-181717?style=for-the-badge&logo=github&logoColor=white">|

### 1-3. 팀원 역할
<table>
  <thead>
    <tr>
      <th style="text-align:center;">구분</th>
      <th style="text-align:center;">팀원</th>
      <th style="text-align:center;">역할</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="text-align:center;">팀장</td>
      <td style="text-align:center;">송용탁</td>
      <td>- 이슈 및 일정 관리 <br> - 로봇 형상 및 기구 설계<br> - 세부 기능 개선 및 코드 통합 <br> - 주행 환경 구축</td>
    </tr>
    <tr>
      <td style="text-align:center;">팀원</td>
      <td style="text-align:center;">김동규</td>
      <td>- ArUco Navigation <br> - 딥러닝 기반 Human Following Robot <br> - Confluence 관리 <br> - 주행 환경 구축</td>
    </tr>
    <tr>
      <td style="text-align:center;">팀원</td>
      <td style="text-align:center;">이재혁</td>
      <td>- SLAM <br> - 다중 로봇 제어 시스템 구현 <br> - 경로 탐색 알고리즘 설계 <br> - 세부 기능 개선 및 코드 통합</td>
    </tr>
    <tr>
      <td style="text-align:center;">팀원</td>
      <td style="text-align:center;">최가은</td>
      <td>- 로봇 통신 서버 구축 <br> - 통신 인터페이스 및 프로토콜 설계 <br> - 데이터베이스 구조 설계 <br> - Github 및 Jira 관리</td>
    </tr>
    <tr>
      <td style="text-align:center;">팀원</td>
      <td style="text-align:center;">유겸희</td>
      <td>- SLAM <br> - 관리자 GUI 설계 <br> - 데이터베이스 구축 <br> - 주행 환경 구축</td>
    </tr>
    <tr>
      <td style="text-align:center;">팀원</td>
      <td style="text-align:center;">장하린</td>
      <td>- SLAM <br> - 소비자 GUI 설계 <br> - 데이터베이스 구축</td>
    </tr>
  </tbody>
</table>


## 2. 📋시스템 구성

### 2-1. 시스템 구성도
<img src= "https://github.com/addinedu-ros-4th/ros-repo-2/assets/118419026/c8ed03cf-49f4-4f77-8416-f3204606b4d6">


