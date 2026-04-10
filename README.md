# Решение Хакатон - соренований по сборке и программированию ROS2/open-source роботов от команды just_misis | ROS Meetup '26

<div align="center">
    <img src="https://github.com/user-attachments/assets/035d7a99-ded4-42b8-a010-f8725f115cb8" alt="rounded-in-photoretrica">
</div>

В данном репозитории представлены все файлы решения команды just_misis для Хакатон - соренований по сборке и программированию ROS2/open-source роботов в рамках мероприятия [ROS Meetup '26](https://rosmeetup.ru/)!

<div align="center">
    <img src="https://github.com/user-attachments/assets/0a232da6-bde4-4101-8d46-427e075f8269" alt="rounded-in-photoretrica">
</div>

# Описание задания хакатона:
Хакатон представлял собой соревнование, целью которого была разработка программного модуля автономной навигации и манипуляции для мобильного робота на базе ROS2 в условиях, приближенных к реальной задаче автоматизации складских помещений. Основная миссия для участников была разбита на две ключевые задачи, которые необходимо было выполнить в едином игровом заезде:


<div align="center">
    <img src="https://github.com/user-attachments/assets/82a4a50a-bc36-40ef-8ac8-a4e6ad097fd9" 
         alt="rounded-in-photoretrica"
         style="width: 70%; max-width: 400px; height: auto;">
</div>

Автономная навигация и локализация: На игровом поле размером 1750×1250 мм, представляющем собой ArUco-доску , одновременно действуют два робота. Конфигурация стартовых позиций и расположение 16 игровых объектов (по 8 на каждой половине) заранее неизвестны участникам и определяются судьей случайным образом. Задача робота — используя бортовые сенсоры (камера, лидар) и внешний модуль камеры на П-образной мачте высотой 1500 мм, автономно перемещаться по полю, локализуясь относительно разметки.

<div align="center" style="text-align: center;">
<div style="display: flex; gap: 20px; justify-content: center; width: 100%;">
    <img src="https://github.com/user-attachments/assets/14354e3f-47b0-48cf-b7ea-0fe8addc32a4" 
         alt="rounded-in-photoretrica" 
         style="width: 400px; height: auto;">
    <img src="https://github.com/user-attachments/assets/2561cc66-e388-498a-95b4-dc725fc8aa20" 
         alt="rounded-in-photoretrica" 
         style="width: 323px; height: auto;">
</div>
<div align="left" style="text-align: left;">

Бортовой ИИ и стратегическое распределение объектов: Во время движения по полю робот должен был в режиме реального времени, с помощью ArUco-маркеров и компьютерного зрения, обнаруживать и классифицировать физические объекты: белые кубы с маркерами (ID 20, 21), красные и синие кубы, красный цилиндр, голубого пингвина, красного осьминога, зеленого кролика. Задача робота — не только найти их, но и доставить в зачетные зоны (поле без корзины, корзину или угловое хранилище с бортиками), размещая объекты друг на друга для получения максимальных бонусов. Например, размещение кролика на кубе с маркером ID 20 приносило 16 баллов вместо базовых 4.

<div align="center" style="text-align: center;">

# Робот команды just_misis - [Роботизированная платформа Frob](https://github.com/dark516/Frob_robot)

<div align="center">
    <img src="https://github.com/user-attachments/assets/4e6b15c5-6662-47f5-8419-bada88828910" 
         alt="rounded-in-photoretrica"
         style="width: 70%; max-width: 400px; height: auto;">
</div>

**Frob** - это недорогой мобильный робот с открытым исходным кодом, созданный для популяризации робототехники и обучения [ROS](https://www.ros.org/). Он предназначен для энтузиастов, студентов и преподавателей, изучающих мир робототехники. Независимо от того, являетесь ли вы начинающим или опытным разработчиком, Frob предоставляет универсальную платформу для решения широкого спектра задач обучения.

[![Wiki](https://img.shields.io/badge/Wiki-Documentation-blue?style=flat-square&logo=github)](https://github.com/dark516/Frob_robot/wiki)
[![Telegram](https://img.shields.io/badge/Telegram-Community-blue?style=flat-square&logo=telegram)](https://t.me/FrobCommunity)
[![License](https://img.shields.io/github/license/dark516/Frob_robot?style=flat-square)](https://github.com/dark516/Frob_robot/blob/main/LICENSE)
[![Issues](https://img.shields.io/github/issues/dark516/Frob_robot?style=flat-square)](https://github.com/dark516/Frob_robot/issues)

[Документация проекта](https://github.com/dark516/Frob_robot/blob/main/MANUAL.md)

[English documentation](https://github.com/dark516/Frob_robot/blob/main/MANUAL.eng.md)

<div align="left" style="text-align: left;">

# Добработка Роботизированной платформы Frob к хакатону

<div align="center">
    <img src="https://github.com/user-attachments/assets/6b8ad8ad-73f9-428c-aceb-7690963b5fc8" 
         alt="rounded-in-photoretrica"
         style="width: 70%; max-width: 400px; height: auto;">
</div>
<div align="center" style="text-align: center;">
<div style="display: flex; gap: 20px; justify-content: center; width: 100%;">
    <img src="https://github.com/user-attachments/assets/4e7c5d74-d18b-485f-8203-c4f3c05cc5ea" 
         alt="rounded-in-photoretrica" 
         style="width: 400px; height: auto;">
    <img src="https://github.com/user-attachments/assets/02e5ffde-c717-467d-b0b5-ae9ad56a1dfb" 
         alt="rounded-in-photoretrica" 
         style="width: 400px; height: auto;">
</div>
<div align="left" style="text-align: left;">


# Работа камеры

<div align="center">
    <img src="https://github.com/user-attachments/assets/97779bf8-f1a0-49d7-8fba-300eef96327c" 
         alt="rounded-in-photoretrica"
         style="width: 90%; max-width: 400px; height: auto;">
</div>


### Когда мы начали разбирать решение задачи на хакатоне, почти сразу наша команда пришла к нескольким выводам:

1. Задача сводиться к локализации робота в 2-мерной плоскости
2. Нет необходимости выражать координаты в натуральных величинах (сантиметрах/метрах)
3. Задача тривиально решается с помощью хорошо обученной нейронное сети

### Разберем первые 2 вывода подробнее:
Поскольку камера смотрит на робота сверху вниз, нам достаточно 2-х координат - x и y, чтобы понять расположение робота на поле. В то же время, если мы получаем расположение робота в координатах относительно угла изображения с камеры, но и координаты объектов мы получаем в той же системе координат. Следовательно, все взаимодействие объектов может происходит в локальной системе координат, без привязки к СИ.

### Теперь же про непосредственно решение:
Для нахождения координат робота мы расположили на его крыше ArUco маркер - маркеры такого типа очень просто можно найти с помощью встроенных инструментов библиотеки opencv, что позволит нам получать устойчивые и правдивые координаты робота (поскольку робот двигает не очень быстро, а камера поддерживает 60+ кадров в секунду, “смазывание” маркера между кадрами не было проблемой). Сопоставив центр робота и центр маркера, по видео с верхней камеры с помощью простого скрипта мы получали координаты и направление, куда наш робот “смотрит” (yaw), считывая ArUco маркер. После этого, эта информация отсылалась в ros-топик как сообщения типа pose2D.
Для обнаружения объектов мы почти сразу решили использовать предобученную нейронную сеть YOLO - отличный инструмент для задач подобного плана. Из-за нехватки коммуникации с организаторами, конвертные объекты для обнаружения не были известны заранее, поэтому датасет для дообучения нейронной сети пришлось собирать прямо на месте. Мы обучили YOLO на кастомных данных находить и идентифицировать все объекты. Информация об их координатах в виде неупорядоченного списка отправлялась в другой ros-топик.


ROS2 MIPT hackathon on Frob robot
