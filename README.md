# ROS_LR_1
Лабораторная работа по "Практикум по робототехнике", 8 семестр

1. Запускаем ubuntu

2. Ставим QT Designer
```
sudo apt-get update
sudo apt-get install python3-pyqt5
sudo apt-get install qtcreator pyqt5-dev-tools
sudo apt-get install qttools5-dev-tools
```

3. Ставим пакет qimage2ndarray
```
pip install qimage2ndarray
```
ИЛИ
```
pip3 install qimage2ndarray
```

4. Переходим на рабочий стол и скачиваем это репозиторий 
```
cd ~/Desktop
git clone https://github.com/XxOinvizioNxX/ROS_LR_1
```

5. Редактируем bashrc
```
gedit ~/.bashrc
```

идём В САМЫЙ НИЗ, и внизу после fi нужно написать это:
```
cd ~/Desktop/ros_workspace/devel
source setup.bash
cd ~
clear
```

далее закомментить или удалить строчку `/opt/ros/noetic/setup.bash`

и в конце написать `export TURTLEBOT3_MODEL=burger`

6. Сохраняем файл (Ctrl + S или кнопкой сверху) и закрываем окно. Далее в терминале пишем `source ~/.bashrc`

7. В терминале переходим в папку ros_workspace.
```
cd ~/Desktop/ros_workspace
```

8. Пишем `catkin_make`

9. Если всё дошло без ошибок до 100%, открываем ещё 2 новые вкладки в терминале (сверху слева значок)

10. В первой вкладке пишем
```
roslaunch turtlebot3_gazebo turtlebot3_autorace.launch
```

и дожидаемся запуска gazebo

11. Во второй вкладке запускаем миисию для этого пишем
```
roslaunch turtlebot3_gazebo turtlebot3_autorace_mission.launch
```

12. Наконец, в третей вкладке запускаем лабу, для этого:
```
roslaunch burgerrace LR1_SEM4.launch
```

дожидаемся когда откроется гуишка

13. Если гуишка открылась успешно, в консоли во вкладке откуда запускали гуишку НЕ срётся никаких ошибок, то на форме переключаем обе камеры во второе положение (знаки и разметка) и выбираем атоматический режим. Робот должен поехать сам по разметке и начать выполнение миссии

14. Если всё работает, жмём и держим несколько раз Ctrl+c во всех вкладках, затем закрываем все открытые окна НЕ ЗАКРЫВАЯ КОНСОЛЬ, Т.е. газебо и гуишку. Нужно убедится что всё закрылось, если нет, в консоли снова жмём ctrl+c пока не закроется

15. Конспирируем гуишку и всё остальное (убираем обводку с прямоугольникв с детектора линий, меняем цвета лидара, текст на знаках и т.д.)
