# ROS Proje

Bu proje, TurtleBot3 robotunun Gazebo ortamında bir harita üzerinde navigasyon yapabilmesi için gerekli adımları içermektedir. Aşağıdaki adımları izleyerek projenizi başlatabilirsiniz.

## Adım 1: ROS'un Başlatılması
Öncelikle, ROS'un temel bileşenlerini başlatmak için aşağıdaki komutu terminalde çalıştırın:
```bash
roscore
```
## Adım 2: Gazebo Ortamında Haritanın Çalıştırılması
Hazırlanmış olan turtlebot3_ev isimli haritanın çalıştırılması için şu komutu kullanın:
```bash
roslaunch turtlebot3_gazebo turtlebot3_ev.launch
```
## Adım 3: Harita Sunucusunun Başlatılması
Harita sunucusunu başlatmak için aşağıdaki komutu terminalde çalıştırın:
```bash
rosrun map_server map_server ~/my_map.yaml
```
## Adım 4: TurtleBot3 İçin Navigasyon ve RViz'in Başlatılması
TurtleBot3 robotunun navigasyon özelliklerini ve RViz programını başlatmak için aşağıdaki komutu kullanın:
```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/my_map.yaml
```
## Adım 5: Hızlandırma
Yukarıdaki dört adımı tek bir terminal satırında çalıştırmak için gorev1.launch dosyasını kullanabilirsiniz. Başlatan kod şu şekildedir:
```bash
roslaunch turtlebot3_gazebo gorev1.launch
```
## Adım 6: Önceden Belirlenmiş Görev Noktalarına Gitme
Robotun, önceden belirlenmiş görev noktalarına ulaşması için hazırlanan kodu çalıştırmak için şu komutu kullanın:
```bash
rosrun turtlebot3_gazebo turtlebot3_gorevler.py
```
