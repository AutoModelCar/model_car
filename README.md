# model_car (AutoNOMOS Mini v2)

## Download and copy it to odroid

```
     $ git clone https://github.com/AutoModelCar/model_car.git
     $ cd model_car
     $ git checkout version-2-kinetic
     $ scp -r catkin_ws root@192.168.1.199:./
```

## Compile and run the code on odroid
```
     $ ssh root@192.168.1.199
     $ cd catkin_ws
     $ catkin build -j2
     $ roslaunch manual_control manual_odroid.launch
```
