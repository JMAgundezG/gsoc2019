# Second post

23th Julio, 2019



## Objectives

- [x] Create a robocomp controller to control a  EV3_LEGO

  



## First task: Finish a E-Puck controller



In the first days of the second delivery, there wasn't an EV3_LEGO to implement in a scene of VREP, so I started to use and complete the EPuck controller developed in the first delivery. The reason for changing some features in this controller is because it wasn't complete and it needs to be compatible with Robocomp. I implemented all the robot components and It works great and It could be used (with some changes) to control an EV3_LEGO model. 

## Second task: Create a EV3_LEGO controller



### EV3_LEGO

EV3_LEGO is a Lego robot which has educational purposes. It can be used in a lot of ways, like using an iPad with the Lego app to be accessible to everyone. The components that makes up the model are referred in the next python dictionary.

```python
__COMPONENTS = {
        'robot': 'LEGO_EV3',
        'camera': 'Camara',
        'camera_bumper': 'Camara_bumper',
        'camera_sonar': 'Camara_sonar',
        'sensor_color_LR': 'Sensor_Color_LR',
        'sensor_color_RC': 'Sensor_Color_RC',
        'sonar': 'Sonar',
        'bumper': 'Bumper',
        'motor_b': 'Motor_B',
        'motor_c': 'Motor_C',
        'slider': 'Slider_SF',
        'giroscope': 'Giroscopio'
    }
```

![EV3_LEGO](https://static1.juguetronica.com/media/catalog/product/l/e/lego_mindstorms_ev3_educ_grua.jpg)



#### Components

The robot has 3 cameras (_Camera_, _camera_bumper_ and _camera_sonar_).

There's 2 light sensors behind EV3, named _sensor_color_LR_ and _sensor_color_RC_, a sonar, a bumper, 2 motors (1 for each wheel), a slider and a giroscope.



### Third task: RoboComp's implementation.

After develop a functional EV3 controller, it's necessary to be implemented as a RoboComp component. In order to make it possible, the component will have a DifferentialRobot interface (see DifferentialRobot.idsl on robocomp interfaces). The component will have all of the necessary methods to implement the interface successfully.