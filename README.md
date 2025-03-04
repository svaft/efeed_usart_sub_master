# CNC lathe efeed JOG remote controller
STM32  powered electro feed/thread cut using encoder&amp;stepper motor
base controller [EFEED](https://github.com/svaft/efeed) ([Download](https://github.com/svaft/efeed/releases))
JOG controller based on stm32f1, connected to main efeed by UART link. Support 3 joystick(X,Z,microstep), bluetooth connection to android app, hw buttons with software debounce and multistate(click, longpress, up, down) to control lathe  
