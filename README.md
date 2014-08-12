# OvenController

用arduino控制器代替原本机械温控的计划。

硬件有：

- Arduino mega 2560
- LCD4884 v1.1 [link](http://wiki.dfrobot.com.cn/index.php/LCD4884_Shield_%E5%85%BC%E5%AE%B9Arduino(SKU:DFR0092))
- PS2 joystick
- MAX6675 温度传感器 [link](https://github.com/ryanjmclaughlin/MAX6675-Library)
- 继电器
- 某常温温度传感器（用于校准，尚未到货）

软件设计：

- 初始化：硬件，timer5，进入默认界面
- loop：啥也不干
- timer5Event：
  1. 刷新所有温度显示
  2. 让指定位置闪烁
  3. 检查温度以控制烤箱
- 界面：显示温度，设置目标温度。