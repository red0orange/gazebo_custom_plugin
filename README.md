# Gazebo可手动控制的仿真人模型

# 使用说明

1. gazebo_extra_plugins是一个ros包，直接放入工作空间编译即可生成需要的插件。
2. actor.world是使用了自己生成的插件的world文件，注意打开前需要将devel/lib加入到Gazebo的环境变量中，否则将无法正常使用。
3. 运行步骤：先roscore，然后再gazebo actor.world运行文件，然后即可rosrun运行包内提供keyboard_actor_pyqt.py或keyboard_actor.py对行人进行控制，控制方式都是i->前进，j->左转，l->右转，pyqt版本的因为能够监控按键的按下和松开事件，实现的控制更加低延时一点。
