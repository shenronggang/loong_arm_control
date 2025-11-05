程序框架

# 使用说明

所有常规操作（编译、运行）在tools内通过脚本完成！

不要自己命令行编译！

# 框架核心思想：隔离！

为防止屎山出现，任何人添加的功能模块，必须以最小成本、最小作用域的形式进行添加。

# 使用说明

所有常规操作（编译、运行）在tools内通过脚本完成！不要自己命令行编译！

* 编译：./make.sh [参数]，参数见该脚本打印，脚本会自动copy生成库至对应文件目录（如默认路径不匹配，需自行修改）
* 离线测试：./run_test.sh
* 仿真：./run_mujoco.sh [延时毫秒]，功能同mujoco目录下的同名脚本

# 框架结构

框架顶层为状态机fsm

添加功能的最大单位被称为plan，plan被add进状态机fsm，fsm负责调度所有的plan

plan内完成各自所需的所有子功能模块，不同plan之间在调度上互相独立。

plan之间复用的模块、数据，推荐以单例形式调用

程序初始会调用每个plan的init

程序运行过程中fms会调用plan的进入(hey)、运行(run)、通知退出(bye)、写日志(log)四个接口，plan自身通过运行(run)的返回值负责自身的退出。另外fms会根据plan的优先级强制plan退出

# 已有plan
* free plan：下使能
* idle plan：空闲
* damp plan：阻尼
* action plan：根据key做一些固定动作
* manipulation plan：根据操作接口数据控制运动


