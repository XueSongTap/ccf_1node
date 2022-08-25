# driver for complementary filter for a imu

serial: read serial hex from the unart port linked by usb cable to the imu module


yesense_driver_node: contains the node that publish imu message, euler message and do check sum from the serial read

six_axis_comp_filter: do comp_filter from the linear accel and the angular velocity

rostopic list:

/imu/euler_raw 原生输出的欧拉角 格式使用VectorStamped

/imu/euler_comp_filter 通过linear accel 和 angular elocity 进行互补滤波后的欧拉角数据  格式使用VectorStamped

/imu/data_raw 原生输出的imu 信息， 包括四元数， 线性加速度， 角速度


roll pitch -> 互补滤波

yaww -> 角加速度直接积分

使用欧拉角的情况下会有一个角无法进行互补滤波


shell脚本用于将rostopic 数据存入csv中， 如果写在一个脚本里， 两个rostopic好像没办法同时存两个数据，第一个读取结束， ctrl+c之后 进行第二个rosptic的读取， 故写在两个脚本里，分别运行

topic_ccf2csv.sh
topic_raw2csv.sh