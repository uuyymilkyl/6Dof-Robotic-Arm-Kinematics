
无需安装任何库，计算简单且通用
请在任意情况下注明来源

逆解方法参考论文： [1]李迎国. 基于6轴机械臂的3D打印相关方法研究[D].西安理工大学,2023.DOI:10.27398/d.cnki.gxalu.2023.000157.


| include：
---|
1. t_KineMatrix.hpp  ：  定义模板类  代替Eigen库进行矩阵描述及运算
2. f_TransPoseDescrib.hpp  ： 功能类 姿态描述转化 四元数-轴角-旋转矩阵-欧拉角 
3. f_Kinematics.hpp  ： 功能类 运动学解算 正解/逆解

| src：
---|
1. f_TransPoseDescrib.cpp
2. f_Kinematics.cpp

| main.cpp  测试历程


2023/12/23  创建，代码已验证未整理 陆续上传中

