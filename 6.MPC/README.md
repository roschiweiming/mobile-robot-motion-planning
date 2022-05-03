# Model Predictive Control

作业链接：https://github.com/roschiweiming/mobile-robot-motion-planning



### Solution

如下为路径跟踪过程的gif动图，存在问题：MPC预测的轨迹波动大

![output](/home/chiweiming/Documents/mobile-robot-motion-planning/6.MPC/picture/output.gif)





延迟补偿

```C++
  VectorX compensateDelay(const VectorX& x0) {
    VectorX x0_delay = x0;
    // TODO: compensate delay
    double cur_x = x0_delay(0);
    double cur_y = x0_delay(1);

    x0_delay(0) += cur_v*cos(cur_phi)*dt_;
    x0_delay(1) += cur_v*sin(cur_phi)*dt_;

    return x0_delay;
  }
```





### Eigen

**Eigen::Matrix4d &Eigen::MatrixBase<Eigen::Matrix4d>::setIdentity()**

1. 用单位矩阵对变量进行了初始化,即设置为单位矩阵 [Eigen库学习 ---- 5.高级初始化操作](https://blog.csdn.net/qq_39400324/article/details/116117391)

**Eigen::SparseMatrix< Scalar_, Options_, StorageIndex_ >::SparseMatrix	(		)**	

​	1.设置稀疏矩阵，默认大小为0 x 0 

```
template<typename Scalar_ , int Options_, typename StorageIndex_ >
Scalar& Eigen::SparseMatrix< Scalar_, Options_, StorageIndex_ >::coeffRef	(	Index 	row,
Index 	col 
)	

```

