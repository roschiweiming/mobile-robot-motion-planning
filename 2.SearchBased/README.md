# SearchBased motion planning

## 第二章作业

### 算法流程及运行效果

A星 算法

```bash
# A*算法流程
# 维持一个最优队列(priority queue)取存储被扩展的节点,代码中采用C++ STL的multimap实现，multimap将{key,value}当做元素，允许重复元素。multimap根据key的排序准则⾃动将元素排序，因此使⽤时只需考虑插⼊和删除操作即可。
# 定义一个启发式函数，代码见getHeu()函数
# 将起点放入最优队列中
# 指定起点的ｇ值为０，其余节点的ｇ值为infinite
# 进入循环
	# 如果队列为空，return false； break；
	# 从最优队列中移除f(n) = g(n) + h(n)最小的节点，该节点作为当前节点，并标志为已扩展
	# 如果当前节点是终点，则 return true; break;
	# 对于当前节点周围未被扩展的邻居节点：
		# 如果 g(m) = infinite
			# 设置 g(m) = g(n) + Cnm;
			# 把当前节点加入最优队列
		# 如果 g(m) > g(n) + Cnm
			# 更新 g(m) = g(n) + Cnm;
	# end
# 结束循环
```

