CSC:压缩列存储格式(稀疏矩阵的一种存储方式）
用三个一维数组表示一个稀疏矩阵(很多 0 的矩阵）
OSQP 强制要求 P 与 A 都用 CSC 格式

**为什么要用 CSC？**
因为稀疏矩阵大多数元素都是 0，用普通二维数组存太浪费
CSC 只存：
非零元素的值
非零元素所在的行
每一列从哪里开始
所以不用存 0，省空间，也加速计算

**CSC 使用三个数组**
一个稀疏矩阵:
$A = \begin{bmatrix}
  1&  0& 2\\
  0&  3& 0\\
  4&  0& 5
\end{bmatrix}$

**A_data**
按列把非零元素存在一起
```cpp
A_data = [1,4, 3, 2,5]
```
**A_indices**
非零元素所在行
```cpp
A_indices = [0,2, 1, 0,2]
```
**A_indptr**
每一列非零元素在 A_data 的起始位置的索引

```cpp
A_indptr = [0, 2, 3, 5]
```
**CSC 用三个一维数组表示一个二维稀疏矩阵：**

```cpp
A = {A_data, A_indices, A_indptr}
```
它是个二维矩阵，只是存储方式变成了 3 个一维数组
**OSQP 就要这种格式**

```cpp
csc_matrix(n, n, P_data->size(), P_data->data(),
           P_indices->data(), P_indptr->data());
```
想象你有一个矩阵：
| 列 | 元素   |
| - | ---- |
| 0 | 1, 4 |
| 1 | 3    |
| 2 | 2, 5 |

CSC 就是把这些列“挤压”成紧凑结构：
data 放所有非零的值
indices 告诉你这些值来自哪一行
indptr 告诉你：某一列的元素从 data 的哪个 index 开始