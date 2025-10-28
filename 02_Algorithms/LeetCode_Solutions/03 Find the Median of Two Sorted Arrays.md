# 寻找两个正序数组的中位数

## 问题描述
给定两个大小分别为 `m` 和 `n` 的正序（从小到大）数组 `nums1` 和 `nums2`，找出并返回这两个正序数组的**中位数**。

## 代码详解

### 核心思路：归并思想
使用双指针模拟归并排序的合并过程，找到中间位置的元素。

```cpp
#include <iostream>
#include <vector>
using namespace std;
/*
思路：
	1.先分别计算两个数组的长度，用totalen来表示两数组长度之和
	2.利用prev和curr两个数，就像两人拉着手往前走，curr读取完成->传递给prev
	3.判断循环到totalen的一半是奇数还是偶数
	4.最终输出prv和curr
错误总结：
	1、容器vector计算数组长度是nums1.size() 而不是 nums1.sizeof()
	2、初始化变量例如i，j,count,特别是prev和curr一定要写到全局，不要写到函数里面
	3、while (count <= totalen / 2)这里是小于等于，不是小于会导致循环执行次数不足
	4、从而漏取计算中位数所需的关键元素，最终输出错误结果
	5、if (i < m && (j >= n || nums1[i] <= nums2[j]))核心代码
	6、用 j > n 来判断 nums2 是否遍历完是逻辑错误，会导致指针越界或取错元素
	7、一定要有返回值
	if (totalLen % 2 == 1) {
            return curr; // 奇数：中间元素就是curr
        } else {
            return (prev + curr) / 2.0; // 偶数：prev和curr的平均值
        }
*/
class Solution {
public:
    double findMedianSortedArrays(vector<int>& nums1, vector<int>& nums2) {
        int m = nums1.size();
        int n = nums2.size();
        int totalLen = m + n;
        int i = 0, j = 0; // 双指针，分别指向nums1和nums2的当前位置
        int count = 0;    // 记录当前找到的元素个数
        int prev = 0, curr = 0; // 记录中间的两个元素（用于偶数长度）

        // 遍历到中间位置（totalLen/2 即可，因为偶数需要前一个元素）
        while (count <= totalLen / 2) {
            prev = curr; // 保存上一个元素（用于偶数情况）
            
            // 移动指针：优先选较小的元素，注意数组越界情况
            if (i < m && (j >= n || nums1[i] <= nums2[j])) {
                curr = nums1[i]; // 取nums1的元素
                i++;
            } else {
                curr = nums2[j]; // 取nums2的元素
                j++;
            }
            count++;
        }

        // 判断总长度是奇数还是偶数
        if (totalLen % 2 == 1) {
            return curr; // 奇数：中间元素就是curr
        } else {
            return (prev + curr) / 2.0; // 偶数：prev和curr的平均值
        }
    }
};
```

### 执行流程示例
以 `nums1 = [1, 3]`, `nums2 = [2, 4]` 为例：

- **初始状态**：`i=0, j=0, count=0, prev=0, curr=0`
- **循环过程**：
  1. `count=0`：`prev=0, curr=1`, `i=1, j=0, count=1`
  2. `count=1`：`prev=1, curr=2`, `i=1, j=1, count=2`
  3. `count=2`：`prev=2, curr=3`, `i=2, j=1, count=3`（停止）

- **结果计算**：总长度4为偶数，返回 `(2 + 3) / 2.0 = 2.5`

## 核心知识点

### 1. 双指针技巧
```cpp
int i = 0, j = 0; // 两个数组的指针
```
- **作用**：分别跟踪两个数组的当前位置
- **优势**：只需遍历到中间位置，无需合并整个数组

### 2. 边界条件处理
```cpp
if (i < m && (j >= n || nums1[i] <= nums2[j]))
```
- **`i < m`**：确保nums1不越界
- **`j >= n`**：nums2已遍历完，只能取nums1
- **`nums1[i] <= nums2[j]`**：选择较小的元素

### 3. 中位数计算逻辑
```cpp
// 奇数情况：直接返回中间元素
if (totalLen % 2 == 1) return curr;

// 偶数情况：返回中间两个元素的平均值
else return (prev + curr) / 2.0;
```

### 4. 关键变量作用
- **`prev`**：记录前一个元素（用于偶数长度）
- **`curr`**：记录当前元素
- **`count`**：控制循环终止条件

## 时间复杂度分析

### 时间复杂度：O(m+n)
- 最坏情况下需要遍历 `(m+n)/2 + 1` 个元素
- 线性时间复杂度

### 空间复杂度：O(1)
- 只使用了固定数量的变量
- 常数级别的额外空间

## 算法优化方向

### 更优解法：二分查找
```cpp
// 时间复杂度 O(log(min(m,n)))
double findMedianSortedArrays(vector<int>& nums1, vector<int>& nums2) {
    if (nums1.size() > nums2.size()) {
        return findMedianSortedArrays(nums2, nums1);
    }
    
    int m = nums1.size(), n = nums2.size();
    int left = 0, right = m;
    
    while (left <= right) {
        int partition1 = (left + right) / 2;
        int partition2 = (m + n + 1) / 2 - partition1;
        
        // 边界条件处理和中位数计算...
    }
}
```

## 面试常见问题与回答

### Q1: 这个算法的时间复杂度是多少？能否优化？
**回答**：当前算法时间复杂度是O(m+n)。可以优化到O(log(min(m,n)))使用二分查找法，在较短的数组上进行二分划分。

### Q2: 为什么需要prev和curr两个变量？
**回答**：当总长度为偶数时，中位数是中间两个数的平均值。prev记录前一个元素，curr记录当前元素，这样在偶数情况下可以直接计算平均值。

### Q3: 如何处理数组越界的情况？
**回答**：通过条件`i < m && (j >= n || nums1[i] <= nums2[j])`确保：
- 当某个数组遍历完时，自动从另一个数组取元素
- 避免访问无效的内存位置

### Q4: 如果输入数组为空怎么办？
**回答**：代码能正确处理空数组情况：
- 如果nums1为空，全程从nums2取元素
- 如果nums2为空，全程从nums1取元素
- 如果都为空，返回0（根据具体实现）

### Q5: 这个算法是否适用于非正序数组？
**回答**：不适用。算法依赖于输入数组已排序的特性。如果数组无序，需要先排序或使用其他方法。

### Q6: 为什么循环条件是count <= totalLen/2？
**回答**：因为我们需要找到中间位置的元素。对于长度为n的数组：
- 奇数：需要第n/2+1个元素
- 偶数：需要第n/2和第n/2+1个元素
循环totalLen/2+1次可以确保获取到所有需要的元素。

### Q7: 这个算法的空间复杂度为什么是O(1)？
**回答**：算法只使用了固定数量的变量(i, j, count, prev, curr等)，不随输入规模增长而增长，因此是常数空间复杂度。

### Q8: 如果数组很大，这个算法是否高效？
**回答**：对于大规模数据，O(m+n)的时间复杂度可能不够高效。在实际应用中，如果数据量很大，应该考虑O(log(min(m,n)))的二分查找解法。

### Q9: 如何处理有重复元素的情况？
**回答**：算法能正确处理重复元素，因为比较条件`nums1[i] <= nums2[j]`在相等时会优先取nums1的元素，这不会影响中位数的计算。

### Q10: 这个方法的优缺点是什么？
**优点**：
- 思路简单直观，容易理解和实现
- 代码简洁，边界条件清晰
- 空间复杂度低

**缺点**：
- 时间复杂度不是最优
- 对于极大数组效率较低

## 总结

这个解法虽然时间复杂度不是最优的，但具有很好的可读性和易于理解的特点，适合在面试中快速实现。掌握这种双指针方法对于解决其他归并类问题也很有帮助。