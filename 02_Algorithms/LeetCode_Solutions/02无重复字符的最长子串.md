## 问题描述
给定一个字符串，找出其中**不含有重复字符**的**最长子串**的长度。

### 示例说明
- 字符串 "abcabcbb" → 最长无重复子串是 "abc"，长度=3
- 字符串 "bbbbb" → 最长无重复子串是 "b"，长度=1  
- 字符串 "pwwkew" → 最长无重复子串是 "wke"，长度=3

---

## 核心思路：滑动窗口算法

### 大白话解释
想象一个能在字符串上左右滑动的"窗户"：
- **右边界**不断向右探索新字符
- 如果新字符导致**窗口内有重复**，**左边界**就向右收缩
- 始终保持**窗口内没有重复字符**
- 记录**历史上最大的窗口长度**

### 可视化理解
```
字符串: a b c a b c b b
步骤1: [a]               ← 长度=1
步骤2: [a b]             ← 长度=2  
步骤3: [a b c]           ← 长度=3
步骤4: a [b c a]         ← 遇到重复，左边界右移
步骤5: a b [c a b]       ← 继续调整...
...
最终结果: 长度=3
```

---

## 完整代码详解

```cpp
#include <iostream>      // 输入输出
#include <unordered_set> // 哈希集合（自动去重）
#include <string>        // 字符串处理
using namespace std;     // 标准命名空间

class Solution {
public:
    int lengthOfLongestSubstring(string s) {
        // 特殊情况：空字符串直接返回0
        if (s.empty()) return 0;
        
        // 核心数据结构
        unordered_set<char> charSet;  // 存储当前窗口内的字符（自动去重）
        int left = 0;                 // 窗口左边界
        int maxLen = 0;               // 记录历史最大长度
        
        // 右边界从0遍历到字符串末尾
        for (int right = 0; right < s.length(); right++) {
            char currentChar = s[right];  // 当前右边界指向的字符
            
            // 关键步骤：如果当前字符在集合中已存在（说明重复了）
            while (charSet.find(currentChar) != charSet.end()) {
                // 不断删除左边界字符，并右移左边界，直到消除重复
                charSet.erase(s[left]);
                left++;
            }
            
            // 将当前字符加入集合（现在窗口内肯定没有重复了）
            charSet.insert(currentChar);
            
            // 计算当前窗口长度，更新历史最大值
            // 窗口长度 = 右边界 - 左边界 + 1
            maxLen = max(maxLen, right - left + 1);
        }
        
        return maxLen;  // 返回找到的最大长度
    }
};

// 测试主函数
int main() {
    Solution solution;           // 创建解决方案实例
    string input;               // 存储用户输入
    
    cout << "请输入一个字符串: ";
    getline(cin, input);        // 读取整行输入（支持空格）
    
    int result = solution.lengthOfLongestSubstring(input);
    cout << "最长无重复字符子串的长度为: " << result << endl;
    
    return 0;
}
```

---

## 关键知识点详解

### 1. 滑动窗口 (Sliding Window)
```cpp
// 窗口维护逻辑
[left...right]  // 当前窗口范围
```
- **右指针**：主动探索，每次循环右移1位
- **左指针**：被动调整，只在发现重复时右移
- **窗口性质**：始终保证窗口内无重复字符

### 2. 哈希集合 (unordered_set)
```cpp
unordered_set<char> charSet;
```
- **特性**：自动去重，相同元素只能存在一个
- **操作**：
  - `charSet.insert(c)` - 插入元素
  - `charSet.erase(c)` - 删除元素  
  - `charSet.find(c)` - 查找元素
- **时间复杂度**：插入、删除、查找都是O(1)

### 3. 时间复杂度分析
- **最好情况**：O(n) - 字符串本身无重复，右指针遍历一次
- **最坏情况**：O(2n) - 每个字符被左右指针各访问一次
- **平均情况**：O(n) - 线性时间复杂度

### 4. 空间复杂度
- **O(min(m, n))**：m字符集大小，n字符串长度
- 最多存储整个字符集的大小

---

## 逐步执行示例

以 "abcabcbb" 为例：

| 步骤 | right | 当前字符 | 窗口内容 | left | 操作 | maxLen |
|------|-------|----------|----------|------|------|--------|
| 1 | 0 | 'a' | "a" | 0 | 直接加入 | 1 |
| 2 | 1 | 'b' | "ab" | 0 | 直接加入 | 2 |
| 3 | 2 | 'c' | "abc" | 0 | 直接加入 | 3 |
| 4 | 3 | 'a' | "bca" | 1 | 删除'a',left++ | 3 |
| 5 | 4 | 'b' | "cab" | 2 | 删除'b',left++ | 3 |
| 6 | 5 | 'c' | "abc" | 3 | 删除'c',left++ | 3 |
| 7 | 6 | 'b' | "cb" | 5 | 删除'b',left++ | 3 |
| 8 | 7 | 'b' | "b" | 7 | 删除'b',left++ | 3 |

**最终结果：3**

---

## 面试常见问题与回答

### Q1: 为什么选择unordered_set而不是其他数据结构？
**A:** unordered_set提供O(1)的查找、插入、删除操作，正好满足我们频繁检查重复的需求。如果用vector查找是O(n)，用set虽然是O(log n)但不如哈希表快。

### Q2: 时间复杂度真的是O(n)吗？while循环嵌套在for里？
**A:** 确实是O(n)。虽然看起来是嵌套循环，但每个字符最多被左、右指针各访问一次，总共2n次操作，所以是O(n)级别。

### Q3: 如果字符串很长（比如几百万字符），这个算法还适用吗？
**A:** 适用。滑动窗口算法是处理这类问题的标准解法，时间复杂度线性，空间复杂度取决于字符集大小（ASCII最多128，Unicode最多几万），内存消耗可控。

### Q4: 如果要求返回具体子串而不仅是长度，怎么修改？
**A:** 可以增加变量记录最大长度时的左右边界：
```cpp
int start = 0, end = 0;  // 记录最长子串的起止位置
// 在更新maxLen时同时更新边界
if (right - left + 1 > maxLen) {
    maxLen = right - left + 1;
    start = left;
    end = right;
}
// 最后返回 s.substr(start, maxLen)
```

### Q5: 这个算法能处理包含空格的字符串吗？
**A:** 完全可以。unordered_set可以存储任何字符包括空格，算法逻辑不受影响。

### Q6: 如果输入是Unicode字符（如中文）还能用吗？
**A:** 可以。unordered_set支持wchar_t等宽字符，只需要将char改为wchar_t，string改为wstring即可。

### Q7: 有没有其他解法？
**A:** 还有两种常见解法：
1. **暴力法**：检查所有子串，O(n³)时间复杂度
2. **优化哈希表法**：记录字符最新位置，遇到重复时直接跳转左边界

### Q8: 这个算法在什么情况下性能最差？
**A:** 当字符串有很多重复字符时，比如"aaaaaa"，左边界需要频繁移动，但时间复杂度仍然是O(n)。

---

## 算法优化思路

### 优化版本（直接跳转）
```cpp
int lengthOfLongestSubstring(string s) {
    unordered_map<char, int> charIndex;  // 存储字符最新位置
    int left = 0, maxLen = 0;
    
    for (int right = 0; right < s.length(); right++) {
        if (charIndex.find(s[right]) != charIndex.end()) {
            // 直接跳转左边界，避免逐步移动
            left = max(left, charIndex[s[right]] + 1);
        }
        charIndex[s[right]] = right;  // 更新字符位置
        maxLen = max(maxLen, right - left + 1);
    }
    return maxLen;
}
```

**优势**：减少左边界移动次数，性能更稳定。

---

## 总结

滑动窗口算法是解决子串/子数组问题的利器，核心思想是：
- **右边界探索**：主动扩展窗口范围
- **左边界调整**：被动维护窗口有效性  
- **实时记录**：持续跟踪最优解

掌握这个模板，可以解决一大类"最长无重复子串"、"最小覆盖子串"等问题。