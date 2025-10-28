## 一、C++ 哈希表使用方法

### 1.1 unordered_map 基本用法

```cpp
#include <unordered_map>
#include <string>
#include <iostream>
using namespace std;

int main() {
    // 创建哈希表
    unordered_map<string, int> scoreMap;
    
    // 插入数据的几种方式
    scoreMap["Alice"] = 95;           // 方式1：使用下标运算符
    scoreMap.insert({"Bob", 88});     // 方式2：使用insert
    scoreMap.insert(pair<string, int>("Charlie", 92)); // 方式3
    scoreMap.emplace("David", 78);    // 方式4：原地构造，效率更高
    
    // 访问元素
    cout << "Alice的分数: " << scoreMap["Alice"] << endl;
    cout << "Bob的分数: " << scoreMap.at("Bob") << endl;
    
    // 查找元素
    auto it = scoreMap.find("Eve");
    if (it != scoreMap.end()) {
        cout << "找到Eve: " << it->second << endl;
    } else {
        cout << "没找到Eve" << endl;
    }
    
    // 删除元素
    scoreMap.erase("David");
    
    // 遍历哈希表
    for (const auto& pair : scoreMap) {
        cout << pair.first << ": " << pair.second << endl;
    }
    
    // 使用迭代器遍历
    for (auto it = scoreMap.begin(); it != scoreMap.end(); it++) {
        cout << it->first << ": " << it->second << endl;
    }
    
    return 0;
}
```

### 1.2 unordered_set 基本用法

```cpp
#include <unordered_set>
#include <iostream>
using namespace std;

int main() {
    // 创建哈希集合
    unordered_set<int> numberSet;
    
    // 插入元素
    numberSet.insert(1);
    numberSet.insert(2);
    numberSet.insert(3);
    numberSet.insert(2);  // 重复元素，不会被插入
    
    // 查找元素
    if (numberSet.find(2) != numberSet.end()) {
        cout << "找到2" << endl;
    }
    
    // 删除元素
    numberSet.erase(3);
    
    // 遍历集合
    for (int num : numberSet) {
        cout << num << " ";
    }
    cout << endl;
    
    return 0;
}
```

## 二、重要注意事项

### 2.1 键的存在性检查

```cpp
unordered_map<string, int> map;

// ❌ 错误方法：会创建不存在的键
if (map["nonexistent"] > 0) {  // 这会创建键"nonexistent"并赋值为0
    // ...
}

// ✅ 正确方法：使用find或count
if (map.find("nonexistent") != map.end()) {
    // 键存在
}

if (map.count("nonexistent") > 0) {
    // 键存在
}
```

### 2.2 自定义类型作为键

```cpp
#include <unordered_map>

// 自定义类
class Student {
public:
    string name;
    int id;
    
    Student(string n, int i) : name(n), id(i) {}
    
    // 必须重载==运算符
    bool operator==(const Student& other) const {
        return name == other.name && id == other.id;
    }
};

// 自定义哈希函数
struct StudentHash {
    size_t operator()(const Student& s) const {
        return hash<string>()(s.name) ^ hash<int>()(s.id);
    }
};

// 使用自定义类型作为键
unordered_map<Student, int, StudentHash> studentScores;

// 或者使用lambda表达式
auto hashFunc = [](const Student& s) {
    return hash<string>()(s.name) ^ hash<int>()(s.id);
};
unordered_map<Student, int, decltype(hashFunc)> studentMap(10, hashFunc);
```

### 2.3 性能优化

```cpp
unordered_map<int, string> map;

// 如果知道大概的元素数量，可以预先分配空间
map.reserve(1000);  // 预留1000个元素的空间

// 查看哈希表状态
cout << "负载因子: " << map.load_factor() << endl;
cout << "最大负载因子: " << map.max_load_factor() << endl;
cout << "桶数量: " << map.bucket_count() << endl;
```

## 三、过程常见问题与回答

### 3.1 基础概念问题

**问题1：哈希表的原理是什么？**

**回答：**
哈希表通过哈希函数将键映射到数组的特定位置，实现快速的数据存取。核心思想是：
- 使用哈希函数计算键的哈希值
- 通过哈希值确定在数组中的存储位置
- 处理哈希冲突（不同键映射到同一位置）

**问题2：unordered_map 和 map 的区别？**

**回答：**
| 特性     | unordered_map            | map          |
| -------- | ------------------------ | ------------ |
| 底层实现 | 哈希表                   | 红黑树       |
| 查找时间 | 平均O(1)，最坏O(n)       | O(log n)     |
| 元素顺序 | 无序                     | 按键排序     |
| 内存占用 | 通常更多                 | 通常较少     |
| 适用场景 | 需要快速查找，不关心顺序 | 需要有序遍历 |

### 3.2 算法实现问题

**问题3：如何解决哈希冲突？**

**回答：**
主要有两种方法：
1. **链地址法**：每个位置存储一个链表，冲突元素放在链表中
2. **开放地址法**：冲突时寻找下一个空位置
   - 线性探测
   - 二次探测
   - 双重哈希

C++的unordered_map使用链地址法。

**问题4：哈希表的时间复杂度分析**

**回答：**
- 插入：平均O(1)，最坏O(n)
- 查找：平均O(1)，最坏O(n)
- 删除：平均O(1)，最坏O(n)

最坏情况发生在所有键都哈希到同一位置时。

### 3.3 实际应用问题

**问题5：两数之和问题的哈希表解法**

**回答：**
```cpp
vector<int> twoSum(vector<int>& nums, int target) {
    unordered_map<int, int> hash_map;
    for (int i = 0; i < nums.size(); i++) {
        int complement = target - nums[i];
        if (hash_map.find(complement) != hash_map.end()) {
            return {hash_map[complement], i};
        }
        hash_map[nums[i]] = i;
    }
    return {};
}
```

**时间复杂度：** O(n)
**空间复杂度：** O(n)

**问题6：如何设计一个好的哈希函数？**

**回答：**
好的哈希函数应该：
1. 计算速度快
2. 分布均匀，减少冲突
3. 确定性，相同键产生相同哈希值

对于自定义类型，需要结合所有关键字段计算哈希值。

### 3.4 进阶问题

**问题7：哈希表扩容机制**

**回答：**
当负载因子（元素数量/桶数量）超过阈值时，哈希表会进行rehash：
1. 创建更大的桶数组（通常是原来的2倍）
2. 重新计算所有元素的哈希值
3. 将元素重新分布到新数组中

**问题8：如何选择unordered_map的初始大小？**

**回答：**
如果知道元素的大概数量，可以使用reserve预分配空间：
```cpp
unordered_map<int, string> map;
map.reserve(expected_size);  // 减少rehash次数
```

## 四、实战技巧总结

### 4.1 使用建议

```cpp
// 1. 使用emplace避免临时对象
map.emplace(key, value);  // 优于 map.insert({key, value})

// 2. 使用结构化绑定(C++17)
for (const auto& [key, value] : map) {
    cout << key << ": " << value << endl;
}

// 3. 批量操作后reserve
vector<pair<int, string>> data = GetData();
unordered_map<int, string> result;
result.reserve(data.size());
for (const auto& item : data) {
    result.insert(item);
}
```

### 4.2 常见陷阱

```cpp
// 陷阱1：迭代器失效
unordered_map<int, int> map = {{1, 10}, {2, 20}};
for (auto it = map.begin(); it != map.end(); it++) {
    if (it->first == 1) {
        map.erase(it);  // ❌ 迭代器失效
        // 应该: it = map.erase(it);
    }
}

// 陷阱2：const引用问题
const unordered_map<int, string>& constMap = GetMap();
// string value = constMap[1];  // ❌ 不能使用下标操作
string value = constMap.at(1);  // ✅ 正确
```

---
## 相关链接
- [[C++容器详解]]
- [[算法复杂度分析]]
- [[面试技巧总结]]

## 标签
#C++  #哈希表 #面试 #数据结构 #算法
