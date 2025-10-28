# C++ 命名空间与 :: 符号详解

## 不使用 :: 符号的条件

**可以省略 `::` 符号的情况：**

### 1. 使用 `using namespace std;`
```cpp
#include <vector>
#include <unordered_map>
using namespace std;  // 引入整个std命名空间

int main() {
    vector<int> nums = {1, 2, 3};        // 不需要 std::
    unordered_map<int, string> map;      // 不需要 std::
    map[1] = "hello";
    return 0;
}
```

### 2. 使用 `using` 声明特定类型
```cpp
#include <vector>
#include <unordered_map>
using std::vector;           // 只引入vector
using std::unordered_map;    // 只引入unordered_map
using std::string;           // 只引入string

int main() {
    vector<int> nums = {1, 2, 3};        // 不需要 std::
    unordered_map<int, string> map;      // 不需要 std::
    return 0;
}
```

## 必须使用 :: 符号的情况

### 1. 没有使用 `using` 指令
```cpp
#include <vector>
#include <unordered_map>
// 没有 using namespace std;

int main() {
    std::vector<int> nums = {1, 2, 3};           // 必须使用 std::
    std::unordered_map<int, std::string> map;    // 必须使用 std::
    return 0;
}
```

### 2. 命名空间冲突时
```cpp
#include <vector>

namespace mylib {
    class vector { /* 自定义vector类 */ };
}

int main() {
    using namespace mylib;
    using namespace std;
    
    std::vector<int> nums;    // 必须明确指定是 std::vector
    mylib::vector myVec;      // 必须明确指定是 mylib::vector
    return 0;
}
```

### 3. 全局命名空间访问
```cpp
int count = 10;  // 全局变量

namespace myns {
    int count = 20;  // 命名空间内的变量
    
    void func() {
        int count = 30;  // 局部变量
        std::cout << count << std::endl;        // 30 (局部变量)
        std::cout << myns::count << std::endl;  // 20 (命名空间变量)
        std::cout << ::count << std::endl;      // 10 (全局变量)
    }
}
```

## 最佳实践建议

### ✅ 推荐做法（在源文件中）
```cpp
// 在 .cpp 文件中可以使用
#include <vector>
#include <unordered_map>
using namespace std;

int main() {
    vector<int> nums;
    unordered_map<string, int> map;
    // 代码更简洁
    return 0;
}
```

### ⚠️ 头文件中的做法
```cpp
// 在 .h 头文件中应该避免 using namespace
#ifndef MYHEADER_H
#define MYHEADER_H

#include <vector>
#include <unordered_map>

class MyClass {
public:
    std::vector<int> getData();  // 明确使用 std::
    void processData(const std::unordered_map<std::string, int>& data);
};

#endif
```

### 🔧 局部使用建议
```cpp
#include <vector>
#include <unordered_map>

int main() {
    // 在函数内部局部使用
    using std::vector;
    using std::unordered_map;
    using std::string;
    
    vector<int> nums;
    unordered_map<string, int> map;
    
    return 0;
}
```

## 实际项目中的选择

### 小型项目/竞赛编程
```cpp
#include <bits/stdc++.h>
using namespace std;

int main() {
    vector<int> v;
    unordered_map<int, int> mp;
    // 代码简洁，方便快速编写
    return 0;
}
```

### 大型项目/生产代码
```cpp
#include <vector>
#include <unordered_map>
#include <string>

class DataProcessor {
public:
    std::vector<int> process(const std::unordered_map<std::string, int>& input) {
        std::vector<int> result;
        // 处理逻辑
        return result;
    }
};
```

## 总结

| 场景                                 | 是否可以省略 `::` | 建议                       |
| ------------------------------------ | ----------------- | -------------------------- |
| 源文件(.cpp) + `using namespace std` | ✅ 可以            | 推荐用于小型项目           |
| 头文件(.h)                           | ❌ 不可以          | 必须使用 `std::`           |
| 有命名冲突时                         | ❌ 不可以          | 必须明确指定命名空间       |
| 大型项目                             | ❌ 不建议          | 使用 `std::` 避免污染      |
| 竞赛编程                             | ✅ 可以            | 使用 `using namespace std` |

**核心原则：**
- 在 `.cpp` 文件中可以使用 `using namespace std` 让代码更简洁
- 在 `.h` 头文件中应该避免，使用完整的 `std::` 前缀
- 大型项目中建议始终使用 `std::` 以避免命名冲突
