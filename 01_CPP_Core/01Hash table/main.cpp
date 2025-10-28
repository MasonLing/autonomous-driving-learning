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