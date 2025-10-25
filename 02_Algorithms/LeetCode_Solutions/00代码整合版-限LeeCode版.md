## 两数之和
[[01数据结构-两数之和]]
[[02无重复字符的最长子串]]
[[03寻找两个正序数组的中位数]]
[[04链表-两数相加]]
[[05链表-反转完全解析]]



```cpp
[[01数据结构-两数之和]]
class Solution {
public:
    vector<int> twoSum(vector<int>& nums, int target) {
        // 使用哈希表存储数字和对应的索引
        // key: 数字的值, value: 数字在数组中的索引
        unordered_map<int, int> hash_map;
        
        // 遍历数组中的每个元素
        for(int i = 0; i < nums.size(); i++) {
            // 计算当前数字需要的补数（目标值减去当前数字）
            int complement = target - nums[i];
            
            // 检查补数是否已经在哈希表中
            // find()方法返回迭代器，如果没找到则返回end()
            if (hash_map.find(complement) != hash_map.end()) {
                // 找到解，返回两个数字的索引
                // hash_map[complement]是之前存储的补数的索引
                // i是当前数字的索引
                return {hash_map[complement], i};
            }
            
            // 将当前数字和索引存入哈希表，供后续查找使用
            // 注意：这里在检查之后才存入，避免使用同一个元素两次
            hash_map[nums[i]] = i;
        }
        
        // 如果没有找到解，返回空数组
        // 根据题目假设，这种情况不会发生
        return {};
    }
};

----------------------------------------------------------------------------
[[02无重复字符的最长子串]]
    
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

[[03寻找两个正序数组的中位数]]
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
