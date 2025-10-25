
/*给定一个整数数组 nums 和一个整数目标值 target，请你在该数组中找出 和为目标值 target  的那 两个 整数，并返回它们的数组下标。

你可以假设每种输入只会对应一个答案，并且你不能使用两次相同的元素。

你可以按任意顺序返回答案。

示例 1：
输入：nums = [2, 7, 11, 15], target = 9
输出：[0, 1]
解释：因为 nums[0] + nums[1] == 9 ，返回[0, 1] 。*/

#include <iostream>
#include <vector>
#include <unordered_map>
using namespace std;

class Solution {
public:
    // 哈希表解法
    vector<int> twoSum(vector<int>& nums, int target) {
     //这是最重要的函数声明

     //vector<int>：函数返回一个整数数组（向量）

     //twoSum：函数名叫"两数之和"

     //vector<int>&nums：参数1，是一个整数数组的引用（可以修改原数组）

     //int target：参数2，整数类型的目标值

        unordered_map<int, int> hash_map;
      /* 创建一个空的哈希表

       格式：数字 → 索引

       比如：2 → 0 表示数字2在数组中的位置是0*/

        for (int i = 0; i < nums.size(); i++) {
            int complement = target - nums[i];//计算"补数" = 目标值 - 当前数字

            if (hash_map.find(complement) != hash_map.end()) {
                return { hash_map[complement], i };
            }

            hash_map[nums[i]] = i;
        }

        return {};
    }
};

// 测试函数
void testTwoSum() {
    Solution solution;

    // 测试用例1
    vector<int> nums1 = { 2, 7, 11, 15 };
    int target1 = 9;
    vector<int> result1 = solution.twoSum(nums1, target1);
    cout << "测试1 - 输入: [2,7,11,15], 目标: 9" << endl;
    cout << "输出: [" << result1[0] << "," << result1[1] << "]" << endl << endl;

    // 测试用例2
    vector<int> nums2 = { 3, 2, 4 };
    int target2 = 6;
    vector<int> result2 = solution.twoSum(nums2, target2);
    cout << "测试2 - 输入: [3,2,4], 目标: 6" << endl;
    cout << "输出: [" << result2[0] << "," << result2[1] << "]" << endl << endl;

    // 测试用例3
    vector<int> nums3 = { 3, 3 };
    int target3 = 6;
    vector<int> result3 = solution.twoSum(nums3, target3);
    cout << "测试3 - 输入: [3,3], 目标: 6" << endl;
    cout << "输出: [" << result3[0] << "," << result3[1] << "]" << endl;
}

int main() {
    testTwoSum();
    system("pause");
    return 0;
}