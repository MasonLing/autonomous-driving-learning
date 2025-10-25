
/*����һ���������� nums ��һ������Ŀ��ֵ target�������ڸ��������ҳ� ��ΪĿ��ֵ target  ���� ���� ���������������ǵ������±ꡣ

����Լ���ÿ������ֻ���Ӧһ���𰸣������㲻��ʹ��������ͬ��Ԫ�ء�

����԰�����˳�򷵻ش𰸡�

ʾ�� 1��
���룺nums = [2, 7, 11, 15], target = 9
�����[0, 1]
���ͣ���Ϊ nums[0] + nums[1] == 9 ������[0, 1] ��*/

#include <iostream>
#include <vector>
#include <unordered_map>
using namespace std;

class Solution {
public:
    // ��ϣ��ⷨ
    vector<int> twoSum(vector<int>& nums, int target) {
     //��������Ҫ�ĺ�������

     //vector<int>����������һ���������飨������

     //twoSum����������"����֮��"

     //vector<int>&nums������1����һ��������������ã������޸�ԭ���飩

     //int target������2���������͵�Ŀ��ֵ

        unordered_map<int, int> hash_map;
      /* ����һ���յĹ�ϣ��

       ��ʽ������ �� ����

       ���磺2 �� 0 ��ʾ����2�������е�λ����0*/

        for (int i = 0; i < nums.size(); i++) {
            int complement = target - nums[i];//����"����" = Ŀ��ֵ - ��ǰ����

            if (hash_map.find(complement) != hash_map.end()) {
                return { hash_map[complement], i };
            }

            hash_map[nums[i]] = i;
        }

        return {};
    }
};

// ���Ժ���
void testTwoSum() {
    Solution solution;

    // ��������1
    vector<int> nums1 = { 2, 7, 11, 15 };
    int target1 = 9;
    vector<int> result1 = solution.twoSum(nums1, target1);
    cout << "����1 - ����: [2,7,11,15], Ŀ��: 9" << endl;
    cout << "���: [" << result1[0] << "," << result1[1] << "]" << endl << endl;

    // ��������2
    vector<int> nums2 = { 3, 2, 4 };
    int target2 = 6;
    vector<int> result2 = solution.twoSum(nums2, target2);
    cout << "����2 - ����: [3,2,4], Ŀ��: 6" << endl;
    cout << "���: [" << result2[0] << "," << result2[1] << "]" << endl << endl;

    // ��������3
    vector<int> nums3 = { 3, 3 };
    int target3 = 6;
    vector<int> result3 = solution.twoSum(nums3, target3);
    cout << "����3 - ����: [3,3], Ŀ��: 6" << endl;
    cout << "���: [" << result3[0] << "," << result3[1] << "]" << endl;
}

int main() {
    testTwoSum();
    system("pause");
    return 0;
}