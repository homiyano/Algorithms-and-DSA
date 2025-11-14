nums = [8,10,2,1]
target = 9
# output = [0,1]

# seen = {}
# complement = 0
# for i,j in enumerate(nums):
#     complement = target - j
#     if complement in seen:
#         print([seen[complement], i])
#         break

#     seen[j] = i


from typing import List

class Solution:
    def twoSum(self, nums: List[int], target: int) -> List[int]:
        seen = {}

        for i, num in enumerate(nums):
            complement = target - num

            if complement in seen:
                return [seen[complement], i]

            seen[num] = i

        return []