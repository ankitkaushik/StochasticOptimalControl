import numpy as np

nums = []
for i in range(100000):
	nums.append(np.random.normal(0.0, np.sqrt(0.1)))
print max(nums)
print min(nums)