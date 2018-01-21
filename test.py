import numpy as np

nums = []
for i in range(1000):
	randomOffset = np.random.normal()
	dtheta = (0.25)*randomOffset/np.sqrt(0.1)
	nums.append(dtheta)

print np.max(nums)
print np.min(nums)
print np.mean(nums)