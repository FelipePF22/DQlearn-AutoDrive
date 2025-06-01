import numpy as np
import matplotlib.pyplot as plt

rewards = np.load('/home/marcos/.ros/q_table.npy') 

plt.figure(figsize=(12, 6))
plt.imshow(q_table, cmap='viridis', interpolation='nearest')
plt.colorbar(label='Q-Valor')
plt.xlabel('Ações (0=esq, 1=frente, 2=dir)')
plt.ylabel('Estados (0 a 26)')
plt.title('Heatmap da Q-Table')
plt.show()
