fig, ax = plt.subplots()
data = plotter.plot_data/0.0945
im = ax.imshow(data.T)
xlabs=[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
ylabs=[0,1,2,3,4,5]
ylabs=[100,50,33,25,20,16]

# We want to show all ticks...
ax.set_xticks(np.arange(len(xlabs)))
ax.set_yticks(np.arange(len(ylabs)))
# ... and label them with the respective list entries
ax.set_xticklabels(xlabs)
ax.set_yticklabels(ylabs)
ax.set_xlabel('Orders')
ax.set_ylabel('% Active')

# Loop over data dimensions and create text annotations.
for i in range(len(ylabs)):
    for j in range(len(xlabs)):
        text = ax.text(j, i, round(data.T[i, j],1), ha="center", va="center", color="w")

ax.set_title("EFD Order and % Active vs Time Averaged Error (normalized)")
fig.tight_layout()
plt.show()

## phase_studies plot
params1=[50.,100.,200.,500.,1000.]
    
# Sensor noise
params2=np.logspace(np.log10(1e-5), np.log10(1e-1), 5)

fig, ax = plt.subplots()
# data = x_mean
data = z_mean
im = ax.imshow(data.T)
xlabs=params1
ylabs=params2

# We want to show all ticks...
ax.set_xticks(np.arange(len(xlabs)))
ax.set_yticks(np.arange(len(ylabs)))
# ... and label them with the respective list entries
ax.set_xticklabels(xlabs)
ax.set_yticklabels(ylabs)
ax.set_xlabel('Update Frequency [Hz]')
ax.set_ylabel('Accelerometer Noise [m/s^2]')

# Loop over data dimensions and create text annotations.
for i in range(len(ylabs)):
    for j in range(len(xlabs)):
        text = ax.text(j, i, round(data.T[i, j],2), ha="center", va="center", color="w")

# ax.set_title("Time Averaged Error: X (normalized)")
ax.set_title("Time Averaged Error: COM (normalized)")
fig.tight_layout()
plt.show()