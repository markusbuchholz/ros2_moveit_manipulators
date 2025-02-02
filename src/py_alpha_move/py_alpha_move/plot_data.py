import pandas as pd
import matplotlib.pyplot as plt
import os
from matplotlib.ticker import AutoMinorLocator  

# Configuration
files = ['joint_states_log_1.txt', 'joint_states_log_1.txt']  # Add your filenames here
colors = ['blue', 'blue']  # Different colors for each file
label_size = 30  # Increased label size
tick_size = 30    # Tick number font size

# Read all files
dataframes = []
for file in files:
    df = pd.read_csv(file, sep='\s*,\s*', engine='python')
    dataframes.append((os.path.splitext(file)[0], df))

# Create figure and subplots
fig, axs = plt.subplots(2, 2, figsize=(14, 10))
#fig.suptitle('Axis Values Comparison', fontsize=14, y=0.98)

# Get global time range for consistent X-axis
max_time = max(max(df['ElapsedTime']) for _, df in dataframes)

# Plot configuration
axes_to_plot = ['Axis_B', 'Axis_C', 'Axis_D', 'Axis_E']
titles = ['axis b', 'axis c', 'axis d', 'axis e']

# Plot data from all files
for ax_idx, (ax, col, title) in enumerate(zip(axs.flat, axes_to_plot, titles)):
    for (label, df), color in zip(dataframes, colors):
        ax.plot(df['ElapsedTime'], df[col], 
                color=color, 
                linewidth=1.8,  # Slightly thicker lines
                label=label)
        
    # Axis configuration
    ax.set_xlim(0, max_time)
    ax.set_title(title, fontsize=label_size+2, pad=12)
    ax.set_xlabel('time [sec]', fontsize=label_size)
    ax.set_ylabel('position [rad]', fontsize=label_size)
    
    # Enhanced grid system
    ax.minorticks_on()
    ax.grid(True, which='major', linestyle='-', linewidth=0.7, alpha=0.5)
    ax.grid(True, which='minor', linestyle=':', linewidth=0.5, alpha=0.3)
    
    # Configure tick parameters
    ax.tick_params(which='both', direction='in', labelsize=tick_size)
    ax.tick_params(which='major', length=6)
    ax.tick_params(which='minor', length=3)
    
    # Add automatic minor tick locators
    ax.xaxis.set_minor_locator(AutoMinorLocator())
    ax.yaxis.set_minor_locator(AutoMinorLocator())
    
    # Add legend to first subplot only
    # if ax_idx == 0:
    #     ax.legend(fontsize=10, loc='upper right', framealpha=0.9)

plt.tight_layout(pad=3.0)
plt.show()