# Simplified Trajectory Data Export

## 🎯 **New Simplified Export Format**

Added clean, minimal trajectory export containing only the essential kinematic data: **position, velocity, and acceleration** in x, y, z coordinates.

## 📊 **Simplified Data Structure**

### **Column Format**
```
time     # Time vector [s]
pos_x    # Position X (altitude) [m] 
pos_y    # Position Y (cross-range) [m]
pos_z    # Position Z (down-range) [m]
vel_x    # Velocity X (altitude) [m/s]
vel_y    # Velocity Y (cross-range) [m/s] 
vel_z    # Velocity Z (down-range) [m/s]
acc_x    # Total acceleration X [m/s²]
acc_y    # Total acceleration Y [m/s²]
acc_z    # Total acceleration Z [m/s²]
```

### **Key Features**
- ✅ **Clean 9-column format** (time + 3×3 kinematic variables)
- ✅ **Total acceleration** (thrust + gravity combined)
- ✅ **Standard coordinate system** (x=altitude, y=cross-range, z=down-range)
- ✅ **SI units throughout** (meters, m/s, m/s²)

## 📂 **Two Export Versions**

### 1. **`gfold_comprehensive_simple_trajectory.csv`**
- **121 data points** at optimization time steps (~0.5s intervals)
- **Direct from solver** - no interpolation
- **Best for analysis** requiring exact optimization points

### 2. **`gfold_comprehensive_simple_interpolated.csv`**  
- **6001 data points** at 0.01s intervals (60s trajectory)
- **High-resolution interpolated** for smooth plotting
- **Best for visualization** and detailed analysis

## 🎨 **Easy Plotting Example**

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load simplified trajectory data
df = pd.read_csv('gfold_comprehensive_simple_interpolated.csv')

# Create comprehensive trajectory plot
fig, axes = plt.subplots(3, 3, figsize=(15, 12))

# Position plots
axes[0,0].plot(df['time'], df['pos_x'])
axes[0,0].set_title('Position X (Altitude)')
axes[0,0].set_ylabel('Position [m]')

axes[0,1].plot(df['time'], df['pos_y']) 
axes[0,1].set_title('Position Y (Cross-range)')

axes[0,2].plot(df['time'], df['pos_z'])
axes[0,2].set_title('Position Z (Down-range)')

# Velocity plots  
axes[1,0].plot(df['time'], df['vel_x'])
axes[1,0].set_title('Velocity X')
axes[1,0].set_ylabel('Velocity [m/s]')

axes[1,1].plot(df['time'], df['vel_y'])
axes[1,1].set_title('Velocity Y')

axes[1,2].plot(df['time'], df['vel_z'])
axes[1,2].set_title('Velocity Z')

# Acceleration plots
axes[2,0].plot(df['time'], df['acc_x'])
axes[2,0].set_title('Acceleration X')
axes[2,0].set_ylabel('Acceleration [m/s²]')
axes[2,0].set_xlabel('Time [s]')

axes[2,1].plot(df['time'], df['acc_y'])
axes[2,1].set_title('Acceleration Y')
axes[2,1].set_xlabel('Time [s]')

axes[2,2].plot(df['time'], df['acc_z'])
axes[2,2].set_title('Acceleration Z')
axes[2,2].set_xlabel('Time [s]')

plt.tight_layout()
plt.show()

# 3D trajectory plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot(df['pos_z'], df['pos_y'], df['pos_x'])
ax.set_xlabel('Down-range [m]')
ax.set_ylabel('Cross-range [m]')
ax.set_zlabel('Altitude [m]')
ax.set_title('3D Trajectory')
plt.show()
```

## 🔍 **Sample Data Analysis**

### **Trajectory Start (t=0.0s)**
```
Position: [20.0, 5.0, 5.0] m
Velocity: [0.0, 0.0, 0.0] m/s  
Acceleration: [-2.98, 0.0, 0.0] m/s²
```
*Interpretation: Starting at altitude with minimal thrust, mostly in free fall*

### **Mid-Trajectory (t=1.0s)**
```
Position: [19.97, 4.997, 4.997] m
Velocity: [-0.18, -0.032, -0.032] m/s
Acceleration: [-0.27, -0.21, -0.21] m/s²  
```
*Interpretation: Controlled descent with balanced thrust and gravity*

## 📋 **Complete Export Summary**

Your GFOLD trajectory generator now provides **6 different data export formats**:

1. **`gfold_trajectory.csv`** - Basic backward-compatible export
2. **`gfold_comprehensive_optimization_stats.csv`** - Complete solver statistics
3. **`gfold_comprehensive_detailed_trajectory.csv`** - Full trajectory with all variables
4. **`gfold_comprehensive_interpolated_trajectory.csv`** - High-res full trajectory
5. **`gfold_comprehensive_simple_trajectory.csv`** - Clean pos/vel/acc only ⭐ **NEW**
6. **`gfold_comprehensive_simple_interpolated.csv`** - High-res pos/vel/acc ⭐ **NEW**

## ✅ **Benefits of Simplified Export**

- 🎯 **Focused data** - Only essential kinematic variables
- 📊 **Easy analysis** - Standard 9-column format
- 🎨 **Quick plotting** - No need to filter complex data
- 📱 **Portable** - Smaller file size for sharing
- 🔄 **Universal** - Works with any analysis tool
- 🎓 **Educational** - Clear for learning trajectory dynamics

The simplified export provides clean, focused trajectory data perfect for analysis, plotting, and sharing without the complexity of the full comprehensive dataset!