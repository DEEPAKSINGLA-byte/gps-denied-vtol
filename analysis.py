import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

# Set global aesthetic theme
sns.set_theme(style="whitegrid")
plt.rcParams.update({'font.sans-serif': 'DejaVu Sans', 'font.family': 'sans-serif'})

# ---------------------------------------------------------
# 1. Master Dataset (15 GPS-Denied UAV Navigation Solutions)
# ---------------------------------------------------------
data = [
    {"Solution": "2D LiDAR + Optical Flow", "Drift_Mid": 1.4, "Power_Mid": 7.0, "Payload_Mid": 150, "Cost_INR": 12500, "Category": "LiDAR/Range"},
    {"Solution": "Stereo Cam V-SLAM (RPi 5)", "Drift_Mid": 0.3, "Power_Mid": 10.5, "Payload_Mid": 85, "Cost_INR": 18000, "Category": "Visual"},
    {"Solution": "2D LiDAR + Monocular RGB", "Drift_Mid": 1.5, "Power_Mid": 8.0, "Payload_Mid": 140, "Cost_INR": 15500, "Category": "LiDAR/Range"},
    {"Solution": "Event-Based Camera Odometry", "Drift_Mid": 1.4, "Power_Mid": 5.5, "Payload_Mid": 42.5, "Cost_INR": 45000, "Category": "Visual"},
    {"Solution": "Visual SLAM", "Drift_Mid": 0.3, "Power_Mid": 17.5, "Payload_Mid": 85, "Cost_INR": 12000, "Category": "Visual"},
    {"Solution": "Acoustic / Ultrasonic Array", "Drift_Mid": 2.5, "Power_Mid": 1.25, "Payload_Mid": 27.5, "Cost_INR": 16500, "Category": "LiDAR/Range"},
    {"Solution": "Optical Flow + Rangefinder", "Drift_Mid": 3.5, "Power_Mid": 2.0, "Payload_Mid": 25, "Cost_INR": 7750, "Category": "Visual"},
    {"Solution": "ArUco / Fiducial Tags", "Drift_Mid": 0.005, "Power_Mid": 2.0, "Payload_Mid": 20, "Cost_INR": 5000, "Category": "Visual"},
    {"Solution": "Three RGB Cameras (RPi 5)", "Drift_Mid": 0.65, "Power_Mid": 13.0, "Payload_Mid": 105, "Cost_INR": 14000, "Category": "Visual"},
    {"Solution": "Depth Camera (RPi 5)", "Drift_Mid": 1.0, "Power_Mid": 10.0, "Payload_Mid": 70, "Cost_INR": 26000, "Category": "Visual"},
    {"Solution": "3D LiDAR SLAM (RPi 5)", "Drift_Mid": 0.125, "Power_Mid": 22.5, "Payload_Mid": 375, "Cost_INR": 425000, "Category": "LiDAR/Range"},
    {"Solution": "FMCW mmWave Radar", "Drift_Mid": 2.0, "Power_Mid": 7.0, "Payload_Mid": 120, "Cost_INR": 77500, "Category": "LiDAR/Range"},
    {"Solution": "UWB Beacon Triangulation", "Drift_Mid": 0.005, "Power_Mid": 2.0, "Payload_Mid": 20, "Cost_INR": 30000, "Category": "RF/Beacons"},
    {"Solution": "MAGNAV", "Drift_Mid": 0.6, "Power_Mid": 3.5, "Payload_Mid": 70, "Cost_INR": 40000, "Category": "Geophysical"},
    {"Solution": "TERCOM / SITAN", "Drift_Mid": 0.45, "Power_Mid": 10.0, "Payload_Mid": 225, "Cost_INR": 235000, "Category": "Geophysical"}
]

df = pd.DataFrame(data)

# ---------------------------------------------------------
# Chart 1: SWaP Trade-Off (Payload Mass vs. Power Draw)
# ---------------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 6))
sns.scatterplot(
    data=df, 
    x="Payload_Mid", 
    y="Power_Mid", 
    hue="Category", 
    size="Drift_Mid", 
    sizes=(40, 400), 
    palette="Set2", 
    alpha=0.85, 
    ax=ax
)

plt.xscale("log")
plt.yscale("log")
plt.title("SWaP Trade-Off: Payload Mass vs. Power Draw (Bubble Size = Drift Rate %)", fontsize=13, fontweight='bold', pad=15)
plt.xlabel("Payload Mass (g) [Log Scale]", fontsize=11)
plt.ylabel("Power Draw (W) [Log Scale]", fontsize=11)

# Annotate key navigation solutions
for _, row in df.iterrows():
    if row["Solution"] in ["3D LiDAR SLAM (RPi 5)", "UWB Beacon Triangulation", "FMCW mmWave Radar", "Optical Flow + Rangefinder", "Stereo Cam V-SLAM (RPi 5)"]:
        ax.annotate(
            row["Solution"], 
            (row["Payload_Mid"], row["Power_Mid"]),
            textcoords="offset points", 
            xytext=(5, 5), 
            ha='left', 
            fontsize=9, 
            weight='bold'
        )

plt.tight_layout()
plt.savefig("1_swap_tradeoff.png", dpi=300)
plt.show()

# ---------------------------------------------------------
# Chart 2: Position Drift Rate Benchmark (Bar Chart)
# ---------------------------------------------------------
df_sorted_drift = df.sort_values(by="Drift_Mid", ascending=True)

fig, ax = plt.subplots(figsize=(10, 7))
palette_drift = sns.color_palette("mako", len(df_sorted_drift))
bars_drift = ax.barh(df_sorted_drift["Solution"], df_sorted_drift["Drift_Mid"], color=palette_drift)

plt.xlabel("Position Drift Rate (% of Distance Traveled) [Lower is Better]", fontsize=11)
plt.title("Position Drift Rate Benchmark Across All 15 Solutions", fontsize=13, fontweight='bold', pad=15)
plt.xlim(0, 4.2)

for bar in bars_drift:
    width = bar.get_width()
    label = f"{width:.2f}%" if width >= 0.01 else "<0.01%"
    ax.text(width + 0.05, bar.get_y() + bar.get_height()/2, label, ha='left', va='center', fontsize=8.5)

plt.tight_layout()
plt.savefig("2_drift_rate_comparison.png", dpi=300)
plt.show()

# ---------------------------------------------------------
# Chart 3: Hardware Cost Comparison (in INR)
# ---------------------------------------------------------
df_sorted_cost = df.sort_values(by="Cost_INR", ascending=True)

fig, ax = plt.subplots(figsize=(11, 7.5))
palette_cost = sns.color_palette("viridis", len(df_sorted_cost))
bars_cost = ax.barh(df_sorted_cost["Solution"], df_sorted_cost["Cost_INR"], color=palette_cost)

plt.xlabel("Estimated Hardware Cost (INR) [Log Scale]", fontsize=11, fontweight='bold')
plt.title("Hardware Cost Benchmark Across 15 Navigation Solutions (in ₹)", fontsize=13, fontweight='bold', pad=15)
plt.xscale("log")
plt.xlim(1000, 10000000)

for bar in bars_cost:
    width = bar.get_width()
    label_text = f" ₹{width/1000:.1f}k" if width < 100000 else f" ₹{width/100000:.2f} Lakhs"
    ax.text(width * 1.05, bar.get_y() + bar.get_height()/2, label_text, ha='left', va='center', fontsize=8.5, weight='bold')

plt.tight_layout()
plt.savefig("3_cost_benchmark_inr.png", dpi=300)
plt.show()

# ---------------------------------------------------------
# Chart 4: Environmental Robustness Heatmap
# ---------------------------------------------------------
robustness_data = {
    "2D LiDAR + Optical Flow": [5, 2, 5, 5, 5],
    "Stereo / Three-Cam V-SLAM": [1, 1, 2, 5, 5],
    "2D LiDAR + Monocular RGB": [5, 2, 3, 5, 5],
    "Event-Based Camera Odometry": [2, 1, 5, 5, 5],
    "Visual SLAM": [1, 1, 2, 5, 5],
    "Acoustic / Ultrasonic Array": [5, 3, 5, 5, 2],
    "Optical Flow + Rangefinder": [2, 1, 3, 5, 5],
    "ArUco / Fiducial Tags": [2, 2, 3, 5, 1],
    "Depth Camera (RPi 5)": [2, 1, 3, 5, 5],
    "3D LiDAR SLAM (RPi 5)": [5, 2, 5, 5, 5],
    "FMCW mmWave Radar": [5, 5, 5, 5, 5],
    "UWB / RF Beacons": [5, 5, 5, 1, 1],
    "Geophysical (MAGNAV/TERCOM)": [5, 5, 5, 5, 2]
}

categories = ["Zero Light", "Smoke/Dust", "Visual Glare", "RF Jamming", "Unmapped Space"]
df_rob = pd.DataFrame(robustness_data, index=categories).T

fig, ax = plt.subplots(figsize=(11, 7))
sns.heatmap(
    df_rob, 
    annot=True, 
    cmap="YlGnBu", 
    cbar_kws={'label': 'Robustness Score (1=Fail, 5=Immune)'}, 
    vmin=1, 
    vmax=5, 
    linewidths=1, 
    ax=ax
)

plt.title("Environmental Robustness Comparison Across Failure Modes", fontsize=13, fontweight='bold', pad=15)
plt.xlabel("Operational Challenge / Failure Mode", fontsize=11)
plt.ylabel("Navigation Modality", fontsize=11)

plt.tight_layout()
plt.savefig("4_robustness_heatmap.png", dpi=300)
plt.show()