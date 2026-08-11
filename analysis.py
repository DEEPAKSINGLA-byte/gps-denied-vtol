import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

# Set global aesthetic theme
sns.set_theme(style="whitegrid")
plt.rcParams.update({'font.sans-serif': 'DejaVu Sans', 'font.family': 'sans-serif'})

# ---------------------------------------------------------
# 1. Master Dataset (20 GPS-Denied UAV Navigation Solutions)
# ---------------------------------------------------------
data = [
    {"Solution": "VIO", "Drift_Mid": 1.0, "Power_Mid": 8.5, "Payload_Mid": 55, "Cost_INR": 55000, "Category": "Visual"},
    {"Solution": "Visual SLAM", "Drift_Mid": 0.3, "Power_Mid": 17.5, "Payload_Mid": 85, "Cost_INR": 97500, "Category": "Visual"},
    {"Solution": "Optical Flow + ToF", "Drift_Mid": 3.5, "Power_Mid": 2.0, "Payload_Mid": 25, "Cost_INR": 7750, "Category": "Visual"},
    {"Solution": "DSMAC", "Drift_Mid": 0.3, "Power_Mid": 14.0, "Payload_Mid": 140, "Cost_INR": 165000, "Category": "Visual"},
    {"Solution": "Event Camera VIO", "Drift_Mid": 1.4, "Power_Mid": 5.5, "Payload_Mid": 42.5, "Cost_INR": 575000, "Category": "Visual"},
    {"Solution": "3D LiDAR SLAM", "Drift_Mid": 0.125, "Power_Mid": 30.0, "Payload_Mid": 725, "Cost_INR": 1125000, "Category": "LiDAR/Range"},
    {"Solution": "mmWave Radar", "Drift_Mid": 2.0, "Power_Mid": 7.0, "Payload_Mid": 120, "Cost_INR": 77500, "Category": "LiDAR/Range"},
    {"Solution": "UWB Anchors", "Drift_Mid": 0.01, "Power_Mid": 2.0, "Payload_Mid": 20, "Cost_INR": 30000, "Category": "RF/Beacons"},
    {"Solution": "Acoustic Array", "Drift_Mid": 2.5, "Power_Mid": 1.25, "Payload_Mid": 27.5, "Cost_INR": 16500, "Category": "LiDAR/Range"},
    {"Solution": "MAGNAV", "Drift_Mid": 0.6, "Power_Mid": 3.5, "Payload_Mid": 70, "Cost_INR": 40000, "Category": "Geophysical"},
    {"Solution": "TERCOM / SITAN", "Drift_Mid": 0.45, "Power_Mid": 10.0, "Payload_Mid": 225, "Cost_INR": 235000, "Category": "Geophysical"},
    {"Solution": "Star Tracker", "Drift_Mid": 0.05, "Power_Mid": 5.5, "Payload_Mid": 165, "Cost_INR": 500000, "Category": "Geophysical"},
    {"Solution": "Signals of Opportunity", "Drift_Mid": 3.0, "Power_Mid": 5.5, "Payload_Mid": 80, "Cost_INR": 34000, "Category": "RF/Beacons"},
    {"Solution": "Pseudolites", "Drift_Mid": 0.01, "Power_Mid": 3.5, "Payload_Mid": 35, "Cost_INR": 325000, "Category": "RF/Beacons"},
    {"Solution": "DoA RF Homing", "Drift_Mid": 2.5, "Power_Mid": 4.0, "Payload_Mid": 60, "Cost_INR": 52500, "Category": "RF/Beacons"},
    {"Solution": "Tactical FOG INS", "Drift_Mid": 0.05, "Power_Mid": 40.0, "Payload_Mid": 2150, "Cost_INR": 2850000, "Category": "Inertial/Other"},
    {"Solution": "Tethered Link", "Drift_Mid": 0.001, "Power_Mid": 0.1, "Payload_Mid": 300, "Cost_INR": 95000, "Category": "Inertial/Other"},
    {"Solution": "ZUPT Perching", "Drift_Mid": 0.001, "Power_Mid": 0.5, "Payload_Mid": 20, "Cost_INR": 11500, "Category": "Inertial/Other"},
    {"Solution": "Swarm Mesh", "Drift_Mid": 1.25, "Power_Mid": 6.5, "Payload_Mid": 47.5, "Cost_INR": 42500, "Category": "Inertial/Other"},
    {"Solution": "Deep Inertial Odometry", "Drift_Mid": 2.0, "Power_Mid": 10.0, "Payload_Mid": 35, "Cost_INR": 60000, "Category": "Inertial/Other"}
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
    if row["Solution"] in ["VIO", "3D LiDAR SLAM", "UWB Anchors", "Tactical FOG INS", "mmWave Radar", "Optical Flow + ToF"]:
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
plt.title("Position Drift Rate Benchmark Across All 20 Solutions", fontsize=13, fontweight='bold', pad=15)
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
plt.title("Hardware Cost Benchmark Across 20 Navigation Solutions (in ₹)", fontsize=13, fontweight='bold', pad=15)
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
    "Visual (VIO / V-SLAM)": [1, 1, 2, 5, 5],
    "Event Camera VIO": [2, 1, 5, 5, 5],
    "3D LiDAR SLAM": [5, 2, 5, 5, 5],
    "FMCW mmWave Radar": [5, 5, 5, 5, 5],
    "UWB / RF Beacons": [5, 5, 5, 1, 1],
    "Geophysical (MAGNAV/TERCOM)": [5, 5, 5, 5, 2],
    "Tactical FOG INS": [5, 5, 5, 5, 5],
    "Deep Inertial Odometry": [5, 5, 5, 5, 4]
}

categories = ["Zero Light", "Smoke/Dust", "Visual Glare", "RF Jamming", "Unmapped Space"]
df_rob = pd.DataFrame(robustness_data, index=categories).T

fig, ax = plt.subplots(figsize=(9.5, 5.5))
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