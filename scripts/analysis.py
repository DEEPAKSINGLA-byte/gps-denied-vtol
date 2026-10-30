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
    {"Solution": "2D LiDAR + Optical Flow", "Drift_Mid": 1.4, "Power_Mid": 7.0, "Payload_Mid": 150, "Cost_INR": 12500, "Update_Hz": 75, "Range_Mid": 10, "Category": "LiDAR/Range"},
    {"Solution": "Stereo Cam V-SLAM (RPi 5)", "Drift_Mid": 0.3, "Power_Mid": 10.5, "Payload_Mid": 85, "Cost_INR": 18000, "Update_Hz": 40, "Range_Mid": 1000, "Category": "Visual"},
    {"Solution": "2D LiDAR + Monocular RGB", "Drift_Mid": 1.5, "Power_Mid": 8.0, "Payload_Mid": 140, "Cost_INR": 15500, "Update_Hz": 35, "Range_Mid": 10, "Category": "LiDAR/Range"},
    {"Solution": "Event-Based Camera Odometry", "Drift_Mid": 1.4, "Power_Mid": 5.5, "Payload_Mid": 42.5, "Cost_INR": 45000, "Update_Hz": 750, "Range_Mid": 100000, "Category": "Visual"},
    {"Solution": "Visual SLAM", "Drift_Mid": 0.3, "Power_Mid": 17.5, "Payload_Mid": 85, "Cost_INR": 12000, "Update_Hz": 40, "Range_Mid": 1000, "Category": "Visual"},
    {"Solution": "Acoustic / Ultrasonic Array", "Drift_Mid": 2.5, "Power_Mid": 1.25, "Payload_Mid": 27.5, "Cost_INR": 16500, "Update_Hz": 30, "Range_Mid": 6, "Category": "LiDAR/Range"},
    {"Solution": "Optical Flow + Rangefinder", "Drift_Mid": 3.5, "Power_Mid": 2.0, "Payload_Mid": 25, "Cost_INR": 7750, "Update_Hz": 75, "Range_Mid": 12.5, "Category": "Visual"},
    {"Solution": "ArUco / Fiducial Tags", "Drift_Mid": 0.005, "Power_Mid": 2.0, "Payload_Mid": 20, "Cost_INR": 5000, "Update_Hz": 20, "Range_Mid": 30, "Category": "Visual"},
    {"Solution": "Three RGB Cameras (RPi 5)", "Drift_Mid": 0.65, "Power_Mid": 13.0, "Payload_Mid": 105, "Cost_INR": 14000, "Update_Hz": 22.5, "Range_Mid": 100000, "Category": "Visual"},
    {"Solution": "Depth Camera (RPi 5)", "Drift_Mid": 1.0, "Power_Mid": 10.0, "Payload_Mid": 70, "Cost_INR": 26000, "Update_Hz": 22.5, "Range_Mid": 3, "Category": "Visual"},
    {"Solution": "3D LiDAR SLAM (RPi 5)", "Drift_Mid": 0.125, "Power_Mid": 22.5, "Payload_Mid": 375, "Cost_INR": 425000, "Update_Hz": 15, "Range_Mid": 5000, "Category": "LiDAR/Range"},
    {"Solution": "FMCW mmWave Radar", "Drift_Mid": 2.0, "Power_Mid": 7.0, "Payload_Mid": 120, "Cost_INR": 77500, "Update_Hz": 35, "Range_Mid": 100000, "Category": "LiDAR/Range"},
    {"Solution": "UWB Beacon Triangulation", "Drift_Mid": 0.005, "Power_Mid": 2.0, "Payload_Mid": 20, "Cost_INR": 30000, "Update_Hz": 75, "Range_Mid": 150, "Category": "RF/Beacons"},
    {"Solution": "MAGNAV", "Drift_Mid": 0.6, "Power_Mid": 3.5, "Payload_Mid": 70, "Cost_INR": 40000, "Update_Hz": 20, "Range_Mid": 100000, "Category": "Geophysical"},
    {"Solution": "TERCOM / SITAN", "Drift_Mid": 0.45, "Power_Mid": 10.0, "Payload_Mid": 225, "Cost_INR": 235000, "Update_Hz": 12.5, "Range_Mid": 100000, "Category": "Geophysical"}
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

# ---------------------------------------------------------
# Chart 5: Cost-Effectiveness (Drift vs. Cost)
# ---------------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 6))
sns.scatterplot(
    data=df, 
    x="Cost_INR", 
    y="Drift_Mid", 
    hue="Category", 
    size="Power_Mid", 
    sizes=(40, 400), 
    palette="Set2", 
    alpha=0.85, 
    ax=ax
)

plt.xscale("log")
plt.yscale("log")
plt.title("Cost-Effectiveness: Position Drift vs. Hardware Cost (Bubble Size = Power Draw W)", fontsize=13, fontweight='bold', pad=15)
plt.xlabel("Estimated Hardware Cost (INR) [Log Scale]", fontsize=11)
plt.ylabel("Position Drift Rate (% Distance) [Log Scale]", fontsize=11)

for _, row in df.iterrows():
    if row["Solution"] in ["3D LiDAR SLAM (RPi 5)", "UWB Beacon Triangulation", "FMCW mmWave Radar", "Optical Flow + Rangefinder", "ArUco / Fiducial Tags", "TERCOM / SITAN"]:
        ax.annotate(
            row["Solution"], 
            (row["Cost_INR"], row["Drift_Mid"]),
            textcoords="offset points", 
            xytext=(5, 5), 
            ha='left', 
            fontsize=9, 
            weight='bold'
        )

plt.tight_layout()
plt.savefig("5_cost_effectiveness.png", dpi=300)
plt.show()

# ---------------------------------------------------------
# Chart 6: Drift vs. Payload Mass
# ---------------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 6))
sns.scatterplot(
    data=df, 
    x="Payload_Mid", 
    y="Drift_Mid", 
    hue="Category", 
    size="Cost_INR", 
    sizes=(40, 400), 
    palette="Set2", 
    alpha=0.85, 
    ax=ax
)

plt.xscale("log")
plt.yscale("log")
plt.title("Weight Penalty for Accuracy: Drift vs. Payload Mass (Bubble Size = Cost INR)", fontsize=13, fontweight='bold', pad=15)
plt.xlabel("Payload Mass (g) [Log Scale]", fontsize=11)
plt.ylabel("Position Drift Rate (% Distance) [Log Scale]", fontsize=11)

for _, row in df.iterrows():
    if row["Solution"] in ["3D LiDAR SLAM (RPi 5)", "TERCOM / SITAN", "Optical Flow + Rangefinder", "UWB Beacon Triangulation", "Acoustic / Ultrasonic Array"]:
        ax.annotate(
            row["Solution"], 
            (row["Payload_Mid"], row["Drift_Mid"]),
            textcoords="offset points", 
            xytext=(5, 5), 
            ha='left', 
            fontsize=9, 
            weight='bold'
        )

plt.tight_layout()
plt.savefig("6_drift_vs_payload.png", dpi=300)
plt.show()

# ---------------------------------------------------------
# Chart 7: Total Robustness Score (Bar Chart)
# ---------------------------------------------------------
rob_sum = df_rob.sum(axis=1).sort_values(ascending=True)

fig, ax = plt.subplots(figsize=(10, 6.5))
palette_rob = sns.color_palette("flare", len(rob_sum))
bars_rob = ax.barh(rob_sum.index, rob_sum.values, color=palette_rob)

plt.xlabel("Total Robustness Score (Sum of 5 Failure-Mode Scores, Max 25)", fontsize=11)
plt.title("Overall Environmental Robustness Ranking (Sum Across Failure Modes)", fontsize=13, fontweight='bold', pad=15)
plt.xlim(0, 25)

for bar in bars_rob:
    width = bar.get_width()
    ax.text(width + 0.15, bar.get_y() + bar.get_height()/2, f"{width:.0f}", ha='left', va='center', fontsize=9, weight='bold')

plt.tight_layout()
plt.savefig("7_robustness_total.png", dpi=300)
plt.show()

# ---------------------------------------------------------
# Chart 8: Cost Distribution per Category (Boxplot)
# ---------------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 6))
sns.boxplot(data=df, x="Category", y="Cost_INR", palette="Set2", ax=ax)
sns.stripplot(data=df, x="Category", y="Cost_INR", color="black", alpha=0.5, size=5, ax=ax)

plt.yscale("log")
plt.title("Hardware Cost Distribution by Solution Category (Log Scale)", fontsize=13, fontweight='bold', pad=15)
plt.xlabel("Solution Category", fontsize=11)
plt.ylabel("Estimated Hardware Cost (INR) [Log Scale]", fontsize=11)
plt.xticks(rotation=15)

plt.tight_layout()
plt.savefig("8_cost_by_category.png", dpi=300)
plt.show()

# ---------------------------------------------------------
# Chart 9: Radar / Spider Chart for Key Modalities
# ---------------------------------------------------------
radar_solutions = ["FMCW mmWave Radar", "UWB Beacon Triangulation", "3D LiDAR SLAM (RPi 5)", "Optical Flow + Rangefinder"]

rob_series = df_rob.sum(axis=1)
rob_key_map = {
    "FMCW mmWave Radar": "FMCW mmWave Radar",
    "UWB Beacon Triangulation": "UWB / RF Beacons",
    "3D LiDAR SLAM (RPi 5)": "3D LiDAR SLAM (RPi 5)",
    "Optical Flow + Rangefinder": "Optical Flow + Rangefinder"
}

def normalize(value, vmin, vmax, invert=False):
    v = (value - vmin) / (vmax - vmin)
    return 1.0 - v if invert else v

drift_min, drift_max = df["Drift_Mid"].min(), df["Drift_Mid"].max()
power_max, payload_max = df["Power_Mid"].max(), df["Payload_Mid"].max()
cost_max, range_max = df["Cost_INR"].max(), df["Range_Mid"].max()
update_max = df["Update_Hz"].max()

radar_labels = ["Accuracy", "Update Rate", "Operational Range", "Low Power", "Light Payload", "Affordability", "Robustness"]

fig, axes = plt.subplots(2, 2, subplot_kw=dict(polar=True), figsize=(12, 10))
for ax, sol in zip(axes.flatten(), radar_solutions):
    row = df[df["Solution"] == sol].iloc[0]
    values = [
        normalize(row["Drift_Mid"], drift_min, drift_max, invert=True),
        normalize(row["Update_Hz"], 0, update_max),
        normalize(row["Range_Mid"], 0, range_max),
        normalize(row["Power_Mid"], 0, power_max, invert=True),
        normalize(row["Payload_Mid"], 0, payload_max, invert=True),
        normalize(row["Cost_INR"], 0, cost_max, invert=True),
        normalize(rob_series.loc[rob_key_map[sol]], 0, 25)
    ]
    angles = [n / float(len(radar_labels)) * 2 * 3.141592653589793 for n in range(len(radar_labels))]
    values += values[:1]
    angles += angles[:1]
    ax.set_title(sol, fontsize=11, fontweight='bold', pad=20)
    ax.plot(angles, values, linewidth=2, linestyle='solid', color="tab:blue")
    ax.fill(angles, values, alpha=0.25, color="tab:blue")
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(radar_labels, fontsize=8)
    ax.set_ylim(0, 1)
    ax.set_yticks([0.25, 0.5, 0.75])

fig.suptitle("Normalized Multi-Criteria Radar Profiles for Key Navigation Modalities", fontsize=14, fontweight='bold', y=1.02)
plt.tight_layout()
plt.savefig("9_radar_profiles.png", dpi=300)
plt.show()

# ---------------------------------------------------------
# Chart 10: Update Rate vs. Drift
# ---------------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 6))
sns.scatterplot(
    data=df, 
    x="Update_Hz", 
    y="Drift_Mid", 
    hue="Category", 
    size="Payload_Mid", 
    sizes=(40, 400), 
    palette="Set2", 
    alpha=0.85, 
    ax=ax
)

plt.xscale("log")
plt.yscale("log")
plt.title("State Update Rate vs. Position Drift (Bubble Size = Payload Mass g)", fontsize=13, fontweight='bold', pad=15)
plt.xlabel("Update Frequency (Hz) [Log Scale]", fontsize=11)
plt.ylabel("Position Drift Rate (% Distance) [Log Scale]", fontsize=11)

for _, row in df.iterrows():
    if row["Solution"] in ["Event-Based Camera Odometry", "3D LiDAR SLAM (RPi 5)", "Acoustic / Ultrasonic Array", "UWB Beacon Triangulation"]:
        ax.annotate(
            row["Solution"], 
            (row["Update_Hz"], row["Drift_Mid"]),
            textcoords="offset points", 
            xytext=(5, 5), 
            ha='left', 
            fontsize=9, 
            weight='bold'
        )

plt.tight_layout()
plt.savefig("10_update_rate_vs_drift.png", dpi=300)
plt.show()

# ---------------------------------------------------------
# Chart 11: Operational Range vs. Drift
# ---------------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 6))
sns.scatterplot(
    data=df, 
    x="Range_Mid", 
    y="Drift_Mid", 
    hue="Category", 
    size="Power_Mid", 
    sizes=(40, 400), 
    palette="Set2", 
    alpha=0.85, 
    ax=ax
)

plt.xscale("log")
plt.yscale("log")
plt.title("Operational Range vs. Position Drift (Bubble Size = Power Draw W)", fontsize=13, fontweight='bold', pad=15)
plt.xlabel("Operational Range (m) [Log Scale]", fontsize=11)
plt.ylabel("Position Drift Rate (% Distance) [Log Scale]", fontsize=11)

for _, row in df.iterrows():
    if row["Solution"] in ["Depth Camera (RPi 5)", "Acoustic / Ultrasonic Array", "3D LiDAR SLAM (RPi 5)", "UWB Beacon Triangulation"]:
        ax.annotate(
            row["Solution"], 
            (row["Range_Mid"], row["Drift_Mid"]),
            textcoords="offset points", 
            xytext=(5, 5), 
            ha='left', 
            fontsize=9, 
            weight='bold'
        )

plt.tight_layout()
plt.savefig("11_range_vs_drift.png", dpi=300)
plt.show()

# ---------------------------------------------------------
# Chart 12: MCDA Weighted Scenario Scores
# ---------------------------------------------------------
scenarios = {
    "Defense / Contested": [
        ("FMCW mmWave Radar Odometry", 8.75),
        ("3D LiDAR SLAM (RPi 5)", 8.30),
        ("Event-Camera Odometry", 7.95)
    ],
    "Underground Inspection": [
        ("3D LiDAR SLAM (RPi 5)", 9.10),
        ("FMCW mmWave Radar Odometry", 8.40),
        ("Depth Camera (RPi 5, IR)", 7.80)
    ],
    "Sub-250g Micro UAV": [
        ("Optical Flow + Rangefinder", 9.15),
        ("UWB Beacon Triangulation", 8.90),
        ("ArUco / Fiducial Tag Tracking", 8.35)
    ]
}

fig, axes = plt.subplots(1, 3, figsize=(15, 6))
colors = ["#2ca02c", "#1f77b4", "#d62728"]

for ax, (scenario, entries) in zip(axes, scenarios.items()):
    names = [e[0] for e in entries][::-1]
    scores = [e[1] for e in entries][::-1]
    ax.barh(names, scores, color=colors)
    ax.set_xlim(0, 10)
    ax.set_xlabel("MCDA Score (0-10)", fontsize=10)
    ax.set_title(scenario, fontsize=11, fontweight='bold')
    for i, s in enumerate(scores):
        ax.text(s + 0.05, i, f"{s:.2f}", ha='left', va='center', fontsize=9, weight='bold')
    ax.tick_params(axis='y', labelsize=8.5)

fig.suptitle("MCDA Top-3 Ranked Solutions by Mission Profile", fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig("12_mcda_scenarios.png", dpi=300)
plt.show()