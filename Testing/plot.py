import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("output.csv")

# Convert timestamp to seconds starting from 0
df["time"] = (df["timestamp"] - df["timestamp"].iloc[0]) / 1000.0

# ----------------------------------------------------------------
# Define plot groups — add new groups here as outputs are added
# Each group: (title, [ (col_suffix, label, color), ... ])
# ----------------------------------------------------------------
channel_bases = ["ax", "ay", "az", "gx", "gy", "gz"]

output_styles = {
    "raw":     ("Raw",      "blue",   0.5),
    "filt":    ("Filtered", "orange", 1.0),
    # Future additions:
    # "squared": ("Squared",  "green",  1.0),
    # "mwi":     ("MWI",      "red",    1.0),
}

# ----------------------------------------------------------------
# Plot each channel base (ax, ay, etc.) as its own figure
# ----------------------------------------------------------------
for ch in channel_bases:
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.set_title(f"{ch}: All Outputs")

    for suffix, (label, color, alpha) in output_styles.items():
        col = f"{ch}_{suffix}"
        if col in df.columns:
            ax.plot(df["time"], df[col], label=label, alpha=alpha, linewidth=1.5)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ch)
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.savefig(f"{ch}_plot.png", dpi=150)
    plt.show()

    