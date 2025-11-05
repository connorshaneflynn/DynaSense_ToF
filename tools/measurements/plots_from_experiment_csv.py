import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path


import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

def plot_tof_csv(csv_path, save=False, save_path=None, range_limit=None, show_labels=False):
    """
    Plot error and standard deviation vs. true distance.
    
    Arguments:
        csv_path (str | Path): Path to CSV file.
        save (bool): If True, save the plot. Default False.
        save_path (str | Path | None): Optional explicit output path.
        range_limit (tuple | None): Optional (min, max) range for True Distance axis.
    """
    csv_path = Path(csv_path)
    df = pd.read_csv(csv_path)

    # Ensure numeric types
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors='coerce')

    # Apply range filter if provided
    if range_limit:
        mask = (df["True Distance"] >= range_limit[0]) & (df["True Distance"] <= range_limit[1])
        df = df[mask]
        plot_min, plot_max = range_limit
    else:
        plot_min, plot_max = df["True Distance"].min(), df["True Distance"].max()

    # Plot Error and Std. Deviation
    fig, ax1 = plt.subplots(figsize=(8, 5))

    ax1.plot(df["True Distance"], df["Error"], marker='o', label='Error (Measured - True)', color='tab:blue')
    ax1.set_xlabel("True Distance [mm]")
    ax1.set_ylabel("Error [mm]", color='tab:blue')
    ax1.tick_params(axis='y', labelcolor='tab:blue')
    ax1.set_xlim(plot_min, plot_max)

    if show_labels:
        for x, y in zip(df["True Distance"], df["Error"]):
            ax1.text(x, y+0.2, f"{y:.2f}", fontsize=7, color='tab:blue', ha='center', va='bottom')

    # Twin y-axis for standard deviation
    ax2 = ax1.twinx()
    ax2.plot(df["True Distance"], df["Std. Deviation"], marker='s', color='tab:green', label='Std. Deviation')
    ax2.set_ylabel("Standard Deviation [mm]", color='tab:green')
    ax2.tick_params(axis='y', labelcolor='tab:green')
    ax2.set_xlim(plot_min, plot_max)

    if show_labels:
        for x, y in zip(df["True Distance"], df["Std. Deviation"]):
            ax2.text(x, y+0.05, f"{y:.2f}", fontsize=7, color='tab:green', ha='center', va='bottom')


    # Titles and grid
    plt.suptitle("ToF Sensor Performance vs True Distance")
    plt.title(csv_path.name, fontsize=9)
    plt.grid(True)
    fig.tight_layout(rect=(0, 0, 1, 0.95))

    # Combined legend
    lines, labels = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines + lines2, labels + labels2, loc='upper left')

    # Save logic
    if save:
        if save_path is None:
            suffix = f"_{int(plot_min)}-{int(plot_max)}mm" if range_limit else ""
            save_path = csv_path.with_stem(csv_path.stem + suffix).with_suffix(".png")
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {save_path}")

    # plt.show()


def plot_tof_csv_multi(csv_paths, names=None, save=False, save_path=None, save_name=None, range_limit=None, show_labels=False):
    """
    Plot error and standard deviation vs. true distance for multiple CSV files in a single plot.

    Arguments:
        csv_paths (list[str | Path]): List of CSV file paths.
        names (list[str] | None): Optional list of names for each dataset.
        save (bool): If True, save the plot. Default False.
        save_path (str | Path | None): Optional explicit output path.
        save_name (str | None): Optional custom name for the saved file (without extension) if save_path=None.
        range_limit (tuple | None): Optional (min, max) range for True Distance axis.
    """
    if isinstance(csv_paths, (str, Path)):
        csv_paths = [csv_paths]
    if names is None:
        names = [Path(p).stem for p in csv_paths]
    if len(names) != len(csv_paths):
        raise ValueError("Length of 'names' must match length of 'csv_paths'.")

    # Define separate colormaps for error and deviation
    blue_cmap = plt.get_cmap('Blues', len(csv_paths)+3)  # +3 to avoid very light colors
    green_cmap = plt.get_cmap('Greens', len(csv_paths)+3)

    fig, ax1 = plt.subplots(figsize=(8, 5))
    ax2 = ax1.twinx()

    all_true = []
    all_error = []

    for idx, (csv_path, label) in enumerate(zip(csv_paths, names)):
        csv_path = Path(csv_path)
        df = pd.read_csv(csv_path)

        # Ensure numeric types
        for col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce')

        # Apply range filter
        if range_limit:
            mask = (df["True Distance"] >= range_limit[0]) & (df["True Distance"] <= range_limit[1])
            df = df[mask]

        true = df["True Distance"]
        error = df["Error"]
        std = df["Std. Deviation"]

        all_true.append(true)
        all_error.append(error)

        # colors
        c_err = blue_cmap(idx + 2)
        c_std = green_cmap(idx + 2)

        # Plot data
        ax1.plot(true, error, color=c_err, label=f"{label} Error (Measured - True)")
        ax2.plot(true, std, color=c_std, label=f"{label} Std. Deviation")

        if show_labels:
            for x, y in zip(true, error):
                ax1.text(x, y+0.2, f"{y:.2f}", fontsize=7, color=c_err, ha='center', va='bottom')
            for x, y in zip(true, std):
                ax2.text(x, y+0.05, f"{y:.2f}", fontsize=7, color=c_std, ha='center', va='bottom')


    # Determine axis limits
    all_true_concat = pd.concat(all_true)
    if range_limit:
        plot_min, plot_max = map(float, range_limit)
    else:
        plot_min = float(all_true_concat.to_numpy().min())
        plot_max = float(all_true_concat.to_numpy().max())

    ax1.set_xlim(plot_min, plot_max)
    ax2.set_xlim(plot_min, plot_max)

    # Axis labels
    ax1.set_xlabel("True Distance [mm]")
    ax1.set_ylabel("Error [mm]", color='tab:blue')
    ax2.set_ylabel("Standard Deviation [mm]", color='tab:green')
    ax1.tick_params(axis='y', labelcolor='tab:blue')
    ax2.tick_params(axis='y', labelcolor='tab:green')

    # Titles and layout
    plt.suptitle("ToF Sensor Performance vs True Distance")
    plt.grid(True)
    fig.tight_layout(rect=(0, 0, 1, 0.95))

    # Add subtitle with file name
    if save_name:
        plt.title(save_name, fontsize=9)

    # Combined legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')

    # Save logic
    if save:
        if save_path is None:
            name_stem = save_name if save_name else "multi_tof_plot"
            suffix = f"_{int(plot_min)}-{int(plot_max)}mm" if range_limit else ""
            save_path = Path(csv_paths[0]).with_stem(name_stem + suffix).with_suffix(".png")
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {save_path}")

    # plt.show()


def plot_tof_deviation(csv_path, save=False, save_path=None, range_limit=None, show_labels=False):
    """
    Plot measured vs. true distance with:
    - Ideal diagonal (y = x)
    - Measured points
    - Standard deviation as error bars and shaded area
    
    Arguments:
        csv_path (str | Path): Path to CSV file.
        save (bool): If True, save the plot. Default False.
        save_path (str | Path | None): Optional explicit output path.
        range_limit (tuple | None): Optional (min, max) range for True Distance axis.
    """
    csv_path = Path(csv_path)
    df = pd.read_csv(csv_path)
    df = df.apply(pd.to_numeric, errors='coerce')
    
    true = df["True Distance"]
    measured = df["Measured Distance"]
    std = df["Std. Deviation"]
    
    # Determine plot range
    if range_limit:
        mask = (true >= range_limit[0]) & (true <= range_limit[1])
        true = true[mask]
        measured = measured[mask]
        std = std[mask]
        plot_min, plot_max = range_limit
    else:
        plot_min, plot_max = 0, max(true.max(), measured.max()) * 1.05
    
    fig, ax = plt.subplots(figsize=(6,6))
    
    # Ideal diagonal
    ax.plot([plot_min, plot_max], [plot_min, plot_max], 'k--', label="Ideal (y = x)")
    
    # Measured points and std. dev.
    ax.errorbar(true, measured, yerr=std, fmt='o', capsize=3, color='tab:blue', label='Measured ± Std. Dev.')
    ax.fill_between(true, measured - std, measured + std, color='tab:blue', alpha=0.2)

    if show_labels:
        for i, (x, y, s) in enumerate(zip(true, measured, std)):
            offset = 4.0
            ax.text(x, y+offset, f"σ={s:.2f}", fontsize=7, color='tab:blue', ha='center', va='bottom')

    
    # Labels and style
    ax.set_xlabel("True Distance [mm]")
    ax.set_ylabel("Measured Distance [mm]")
    plt.suptitle("Measured vs True Distance with Standard Deviation")
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)
    ax.legend()
    
    # Add subtitle with file name
    subtitle = csv_path.name
    ax.set_title(subtitle, fontsize=9)
    
    plt.tight_layout(rect=(0, 0, 1, 0.95))
    
    # Save logic
    if save:
        if save_path is None:
            save_path = csv_path.with_stem(csv_path.stem + "_deviation" + f"_{int(plot_min)}-{int(plot_max)}mm").with_suffix(".png")
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {save_path}")
    
    # plt.show()

def plot_tof_deviation_multi(csv_paths, names=None, save=False, save_path=None, save_name=None, range_limit=None, show_labels=False):
    """
    Plot multiple ToF datasets (measured vs. true distance) in a single figure.

    Each dataset is shown with:
      - Ideal diagonal (y = x)
      - Measured points
      - Standard deviation as error bars and shaded area

    Args:
        csv_paths (list[str | Path]): List of CSV file paths.
        names (list[str] | None): Optional names for each dataset (for legend).
        save (bool): Whether to save the plot. Default: False.
        save_path (str | Path | None): Path for saving if `save=True`.
        save_name (str | None): Optional custom name for the saved file (without extension) if save_path=None.
        range_limit (tuple | None): (min, max) for the True Distance axis.
    """
    if names is None:
        names = [Path(p).stem for p in csv_paths]

    # Prepare the figure
    fig, ax = plt.subplots(figsize=(6, 6))

    all_true = []
    all_measured = []

    cmap = plt.get_cmap("tab10")
    colors = [cmap(i) for i in range(len(csv_paths))]

    for i, (csv_path, name, color) in enumerate(zip(csv_paths, names, colors)):
        csv_path = Path(csv_path)
        df = pd.read_csv(csv_path)
        df = df.apply(pd.to_numeric, errors='coerce').dropna(subset=["Measured Distance"])

        true = df["True Distance"]
        measured = df["Measured Distance"]
        std = df["Std. Deviation"]

        if range_limit:
            mask = (true >= range_limit[0]) & (true <= range_limit[1])
            true = true[mask]
            measured = measured[mask]
            std = std[mask]

        ax.errorbar(true, measured, yerr=std, fmt='o', capsize=3, label=f"{name} ± Std.Dev.", color=color)
        ax.fill_between(true, measured - std, measured + std, color=color, alpha=0.15)

        if show_labels:
            offset = (10.0 if not range_limit else range_limit[1]*0.08)
            offset = (offset*(i+1) if i%2==0 else -offset*i)  # alternate label position and move outwards
            for x, y, s in zip(true, measured, std):
                ax.text(x+offset, y, f"σ={s:.2f}", fontsize=7, color=color, ha='center', va='bottom')


        all_true.append(true)
        all_measured.append(measured)

    # Determine plot limits
    all_true = pd.concat(all_true)
    all_measured = pd.concat(all_measured)
    plot_min = float(range_limit[0]) if range_limit else 0.0
    plot_max = float(range_limit[1]) if range_limit else float(max(all_true.to_numpy().max(), all_measured.to_numpy().max()) * 1.05)

    # Plot ideal diagonal (force floats)
    ax.plot([plot_min, plot_max], [plot_min, plot_max], linestyle='--', color='black', label="Ideal (y = x)")

    # Labels, title, and style
    ax.set_xlabel("True Distance [mm]")
    ax.set_ylabel("Measured Distance [mm]")
    plt.suptitle("Measured vs True Distance with Standard Deviation")
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)
    ax.legend()
    plt.tight_layout(rect=(0, 0, 1, 0.95))

    # Add subtitle with file name
    if save_name:
        plt.title(save_name, fontsize=9)

    # Save logic
    if save:
        if save_path is None:
            name_stem = save_name if save_name else "multi_tof_plot"
            suffix = f"_{int(plot_min)}-{int(plot_max)}mm" if range_limit else ""
            save_path = Path(csv_paths[0]).with_stem(name_stem + "_deviation" + suffix).with_suffix(".png")
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {save_path}")

    # plt.show()
    

### Single Plots ###

file_path = Path("experiment_logs/Sensor1_1.a.C_distance_alu.csv")

plot_tof_deviation(file_path, save=True, range_limit=(0, 750), show_labels=True)
plot_tof_csv(file_path, save=True, range_limit=(0, 750), show_labels=True)

plot_tof_deviation(file_path, save=True, range_limit=(0, 350), show_labels=True)
plot_tof_csv(file_path, save=True, range_limit=(0, 350), show_labels=True)

plot_tof_deviation(file_path, save=True, range_limit=(0, 100), show_labels=True)
plot_tof_csv(file_path, save=True, range_limit=(0, 100), show_labels=True)

plt.show()

### Multi Plots ###

# file_paths = [
#     Path("experiment_logs/Sensor1_1.a.D_distance_metal_center.csv"),
#     Path("experiment_logs/Sensor1_1.a.D_distance_metal_outer.csv"),
#     # Path("experiment_logs/Sensor1_1.a.D_distance_metal_outer.csv")
# ]
# save_name = "Sensor1_1.a.D_distance_metal_combined"

# plot_tof_deviation_multi(file_paths, names=["Center", "Outer"], save=True, save_name=save_name, range_limit=(0, 750), show_labels=True)
# plot_tof_deviation_multi(file_paths, names=["Center", "Outer"], save=True, save_name=save_name, range_limit=(0, 350), show_labels=True)
# plot_tof_deviation_multi(file_paths, names=["Center", "Outer"], save=True, save_name=save_name, range_limit=(0, 100), show_labels=True)

# plot_tof_csv_multi(file_paths, names=["Center", "Outer"], save=True, save_name=save_name, range_limit=(0, 750), show_labels=True)
# plot_tof_csv_multi(file_paths, names=["Center", "Outer"], save=True, save_name=save_name, range_limit=(0, 350), show_labels=True)
# plot_tof_csv_multi(file_paths, names=["Center", "Outer"], save=True, save_name=save_name, range_limit=(0, 100), show_labels=True)

# plt.show()