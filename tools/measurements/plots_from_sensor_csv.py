import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

class ToFDataAnalyzer:
    def __init__(self, csv_path: str):
        self.csv_path = Path(csv_path)
        self.df = self._load_data()

    def _load_data(self):
        df = pd.read_csv(self.csv_path)
        dist_cols = [c for c in df.columns if c.startswith("dist_")]
        status_cols = [c for c in df.columns if c.startswith("status_")]
        df[dist_cols] = df[dist_cols].astype(np.float32)
        df[status_cols] = df[status_cols].astype(np.int8)
        return df

    def _get_zone_columns(self):
        dist_cols = [c for c in self.df.columns if c.startswith("dist_")]
        status_cols = [c for c in self.df.columns if c.startswith("status_")]
        return dist_cols, status_cols
    
    def _save_results(self, df_out, fig, name, out_dir):
        out_csv = out_dir / f"{name}.csv"
        out_png = out_dir / f"{name}.png"
        df_out.to_csv(out_csv, index=False)
        fig.savefig(out_png, dpi=300, bbox_inches="tight")
        print(f"Saved: {out_png.name}, {out_csv.name}")
    
    def _bin_data(self, distances, values, dist_range, bin_size):
        bins = np.arange(dist_range[0], dist_range[1] + bin_size, bin_size)
        bin_indices = np.digitize(distances, bins)
        binned = {b: [] for b in range(1, len(bins))}
        for d, v, bi in zip(distances, values, bin_indices):
            if 1 <= bi < len(bins):
                binned[bi].append(v)
        return bins, binned
    
    # ------------------------ Noise Curves ------------------------

    @staticmethod
    def rolling_noise_curve(distances, window=200):
        s = pd.Series(distances).sort_values(ignore_index=True)
        means = s.rolling(window, center=True).mean()
        vars_ = s.rolling(window, center=True).var(ddof=1)
        return means, vars_
    
    def rolling_noise_curve_by_distance(self, distances, window_mm=50):
        s = pd.Series(distances).sort_values(ignore_index=True)
        means, vars_ = [], []
        step = max(1, len(s)//2000)  # control resolution
        for i in range(0, len(s), step):
            center = s[i]
            in_window = s[(s >= center - window_mm/2) & (s <= center + window_mm/2)]
            if len(in_window) > 1:
                means.append(center)
                vars_.append(np.var(in_window, ddof=1))
        return np.array(means), np.array(vars_)
    
    def noise_by_static_segments(self, distances, segment_tolerance=10, min_len=10):
        """
        Compute sensor noise by identifying segments where distance is nearly constant.
        
        distances: 1D array of measured distances over time
        segment_tolerance: max change in mm to consider as "same segment"
        min_len: minimum number of samples in a segment to compute variance
        """
        distances = np.array(distances)
        segments = []
        start = 0
        for i in range(1, len(distances)):
            if abs(distances[i] - distances[i-1]) > segment_tolerance:
                if i - start >= min_len:
                    seg = distances[start:i]
                    segments.append((np.mean(seg), np.var(seg, ddof=1)))
                start = i
        # handle last segment
        if len(distances) - start >= min_len:
            seg = distances[start:]
            segments.append((np.mean(seg), np.var(seg, ddof=1)))
        means, vars_ = zip(*segments) if segments else ([], [])
        return np.array(means), np.array(vars_)


    def _noise_curve_combined(self, window):
        dist_cols, _ = self._get_zone_columns()
        all_dist = self.df[dist_cols].values.flatten()
        all_dist = all_dist[~np.isnan(all_dist)]
        mean_d, var_d = self.rolling_noise_curve_by_distance(all_dist, window)
        return pd.DataFrame({"distance_mm": mean_d, "variance_mm2": var_d})

    def _noise_curve_per_zone(self, window):
        dist_cols, _ = self._get_zone_columns()
        all_dfs = []
        for zone, col in enumerate(dist_cols):
            d = self.df[col].dropna().astype(float)
            mean_d, var_d = self.rolling_noise_curve_by_distance(d, window)
            df_zone = pd.DataFrame({
                "distance_mm": mean_d,
                "variance_mm2": var_d,
                "zone": zone
            })
            all_dfs.append(df_zone)
        return pd.concat(all_dfs, ignore_index=True)
    
    def plot_noise_per_distance(self, dist_range=(0, 4000), window_mm=10, combine_zones=True):
        if combine_zones:
            df_out = self._noise_curve_combined(window_mm)
            df_out = df_out[df_out["distance_mm"].between(dist_range[0], dist_range[1])]
            fig, ax = plt.subplots()
            ax.plot(df_out["distance_mm"], df_out["variance_mm2"], color="black", label="Combined")
        else:
            df_out = self._noise_curve_per_zone(window_mm)
            df_out = df_out[df_out["distance_mm"].between(dist_range[0], dist_range[1])]
            fig, ax = plt.subplots()
            for zone, group in df_out.groupby("zone"):
                ax.plot(group["distance_mm"], group["variance_mm2"], alpha=0.6, label=f"Zone {zone}")

        df_out = df_out[df_out["distance_mm"].between(dist_range[0], dist_range[1])]

        ax.set_xlabel("Distance [mm]")
        ax.set_ylabel("Variance [mm²]")
        fig.suptitle(self.csv_path.name, fontsize=10)
        ax.set_title(f"Rolling Noise Curve ({window_mm} samples)")
        ax.grid(True)
        ax.legend()
        self._save_results(df_out, fig, "rolling_noise_curve", self.csv_path.parent)


    def plot_noise_per_distance_(self, dist_range=(0, 4000), bin_size=10, plot_all = False):
        dist_cols, _ = self._get_zone_columns()
        zone_variances = {}
        combined_variances = []

        for zone, col in enumerate(dist_cols):
            distances = self.df[col].dropna()
            bins, binned = self._bin_data(distances, distances, dist_range, bin_size)
            variances = [np.var(binned[b]) if binned[b] else np.nan for b in range(1, len(bins))]
            zone_variances[zone] = variances
            combined_variances.append(variances)

        combined_variances_mean = np.nanmean(combined_variances, axis=0)

        plt.figure()
        if plot_all:
            for zone, v in zone_variances.items():
                plt.plot(bins[:-1], v, alpha=0.3, label=f"Zone {zone}")
        plt.plot(bins[:-1], combined_variances_mean, color="black", label="Mean Noise", linewidth=2)
        plt.xlabel("Distance [mm]")
        plt.ylabel("Variance [mm²]")
        plt.suptitle(f"Sensor Noise per Distance Bin ({bin_size} mm bins)")
        plt.title(f"({self.csv_path.name})", fontsize=10)
        # plt.legend()
        plt.grid(True)
        self._save_results("noise_per_distance", bins, combined_variances_mean, "variance_mm2")

        # plt.ion()
        # plt.show(block=False)
        # plt.pause(0.1)

        # print("Combined noise variance per distance bin:")
        # for b, v in zip(bins[:-1], combined_mean):
        #     print(f"Distance {b:5.0f} mm: Variance = {v:.2f}")


# ------------------------ Validity Curves ------------------------

    @staticmethod
    def map_validity(status_series):
        return status_series.map({5: 1.0, 9: 0.5, 10: 0.5}).fillna(0.0)

    def _validity_curve_combined(self, window):
        dist_cols, status_cols = self._get_zone_columns()
        distances = self.df[dist_cols].values.flatten()
        statuses = self.df[status_cols].values.flatten()
        mask = ~np.isnan(distances)
        distances = distances[mask]
        statuses = statuses[mask]
        weights = self.map_validity(pd.Series(statuses))
        df_sorted = pd.DataFrame({"dist": distances, "w": weights}).sort_values("dist", ignore_index=True)
        mean_d = df_sorted["dist"].rolling(window, center=True).mean()
        mean_w = df_sorted["w"].rolling(window, center=True).mean()
        return pd.DataFrame({"distance_mm": mean_d, "validity_factor": mean_w})

    def _validity_curve_per_zone(self, window):
        dist_cols, status_cols = self._get_zone_columns()
        all_dfs = []
        for zone, (dist_col, stat_col) in enumerate(zip(dist_cols, status_cols)):
            distances = self.df[dist_col].dropna().astype(float)
            statuses = self.df[stat_col].dropna().astype(float)
            weights = self.map_validity(statuses)
            df_zone = pd.DataFrame({"dist": distances, "w": weights}).sort_values("dist", ignore_index=True)
            mean_d = df_zone["dist"].rolling(window, center=True).mean()
            mean_w = df_zone["w"].rolling(window, center=True).mean()
            all_dfs.append(pd.DataFrame({
                "distance_mm": mean_d,
                "validity_factor": mean_w,
                "zone": zone
            }))
        return pd.concat(all_dfs, ignore_index=True)

    def plot_validity_per_distance(self, dist_range=(0, 4000), window=200, combine_zones=True):
        if combine_zones:
            df_out = self._validity_curve_combined(window)
            df_out = df_out[df_out["distance_mm"].between(dist_range[0], dist_range[1])]
            fig, ax = plt.subplots()
            ax.plot(df_out["distance_mm"], df_out["validity_factor"], color="black", label="Combined")
        else:
            df_out = self._validity_curve_per_zone(window)
            df_out = df_out[df_out["distance_mm"].between(dist_range[0], dist_range[1])]
            fig, ax = plt.subplots()
            for zone, group in df_out.groupby("zone"):
                ax.plot(group["distance_mm"], group["validity_factor"], alpha=0.6, label=f"Zone {zone}")

        ax.set_xlabel("Distance [mm]")
        ax.set_ylabel("Validity Factor")
        ax.set_title(f"Rolling Validity Curve ({window} samples)")
        fig.suptitle(self.csv_path.name, fontsize=10)
        ax.grid(True)
        ax.legend()
        self._save_results(df_out, fig, "rolling_validity_curve", self.csv_path.parent)

    def plot_validity_per_distance_(self, dist_range=(0, 4000), bin_size=10, plot_all=False, zones=None):
        """
        Plot validity factor per distance bin, optionally for specific range and zones.
        dist_range: (min, max) distance in mm to consider
        bin_size: size of each distance bin in mm
        plot_all: if True, plot individual zone curves; otherwise combine them
        zones: 'center', 'outer, or None for all zones
        """
        dist_cols, status_cols = self._get_zone_columns()
        zone_validity = {}
        combined_validity = []

        # Extract Specific Zones
        match zones:
            case "center":
                center_idx = [5, 6, 9, 10]
                dist_cols = [dist_cols[idx] for idx in center_idx]
                status_cols = [status_cols[idx] for idx in center_idx]
            case "outer":
                outer_idx = [0, 1, 2, 3, 4, 7, 8, 11, 12, 13, 14, 15]
                dist_cols = [dist_cols[idx] for idx in outer_idx]
                status_cols = [status_cols[idx] for idx in outer_idx]
            case _: # all zones
                pass

        for zone, (dist_col, status_col) in enumerate(zip(dist_cols, status_cols)):
            distances = self.df[dist_col]
            statuses = self.df[status_col]
            weights = statuses.map({5: 1.0, 9: 1.0, 10: 1.0}).fillna(0.0)
            bins, binned = self._bin_data(distances, weights, dist_range, bin_size)
            validity = [np.nanmean(binned[b]) if binned[b] else np.nan for b in range(1, len(bins))]
            zone_validity[zone] = validity
            combined_validity.append(validity)

        combined_validity_mean = np.nanmean(combined_validity, axis=0)

        # --- Create plot ---
        fig, ax = plt.subplots()
        if plot_all:
            for zone, v in zone_validity.items():
                ax.plot(bins[:-1], v, alpha=0.3, label=f"Zone {zone}")
        ax.plot(bins[:-1], combined_validity_mean, color="black", label="Mean Validity", linewidth=2)
        ax.set_xlabel("Distance [mm]")
        ax.set_ylabel("Validity Factor")
        ax.grid(True)
        ax.legend()
        ax.set_title(self.csv_path.name, fontsize=10)
        fig.suptitle(f"Measurement Validity per Distance Bin ({bin_size} mm bins)")

        # --- Prepare DataFrame for saving ---
        df_out = pd.DataFrame({
            "distance_mm": bins[:-1],
            "validity_factor": combined_validity_mean
        })

        # --- Save plot and data ---
        self._save_results(df_out, fig, "validity_per_distance", self.csv_path.parent)

        
    # ------------------------ Manual Measurements ------------------------
    def _get_zone_data(self, zones):
        """Return list of valid numpy arrays (one per selected zone)."""
        dist_cols, status_cols = self._get_zone_columns()

        # --- zone selection ---
        if isinstance(zones, list):
            dist_cols = [dist_cols[idx] for idx in zones]
            status_cols = [status_cols[idx] for idx in zones]
        else:
            match zones:
                case "center":
                    idx = [5, 6, 9, 10]
                case "outer":
                    idx = [0, 1, 2, 3, 4, 7, 8, 11, 12, 13, 14, 15]
                case "corners":
                    idx = [0, 3, 12, 15]
                case "edges":
                    idx = [1, 2, 4, 7, 8, 11, 13, 14]
                case _:
                    idx = list(range(len(dist_cols)))
            dist_cols = [dist_cols[i] for i in idx]
            status_cols = [status_cols[i] for i in idx]

        # --- gather valid values per zone ---
        zone_data = []
        for dcol, scol in zip(dist_cols, status_cols):
            vals = self.df.loc[self.df[scol].isin([5, 9, 10]), dcol].dropna().to_numpy()
            if vals.size:
                zone_data.append(vals)
        return zone_data


    def analyze_static_distance_error(self, zones=None, print_results=True):
        """
        Compute per-zone mean and variance, then average them.
        Removes invalid measurements (using status).
        """
        zone_data = self._get_zone_data(zones)
        if not zone_data:
            if print_results:
                print(f"No valid data in {zones if zones else 'all'} zones.")
            return

        zone_means = [vals.mean() for vals in zone_data]
        zone_vars = [vals.var(ddof=1) for vals in zone_data]
        mean_val = np.mean(zone_means)
        var_val = np.mean(zone_vars)
        std_val = np.sqrt(var_val)
        total_samples = sum(len(v) for v in zone_data)

        if print_results:
            region = zones if zones else "all"
            print(f"\nManual Static Measurement Analysis ({region} zones):")
            print(f"Number of samples: {total_samples}")
            print(f"Mean distance: {mean_val:.2f} mm")
            print(f"Variance: {var_val:.2f} mm²")
            print(f"Standard deviation: {std_val:.2f} mm")

        return mean_val, var_val, std_val


    def analyze_static_distance_percentiles(
        self,
        zones=None,
        percentages=(0.5, 0.68, 0.9, 0.99, 1.0),
        plot=False,
        print_results=True,
        save_plot=False
    ):
        """
        Computes symmetric ± distance ranges that include a given percentage of valid data,
        after removing per-zone mean offsets.
        0.68 is 1 Std. Dev. if assuming normal distribution
        """
        import matplotlib.pyplot as plt

        zone_data = self._get_zone_data(zones)
        if not zone_data:
            if print_results:
                print(f"No valid data in {zones if zones else 'all'} zones.")
            return

        # remove per-zone mean
        residuals = np.concatenate([vals - vals.mean() for vals in zone_data])
        residuals.sort()

        mean = np.mean([vals.mean() for vals in zone_data])

        # calculate symmetric ± ranges
        results = {}
        for p in percentages:
            if p >= 1.0:
                lower, upper = residuals.min(), residuals.max()
            else:
                lo_q = (1 - p) / 2 * 100
                hi_q = (1 + p) / 2 * 100
                lower = np.percentile(residuals, lo_q)
                upper = np.percentile(residuals, hi_q)
            results[p] = float((upper - lower) / 2)

        if print_results:
            region = zones if zones else "all"
            print(f"\nStatic Distance Percentile Analysis ({region} zones, mean-removed):")
            print(f"Number of valid samples: {len(residuals)}")
            for p, half in results.items():
                print(f"{int(p*100)}% of data within ±{half:.2f} mm")

        if plot:
            plt.figure()
            plt.hist(residuals, bins=60, alpha=0.7)
            plt.xlabel("Residual distance (mm, mean-removed)")
            plt.ylabel("Count")
            plt.suptitle(f"Distribution of residuals ({zones if zones else 'all'} zones)")
            plt.title(f"Avg. Distance {round(mean)} mm")
            plt.grid(True, alpha=0.3)
            if save_plot:
                out_png = self.csv_path.parent / f"residuals_hist_{zones if zones else 'all'}_{round(mean)}mm.png"
                plt.savefig(out_png, dpi=300, bbox_inches="tight")
                print(f"Saved: {out_png.name}")
            plt.show()

        return results


    def show_plots(self):
        plt.show()



analyzer = ToFDataAnalyzer(r"sensor_logs/sensor_2_white_100mm.csv")
# analyzer.df.info()
# analyzer.plot_noise_per_distance(dist_range=(0, 1000), window_mm=60, combine_zones=False)
# analyzer.plot_validity_per_distance(dist_range=(0, 1000), window=60, combine_zones=True)

# analyzer.plot_validity_per_distance_(dist_range=(0, 100), bin_size=5, plot_all = True, zones='')
# analyzer.plot_validity_per_distance_(dist_range=(0, 350), bin_size=5, plot_all = True, zones='')
# analyzer.plot_validity_per_distance_(dist_range=(0, 1000), bin_size=5, plot_all = True, zones='')
# analyzer.show_plots()

analyzer.analyze_static_distance_error(None)
analyzer.analyze_static_distance_error("center")
analyzer.analyze_static_distance_error("outer")

zone_results = np.zeros((16,3))
for z in range(16):
    zone_results[z, :] = analyzer.analyze_static_distance_error([z], print_results=False)

print("\nStandard Deviation per Zone:")
for i, z in enumerate(zone_results[:,2]):
    print(f"Std.Dev {i:02d}:\t{z:.2f}")

print("\nDistance per Zone:")
for i, z in enumerate(zone_results[:,0]):
    print(f"Dist {i:02d}:\t{z:.2f}")



tests = ['center', 'edges', 'corners']
zone_results = np.zeros((16,3))
for t in tests:
    analyzer.analyze_static_distance_error(t)


analyzer.analyze_static_distance_percentiles(zones=None, plot=True, save_plot=True)
