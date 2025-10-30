# Custom Plot Generation from Analysis Results

## Overview

The `plot_from_summary.py` script creates publication-quality plots from your saved analysis results without needing to re-run the full analysis.

## Features

- **High resolution**: 300 DPI output (vs 150 DPI in analyzer)
- **Zoomed X-axis**: ±50cm range for better detail
- **Customizable styling**: Easier to modify for your needs
- **Multiple outputs**:
  - Individual best frame plots
  - Individual worst frame plots
  - Side-by-side comparison plot

## Usage

### Basic Usage

```bash
cd ~/ros2_ws/src/windrow_centerline_node/eval
python3 plot_from_summary.py
```

This looks for `./centerline_analysis/analysis_summary.json` by default.

### Custom Path

```bash
python3 plot_from_summary.py /path/to/your/analysis_summary.json
```

## Output Files

The script generates the following files in the same directory as the JSON:

### Individual Plots
- `custom_best_01_score_X.XXX.png` - Best agreement frame #1
- `custom_best_02_score_X.XXX.png` - Best agreement frame #2
- `custom_best_03_score_X.XXX.png` - Best agreement frame #3
- `custom_worst_01_score_X.XXX.png` - Worst agreement frame #1
- `custom_worst_02_score_X.XXX.png` - Worst agreement frame #2
- `custom_worst_03_score_X.XXX.png` - Worst agreement frame #3

### Comparison Plot
- `comparison_best_vs_worst.png` - Side-by-side: worst (left) vs best (right)

## Plot Specifications

### X-axis Range
- **Range**: -0.5m to +0.5m (±50cm)
- Better zoom than original ±1m range

### Font Sizes
- **Title (main)**: 28pt
- **Title (score/timestamp)**: 22pt (smaller than main title)
- **Axis labels**: 24pt
- **Legend**: 22pt
- **Tick labels**: 20pt

### Resolution
- **DPI**: 300 (publication quality)
- **Format**: PNG

### Styling
- **ZED**: Blue line/points
- **Ouster**: Red line/points
- **Line width**: 3pt
- **Point size**: 40
- **Grid**: Light alpha=0.3

## Customization

You can easily modify the script to:

### Change X-axis limits
```python
# Line 67 and 107
ax.set_xlim(-0.5, 0.5)  # Change to your desired range
```

### Change font sizes
```python
# Lines 57-65
ax.set_xlabel('X (m) - Lateral', fontsize=24)  # Modify fontsize
```

### Change colors
```python
# Lines 44-45
ax.plot(zed_x, zed_y, 'b-', ...)  # Change 'b-' to different color
```

### Change output resolution
```python
# Line 75
plt.savefig(output_path, dpi=300, ...)  # Change dpi value
```

## Example Workflow

1. **Run your analysis** (generates `analysis_summary.json`)
   ```bash
   python3 centerline_agreement_analyzer.py
   ```

2. **Generate custom plots**
   ```bash
   python3 plot_from_summary.py
   ```

3. **Check output**
   ```bash
   ls -lh centerline_analysis/custom_*.png
   ls -lh centerline_analysis/comparison_*.png
   ```

4. **Use in paper/presentation**
   - High-resolution plots ready for publication
   - Consistent styling across all figures
   - Easy to regenerate if you need changes

## Comparison with Analyzer Plots

| Feature | Analyzer | plot_from_summary.py |
|---------|----------|---------------------|
| DPI | 150 | 300 |
| X-axis range | ±1m | ±50cm |
| Title style | Single line, 28pt | Two lines, 28pt + 22pt |
| Regeneration | Requires full replay | Instant from JSON |
| Customization | Edit node code | Simple Python script |

## Tips

1. **For papers**: Use `comparison_best_vs_worst.png` for a nice figure showing both cases

2. **For presentations**: Use individual plots with custom titles

3. **Quick iterations**: Modify the script and re-run - no need to replay bags!

4. **Multiple analyses**: Point to different JSON files to compare datasets

## Troubleshooting

**Error: "Could not find analysis_summary.json"**
- Make sure you've run the analyzer first
- Or specify the full path to your JSON file

**Plots look wrong**
- Check that your JSON file has the `zed_centerline` and `ouster_centerline` fields
- These were added in recent versions of the analyzer

**Need different styling**
- Edit the script! It's designed to be easy to customize
- All plotting code is in the `plot_frame()` and `plot_comparison()` functions

