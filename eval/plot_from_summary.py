#!/usr/bin/env python3
"""
Standalone script to create publication-quality plots from analysis_summary.json

Usage:
    python3 plot_from_summary.py [path_to_analysis_summary.json]
    
If no path is provided, looks for ./centerline_analysis/analysis_summary.json
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import sys
import os


def load_summary(json_path):
    """Load the analysis summary JSON file"""
    with open(json_path, 'r') as f:
        return json.load(f)


def plot_frame(zed_centerline, ouster_centerline, score, timestamp, 
               output_path, frame_type=""):
    """Create a single frame plot with custom styling"""
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Plot ZED centerline (BLUE)
    if zed_centerline:
        zed_x, zed_y = zip(*zed_centerline)
        ax.plot(zed_x, zed_y, 'b-', linewidth=3, label='ZED', alpha=0.7)
        ax.scatter(zed_x, zed_y, c='blue', s=40, alpha=0.6, edgecolors='darkblue', linewidth=0.5)
    
    # Plot Ouster centerline (RED)
    if ouster_centerline:
        oust_x, oust_y = zip(*ouster_centerline)
        ax.plot(oust_x, oust_y, 'r-', linewidth=3, label='Ouster', alpha=0.7)
        ax.scatter(oust_x, oust_y, c='red', s=40, alpha=0.6, edgecolors='darkred', linewidth=0.5)
    
    # Labels and title
    ax.set_xlabel('X (m) - Lateral', fontsize=24)
    ax.set_ylabel('Y (m) - Forward', fontsize=24)
    
    # Title with score and timestamp (same style as analyzer)
    ax.set_title(f'Agreement Score: {score:.3f} | Timestamp: {timestamp:.2f}s', 
                fontsize=22, fontweight='bold')
    
    ax.legend(fontsize=22, loc='best')
    ax.grid(True, alpha=0.3)
    
    # Set X-axis limits to ±50cm
    ax.set_xlim(-0.5, 0.5)
    
    # Increase tick label size
    ax.tick_params(axis='both', which='major', labelsize=20)
    
    # Keep aspect ratio but allow different scales for x and y
    ax.set_aspect('auto')
    
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"Saved: {output_path}")


def plot_comparison(best_frame, worst_frame, output_path):
    """Create a side-by-side comparison plot"""
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))
    
    # Plot worst frame (left)
    if worst_frame['zed_centerline']:
        zed_x, zed_y = zip(*worst_frame['zed_centerline'])
        ax1.plot(zed_x, zed_y, 'b-', linewidth=3, label='ZED', alpha=0.7)
        ax1.scatter(zed_x, zed_y, c='blue', s=40, alpha=0.6, edgecolors='darkblue', linewidth=0.5)
    
    if worst_frame['ouster_centerline']:
        oust_x, oust_y = zip(*worst_frame['ouster_centerline'])
        ax1.plot(oust_x, oust_y, 'r-', linewidth=3, label='Ouster', alpha=0.7)
        ax1.scatter(oust_x, oust_y, c='red', s=40, alpha=0.6, edgecolors='darkred', linewidth=0.5)
    
    ax1.set_xlabel('X (m) - Lateral', fontsize=24)
    ax1.set_ylabel('Y (m) - Forward', fontsize=24)
    ax1.set_title(f'Agreement Score: {worst_frame["score"]:.3f} | Timestamp: {worst_frame["timestamp"]:.2f}s', 
                  fontsize=22, fontweight='bold')
    ax1.legend(fontsize=22)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(-0.5, 0.5)
    ax1.tick_params(axis='both', which='major', labelsize=20)
    ax1.set_aspect('auto')
    
    # Plot best frame (right)
    if best_frame['zed_centerline']:
        zed_x, zed_y = zip(*best_frame['zed_centerline'])
        ax2.plot(zed_x, zed_y, 'b-', linewidth=3, label='ZED', alpha=0.7)
        ax2.scatter(zed_x, zed_y, c='blue', s=40, alpha=0.6, edgecolors='darkblue', linewidth=0.5)
    
    if best_frame['ouster_centerline']:
        oust_x, oust_y = zip(*best_frame['ouster_centerline'])
        ax2.plot(oust_x, oust_y, 'r-', linewidth=3, label='Ouster', alpha=0.7)
        ax2.scatter(oust_x, oust_y, c='red', s=40, alpha=0.6, edgecolors='darkred', linewidth=0.5)
    
    ax2.set_xlabel('X (m) - Lateral', fontsize=24)
    ax2.set_ylabel('Y (m) - Forward', fontsize=24)
    ax2.set_title(f'Agreement Score: {best_frame["score"]:.3f} | Timestamp: {best_frame["timestamp"]:.2f}s', 
                  fontsize=22, fontweight='bold')
    ax2.legend(fontsize=22)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(-0.5, 0.5)
    ax2.tick_params(axis='both', which='major', labelsize=20)
    ax2.set_aspect('auto')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"Saved comparison: {output_path}")


def main():
    # Get JSON path from command line or use default
    if len(sys.argv) > 1:
        json_path = sys.argv[1]
    else:
        json_path = './centerline_analysis/analysis_summary.json'
    
    if not os.path.exists(json_path):
        print(f"Error: Could not find {json_path}")
        print("Usage: python3 plot_from_summary.py [path_to_analysis_summary.json]")
        sys.exit(1)
    
    # Load the summary
    print(f"Loading {json_path}...")
    summary = load_summary(json_path)
    
    # Get output directory (same as JSON location)
    output_dir = os.path.dirname(json_path)
    if not output_dir:
        output_dir = '.'
    
    # Plot best frames
    print("\nGenerating best frame plots...")
    for bf in summary['best_frames']:
        output_path = os.path.join(output_dir, 
                                   f'custom_best_{bf["rank"]:02d}_score_{bf["score"]:.3f}.png')
        plot_frame(bf['zed_centerline'], bf['ouster_centerline'], 
                  bf['score'], bf['timestamp'], output_path, 
                  f"Best Agreement #{bf['rank']}")
    
    # Plot worst frames
    print("\nGenerating worst frame plots...")
    for wf in summary['worst_frames']:
        output_path = os.path.join(output_dir,
                                   f'custom_worst_{wf["rank"]:02d}_score_{wf["score"]:.3f}.png')
        plot_frame(wf['zed_centerline'], wf['ouster_centerline'],
                  wf['score'], wf['timestamp'], output_path,
                  f"Worst Agreement #{wf['rank']}")
    
    # Create side-by-side comparison of #1 best and #1 worst
    if summary['best_frames'] and summary['worst_frames']:
        print("\nGenerating comparison plot...")
        comparison_path = os.path.join(output_dir, 'comparison_best_vs_worst.png')
        plot_comparison(summary['best_frames'][0], summary['worst_frames'][0], 
                       comparison_path)
    
    # Print summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    print(f"Total frames analyzed: {summary['total_frames']}")
    print(f"Mean agreement score: {summary['mean_score']:.4f}")
    print(f"\nBest score:  {summary['max_score']:.4f}")
    print(f"Worst score: {summary['min_score']:.4f}")
    print(f"\nPlots saved to: {output_dir}/")
    print("="*60)


if __name__ == '__main__':
    main()

