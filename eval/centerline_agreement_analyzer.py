#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import json
from datetime import datetime
import os

class CenterlineAgreementAnalyzer(Node):
    def __init__(self):
        super().__init__('centerline_agreement_analyzer')
        
        # Declare parameters
        self.declare_parameter('output_dir', './centerline_analysis')
        self.declare_parameter('save_all_frames', False)
        self.declare_parameter('top_n_best', 3)
        self.declare_parameter('top_n_worst', 3)
        
        self.output_dir = self.get_parameter('output_dir').value
        self.save_all_frames = self.get_parameter('save_all_frames').value
        self.top_n_best = self.get_parameter('top_n_best').value
        self.top_n_worst = self.get_parameter('top_n_worst').value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.zed_centerline = None
        self.ouster_centerline = None
        self.zed_timestamp = None
        self.ouster_timestamp = None
        
        self.zed_sub = self.create_subscription(
            PoseArray, '/zed/windrow_centerline',
            self.zed_callback, 10)
        
        self.ouster_sub = self.create_subscription(
            PoseArray, '/ouster/windrow_centerline',
            self.ouster_callback, 10)
        
        # Timer to analyze and plot
        self.timer = self.create_timer(1.0, self.analyze_agreement)
        
        self.frame_count = 0
        
        # Track all scores and metadata
        self.scores_history = []  # List of (score, timestamp, frame_count)
        self.best_frames = []  # List of (score, timestamp, frame_count, zed_centerline, ouster_centerline)
        self.worst_frames = []  # List of (score, timestamp, frame_count, zed_centerline, ouster_centerline)
        
        # Track lateral offsets (X positions relative to base_link)
        self.zed_x_offsets = []  # List of X positions for ZED centerline
        self.ouster_x_offsets = []  # List of X positions for Ouster centerline
        
    def zed_callback(self, msg):
        self.zed_centerline = [(p.position.x, p.position.y) 
                               for p in msg.poses]
        self.zed_timestamp = msg.header.stamp
    
    def ouster_callback(self, msg):
        self.ouster_centerline = [(p.position.x, p.position.y) 
                                  for p in msg.poses]
        self.ouster_timestamp = msg.header.stamp
    
    def analyze_agreement(self):
        if self.zed_centerline is None or self.ouster_centerline is None:
            return
        
        # Use the most recent timestamp (or average them)
        if self.zed_timestamp and self.ouster_timestamp:
            # Use ouster timestamp as reference
            timestamp_sec = self.ouster_timestamp.sec + self.ouster_timestamp.nanosec * 1e-9
        else:
            timestamp_sec = self.get_clock().now().seconds_nanoseconds()[0]
        
        # Calculate agreement metrics
        agreement_score = self.compute_agreement()
        
        # Track lateral offsets (X positions) for each sensor
        if self.zed_centerline:
            zed_x_values = [x for x, y in self.zed_centerline]
            if zed_x_values:
                self.zed_x_offsets.extend(zed_x_values)
        
        if self.ouster_centerline:
            ouster_x_values = [x for x, y in self.ouster_centerline]
            if ouster_x_values:
                self.ouster_x_offsets.extend(ouster_x_values)
        
        # Store score and timestamp
        self.scores_history.append({
            'score': agreement_score,
            'timestamp': timestamp_sec,
            'frame': self.frame_count
        })
        
        # Update best and worst frames
        self.update_best_worst_frames(agreement_score, timestamp_sec)
        
        # Plot top-down view (conditionally)
        if self.save_all_frames:
            self.plot_overlay(agreement_score, timestamp_sec, save=True)
        
        self.frame_count += 1
        
        # Log progress every 10 frames
        if self.frame_count % 10 == 0:
            zed_avg = np.mean(self.zed_x_offsets) if self.zed_x_offsets else 0.0
            ouster_avg = np.mean(self.ouster_x_offsets) if self.ouster_x_offsets else 0.0
            self.get_logger().info(
                f'Processed {self.frame_count} frames. '
                f'Score: {agreement_score:.3f} (Best: {max(self.scores_history, key=lambda x: x["score"])["score"]:.3f}, '
                f'Worst: {min(self.scores_history, key=lambda x: x["score"])["score"]:.3f}) | '
                f'Avg Offset: ZED={zed_avg:+.3f}m, Ouster={ouster_avg:+.3f}m'
            )
    
    def update_best_worst_frames(self, score, timestamp):
        """Update lists of best and worst frames"""
        frame_data = {
            'score': score,
            'timestamp': timestamp,
            'frame': self.frame_count,
            'zed_centerline': list(self.zed_centerline),
            'ouster_centerline': list(self.ouster_centerline)
        }
        
        # Update best frames
        self.best_frames.append(frame_data)
        self.best_frames.sort(key=lambda x: x['score'], reverse=True)
        self.best_frames = self.best_frames[:self.top_n_best]
        
        # Update worst frames
        self.worst_frames.append(frame_data)
        self.worst_frames.sort(key=lambda x: x['score'])
        self.worst_frames = self.worst_frames[:self.top_n_worst]
    
    def compute_agreement(self):
        """Compute agreement score between centerlines"""
        if not self.zed_centerline or not self.ouster_centerline:
            return 0.0
        
        distances = []
        for zx, zy in self.zed_centerline:
            min_dist = float('inf')
            for ox, oy in self.ouster_centerline:
                dist = np.sqrt((zx - ox)**2 + (zy - oy)**2)
                min_dist = min(min_dist, dist)
            distances.append(min_dist)
        
        avg_distance = np.mean(distances)
        
        # Agreement score: closer = better
        # 1.0 = perfect, 0.0 = >1m difference
        score = max(0.0, 1.0 - avg_distance)
        
        return score
    
    def plot_overlay(self, score, timestamp, save=False, filename=None):
        """Create top-down overlay plot"""
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Plot ZED centerline (BLUE)
        if self.zed_centerline:
            zed_x, zed_y = zip(*self.zed_centerline)
            ax.plot(zed_x, zed_y, 'b-', linewidth=2, label='ZED', alpha=0.7)
            ax.scatter(zed_x, zed_y, c='blue', s=20, alpha=0.5)
        
        # Plot Ouster centerline (RED)
        if self.ouster_centerline:
            oust_x, oust_y = zip(*self.ouster_centerline)
            ax.plot(oust_x, oust_y, 'r-', linewidth=2, label='LiDAR', alpha=0.7)
            ax.scatter(oust_x, oust_y, c='red', s=20, alpha=0.5)
        
        ax.set_xlabel('X (m) - Lateral', fontsize=12)
        ax.set_ylabel('Y (m) - Forward', fontsize=12)
        ax.set_title(f'Agreement Score: {score:.3f} | Timestamp: {timestamp:.2f}s', 
                    fontsize=14, fontweight='bold')
        ax.legend(fontsize=11)
        ax.grid(True, alpha=0.3)
        
        # Set X-axis limits to ±1m for better zoom
        ax.set_xlim(-1.0, 1.0)
        
        # Keep aspect ratio but allow different scales for x and y
        ax.set_aspect('auto')
        
        if save:
            if filename is None:
                filename = f'centerline_overlay_frame_{self.frame_count:04d}.png'
            filepath = os.path.join(self.output_dir, filename)
            plt.savefig(filepath, dpi=150, bbox_inches='tight')
            self.get_logger().info(f'Saved frame {self.frame_count}, score={score:.3f}')
        
        plt.close()
    
    def save_best_worst_frames(self):
        """Save plots of best and worst frames"""
        self.get_logger().info('=' * 80)
        self.get_logger().info('Saving best and worst frames...')
        
        # Save best frames
        for i, frame_data in enumerate(self.best_frames):
            self.get_logger().info(
                f'Best #{i+1}: Score={frame_data["score"]:.3f}, '
                f'Frame={frame_data["frame"]}, '
                f'Timestamp={frame_data["timestamp"]:.2f}s'
            )
            
            # Restore centerlines for plotting
            self.zed_centerline = frame_data['zed_centerline']
            self.ouster_centerline = frame_data['ouster_centerline']
            
            filename = f'best_{i+1:02d}_frame_{frame_data["frame"]:04d}_score_{frame_data["score"]:.3f}.png'
            self.plot_overlay(frame_data['score'], frame_data['timestamp'], 
                            save=True, filename=filename)
        
        # Save worst frames
        for i, frame_data in enumerate(self.worst_frames):
            self.get_logger().info(
                f'Worst #{i+1}: Score={frame_data["score"]:.3f}, '
                f'Frame={frame_data["frame"]}, '
                f'Timestamp={frame_data["timestamp"]:.2f}s'
            )
            
            # Restore centerlines for plotting
            self.zed_centerline = frame_data['zed_centerline']
            self.ouster_centerline = frame_data['ouster_centerline']
            
            filename = f'worst_{i+1:02d}_frame_{frame_data["frame"]:04d}_score_{frame_data["score"]:.3f}.png'
            self.plot_overlay(frame_data['score'], frame_data['timestamp'], 
                            save=True, filename=filename)
    
    def generate_summary_report(self):
        """Generate a summary JSON and text report"""
        if not self.scores_history:
            self.get_logger().warn('No frames processed, cannot generate summary')
            return
        
        scores = [s['score'] for s in self.scores_history]
        
        # Calculate lateral offset statistics
        zed_mean_offset = float(np.mean(self.zed_x_offsets)) if self.zed_x_offsets else 0.0
        zed_std_offset = float(np.std(self.zed_x_offsets)) if self.zed_x_offsets else 0.0
        ouster_mean_offset = float(np.mean(self.ouster_x_offsets)) if self.ouster_x_offsets else 0.0
        ouster_std_offset = float(np.std(self.ouster_x_offsets)) if self.ouster_x_offsets else 0.0
        
        summary = {
            'total_frames': len(self.scores_history),
            'mean_score': float(np.mean(scores)),
            'median_score': float(np.median(scores)),
            'std_score': float(np.std(scores)),
            'min_score': float(np.min(scores)),
            'max_score': float(np.max(scores)),
            'lateral_offsets': {
                'zed': {
                    'mean': zed_mean_offset,
                    'std': zed_std_offset,
                    'total_points': len(self.zed_x_offsets)
                },
                'ouster': {
                    'mean': ouster_mean_offset,
                    'std': ouster_std_offset,
                    'total_points': len(self.ouster_x_offsets)
                }
            },
            'best_frames': [
                {
                    'rank': i + 1,
                    'frame': f['frame'],
                    'score': f['score'],
                    'timestamp': f['timestamp'],
                    'zed_centerline': f['zed_centerline'],
                    'ouster_centerline': f['ouster_centerline']
                } for i, f in enumerate(self.best_frames)
            ],
            'worst_frames': [
                {
                    'rank': i + 1,
                    'frame': f['frame'],
                    'score': f['score'],
                    'timestamp': f['timestamp'],
                    'zed_centerline': f['zed_centerline'],
                    'ouster_centerline': f['ouster_centerline']
                } for i, f in enumerate(self.worst_frames)
            ],
            'all_scores': self.scores_history
        }
        
        # Save JSON report
        json_path = os.path.join(self.output_dir, 'analysis_summary.json')
        with open(json_path, 'w') as f:
            json.dump(summary, f, indent=2)
        self.get_logger().info(f'Saved JSON summary to {json_path}')
        
        # Save text report
        text_path = os.path.join(self.output_dir, 'analysis_summary.txt')
        with open(text_path, 'w') as f:
            f.write('=' * 80 + '\n')
            f.write('CENTERLINE AGREEMENT ANALYSIS SUMMARY\n')
            f.write('=' * 80 + '\n\n')
            
            f.write(f'Total Frames Analyzed: {summary["total_frames"]}\n\n')
            
            f.write('AGREEMENT STATISTICS:\n')
            f.write(f'  Mean Score:   {summary["mean_score"]:.4f}\n')
            f.write(f'  Median Score: {summary["median_score"]:.4f}\n')
            f.write(f'  Std Dev:      {summary["std_score"]:.4f}\n')
            f.write(f'  Min Score:    {summary["min_score"]:.4f}\n')
            f.write(f'  Max Score:    {summary["max_score"]:.4f}\n\n')
            
            f.write('LATERAL OFFSET STATISTICS (X position relative to base_link):\n')
            f.write(f'  ZED Centerline:\n')
            f.write(f'    Mean Offset:  {summary["lateral_offsets"]["zed"]["mean"]:+.4f} m ')
            f.write(f'({"left" if summary["lateral_offsets"]["zed"]["mean"] < 0 else "right"} of center)\n')
            f.write(f'    Std Dev:      {summary["lateral_offsets"]["zed"]["std"]:.4f} m\n')
            f.write(f'    Total Points: {summary["lateral_offsets"]["zed"]["total_points"]}\n')
            f.write(f'  Ouster Centerline:\n')
            f.write(f'    Mean Offset:  {summary["lateral_offsets"]["ouster"]["mean"]:+.4f} m ')
            f.write(f'({"left" if summary["lateral_offsets"]["ouster"]["mean"] < 0 else "right"} of center)\n')
            f.write(f'    Std Dev:      {summary["lateral_offsets"]["ouster"]["std"]:.4f} m\n')
            f.write(f'    Total Points: {summary["lateral_offsets"]["ouster"]["total_points"]}\n')
            f.write(f'  Lateral Difference (ZED - Ouster):\n')
            f.write(f'    Mean:         {summary["lateral_offsets"]["zed"]["mean"] - summary["lateral_offsets"]["ouster"]["mean"]:+.4f} m\n\n')
            
            f.write('-' * 80 + '\n')
            f.write('BEST FRAMES (Highest Agreement):\n')
            f.write('-' * 80 + '\n')
            for bf in summary['best_frames']:
                f.write(f'  #{bf["rank"]}: Frame {bf["frame"]:4d} | '
                       f'Score: {bf["score"]:.4f} | '
                       f'Timestamp: {bf["timestamp"]:.2f}s\n')
                f.write(f'    ZED Centerline (x,y): {bf["zed_centerline"]}\n')
                f.write(f'    Ouster Centerline (x,y): {bf["ouster_centerline"]}\n\n')
            
            f.write('-' * 80 + '\n')
            f.write('WORST FRAMES (Lowest Agreement):\n')
            f.write('-' * 80 + '\n')
            for wf in summary['worst_frames']:
                f.write(f'  #{wf["rank"]}: Frame {wf["frame"]:4d} | '
                       f'Score: {wf["score"]:.4f} | '
                       f'Timestamp: {wf["timestamp"]:.2f}s\n')
                f.write(f'    ZED Centerline (x,y): {wf["zed_centerline"]}\n')
                f.write(f'    Ouster Centerline (x,y): {wf["ouster_centerline"]}\n\n')
            
            f.write('\n' + '=' * 80 + '\n')
        
        self.get_logger().info(f'Saved text summary to {text_path}')
        
        # Generate score timeline plot
        self.plot_score_timeline()
    
    def plot_score_timeline(self):
        """Plot agreement score over time"""
        if not self.scores_history:
            return
        
        timestamps = [s['timestamp'] for s in self.scores_history]
        scores = [s['score'] for s in self.scores_history]
        frames = [s['frame'] for s in self.scores_history]
        
        # Normalize timestamps to start from 0
        min_time = min(timestamps)
        timestamps = [t - min_time for t in timestamps]
        
        fig, ax = plt.subplots(figsize=(14, 6))
        
        # Plot score timeline
        ax.plot(timestamps, scores, 'b-', linewidth=1.5, alpha=0.7)
        ax.scatter(timestamps, scores, c=scores, cmap='RdYlGn', s=30, alpha=0.6, 
                  vmin=0, vmax=1, edgecolors='black', linewidth=0.5)
        
        # Mark best frames
        for bf in self.best_frames:
            t = bf['timestamp'] - min_time
            ax.scatter(t, bf['score'], marker='*', s=300, c='green', 
                      edgecolors='darkgreen', linewidth=2, zorder=10, 
                      label=f'Best (Frame {bf["frame"]})' if bf == self.best_frames[0] else '')
        
        # Mark worst frames
        for wf in self.worst_frames:
            t = wf['timestamp'] - min_time
            ax.scatter(t, wf['score'], marker='X', s=300, c='red', 
                      edgecolors='darkred', linewidth=2, zorder=10,
                      label=f'Worst (Frame {wf["frame"]})' if wf == self.worst_frames[0] else '')
        
        ax.set_xlabel('Time (seconds)', fontsize=12)
        ax.set_ylabel('Agreement Score', fontsize=12)
        ax.set_title('Centerline Agreement Score Over Time', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-0.05, 1.05)
        
        # Add colorbar
        sm = plt.cm.ScalarMappable(cmap='RdYlGn', norm=plt.Normalize(vmin=0, vmax=1))
        sm.set_array([])
        cbar = plt.colorbar(sm, ax=ax)
        cbar.set_label('Agreement Score', rotation=270, labelpad=20)
        
        ax.legend(loc='best')
        
        filepath = os.path.join(self.output_dir, 'score_timeline.png')
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close()
        
        self.get_logger().info(f'Saved score timeline to {filepath}')
    
    def cleanup(self):
        """Called when node is shutting down"""
        self.get_logger().info('\n' + '=' * 80)
        self.get_logger().info('Shutting down - generating final analysis...')
        self.get_logger().info('=' * 80)
        
        self.save_best_worst_frames()
        self.generate_summary_report()
        
        self.get_logger().info('\n' + '=' * 80)
        self.get_logger().info(f'Analysis complete! Results saved to: {self.output_dir}')
        self.get_logger().info('=' * 80 + '\n')

def main():
    rclpy.init()
    node = CenterlineAgreementAnalyzer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()