#!/usr/bin/env python3
"""
Simple RL Plotter - 4 Focused Graphs with Light Purple Theme
Generates good signal % and average RSRP trends for training and testing

Usage:
    python3 simple_plotter.py --training logs/history_*.json --testing logs/test_results_*.json --output plots
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patheffects as path_effects
from scipy.interpolate import make_interp_spline
from scipy.ndimage import uniform_filter1d
import argparse


def plot_good_signal_percentage(data, title, save_path, mode='training'):
    """
    Fluid line graph with smooth interpolation: Good signal percentage trend
    """
    # Filter successful episodes only
    if mode == 'training':
        successful = [ep for ep in data if ep.get('success', False)]
    else:  # testing
        successful = [ep for ep in data if ep.get('success', True)]
    
    if not successful:
        print(f"⚠️ No successful episodes found for {title}")
        return
    
    episodes = np.array([ep['episode'] for ep in successful])
    
    # Calculate good signal percentage
    good_signal_pct = []
    for ep in successful:
        if ep['steps'] > 0:
            pct = (ep['good_signal_steps'] / ep['steps']) * 100
            good_signal_pct.append(pct)
        else:
            good_signal_pct.append(0)
    
    good_signal_pct = np.array(good_signal_pct)
    
    # Apply gentle smoothing
    if len(good_signal_pct) > 3:
        good_signal_pct_smooth = uniform_filter1d(good_signal_pct, size=3, mode='nearest')
    else:
        good_signal_pct_smooth = good_signal_pct
    
    # Create smooth interpolated curve if enough points
    if len(episodes) > 3:
        episodes_smooth = np.linspace(episodes.min(), episodes.max(), 300)
        spl = make_interp_spline(episodes, good_signal_pct_smooth, k=min(3, len(episodes)-1))
        good_signal_smooth = spl(episodes_smooth)
        good_signal_smooth = np.clip(good_signal_smooth, 0, 100)  # Keep in bounds
    else:
        episodes_smooth = episodes
        good_signal_smooth = good_signal_pct_smooth
    
    # Calculate average
    overall_avg = np.mean(good_signal_pct)
    
    # Create figure with gradient background
    fig, ax = plt.subplots(figsize=(14, 7), facecolor='#FDFCFF')
    
    # Create subtle gradient background
    gradient = np.linspace(0, 1, 100).reshape(-1, 1)
    gradient = np.hstack((gradient, gradient))
    ax.imshow(gradient, extent=[episodes.min(), episodes.max(), 0, 100], 
              aspect='auto', alpha=0.15, cmap='Purples', zorder=0)
    ax.set_facecolor('#FCFAFF')
    
    # Plot smooth line with glow effect (shadow)
    ax.plot(episodes_smooth, good_signal_smooth, color='#9370DB', linewidth=5, 
            alpha=0.2, zorder=2, solid_capstyle='round', solid_joinstyle='round')
    ax.plot(episodes_smooth, good_signal_smooth, color='#9370DB', linewidth=3, 
            alpha=0.6, zorder=3, solid_capstyle='round', solid_joinstyle='round')
    
    # Main line (no markers for fluid look)
    line = ax.plot(episodes_smooth, good_signal_smooth, color='#7B68B8', linewidth=2.5, 
                   zorder=4, solid_capstyle='round', solid_joinstyle='round')[0]
    
    # Add subtle fill under curve
    ax.fill_between(episodes_smooth, 0, good_signal_smooth, 
                     color='#B8A4D9', alpha=0.2, zorder=1)
    
    # Average line with glow
    ax.axhline(y=overall_avg, color='#6B4BA1', linestyle='--', linewidth=2.5, 
               alpha=0.4, zorder=5)
    ax.axhline(y=overall_avg, color='#6B4BA1', linestyle='--', linewidth=1.5, 
               alpha=0.8, zorder=6, label=f'Average: {overall_avg:.1f}%')
    
    # Labels with soft shadow
    xlabel = ax.set_xlabel('Episode', fontsize=14, fontweight='bold', color='#4A4A4A')
    ylabel = ax.set_ylabel('Good Signal Percentage (%)', fontsize=14, fontweight='bold', color='#4A4A4A')
    title_text = ax.set_title(title, fontsize=16, fontweight='bold', pad=20, color='#6B4BA1')
    
    # Add subtle shadow to text
    for text in [xlabel, ylabel, title_text]:
        text.set_path_effects([path_effects.withStroke(linewidth=3, foreground='white', alpha=0.7)])
    
    # Legend with rounded corners
    legend = ax.legend(fontsize=12, facecolor='#FAF8FF', edgecolor='#B8A4D9', 
                       framealpha=0.95, loc='best')
    legend.get_frame().set_linewidth(1.5)
    
    # Softer grid
    ax.grid(True, alpha=0.25, color='#C8B4E8', linestyle='-', linewidth=0.8)
    ax.set_ylim(0, 100)
    
    # Customize spines for modern look
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_color('#B8A4D9')
    ax.spines['bottom'].set_color('#B8A4D9')
    ax.spines['left'].set_linewidth(1.5)
    ax.spines['bottom'].set_linewidth(1.5)
    
    # Customize tick colors
    ax.tick_params(colors='#4A4A4A', width=1.5)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=600, bbox_inches='tight', facecolor='#FDFCFF')  # High DPI for PPT
    print(f"✅ Saved: {save_path}")
    plt.close()


def plot_average_rsrp(data, title, save_path, mode='training'):
    """
    Fluid line graph with smooth interpolation: Average RSRP trend
    """
    # Filter successful episodes only
    if mode == 'training':
        successful = [ep for ep in data if ep.get('success', False)]
    else:  # testing
        successful = [ep for ep in data if ep.get('success', True)]
    
    if not successful:
        print(f"⚠️ No successful episodes found for {title}")
        return
    
    episodes = np.array([ep['episode'] for ep in successful])
    avg_rsrp = np.array([ep['avg_rsrp'] for ep in successful])
    
    # Apply gentle smoothing
    if len(avg_rsrp) > 3:
        avg_rsrp_smooth = uniform_filter1d(avg_rsrp, size=3, mode='nearest')
    else:
        avg_rsrp_smooth = avg_rsrp
    
    # Create smooth interpolated curve if enough points
    if len(episodes) > 3:
        episodes_smooth = np.linspace(episodes.min(), episodes.max(), 300)
        spl = make_interp_spline(episodes, avg_rsrp_smooth, k=min(3, len(episodes)-1))
        rsrp_smooth = spl(episodes_smooth)
        rsrp_smooth = np.clip(rsrp_smooth, -100, -80)  # Keep in bounds
    else:
        episodes_smooth = episodes
        rsrp_smooth = avg_rsrp_smooth
    
    # Calculate average
    overall_avg = np.mean(avg_rsrp)
    
    # Create figure with gradient background
    fig, ax = plt.subplots(figsize=(14, 7), facecolor='#FDFCFF')
    
    # Create subtle gradient background
    gradient = np.linspace(0, 1, 100).reshape(-1, 1)
    gradient = np.hstack((gradient, gradient))
    ax.imshow(gradient, extent=[episodes.min(), episodes.max(), -100, -80], 
              aspect='auto', alpha=0.15, cmap='Purples', zorder=0)
    ax.set_facecolor('#FCFAFF')
    
    # Plot smooth line with glow effect (shadow)
    ax.plot(episodes_smooth, rsrp_smooth, color='#9370DB', linewidth=5, 
            alpha=0.2, zorder=2, solid_capstyle='round', solid_joinstyle='round')
    ax.plot(episodes_smooth, rsrp_smooth, color='#9370DB', linewidth=3, 
            alpha=0.6, zorder=3, solid_capstyle='round', solid_joinstyle='round')
    
    # Main line (no markers for fluid look)
    line = ax.plot(episodes_smooth, rsrp_smooth, color='#7B68B8', linewidth=2.5, 
                   zorder=4, solid_capstyle='round', solid_joinstyle='round')[0]
    
    # Add subtle fill under curve
    ax.fill_between(episodes_smooth, -100, rsrp_smooth, 
                     color='#B8A4D9', alpha=0.2, zorder=1)
    
    # Average line with glow
    ax.axhline(y=overall_avg, color='#6B4BA1', linestyle='--', linewidth=2.5, 
               alpha=0.4, zorder=5)
    ax.axhline(y=overall_avg, color='#6B4BA1', linestyle='--', linewidth=1.5, 
               alpha=0.8, zorder=6, label=f'Average: {overall_avg:.1f} dBm')
    
    # Labels with soft shadow
    xlabel = ax.set_xlabel('Episode', fontsize=14, fontweight='bold', color='#4A4A4A')
    ylabel = ax.set_ylabel('Average RSRP (dBm)', fontsize=14, fontweight='bold', color='#4A4A4A')
    title_text = ax.set_title(title, fontsize=16, fontweight='bold', pad=20, color='#6B4BA1')
    
    # Add subtle shadow to text
    for text in [xlabel, ylabel, title_text]:
        text.set_path_effects([path_effects.withStroke(linewidth=3, foreground='white', alpha=0.7)])
    
    # Legend with rounded corners
    legend = ax.legend(fontsize=12, facecolor='#FAF8FF', edgecolor='#B8A4D9', 
                       framealpha=0.95, loc='best')
    legend.get_frame().set_linewidth(1.5)
    
    # Softer grid
    ax.grid(True, alpha=0.25, color='#C8B4E8', linestyle='-', linewidth=0.8)
    ax.set_ylim(-100, -80)
    
    # Customize spines for modern look
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_color('#B8A4D9')
    ax.spines['bottom'].set_color('#B8A4D9')
    ax.spines['left'].set_linewidth(1.5)
    ax.spines['bottom'].set_linewidth(1.5)
    
    # Customize tick colors
    ax.tick_params(colors='#4A4A4A', width=1.5)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=600, bbox_inches='tight', facecolor='#FDFCFF')  # High DPI for PPT
    print(f"✅ Saved: {save_path}")
    plt.close()


def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='Generate 4 focused RL plots')
    parser.add_argument('--training', type=str, required=True,
                       help='Path to training history JSON')
    parser.add_argument('--testing', type=str, required=True,
                       help='Path to testing results JSON')
    parser.add_argument('--output', type=str, default='plots',
                       help='Output directory for plots')
    
    args = parser.parse_args()
    
    # Load data
    print("📂 Loading data...")
    with open(args.training, 'r') as f:
        training_data = json.load(f)
    print(f"   Training: {len(training_data)} episodes")
    
    with open(args.testing, 'r') as f:
        testing_data = json.load(f)
    print(f"   Testing: {len(testing_data)} episodes")
    
    # Create output directory
    import os
    os.makedirs(args.output, exist_ok=True)
    
    print("\n" + "=" * 70)
    print("📊 GENERATING 4 FOCUSED PLOTS")
    print("=" * 70)
    
    # Graph 1: Training - Good Signal %
    plot_good_signal_percentage(
        training_data,
        'Training: Good Signal Percentage (RSRP > -90 dBm)\nSuccessful Episodes Only',
        os.path.join(args.output, '1_training_good_signal_pct.png'),
        mode='training'
    )
    
    # Graph 2: Training - Average RSRP
    plot_average_rsrp(
        training_data,
        'Training: Average RSRP per Episode\nSuccessful Episodes Only',
        os.path.join(args.output, '2_training_avg_rsrp.png'),
        mode='training'
    )
    
    # Graph 3: Testing - Good Signal %
    plot_good_signal_percentage(
        testing_data,
        'Testing: Good Signal Percentage (RSRP > -90 dBm)\nSuccessful Episodes Only',
        os.path.join(args.output, '3_testing_good_signal_pct.png'),
        mode='testing'
    )
    
    # Graph 4: Testing - Average RSRP
    plot_average_rsrp(
        testing_data,
        'Testing: Average RSRP per Episode\nSuccessful Episodes Only',
        os.path.join(args.output, '4_testing_avg_rsrp.png'),
        mode='testing'
    )
    
    print("=" * 70)
    print(f"✅ All 4 plots saved to: {args.output}/")
    print("=" * 70)
    
    # Print summary statistics
    print("\n📈 SUMMARY STATISTICS")
    print("=" * 70)
    
    # Training stats
    train_success = [ep for ep in training_data if ep.get('success', False)]
    if train_success:
        train_good_signal = [(ep['good_signal_steps'] / ep['steps']) * 100 
                             for ep in train_success if ep['steps'] > 0]
        train_rsrp = [ep['avg_rsrp'] for ep in train_success]
        
        print(f"\n🎓 TRAINING ({len(train_success)} successful episodes):")
        print(f"   Good Signal %: {np.mean(train_good_signal):.1f}% (avg)")
        print(f"   Average RSRP:  {np.mean(train_rsrp):.1f} dBm (avg)")
    
    # Testing stats
    test_success = [ep for ep in testing_data if ep.get('success', True)]
    if test_success:
        test_good_signal = [(ep['good_signal_steps'] / ep['steps']) * 100 
                            for ep in test_success if ep['steps'] > 0]
        test_rsrp = [ep['avg_rsrp'] for ep in test_success]
        
        print(f"\n🧪 TESTING ({len(test_success)} successful episodes):")
        print(f"   Good Signal %: {np.mean(test_good_signal):.1f}% (avg)")
        print(f"   Average RSRP:  {np.mean(test_rsrp):.1f} dBm (avg)")
    
    print("\n" + "=" * 70)
    print("✅ Done! Check the plots folder.")
    print("=" * 70)


if __name__ == '__main__':
    main()