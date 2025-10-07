#!/usr/bin/env python3
"""
Robot Calibration Tool
This script helps you measure and calibrate your robot's physical parameters.
"""

import math

def measure_robot_parameters():
    print("=== Robot Physical Parameter Calculation ===")
    print("Using your known robot specifications:")
    print()
    
    # Known wheel specifications
    print("1. WHEEL SPECIFICATIONS (from your data):")
    wheel_diameter_mm = 85  # Your wheels are 85mm diameter
    wheel_diameter_cm = wheel_diameter_mm / 10.0
    wheel_radius = (wheel_diameter_cm / 100.0) / 2.0  # Convert to meters
    print(f"   Wheel diameter: {wheel_diameter_mm}mm ({wheel_diameter_cm}cm)")
    print(f"   Calculated wheel radius: {wheel_radius:.6f} meters")
    print()
    
    # Known base width
    print("2. BASE WIDTH (from your measurements):")
    base_width_cm = 20.8  # Your measured wheelbase
    base_width = base_width_cm / 100.0  # Convert cm to m
    print(f"   Wheelbase (center to center): {base_width_cm}cm")
    print(f"   Calculated base width: {base_width:.6f} meters")
    print()
    
    # Motor specifications
    print("3. MOTOR SPECIFICATIONS:")
    print("   Model: 25GA370 DC 12V micro gear reduction motor")
    print("   Type: D-shaped Hall encoder")
    print("   Wiring: Yellow/Green = encoder signals (11 pulses per revolution mentioned)")
    print()
    
    # Encoder ticks determination
    print("4. ENCODER TICKS PER REVOLUTION:")
    print("   Your motor spec mentions '11 engine signals'")
    print("   For Hall encoders, this typically means:")
    print("   - 11 pulses per motor revolution (before gearing)")
    print("   - With gear reduction, final output ticks could be much higher")
    print()
    
    print("   Let's determine the correct value:")
    print("   a) Test with common values: 44, 120, 334")
    print("   b) Enter custom value if you know the gear ratio")
    print("   c) Use empirical testing (recommended)")
    
    choice = input("   Choose option (a/b/c): ").lower().strip()
    
    if choice == 'a':
        print("   Testing with common encoder values:")
        encoder_options = [44, 120, 334]
        for i, ticks in enumerate(encoder_options):
            print(f"   {i+1}. {ticks} ticks/rev")
        
        selection = int(input("   Select option (1-3): ")) - 1
        encoder_ticks = encoder_options[selection]
    elif choice == 'b':
        encoder_ticks = int(input("   Enter custom encoder ticks per revolution: "))
    else:  # choice == 'c'
        print("   For empirical testing, we'll start with 44 ticks (most common)")
        print("   You can adjust this after testing robot movement")
        encoder_ticks = 44
    
    print(f"   Using: {encoder_ticks} ticks per revolution")
    print()
    
    # Calculate useful metrics
    wheel_circumference = 2 * math.pi * wheel_radius
    distance_per_tick = wheel_circumference / encoder_ticks
    
    print("=== CALCULATED PARAMETERS ===")
    print(f"Wheel radius:              {wheel_radius:.6f} m")
    print(f"Base width:                {base_width:.6f} m") 
    print(f"Encoder ticks per rev:     {encoder_ticks}")
    print(f"Wheel circumference:       {wheel_circumference:.6f} m")
    print(f"Distance per encoder tick: {distance_per_tick:.6f} m")
    print()
    
    # Generate launch file parameters
    print("=== LAUNCH FILE PARAMETERS ===")
    print("Add these parameters to your launch files:")
    print(f"            'wheel_radius': {wheel_radius:.6f},")
    print(f"            'base_width': {base_width:.6f},")
    print(f"            'encoder_ticks_per_rev': {encoder_ticks},")
    print()
    
    # Test calculations
    print("=== TURNING RADIUS TEST ===")
    print("For a 90-degree turn, each wheel should move:")
    quarter_circle = (math.pi * base_width) / 4  # Quarter of circle around base_width
    ticks_for_90_turn = quarter_circle / distance_per_tick
    print(f"   Distance: {quarter_circle:.4f} meters")
    print(f"   Encoder ticks: {ticks_for_90_turn:.1f} ticks")
    print()
    
    print("=== RECOMMENDATIONS ===")
    if wheel_radius < 0.03 or wheel_radius > 0.15:
        print("⚠️  WARNING: Wheel radius seems unusual. Please double-check measurement.")
    if base_width < 0.1 or base_width > 0.5:
        print("⚠️  WARNING: Base width seems unusual. Please double-check measurement.")
    # Sanity checks
    if encoder_ticks not in [283, 120, 334, 1440]:
        print(f"   ⚠️  Unusual encoder_ticks value: {encoder_ticks}")
        print(f"      Common values are 283, 120, 334, or 1440")
        confirm = input("   Continue anyway? (y/n): ").strip().lower()
    
    print("✅ For better SLAM performance:")
    print("   - Ensure wheels are properly aligned")
    print("   - Check that encoders count in both directions")
    print("   - Test turning accuracy with measured parameters")

if __name__ == "__main__":
    measure_robot_parameters()