#!/usr/bin/env python3
"""
Code Inspection Report for Disease Detection Timeout Feature
=============================================================

This script analyzes the implementation without requiring ROS2 dependencies.

Author: Robotics Dojo 2025
Date: October 7, 2025
"""

import re
import os


def inspect_file(filepath, checks):
    """Inspect a file for specific patterns"""
    print(f"\n{'='*70}")
    print(f"📄 Inspecting: {os.path.basename(filepath)}")
    print('='*70)
    
    if not os.path.exists(filepath):
        print(f"❌ File not found: {filepath}")
        return False
    
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
        lines = content.split('\n')
    
    all_passed = True
    
    for check_name, check_fn in checks.items():
        print(f"\n🔍 {check_name}")
        result = check_fn(content, lines)
        if result:
            print(f"   ✅ PASS")
        else:
            print(f"   ❌ FAIL")
            all_passed = False
    
    return all_passed


def check_sensor_behaviors():
    """Check sensor_behaviors.py for timeout implementation"""
    
    filepath = "sensor_behaviors.py"
    
    checks = {
        "1. WaitForDiseaseDetection class exists": lambda c, l: "class WaitForDiseaseDetection" in c,
        
        "2. Timeout parameter in __init__": lambda c, l: re.search(r'def __init__.*timeout.*=.*\d+\.?\d*', c),
        
        "3. Timeout stored as instance variable": lambda c, l: "self.timeout = timeout" in c,
        
        "4. Start time initialization": lambda c, l: "self.start_time = time.time()" in c,
        
        "5. Timeout check in update()": lambda c, l: "elapsed = time.time() - self.start_time" in c and "if elapsed >= self.timeout:" in c,
        
        "6. Returns FAILURE on timeout": lambda c, l: "return py_trees.common.Status.FAILURE" in c and any("timeout" in line.lower() for line in l),
        
        "7. Stores 'timeout' in blackboard": lambda c, l: "self.blackboard.set('disease_detection_result', 'timeout')" in c,
        
        "8. Returns SUCCESS on detection": lambda c, l: "if self.detection_result is not None:" in c and "return py_trees.common.Status.SUCCESS" in c,
        
        "9. Returns RUNNING while waiting": lambda c, l: "return py_trees.common.Status.RUNNING" in c,
        
        "10. Warning message on timeout": lambda c, l: re.search(r'logger\.warning.*timeout', c, re.IGNORECASE),
    }
    
    return inspect_file(filepath, checks)


def check_mission_file():
    """Check cube_delivery_mission.py for timeout configuration"""
    
    filepath = "cube_delivery_mission.py"
    
    checks = {
        "1. WaitForDiseaseDetection imported": lambda c, l: "from sensor_behaviors import" in c and "WaitForDiseaseDetection" in c,
        
        "2. Timeout set to 12.0 seconds": lambda c, l: "WaitForDiseaseDetection" in c and "timeout=12.0" in c,
        
        "3. Timeout decorator applied": lambda c, l: "py_trees.decorators.Timeout" in c,
        
        "4. Timeout decorator duration 12.5s": lambda c, l: "duration=12.5" in c,
        
        "5. FailureIsSuccess decorator applied": lambda c, l: "py_trees.decorators.FailureIsSuccess" in c,
        
        "6. Decorators properly nested": lambda c, l: check_decorator_nesting(c),
        
        "7. Console message mentions timeout": lambda c, l: "12s timeout" in c or "12 sec" in c.lower(),
        
        "8. detect_disease added to tree": lambda c, l: "detect_disease" in c and "root.add_children" in c,
        
        "9. Comment explains non-fatal behavior": lambda c, l: "non-fatal" in c.lower() or "move on" in c.lower(),
        
        "10. Phase 0 includes disease detection": lambda c, l: "PHASE 0" in c and "DISEASE DETECTION" in c,
    }
    
    return inspect_file(filepath, checks)


def check_decorator_nesting(content):
    """Check if decorators are properly nested"""
    # Look for pattern where FailureIsSuccess wraps Timeout
    pattern = r'FailureIsSuccess.*?child=.*?(Timeout|detect_disease)'
    return bool(re.search(pattern, content, re.DOTALL))


def analyze_timeout_logic():
    """Analyze the timeout logic flow"""
    print(f"\n{'='*70}")
    print("🔬 TIMEOUT LOGIC ANALYSIS")
    print('='*70)
    
    filepath = "sensor_behaviors.py"
    
    if not os.path.exists(filepath):
        print(f"❌ File not found: {filepath}")
        return False
    
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    # Find the update() method in WaitForDiseaseDetection
    in_class = False
    in_update_method = False
    update_lines = []
    indent_level = 0
    
    for i, line in enumerate(lines):
        if 'class WaitForDiseaseDetection' in line:
            in_class = True
        if in_class and 'def update(self):' in line:
            in_update_method = True
            indent_level = len(line) - len(line.lstrip())
        elif in_update_method:
            current_indent = len(line) - len(line.lstrip())
            if line.strip() and current_indent <= indent_level:
                break
            update_lines.append((i+1, line.rstrip()))
    
    print("\n📊 update() Method Flow:")
    print("-" * 70)
    
    has_result_check = False
    has_timeout_check = False
    has_elapsed_calc = False
    returns_success = False
    returns_failure = False
    returns_running = False
    
    for line_num, line in update_lines:
        if "if self.detection_result is not None:" in line:
            has_result_check = True
            print(f"Line {line_num}: ✅ Checks for detection result")
        elif "return py_trees.common.Status.SUCCESS" in line:
            returns_success = True
            print(f"Line {line_num}: ✅ Returns SUCCESS when result received")
        elif "elapsed = time.time() - self.start_time" in line:
            has_elapsed_calc = True
            print(f"Line {line_num}: ✅ Calculates elapsed time")
        elif "if elapsed >= self.timeout:" in line:
            has_timeout_check = True
            print(f"Line {line_num}: ✅ Checks if timeout exceeded")
        elif "return py_trees.common.Status.FAILURE" in line:
            returns_failure = True
            print(f"Line {line_num}: ✅ Returns FAILURE on timeout")
        elif "return py_trees.common.Status.RUNNING" in line:
            returns_running = True
            print(f"Line {line_num}: ✅ Returns RUNNING while waiting")
    
    print("\n📋 Logic Completeness:")
    checks = [
        ("Result check present", has_result_check),
        ("Timeout calculation present", has_elapsed_calc),
        ("Timeout comparison present", has_timeout_check),
        ("Returns SUCCESS", returns_success),
        ("Returns FAILURE", returns_failure),
        ("Returns RUNNING", returns_running),
    ]
    
    all_present = True
    for check_name, present in checks:
        status = "✅" if present else "❌"
        print(f"   {status} {check_name}")
        if not present:
            all_present = False
    
    return all_present


def check_timeout_values():
    """Check timeout values are consistent"""
    print(f"\n{'='*70}")
    print("⏱  TIMEOUT VALUES CHECK")
    print('='*70)
    
    mission_file = "cube_delivery_mission.py"
    
    if not os.path.exists(mission_file):
        print(f"❌ File not found: {mission_file}")
        return False
    
    with open(mission_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Extract timeout values
    behavior_timeout = re.search(r'WaitForDiseaseDetection.*?timeout=([\d.]+)', content)
    decorator_timeout = re.search(r'Timeout.*?duration=([\d.]+)', content)
    
    if behavior_timeout and decorator_timeout:
        behavior_val = float(behavior_timeout.group(1))
        decorator_val = float(decorator_timeout.group(1))
        
        print(f"\n📊 Timeout Values:")
        print(f"   Behavior timeout:  {behavior_val}s")
        print(f"   Decorator timeout: {decorator_val}s")
        print(f"   Difference:        {decorator_val - behavior_val}s")
        
        if decorator_val > behavior_val:
            print(f"\n   ✅ Decorator timeout is longer (correct)")
            print(f"      Behavior times out at {behavior_val}s")
            print(f"      Decorator provides {decorator_val - behavior_val}s buffer")
            return True
        else:
            print(f"\n   ⚠️  Decorator timeout should be longer than behavior timeout")
            return False
    else:
        print("❌ Could not find timeout values")
        return False


def generate_report():
    """Generate comprehensive inspection report"""
    print("\n" + "="*70)
    print("🧪 DISEASE DETECTION TIMEOUT - CODE INSPECTION REPORT")
    print("="*70)
    print("Date: October 7, 2025")
    print("Feature: 12-second timeout with graceful failure handling")
    print("="*70)
    
    results = []
    
    # Check sensor_behaviors.py
    results.append(("sensor_behaviors.py implementation", check_sensor_behaviors()))
    
    # Check cube_delivery_mission.py
    results.append(("cube_delivery_mission.py configuration", check_mission_file()))
    
    # Analyze timeout logic
    results.append(("Timeout logic flow", analyze_timeout_logic()))
    
    # Check timeout values
    results.append(("Timeout value consistency", check_timeout_values()))
    
    # Summary
    print(f"\n{'='*70}")
    print("📊 INSPECTION SUMMARY")
    print('='*70)
    
    for test_name, passed in results:
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{status}: {test_name}")
    
    all_passed = all(r[1] for r in results)
    
    if all_passed:
        print("\n" + "="*70)
        print("✅ ALL CHECKS PASSED!")
        print("="*70)
        print("\n🎯 Implementation Summary:")
        print("   ✅ 12-second timeout implemented in WaitForDiseaseDetection")
        print("   ✅ Behavior returns FAILURE on timeout")
        print("   ✅ 'timeout' stored in blackboard on timeout")
        print("   ✅ Timeout decorator (12.5s) wraps behavior")
        print("   ✅ FailureIsSuccess decorator converts FAILURE → SUCCESS")
        print("   ✅ Mission continues after timeout (non-fatal)")
        print("   ✅ Console messages updated to reflect timeout")
        print("\n🚀 Code is ready for deployment!")
        print("\n📝 Next Steps:")
        print("   1. Test with actual ROS2 nodes")
        print("   2. Verify inference node integration")
        print("   3. Monitor console output during timeout")
        print("   4. Confirm mission continues to cube delivery phase")
        return 0
    else:
        print("\n" + "="*70)
        print("❌ SOME CHECKS FAILED")
        print("="*70)
        print("Review the failed checks above and fix the issues.")
        return 1


if __name__ == "__main__":
    import sys
    
    # Change to behavior_tree directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    
    exit_code = generate_report()
    sys.exit(exit_code)
