#!/usr/bin/env python3
"""
Test script to verify all debug tools are working correctly.
This script performs basic functionality tests without requiring a real robot connection.
"""

import sys
import os
import subprocess
import time

def test_script_exists(script_path):
    """Test if a script exists and is executable"""
    if not os.path.exists(script_path):
        print(f"❌ FAIL: Script {script_path} does not exist")
        return False
    
    if not os.access(script_path, os.X_OK):
        print(f"❌ FAIL: Script {script_path} is not executable")
        return False
    
    print(f"✅ PASS: Script {script_path} exists and is executable")
    return True

def test_script_help(script_path):
    """Test if script shows help when called with --help"""
    try:
        result = subprocess.run([script_path, '--help'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"✅ PASS: Script {script_path} shows help correctly")
            return True
        else:
            print(f"❌ FAIL: Script {script_path} help failed with code {result.returncode}")
            return False
    except subprocess.TimeoutExpired:
        print(f"❌ FAIL: Script {script_path} help timed out")
        return False
    except Exception as e:
        print(f"❌ FAIL: Script {script_path} help error: {e}")
        return False

def test_python_imports():
    """Test if required Python modules can be imported"""
    required_modules = [
        'socket',
        'struct', 
        'time',
        'argparse',
        'statistics',
        'collections'
    ]
    
    all_passed = True
    for module in required_modules:
        try:
            __import__(module)
            print(f"✅ PASS: Module {module} imported successfully")
        except ImportError as e:
            print(f"❌ FAIL: Module {module} import failed: {e}")
            all_passed = False
    
    return all_passed

def test_ros_imports():
    """Test if ROS modules can be imported (if ROS is available)"""
    try:
        import rospy
        from fanuc_driver.msg import WeldState
        print("✅ PASS: ROS modules imported successfully")
        return True
    except ImportError as e:
        print(f"⚠️  WARN: ROS modules not available: {e}")
        print("         This is expected if ROS is not installed or sourced")
        return False

def main():
    print("Fanuc Welding Debug Tools Test")
    print("=" * 50)
    
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Test script files
    scripts_to_test = [
        'raw_message_capture.py',
        'weld_state_validator.py', 
        'byte_order_analyzer.py',
        'weld_state_monitor.py'
    ]
    
    print("\n1. Testing script existence and permissions...")
    script_tests_passed = 0
    for script in scripts_to_test:
        script_path = os.path.join(script_dir, script)
        if test_script_exists(script_path):
            script_tests_passed += 1
    
    print(f"\nScript tests: {script_tests_passed}/{len(scripts_to_test)} passed")
    
    print("\n2. Testing script help functionality...")
    help_tests_passed = 0
    for script in scripts_to_test:
        script_path = os.path.join(script_dir, script)
        if os.path.exists(script_path):
            if test_script_help(script_path):
                help_tests_passed += 1
    
    print(f"\nHelp tests: {help_tests_passed}/{len(scripts_to_test)} passed")
    
    print("\n3. Testing Python module imports...")
    python_imports_ok = test_python_imports()
    
    print("\n4. Testing ROS module imports...")
    ros_imports_ok = test_ros_imports()
    
    print("\n5. Testing KAREL file existence...")
    karel_dir = os.path.join(os.path.dirname(script_dir), 'karel')
    karel_files = [
        'debug/weld_io_test.kl',
        'include/libind_weld_t.kl',
        'include/libind_weld_h.kl'
    ]
    
    karel_tests_passed = 0
    for karel_file in karel_files:
        karel_path = os.path.join(karel_dir, karel_file)
        if os.path.exists(karel_path):
            print(f"✅ PASS: KAREL file {karel_file} exists")
            karel_tests_passed += 1
        else:
            print(f"❌ FAIL: KAREL file {karel_file} does not exist")
    
    print(f"\nKAREL tests: {karel_tests_passed}/{len(karel_files)} passed")
    
    # Summary
    print("\n" + "=" * 50)
    print("TEST SUMMARY")
    print("=" * 50)
    
    total_tests = len(scripts_to_test) * 2 + len(karel_files) + 2  # scripts + help + karel + python + ros
    passed_tests = script_tests_passed + help_tests_passed + karel_tests_passed
    
    if python_imports_ok:
        passed_tests += 1
    if ros_imports_ok:
        passed_tests += 1
    
    print(f"Total tests: {total_tests}")
    print(f"Passed: {passed_tests}")
    print(f"Failed: {total_tests - passed_tests}")
    
    if passed_tests == total_tests:
        print("\n🎉 ALL TESTS PASSED! Debug tools are ready to use.")
        return 0
    elif passed_tests >= total_tests * 0.8:
        print("\n⚠️  MOST TESTS PASSED. Some optional features may not work.")
        return 0
    else:
        print("\n❌ MANY TESTS FAILED. Please check the installation.")
        return 1

if __name__ == '__main__':
    sys.exit(main()) 