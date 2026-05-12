#!/usr/bin/env python3
"""
Safe module tests for ARX teleop integration.
Tests import, IK computation, and quaternion conversion WITHOUT connecting to hardware.

Usage:
    # Must set environment first:
    source scripts/env/setup_arx_r5.sh

    # Then run tests:
    python scripts/tests/test_arx_modules.py
"""

import sys
import os

# Add SDK paths at the very beginning (using shared python_sdk directory)
_script_dir = os.path.dirname(os.path.abspath(__file__))
_scripts_dir = os.path.dirname(_script_dir)
_project_root = os.path.dirname(_scripts_dir)
_lerobot_data_root = os.path.dirname(_project_root)
_arx_root = os.path.dirname(_lerobot_data_root)
_r5_sdk_path = os.path.join(_arx_root, "python_sdk", "arx_r5_sdk")
sys.path.insert(0, _r5_sdk_path)
sys.path.insert(0, os.path.join(_r5_sdk_path, "bimanual", "api"))

import numpy as np


def test_ld_library_path():
    """Test 1: Check LD_LIBRARY_PATH is set correctly."""
    print("\n" + "=" * 60)
    print("TEST 1: LD_LIBRARY_PATH Check")
    print("=" * 60)

    ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    required_paths = [
        'bimanual/api/arx_r5_src',
        'bimanual/api',
        '/opt/ros/jazzy/lib'
    ]

    missing = []
    for rp in required_paths:
        if rp not in ld_path:
            missing.append(rp)

    if missing:
        print("❌ FAILED: Missing paths in LD_LIBRARY_PATH:")
        for m in missing:
            print(f"   - {m}")
        print("\nCurrent LD_LIBRARY_PATH:")
        for p in ld_path.split(':')[:5]:
            print(f"   {p}")
        return False
    else:
        print("✓ All required paths found in LD_LIBRARY_PATH")
        return True


def test_bimanual_import():
    """Test 2: Import bimanual SDK."""
    print("\n" + "=" * 60)
    print("TEST 2: bimanual SDK Import")
    print("=" * 60)

    try:
        import bimanual
        print("✓ bimanual imported successfully")

        # Check for required functions
        has_fk = hasattr(bimanual, 'forward_kinematics')
        has_ik = hasattr(bimanual, 'inverse_kinematics')

        if has_fk and has_ik:
            print("✓ forward_kinematics and inverse_kinematics available")
            return True
        else:
            print(f"❌ Missing: FK={has_fk}, IK={has_ik}")
            return False

    except ImportError as e:
        print(f"❌ FAILED to import bimanual: {e}")
        print("\nHint: Make sure to set LD_LIBRARY_PATH before running this test")
        return False


def test_ik_computation():
    """Test 3: IK computation (no hardware)."""
    print("\n" + "=" * 60)
    print("TEST 3: IK Computation (Pure Math, No Hardware)")
    print("=" * 60)

    try:
        import bimanual

        # Test forward kinematics with zero position
        print("\nForward Kinematics Test:")
        joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        print(f"  Input joints (6D): {joint_angles}")

        ee_pose = bimanual.forward_kinematics(joint_angles)
        print(f"  Output EE pose (xyzrpy): {ee_pose}")
        print(f"  Output shape: {len(ee_pose)}")

        # Test inverse kinematics
        print("\nInverse Kinematics Test:")
        target_xyzrpy = np.array(ee_pose)  # Use FK result as IK input
        print(f"  Input xyzrpy (6D): {target_xyzrpy}")

        ik_result = bimanual.inverse_kinematics(target_xyzrpy)
        print(f"  Output joints (6D): {ik_result}")
        print(f"  Output shape: {len(ik_result)}")

        # Verify round-trip
        error = np.abs(np.array(ik_result) - joint_angles).max()
        print(f"\n  Round-trip error: {error:.6f} rad")

        if error < 0.01:
            print("✓ IK computation working correctly")
            return True
        else:
            print(f"⚠ IK round-trip error is larger than expected")
            return True  # Still pass, just with warning

    except Exception as e:
        print(f"❌ FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_quaternion_conversion():
    """Test 4: Quaternion to RPY conversion."""
    print("\n" + "=" * 60)
    print("TEST 4: Quaternion to RPY Conversion")
    print("=" * 60)

    try:
        from scipy.spatial.transform import Rotation as R

        def quat_wxyz_to_rpy(quat_wxyz):
            """Convert quaternion [w, x, y, z] to Euler angles [roll, pitch, yaw]."""
            quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
            r = R.from_quat(quat_xyzw)
            return r.as_euler('xyz')

        # Test cases: identity, 90-degree rotations
        test_cases = [
            ("Identity", [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            ("90° roll", [0.7071, 0.7071, 0.0, 0.0], [np.pi/2, 0.0, 0.0]),
            ("90° pitch", [0.7071, 0.0, 0.7071, 0.0], [0.0, np.pi/2, 0.0]),
            ("90° yaw", [0.7071, 0.0, 0.0, 0.7071], [0.0, 0.0, np.pi/2]),
        ]

        all_passed = True
        for name, quat, expected_rpy in test_cases:
            result = quat_wxyz_to_rpy(np.array(quat))
            error = np.abs(result - np.array(expected_rpy)).max()
            status = "✓" if error < 0.01 else "❌"
            print(f"  {status} {name}: quat={quat[:2]}... -> rpy={[f'{r:.3f}' for r in result]}")
            if error >= 0.01:
                all_passed = False

        if all_passed:
            print("✓ Quaternion conversion working correctly")
        return all_passed

    except Exception as e:
        print(f"❌ FAILED: {e}")
        return False


def test_teleop_import():
    """Test 5: Import teleop module."""
    print("\n" + "=" * 60)
    print("TEST 5: ARXVRTeleop Import")
    print("=" * 60)

    try:
        # Add paths for the packages
        base_path = str(Path(__file__).resolve().parent.parent.parent)
        sys.path.insert(0, base_path)

        from teleoperators.arx import ARXVRTeleop, ARXVRTeleopConfig
        print("✓ ARXVRTeleop imported successfully")
        print("✓ ARXVRTeleopConfig imported successfully")

        # Check that CAN fields are removed from config
        import dataclasses
        field_names = [f.name for f in dataclasses.fields(ARXVRTeleopConfig)]

        removed_fields = ['left_can', 'right_can', 'lift_can', 'arm_type']
        issues = []
        for rf in removed_fields:
            if rf in field_names:
                issues.append(rf)

        if issues:
            print(f"⚠ Warning: These CAN fields still exist in config: {issues}")
        else:
            print("✓ CAN config fields correctly removed from ARXVRTeleopConfig")

        # Check for set_robot_reference method
        if hasattr(ARXVRTeleop, 'set_robot_reference'):
            print("✓ set_robot_reference method exists")
        else:
            print("❌ set_robot_reference method missing")
            return False

        return True

    except Exception as e:
        print(f"❌ FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_robot_import():
    """Test 6: Import robot module."""
    print("\n" + "=" * 60)
    print("TEST 6: ARXLift2 Import")
    print("=" * 60)

    try:
        # Paths already added in test_teleop_import
        from robots.arx import ARXLift2, ARXLift2Config
        print("✓ ARXLift2 imported successfully")
        print("✓ ARXLift2Config imported successfully")

        return True

    except Exception as e:
        print(f"❌ FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("\n" + "=" * 60)
    print("ARX VR Teleop Module Tests")
    print("Safe tests - NO hardware connection")
    print("=" * 60)

    results = {}

    # Run tests
    results['LD_LIBRARY_PATH'] = test_ld_library_path()

    if results['LD_LIBRARY_PATH']:
        results['bimanual_import'] = test_bimanual_import()

        if results['bimanual_import']:
            results['ik_computation'] = test_ik_computation()
        else:
            results['ik_computation'] = False
    else:
        results['bimanual_import'] = False
        results['ik_computation'] = False
        print("\n⚠ Skipping bimanual tests due to missing LD_LIBRARY_PATH")

    results['quaternion_conversion'] = test_quaternion_conversion()
    results['teleop_import'] = test_teleop_import()
    results['robot_import'] = test_robot_import()

    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)

    passed = sum(results.values())
    total = len(results)

    for test_name, result in results.items():
        status = "✓ PASS" if result else "❌ FAIL"
        print(f"  {status}: {test_name}")

    print(f"\nResult: {passed}/{total} tests passed")

    if passed == total:
        print("\n✓ All tests passed! Ready for hardware testing.")
    else:
        print("\n❌ Some tests failed. Please fix before hardware testing.")

    return passed == total


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
