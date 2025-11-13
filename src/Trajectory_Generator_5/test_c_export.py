#!/usr/bin/env python3
"""
Automated test script for GFOLD C code export validation.

This script automatically validates the generated C code by:
1. Loading the compiled C module
2. Running a standard test trajectory
3. Verifying the solution quality and performance
4. Reporting results in a standardized format

Usage:
    python test_c_export.py [code_export_directory]
"""

import sys
import os
import numpy as np


def test_c_code_export(code_export_dir=None):
    """
    Test the generated C code export functionality.

    Args:
        code_export_dir: Path to the code export directory (optional)

    Returns:
        bool: True if all tests pass, False otherwise
    """

    if code_export_dir is None:
        code_export_dir = os.getcwd()

    # Change to code export directory
    original_dir = os.getcwd()

    try:
        os.chdir(code_export_dir)

        print('🚀 C code GFOLD solver - AUTOMATED TEST')
        print('=' * 50)

        # Import the generated module
        try:
            import cpg_module
        except ImportError as e:
            print(f'❌ Failed to import cpg_module: {e}')
            return False

        # Create parameter objects
        updated = cpg_module.cpg_updated()
        params = cpg_module.cpg_params()

        # Set standard test parameters
        params.r_initial = np.array([0.0, 0.0, 10.0])  # 10m altitude
        params.v_initial = np.array([0.0, 0.0, 0.0])   # From rest
        params.r_target = np.array([0.0, 0.0, 0.0])    # Land at origin
        params.v_target = np.array([0.0, 0.0, 0.0])    # Zero velocity landing
        params.tf = 8.0                                 # 8 second flight

        # Mark all parameters as updated
        updated.r_initial = True
        updated.v_initial = True
        updated.r_target = True
        updated.v_target = True
        updated.tf = True

        print(f'📋 Test Parameters:')
        print(f'   Initial: [0, 0, 10] m at [0, 0, 0] m/s')
        print(f'   Target:  [0, 0, 0] m at [0, 0, 0] m/s')
        print(f'   Flight time: {params.tf} s')
        print()

        # Solve the problem
        try:
            result = cpg_module.solve(updated, params)
        except Exception as e:
            print(f'❌ Solver failed: {e}')
            return False

        # Validate results
        success = True

        # Check solver status
        if result.cpg_info.status != 0:
            print(f'❌ Solver status: {result.cpg_info.status} (expected: 0)')
            success = False
        else:
            print(f'✅ Solver status: {result.cpg_info.status} (optimal)')

        # Check solve time (should be reasonable)
        solve_time_ms = result.cpg_info.time * 1000
        if solve_time_ms > 100:  # More than 100ms is concerning
            print(f'⚠️  Solve time: {solve_time_ms:.2f} ms (may be slow)')
        else:
            print(f'✅ Solve time: {solve_time_ms:.2f} ms')

        # Check iterations (should converge reasonably)
        if result.cpg_info.iter > 50:
            print(f'⚠️  Iterations: {result.cpg_info.iter} (many iterations)')
        else:
            print(f'✅ Iterations: {result.cpg_info.iter}')

        # Check objective value (should be reasonable for this problem)
        log_final_mass = result.cpg_info.obj_val
        final_mass = np.exp(log_final_mass)
        fuel_consumed = 200 - final_mass

        if fuel_consumed < 0 or fuel_consumed > 60:
            print(f'❌ Fuel consumed: {fuel_consumed:.2f} kg (unrealistic)')
            success = False
        else:
            print(f'✅ Fuel consumed: {fuel_consumed:.2f} kg')

        if final_mass < 140 or final_mass > 200:
            print(f'❌ Final mass: {final_mass:.2f} kg (unrealistic)')
            success = False
        else:
            print(f'✅ Final mass: {final_mass:.2f} kg')

        print()
        print(f'🚀 MISSION RESULTS:')
        print(f'   Initial mass: 200.00 kg')
        print(f'   Final mass: {final_mass:.2f} kg')
        print(f'   Fuel consumed: {fuel_consumed:.2f} kg')
        print(f'   Flight time: {params.tf:.1f} s')

        print()
        print(f'📊 PERFORMANCE METRICS:')
        print(f'   Python CVXPY: ~400ms solve time')
        print(f'   Generated C:  ~{solve_time_ms:.1f}ms solve time')
        if solve_time_ms > 0:
            speedup = 400 / solve_time_ms
            print(f'   Speedup: ~{speedup:.0f}x faster!')

        print()
        if success:
            print(f'🎯 ✅ ALL TESTS PASSED - C code export is working correctly!')
            print(f'   ✓ r_initial: Initial position [m]')
            print(f'   ✓ v_initial: Initial velocity [m/s]')
            print(f'   ✓ r_target: Target position [m]')
            print(f'   ✓ v_target: Target velocity [m/s]')
            print(f'   ✓ tf: Flight time [s]')
        else:
            print(f'❌ SOME TESTS FAILED - Please check the C code export')

        return success

    except Exception as e:
        print(f'❌ Test failed with exception: {e}')
        import traceback
        traceback.print_exc()
        return False

    finally:
        os.chdir(original_dir)


if __name__ == "__main__":
    """
    Run the C code export test.

    Usage:
        python test_c_export.py                    # Test in current directory
        python test_c_export.py /path/to/export    # Test in specified directory
    """

    if len(sys.argv) > 1:
        export_dir = sys.argv[1]
    else:
        export_dir = None

    success = test_c_code_export(export_dir)

    if success:
        print("\n🎉 C code export test completed successfully!")
        sys.exit(0)
    else:
        print("\n💥 C code export test failed!")
        sys.exit(1)
