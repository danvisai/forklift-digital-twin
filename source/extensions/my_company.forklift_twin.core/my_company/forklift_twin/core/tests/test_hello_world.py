# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add
#   suport for async/await tests
#   For most things refer to unittest docs:
#   https://docs.python.org/3/library/unittest.html
import omni.kit.test

from my_company.forklift_twin.core.extension import (
    NVIDIA_ASSET_UNIT_SCALE,
    RACK_COLLIDER_SIZE,
)


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the
# root of module will make it auto-discoverable by omni.kit.test
class Test(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        pass

    # After running each test
    async def tearDown(self):
        pass

    async def test_warehouse_asset_units_match_meter_stage(self):
        self.assertEqual(NVIDIA_ASSET_UNIT_SCALE, 0.01)
        self.assertAlmostEqual(RACK_COLLIDER_SIZE[0], 3.30)
        self.assertAlmostEqual(RACK_COLLIDER_SIZE[1], 0.60)
        self.assertAlmostEqual(RACK_COLLIDER_SIZE[2], 2.00)
