/*
 * Copyright (c) 2020-2021 Thornbots
 *
 * This file is part of MCB.
 *
 * MCB is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MCB is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCB.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include "subsystems/drivetrain/ChassisController.hpp"
using namespace subsystems;

ChassisController controller{};

TEST(hello, world) { 
    Pose2d pose1{0, 1, 2};
    Pose2d pose2{0, 1, 2};

    EXPECT_EQ(pose1, pose2); 
}
