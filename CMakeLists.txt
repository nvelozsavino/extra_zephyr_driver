#
# Copyright (c) 2020 Circuit Dojo LLC
#
# SPDX-License-Identifier: Apache-2.0
#


add_subdirectory(drivers)
add_subdirectory_ifdef(CONFIG_EXTRA_UTILS utils)
zephyr_include_directories(include)
zephyr_include_directories(.)
